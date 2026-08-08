#include "AOCS.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

AOCS::AOCS() 
    : _controller("/dev/spidev0.0", 100000), _attitudeSensor(),
      _targetAttitudeRad(0.0f), _lastAttitudeRad(0.0f), _isFirstIteration(true),
      _hasReceivedAnyTelemetry(false) {
          _lastValidTelemetryTime = std::chrono::steady_clock::now();
          _lastReportTime = _lastValidTelemetryTime;
          _controlAlgorithm = ControlAlgorithm();
      }

bool AOCS::init(bool runCalibration) {
    bool aocsControllerOk = _controller.init();
    bool sensorsOk = _attitudeSensor.init();
    
    if (!aocsControllerOk || !sensorsOk) {
        std::cerr << "[AOCS SYSTEM] Initialisation failed: "
                  << (!aocsControllerOk ? "AOCS CONTROLLER " : "") << (!sensorsOk ? "SENSORS" : "") << std::endl;
        if (!sensorsOk) {
            // For now, we know that the only error that could prevent initialisation is the following.
            // TODO improve on this with fault codes etc. as more sensors are added.
            std::cerr << "[AOCS SYSTEM] Couldn't establish I2C link to Magnetometer." << std::endl;
        }
        return false;
    }

    if (runCalibration) {
        std::cout << "[AOCS SYSTEM] Boot sequence complete. Initiating open-loop calibration maneuver..." << std::endl;
        calibrateSensors(15000);
    } else {
        std::cout << "[AOCS SYSTEM] Bypassing sensor and platform calibration via command line flag override." << std::endl;
    }

    _lastExecutionTime = std::chrono::steady_clock::now();
    _lastReportTime = _lastExecutionTime;
    return true;
}

void AOCS::setTargetAttitude(float targetRad) {
    _targetAttitudeRad = std::atan2(std::sin(targetRad), std::cos(targetRad));
    std::cout << "[GUIDANCE] New target attitude set: " << _targetAttitudeRad << " rad" << std::endl;
}

void AOCS::calibrateSensors(uint32_t durationMs) {
    float alphaSat = CalibrationProcess::TOTAL_TARGET_RAD / 
                     (CalibrationProcess::ACCEL_TIME_S * (CalibrationProcess::ACCEL_TIME_S + CalibrationProcess::COAST_TIME_S));
    float targetTorqueNm = ControlAlgorithm::AirSatConstraints::SATELLITE_INERTIA * alphaSat;

    _attitudeSensor.requestCalibrationStart();
    auto startTime = std::chrono::steady_clock::now();
    auto samplePeriod = std::chrono::milliseconds(10);
    
    float elapsedSeconds = 0.0f;
    float currentCommandedTorque = 0.0f;
    float dummyThrusts[] = {0.0f, 0.0f, 0.0f, 0.0f};

    while ((elapsedSeconds * 1000.0f) < durationMs) {
        auto iterStart = std::chrono::steady_clock::now();

        if (elapsedSeconds < CalibrationProcess::ACCEL_TIME_S) {
            currentCommandedTorque = targetTorqueNm;
        } else if (elapsedSeconds < (CalibrationProcess::ACCEL_TIME_S + CalibrationProcess::COAST_TIME_S)) {
            currentCommandedTorque = 0.0f;
        } else if (elapsedSeconds < (CalibrationProcess::ACCEL_TIME_S + CalibrationProcess::COAST_TIME_S + CalibrationProcess::DECEL_TIME_S)) {
            currentCommandedTorque = -targetTorqueNm;
        } else {
            currentCommandedTorque = 0.0f;
        }

        // Capture returned handshake frames during calibration rotation sweeps
        if (_controller.sendNewCommand(currentCommandedTorque, dummyThrusts)) {
            _lastValidTelemetryTime = std::chrono::steady_clock::now();
            _hasReceivedAnyTelemetry = true;
        }
        
        _attitudeSensor.updateCalibration();

        auto iterEnd = std::chrono::steady_clock::now();
        elapsedSeconds = std::chrono::duration<float>(iterEnd - startTime).count();

        auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart);
        if (timeSpent < samplePeriod) {
            std::this_thread::sleep_for(samplePeriod - timeSpent);
        }
    }

    currentCommandedTorque = 0.0f;
    _controller.sendNewCommand(currentCommandedTorque, dummyThrusts);
    _attitudeSensor.requestCalibrationEnd();
}

void AOCS::runIteration() {
    auto currentTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(currentTime - _lastExecutionTime).count();
    _lastExecutionTime = currentTime;

    // Make sure all the subsystems are doing their thing
    _attitudeSensor.update();
    bool telemetryReceived = _controller.update();
    if (telemetryReceived) {
        _lastValidTelemetryTime = currentTime;
        _hasReceivedAnyTelemetry = true;
    }
    
    // Safety first...
    if (!_attitudeSensor.isSensorHealthy()) {
        float safeThrust[] = {0.0f, 0.0f, 0.0f, 0.0f};
        _controller.sendNewCommand(0.0f, safeThrust);
    }

    // Update all the readings
    float airsatAttitudeRad = _attitudeSensor.getEstimatedAttitude();
    float airsatAngularVelocityRadS = 0.0f;
    if (!_isFirstIteration) {
        // We need time to have passed to calculate a velocity
        if (dt > 0.00001f) {
            float delta = airsatAttitudeRad - _lastAttitudeRad;
            delta = std::atan2(std::sin(delta), std::cos(delta));
            airsatAngularVelocityRadS = delta / dt; 
        }
    } else {
        _isFirstIteration = false;
        _lastAttitudeRad = airsatAttitudeRad;
        return; 
    }
    _lastAttitudeRad = airsatAttitudeRad;
    float angularMomentum = _controller.getLatestMomentum();
    float remainingPropellant = _controller.getLatestPropellant();

    // Work out what to tell the actuators to do
    ControlAlgorithm::ControlCommands commands = _controlAlgorithm.computeControlCommands(
        _targetAttitudeRad, airsatAttitudeRad, airsatAngularVelocityRadS, angularMomentum, remainingPropellant
    );

    // Action the commands
    _controller.sendNewCommand(commands.torque, commands.thrust);

    // Do some reporting if it's that time again
    if (currentTime - _lastReportTime >= REPORT_INTERVAL) {
        _lastReportTime = currentTime;

        float telemetryAgeSeconds = std::chrono::duration<float>(currentTime - _lastValidTelemetryTime).count();

        auto radToDeg = [](double radians) -> double {
            return radians * (180.0 / M_PI);
        };
        
        std::cout << "[SENSORS STATUS] Health: " << (_attitudeSensor.isSensorHealthy() ? "OK" : "FAULT") 
                  << " | Faults: " << _attitudeSensor.getSensorFaultCount() << std::endl;

        std::cout << "[SENSORS] Attitude: " << radToDeg(airsatAttitudeRad) << "° | Angular Velocity: " << radToDeg(airsatAngularVelocityRadS) << "°/s" 
                  << " | Momentum: " << angularMomentum << " kg*m^2/s"
                  << " | Remaining Propellant: " << remainingPropellant << " kg" << std::endl;
        
        std::cout << "[CONTROL LOOP] Target Attitude: " << radToDeg(_targetAttitudeRad) << "° | Commanded Torque: " << commands.torque << " Nm" << std::endl;
                  

        std::cout << "[AOCS Controller Link] Tx State: " << (_controller.isLastTransactionValid() ? "SUCCESS" : "BUS IDLE/DROP")
                  << " | Pi Rx Drops: " << _controller.getLocalRxErrorCount()
                  << " | Teensy Rx Drops: " << _controller.getTeensyRxErrorCount();
                  
        if (_hasReceivedAnyTelemetry) {
            std::cout << " | Last Telem Received: " << telemetryAgeSeconds << "s ago" << std::endl;
        } else {
            std::cout << " | Last Telem Received: NEVER" << std::endl;
        }
        std::cout << "------------------------------------------------------------------------" << std::endl;
    }
}


