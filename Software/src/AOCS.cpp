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
          _controlAlgorithm = ControlAlgorithm();
      }

bool AOCS::init(bool runCalibration) {
    bool spiOk = _controller.init();
    bool i2cOk = _attitudeSensor.init();
    
    if (!spiOk || !i2cOk) {
        std::cerr << "[AOCS SYSTEM] Initialisation halted due to hardware bus failure: "
                  << (!spiOk ? "SPI " : "") << (!i2cOk ? "I2C" : "") << std::endl;
        return false;
    }

    if (runCalibration) {
        std::cout << "[AOCS SYSTEM] Boot sequence complete. Initiating open-loop calibration maneuver..." << std::endl;
        calibrateSensors(15000);
    } else {
        std::cout << "[AOCS SYSTEM] Bypassing sensor and platform calibration via command line flag override." << std::endl;
    }

    _lastExecutionTime = std::chrono::steady_clock::now();
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
        if (_controller.updateActuators(currentCommandedTorque, dummyThrusts)) {
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
    _controller.updateActuators(currentCommandedTorque, dummyThrusts);
    _attitudeSensor.requestCalibrationEnd();
}

void AOCS::runIteration() {
    auto currentTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(currentTime - _lastExecutionTime).count();
    _lastExecutionTime = currentTime;

    _attitudeSensor.update();
    if (!_attitudeSensor.isSensorHealthy()) {
        float safeThrust[] = {0.0f, 0.0f, 0.0f, 0.0f};
        _controller.updateActuators(0.0f, safeThrust);
        return;
    }

    float currentAttitude = _attitudeSensor.getEstimatedAttitude();
    float satelliteVelocityRadS = 0.0f;
    if (!_isFirstIteration) {
        if (dt > 0.00001f) {
            float delta = currentAttitude - _lastAttitudeRad;
            delta = std::atan2(std::sin(delta), std::cos(delta));
            satelliteVelocityRadS = delta / dt; 
        }
    } else {
        _isFirstIteration = false;
        _lastAttitudeRad = currentAttitude;
        return; 
    }
    _lastAttitudeRad = currentAttitude;

    ControlAlgorithm::ControlCommands commands = _controlAlgorithm.computeControlCommands(
        currentAttitude, satelliteVelocityRadS, _controller.getLatestMomentum(), _controller.getLatestPropellant()
    );

    // This returns true ONLY if a structurally integral validation handshake packet returns.
    bool freshHandshakeReceived = _controller.updateActuators(commands.torque, commands.thrust);
    
    if (freshHandshakeReceived) {
        _lastValidTelemetryTime = currentTime;
        _hasReceivedAnyTelemetry = true;
    }

    float telemetryAgeSeconds = std::chrono::duration<float>(currentTime - _lastValidTelemetryTime).count();

    auto radToDeg = [](double radians) -> double {
        return radians * (180.0 / M_PI);
    };
    
    std::cout << "[CONTROL LOOP] Target: " << radToDeg(_targetAttitudeRad) << "° | Current: " << radToDeg(currentAttitude) 
              << "° | Torque: " << commands.torque << " Nm" << std::endl;
              
    std::cout << "[BUS DIAGNOSTICS] Tx State: " << (_controller.isLastTransactionValid() ? "SUCCESS" : "BUS IDLE/DROP")
              << " | Pi Rx Drops: " << _controller.getLocalRxErrors()
              << " | Teensy Rx Drops: " << _controller.getTeensyRxErrorCount();
              
    if (_hasReceivedAnyTelemetry) {
        std::cout << " | Last Telem Received: " << telemetryAgeSeconds << "s ago" << std::endl;
    } else {
        std::cout << " | Last Telem Received: NEVER" << std::endl;
    }
    std::cout << "------------------------------------------------------------------------" << std::endl;
}


