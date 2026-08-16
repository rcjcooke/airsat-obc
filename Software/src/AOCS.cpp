#include "AOCS.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

AOCS::AOCS() 
    : m_aocsHardwareLink("/dev/spidev0.0", AOCSController::CommConstants::DEFAULT_SPI_SPEED_HZ), 
      m_attitudeSensor(),
      m_controlAlgorithm() {}

bool AOCS::init(bool runCalibration) {
    bool aocsControllerOk = m_aocsHardwareLink.init();
    bool sensorsOk = m_attitudeSensor.init();
    
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

    m_lastExecutionTime = std::chrono::steady_clock::now();
    m_lastReportTime = m_lastExecutionTime;
    return true;
}

void AOCS::setTargetAttitude(float targetRad) {
    m_targetAttitudeRad = std::atan2(std::sin(targetRad), std::cos(targetRad));
    std::cout << "[GUIDANCE] New target attitude set: " << m_targetAttitudeRad << " rad" << std::endl;
}

void AOCS::calibrateSensors(uint32_t durationMs) {
    float alphaSat = CalibrationProcess::TOTAL_TARGET_RAD / 
                     (CalibrationProcess::ACCEL_TIME_S * (CalibrationProcess::ACCEL_TIME_S + CalibrationProcess::COAST_TIME_S));
    float targetTorqueNm = ControlAlgorithm::AirSatConstraints::SATELLITE_INERTIA * alphaSat;

    m_attitudeSensor.requestCalibrationStart();
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
        m_aocsHardwareLink.sendNewCommand(currentCommandedTorque, dummyThrusts);
        if (m_aocsHardwareLink.update()) {
            m_lastValidTelemetryTime = std::chrono::steady_clock::now();
            m_hasReceivedAnyTelemetry = true;
        }
        
        m_attitudeSensor.updateCalibration();

        auto iterEnd = std::chrono::steady_clock::now();
        elapsedSeconds = std::chrono::duration<float>(iterEnd - startTime).count();

        auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart);
        if (timeSpent < samplePeriod) {
            std::this_thread::sleep_for(samplePeriod - timeSpent);
        }
    }

    currentCommandedTorque = 0.0f;
    m_aocsHardwareLink.sendNewCommand(currentCommandedTorque, dummyThrusts);
    m_attitudeSensor.requestCalibrationEnd();
}

void AOCS::updateSubsystems(std::chrono::steady_clock::time_point time) {
    m_attitudeSensor.update();
    bool telemetryReceived = m_aocsHardwareLink.update();
    if (telemetryReceived) {
        m_lastValidTelemetryTime = time;
        m_hasReceivedAnyTelemetry = true;
    }
}

void AOCS::executeSafetyChecks() {
    if (!m_attitudeSensor.isSensorHealthy()) {
        float safeThrust[] = {0.0f, 0.0f, 0.0f, 0.0f};
        m_aocsHardwareLink.sendNewCommand(0.0f, safeThrust);
    }
}

void AOCS::updateTelemetry(std::chrono::steady_clock::time_point time) {
    // Get time delta since last update
    float dt = std::chrono::duration<float>(time - m_lastExecutionTime).count();

    // Get latest readings from sensors and AOCS hardware
    m_airsatAttitudeRad = m_attitudeSensor.getEstimatedAttitude();
    m_angularMomentum = m_aocsHardwareLink.getLatestMomentum();
    m_remainingPropellant = m_aocsHardwareLink.getLatestPropellant();

    // Compute angular velocity based on change in attitude over time
    m_airsatAngularVelocityRadS = 0.0f;
    if (!m_isFirstIteration) {
        // We need time to have passed to calculate a velocity
        if (dt > 0.00001f) {
            float delta = m_airsatAttitudeRad - m_lastAttitudeRad;
            delta = std::atan2(std::sin(delta), std::cos(delta));
            m_airsatAngularVelocityRadS = delta / dt; 
        }
    } else {
        m_isFirstIteration = false;
        m_lastAttitudeRad = m_airsatAttitudeRad;
        return; 
    }
    m_lastAttitudeRad = m_airsatAttitudeRad;
}

void AOCS::printTelemetryReport(std::chrono::steady_clock::time_point time, const ControlAlgorithm::ControlCommands& commands) {
    std::cout << "-------------------- AOCS TELEMETRY REPORT --------------------" << std::endl;
    std::cout << "[TIME] Current Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << " ms" << std::endl;

    float telemetryAgeSeconds = std::chrono::duration<float>(time - m_lastValidTelemetryTime).count();

    auto radToDeg = [](double radians) -> double {
        return radians * (180.0 / M_PI);
    };
    
    std::cout << "[SENSORS STATUS] Health: " << (m_attitudeSensor.isSensorHealthy() ? "OK" : "FAULT") 
                << " | Faults: " << m_attitudeSensor.getSensorFaultCount() << std::endl;

    std::cout << "[SENSORS] Attitude: " << radToDeg(m_airsatAttitudeRad) << "° | Angular Velocity: " << radToDeg(m_airsatAngularVelocityRadS) << "°/s" 
                << " | Momentum: " << m_angularMomentum << " kg.m²/s"
                << " | Remaining Propellant: " << m_remainingPropellant << " kg" << std::endl;
    
    std::cout << "[CONTROL LOOP] Target Attitude: " << radToDeg(m_targetAttitudeRad) << "° | Commanded Torque: " << commands.torque << " Nm" << std::endl;

    std::cout << "[AOCS Controller Link] Tx State: " << (m_aocsHardwareLink.isLastTransactionValid() ? "SUCCESS" : "BUS IDLE/DROP")
                << " | Pi Rx Drops: " << m_aocsHardwareLink.getLocalRxErrorCount()
                << " | Teensy Rx Drops: " << m_aocsHardwareLink.getTeensyRxErrorCount();
                
    if (m_hasReceivedAnyTelemetry) {
        std::cout << " | Last Telem Received: " << telemetryAgeSeconds << "s ago" << std::endl;
    } else {
        std::cout << " | Last Telem Received: NEVER" << std::endl;
    }
    std::cout << "---------------------------------------------------------------" << std::endl;
}

void AOCS::update() {
    std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
    m_lastExecutionTime = currentTime;

    // Make sure all the subsystems are doing their thing
    updateSubsystems(currentTime);
    // Safety first...
    executeSafetyChecks();
    // Update all the readings
    updateTelemetry(currentTime);

    // Work out what to tell the actuators to do
    ControlAlgorithm::ControlCommands commands = m_controlAlgorithm.computeControlCommands(
        m_targetAttitudeRad, m_airsatAttitudeRad, m_airsatAngularVelocityRadS, m_angularMomentum, m_remainingPropellant
    );
    // Action the commands
    m_aocsHardwareLink.sendNewCommand(commands.torque, commands.thrust);

    // Do some reporting if it's that time again
    if (currentTime - m_lastReportTime >= REPORT_INTERVAL) {
        m_lastReportTime = currentTime;
        printTelemetryReport(currentTime, commands);
    }
}


