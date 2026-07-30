#include "AOCS.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

AOCS::AOCS() : _controller("/dev/spidev0.0", 2000000), _attitudeSensor() {}

bool AOCS::initialize() {
    bool spiOk = _controller.init();
    bool i2cOk = _attitudeSensor.init();
    
    if (!spiOk || !i2cOk) {
        std::cerr << "[AOCS SYSTEM] Initialization halted due to hardware bus failure." << std::endl;
        return false;
    }

    std::cout << "[AOCS SYSTEM] Boot sequence complete. Initiating open-loop calibration maneuver..." << std::endl;
    calibrateSensors(15000); // 15 seconds matches our 3s + 9s + 3s profile exactly

    return true;
}

void AOCS::calibrateSensors(uint32_t durationMs) {
    std::cout << "[AOCS MANEUVER] Command: Perform open-loop 360-degree calibration turn." << std::endl;
    
    // 1. Pre-calculate the exact torque profile boundaries based on satellite inertia
    // alpha = total_angle / (t_accel * (t_accel + t_coast))
    float alphaSat = PhysicsConstants::TOTAL_TARGET_RAD / 
                     (PhysicsConstants::ACCEL_TIME_S * (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S));
                     
    // Torque = I * alpha
    float targetTorqueNm = PhysicsConstants::SATELLITE_INERTIA * alphaSat;
    
    std::cout << "[AOCS MANEUVER] Calculated Open-Loop Torque Step: " << targetTorqueNm << " N*m" << std::endl;

    // Signal sensor suite to begin tracking limits
    _attitudeSensor.requestCalibrationStart();

    auto startTime = std::chrono::steady_clock::now();
    auto samplePeriod = std::chrono::milliseconds(10); // Maintain highly accurate 100Hz execution steps
    
    uint32_t lastLogTimeMs = 0;
    float elapsedSeconds = 0.0f;
    float currentCommandedTorque = 0.0f;
    float dummyThrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    while ((elapsedSeconds * 1000.0f) < durationMs) {
        auto iterStart = std::chrono::steady_clock::now();

        // 2. Determine required torque output based on elapsed trajectory timeline
        if (elapsedSeconds < PhysicsConstants::ACCEL_TIME_S) {
            // Phase 1: Accelerate the platform smoothly forward
            currentCommandedTorque = targetTorqueNm;
        } 
        else if (elapsedSeconds < (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S)) {
            // Phase 2: Constant speed coasting window (Zero acceleration torque required)
            currentCommandedTorque = 0.0f;
        } 
        else if (elapsedSeconds < (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S + PhysicsConstants::DECEL_TIME_S)) {
            // Phase 3: Decelerate the platform to bring it to a precise halt at 360 degrees
            currentCommandedTorque = -targetTorqueNm;
        } 
        else {
            currentCommandedTorque = 0.0f;
        }

        // 3. Simultaneously push open-loop torque to Teensy AND sample the magnetometer lines
        _controller.transmitCommand(currentCommandedTorque, dummyThrusts);
        _attitudeSensor.updateCalibration();

        auto iterEnd = std::chrono::steady_clock::now();
        elapsedSeconds = std::chrono::duration<float>(iterEnd - startTime).count();

        // Log ongoing progress to console once per second
        uint32_t elapsedMs = static_cast<uint32_t>(elapsedSeconds * 1000.0f);
        if (elapsedMs - lastLogTimeMs >= 1000) {
            lastLogTimeMs = elapsedMs;
            std::cout << "[AOCS MANEUVER] Rotating... Time: " << elapsedSeconds << "s / " 
                      << (durationMs / 1000) << "s | Out Torque: " << currentCommandedTorque << " Nm" << std::endl;
        }

        auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart);
        if (timeSpent < samplePeriod) {
            std::this_thread::sleep_for(samplePeriod - timeSpent);
        }
    }

    // 4. Force safe-state brake command to reset the wheel profile post-calibration
    currentCommandedTorque = 0.0f;
    _controller.transmitCommand(currentCommandedTorque, dummyThrusts);

    _attitudeSensor.requestCalibrationEnd();
    std::cout << "[AOCS MANEUVER] Rotation complete. Sensor offsets successfully saved. Entering Flight Mode." << std::endl;
}

void AOCS::runIteration() {
    _attitudeSensor.update();
    
    float currentYaw = _attitudeSensor.getEstimatedYawHeading();
    float targetTorque = 0.0f; 
    float mockThrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Ensure brackets are used here too if missing

    if (_attitudeSensor.isSensorHealthy()) {
        float headingError = 0.0f - currentYaw;
        targetTorque = headingError * 0.1f; 
    }

    if (!_controller.transmitCommand(targetTorque, mockThrusts)) {
        std::cerr << "[AOCS WARNING] SPI Bus Drop: Handshake with Teensy failed." << std::endl;
    }

    std::cout << "[FLIGHT MODE] Yaw: " << currentYaw << " rad | "
              << "Torque: " << targetTorque << " Nm | "
              << "Wheel Momentum: " << _controller.getLatestMomentum() << " kg*m^2/s" << std::endl;
}
