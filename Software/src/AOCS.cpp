#include "AOCS.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

AOCS::AOCS() 
    : _controller("/dev/spidev0.0", 2000000), _attitudeSensor(),
      _targetAttitudeRad(0.0f), _lastYawRad(0.0f), _isFirstIteration(true) {}

bool AOCS::initialize() {
    bool spiOk = _controller.init();
    bool i2cOk = _attitudeSensor.init();
    
    if (!spiOk || !i2cOk) {
        std::cerr << "[AOCS SYSTEM] Initialization halted due to hardware bus failure." << std::endl;
        return false;
    }

    std::cout << "[AOCS SYSTEM] Boot sequence complete. Initiating open-loop calibration maneuver..." << std::endl;
    calibrateSensors(15000);

    // Seed the initial timestamp immediately after calibration concludes
    _lastExecutionTime = std::chrono::steady_clock::now();

    return true;
}

void AOCS::setTargetAttitude(float targetRad) {
    _targetAttitudeRad = std::atan2(std::sin(targetRad), std::cos(targetRad));
    std::cout << "[GUIDANCE] New target attitude set: " << _targetAttitudeRad << " rad" << std::endl;
}

void AOCS::calibrateSensors(uint32_t durationMs) {
    float alphaSat = PhysicsConstants::TOTAL_TARGET_RAD / 
                     (PhysicsConstants::ACCEL_TIME_S * (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S));
    float targetTorqueNm = AirSatConstraints::SATELLITE_INERTIA * alphaSat;

    _attitudeSensor.requestCalibrationStart();
    auto startTime = std::chrono::steady_clock::now();
    auto samplePeriod = std::chrono::milliseconds(10);
    
    float elapsedSeconds = 0.0f;
    float currentCommandedTorque = 0.0f;
    float dummyThrusts[] = {0.0f, 0.0f, 0.0f, 0.0f};

    while ((elapsedSeconds * 1000.0f) < durationMs) {
        auto iterStart = std::chrono::steady_clock::now();

        if (elapsedSeconds < PhysicsConstants::ACCEL_TIME_S) {
            currentCommandedTorque = targetTorqueNm;
        } else if (elapsedSeconds < (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S)) {
            currentCommandedTorque = 0.0f;
        } else if (elapsedSeconds < (PhysicsConstants::ACCEL_TIME_S + PhysicsConstants::COAST_TIME_S + PhysicsConstants::DECEL_TIME_S)) {
            currentCommandedTorque = -targetTorqueNm;
        } else {
            currentCommandedTorque = 0.0f;
        }

        _controller.transmitCommand(currentCommandedTorque, dummyThrusts);
        _attitudeSensor.updateCalibration();

        auto iterEnd = std::chrono::steady_clock::now();
        elapsedSeconds = std::chrono::duration<float>(iterEnd - startTime).count();

        auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart);
        if (timeSpent < samplePeriod) {
            std::this_thread::sleep_for(samplePeriod - timeSpent);
        }
    }

    currentCommandedTorque = 0.0f;
    _controller.transmitCommand(currentCommandedTorque, dummyThrusts);
    _attitudeSensor.requestCalibrationEnd();
}

void AOCS::runIteration() {
    // 1. Calculate the exact dynamic time delta (dt) since the last execution path
    auto currentTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(currentTime - _lastExecutionTime).count();
    _lastExecutionTime = currentTime; // Update the tracking baseline marker immediately

    // 2. Refresh hardware sensor parameters
    _attitudeSensor.update();
    
    if (!_attitudeSensor.isSensorHealthy()) {
        std::cerr << "[AOCS FAULT] Sensor stream failure. Forcing zero-torque emergency state." << std::endl;
        float safeThrust[] = {0.0f, 0.0f, 0.0f, 0.0f};
        _controller.transmitCommand(0.0f, safeThrust);
        return;
    }

    float currentYaw = _attitudeSensor.getEstimatedYawHeading();

    // 3. Dynamic Velocity Derivative Calculation
    float satelliteVelocityRadS = 0.0f;
    if (!_isFirstIteration) {
        // Guard against zero-division errors if the loop executes faster than system clock resolution
        if (dt > 0.00001f) {
            float deltaYaw = currentYaw - _lastYawRad;
            // Correct for boundary wrap-around conditions over the -PI/+PI limits
            deltaYaw = std::atan2(std::sin(deltaYaw), std::cos(deltaYaw));
            satelliteVelocityRadS = deltaYaw / dt; 
        }
    } else {
        _isFirstIteration = false;
        _lastYawRad = currentYaw;
        return; // Skip the rest of first step since velocity cannot be established yet
    }
    _lastYawRad = currentYaw;

    // 4. Compute Tracking Errors
    float headingError = _targetAttitudeRad - currentYaw;
    headingError = std::atan2(std::sin(headingError), std::cos(headingError));

    // 5. Apply the Attitude Control Model Law using dynamic damping
    float kp = AirSatConstraints::KP_GAIN;
    float kd = 2.0f * std::sqrt(kp * AirSatConstraints::SATELLITE_INERTIA);
    float requestedTorque = (kp * headingError) - (kd * satelliteVelocityRadS);

    // 6. Enforce Physical Hardware Constraints
    if (requestedTorque > AirSatConstraints::MAX_MOTOR_TORQUE)  requestedTorque = AirSatConstraints::MAX_MOTOR_TORQUE;
    if (requestedTorque < -AirSatConstraints::MAX_MOTOR_TORQUE) requestedTorque = -AirSatConstraints::MAX_MOTOR_TORQUE;

    float currentWheelMomentum = _controller.getLatestMomentum();
    if (currentWheelMomentum >= AirSatConstraints::MAX_WHEEL_MOMENTUM && requestedTorque < 0.0f) {
        requestedTorque = 0.0f; 
    }
    else if (currentWheelMomentum <= -AirSatConstraints::MAX_WHEEL_MOMENTUM && requestedTorque > 0.0f) {
        requestedTorque = 0.0f; 
    }

    // 7. Route verified variables down over the SPI link to the platform actuators
    float mockThrusts[] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (!_controller.transmitCommand(requestedTorque, mockThrusts)) {
        std::cerr << "[AOCS WARNING] SPI frame dropped during active control loop step." << std::endl;
    }

    // 8. Debug Telemetry Readout showing precise dynamic dt performance metrics
    std::cout << "[CONTROL LOOP] dt: " << dt << "s | Target: " << _targetAttitudeRad 
              << " | Current: " << currentYaw << " | Error: " << headingError 
              << " | Vel: " << satelliteVelocityRadS << " rad/s | Out Torque: " << requestedTorque << " Nm" << std::endl;
}
