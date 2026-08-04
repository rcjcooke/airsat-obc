#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"
#include <chrono>

class AOCS {
public:
    struct AirSatConstraints {
        static constexpr float SATELLITE_INERTIA   = 0.0500f; // I_sat (kg*m^2)
        static constexpr float MAX_MOTOR_TORQUE    = 0.1500f; // Max Torque limit (N*m)
        static constexpr float MAX_WHEEL_MOMENTUM  = 0.0200f; // Wheel saturation limit (kg*m^2/s)
        static constexpr float KP_GAIN             = 0.8000f; 
    };

    struct PhysicsConstants {
        static constexpr float TOTAL_TARGET_RAD  = 2.0f * 3.14159265f;
        static constexpr float ACCEL_TIME_S      = 3.0f;
        static constexpr float COAST_TIME_S      = 9.0f;
        static constexpr float DECEL_TIME_S      = 3.0f;
    };

    AOCS();
    
    // Updated: Accept a runCalibration configuration flag
    bool init(bool runCalibration = true);
    
    void runIteration();
    void setTargetAttitude(float targetRad);
    void calibrateSensors(uint32_t durationMs = 15000);

private:
    AOCSController _controller;
    FusedAttitudeSensor _attitudeSensor;

    float _targetAttitudeRad;
    float _lastAttitudeRad;
    bool _isFirstIteration;
    std::chrono::steady_clock::time_point _lastExecutionTime;

    std::chrono::steady_clock::time_point _lastValidTelemetryTime;
    bool _hasReceivedAnyTelemetry;
};

#endif
