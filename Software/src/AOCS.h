#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"

class AOCS {
public:
    // Physical constraints and design constants for the AirSat Platform
    struct AirSatConstraints {
        static constexpr float SATELLITE_INERTIA   = 0.0500f; // I_sat (kg*m^2)
        static constexpr float MAX_MOTOR_TORQUE    = 0.1500f; // Max Torque limit (N*m)
        static constexpr float MAX_WHEEL_MOMENTUM  = 0.0200f; // Wheel saturation limit (kg*m^2/s)
        
        // Controller Tuning (Proportional Gain chosen for crisp response)
        static constexpr float KP_GAIN             = 0.8000f; 
    };

    struct PhysicsConstants {
        static constexpr float TOTAL_TARGET_RAD  = 2.0f * 3.14159265f;
        static constexpr float ACCEL_TIME_S      = 3.0f;
        static constexpr float COAST_TIME_S      = 9.0f;
        static constexpr float DECEL_TIME_S      = 3.0f;
    };

    AOCS();
    bool initialize();
    
    // Core Timed Execution Path (10Hz)
    void runIteration();

    // Guidance Command API
    void setTargetAttitude(float targetRad);

    void calibrateSensors(uint32_t durationMs = 15000);

private:
    AOCSController _controller;
    FusedAttitudeSensor _attitudeSensor;

    // Control Model State Tracking Variables
    float _targetAttitudeRad;
    float _lastYawRad;
    bool _isFirstIteration;
};

#endif
