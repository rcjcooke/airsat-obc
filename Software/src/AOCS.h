#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"

class AOCS {
public:
    // Structural constants for the open-loop physics model
    struct PhysicsConstants {
        static constexpr float SATELLITE_INERTIA = 0.0500f; // Satellite I (kg*m^2)
        static constexpr float TOTAL_TARGET_RAD  = 2.0f * 3.14159265f; // 360 degrees
        static constexpr float ACCEL_TIME_S      = 3.0f;    // 3 seconds acceleration
        static constexpr float COAST_TIME_S      = 9.0f;    // 9 seconds constant speed
        static constexpr float DECEL_TIME_S      = 3.0f;    // 3 seconds deceleration
    };

    AOCS();
    bool initialize();
    void runIteration();
    void calibrateSensors(uint32_t durationMs = 15000);

private:
    AOCSController _controller;
    FusedAttitudeSensor _attitudeSensor;
};

#endif
