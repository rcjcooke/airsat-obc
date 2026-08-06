#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"
#include "ControlAlgorithm.h"
#include <chrono>

class AOCS {
public:

    struct CalibrationProcess {
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

    ControlAlgorithm _controlAlgorithm;

    float _targetAttitudeRad;
    float _lastAttitudeRad;
    bool _isFirstIteration;
    std::chrono::steady_clock::time_point _lastExecutionTime;

    std::chrono::steady_clock::time_point _lastValidTelemetryTime;
    bool _hasReceivedAnyTelemetry;
};

#endif
