#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"
#include "ControlAlgorithm.h"
#include <chrono>

class AOCS {
public:
    static constexpr auto REPORT_INTERVAL = std::chrono::milliseconds(500);

    struct CalibrationProcess {
        static constexpr float TOTAL_TARGET_RAD  = 2.0f * 3.14159265f;
        static constexpr float ACCEL_TIME_S      = 3.0f;
        static constexpr float COAST_TIME_S      = 9.0f;
        static constexpr float DECEL_TIME_S      = 3.0f;
    };

    AOCS();
    
    // Updated: Accept a runCalibration configuration flag
    bool init(bool runCalibration = true);
    
    void updateSubsystems(std::chrono::steady_clock::time_point time);
    void executeSafetyChecks();
    void updateTelemetry(std::chrono::steady_clock::time_point time);
    void printTelemetryReport(std::chrono::steady_clock::time_point time, const ControlAlgorithm::ControlCommands& commands);
    void update();

    void setTargetAttitude(float targetRad);
    void calibrateSensors(uint32_t durationMs = 15000);

private:

    // Hardware links
    AOCSController m_aocsHardwareLink;
    FusedAttitudeSensor m_attitudeSensor;

    // Control Algorithm
    ControlAlgorithm m_controlAlgorithm;

    // Execution state variables
    std::chrono::steady_clock::time_point m_lastExecutionTime {std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point m_lastReportTime {std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point m_lastValidTelemetryTime {std::chrono::steady_clock::now()};
    bool m_isFirstIteration {true};
    bool m_hasReceivedAnyTelemetry {false};

    // Telemetry state variables
    float m_targetAttitudeRad {0.0f};
    float m_lastAttitudeRad {0.0f};
    float m_airsatAttitudeRad {0.0f};
    float m_airsatAngularVelocityRadS {0.0f};
    float m_angularMomentum {0.0f};
    float m_remainingPropellant {0.0f};

};

#endif
