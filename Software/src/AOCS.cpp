#include "AOCS.h"
#include <iostream>

AOCS::AOCS() : _controller("/dev/spidev0.0", 2000000), _attitudeSensor() {}

bool AOCS::initialize() {
    return (_controller.init() && _attitudeSensor.init());
}

void AOCS::runIteration() {
    static uint32_t iterationCounter = 0;
    iterationCounter++;

    // 1. Service state tracking loops
    _attitudeSensor.update();

    // 2. Automated Calibration Lifecycle Sequence:
    // First 15 seconds (150 iterations at 10Hz): Run the calibration routine
    if (iterationCounter < 150) {
        if (iterationCounter == 1) {
            _attitudeSensor.requestCalibrationStart();
        }
        if (iterationCounter % 10 == 0) {
            std::cout << "[AOCS SYSTEM] Calibrating Magnetometer... T-Minus " 
                      << (15 - (iterationCounter / 10)) << "s" << std::endl;
        }
        return; // Halt command processing execution until calibration finishes
    } 
    else if (iterationCounter == 150) {
        _attitudeSensor.requestCalibrationEnd();
        std::cout << "[AOCS SYSTEM] Ready for Flight Mode initialization profiles." << std::endl;
        return;
    }

    // 3. Normal Flight Control Space (Begins at T + 15 seconds)
    float currentYaw = _attitudeSensor.getEstimatedYawHeading();
    float targetTorque = 0.0f; 
    float mockThrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (_attitudeSensor.isSensorHealthy()) {
        float headingError = 0.0f - currentYaw;
        targetTorque = headingError * 0.1f; 
    }

    bool telemFresh = _controller.transmitCommand(targetTorque, mockThrusts);

    std::cout << "[FLIGHT MODE] Calibrated Yaw: " << currentYaw << " rad | "
              << "Torque Output: " << targetTorque << " Nm | "
              << "Link: " << (telemFresh ? "OK" : "DROP ⚠️") << std::endl;
}
