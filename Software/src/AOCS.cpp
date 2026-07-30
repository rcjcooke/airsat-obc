#include "AOCS.h"
#include <iostream>
#include <thread>
#include <chrono>

AOCS::AOCS() : _controller("/dev/spidev0.0", 2000000), _attitudeSensor() {}

bool AOCS::initialize() {
    bool spiOk = _controller.init();
    bool i2cOk = _attitudeSensor.init();
    
    if (!spiOk || !i2cOk) {
        std::cerr << "[AOCS SYSTEM] Initialization halted due to hardware bus failure." << std::endl;
        return false;
    }

    std::cout << "[AOCS SYSTEM] Beginning pre-flight sensor initialization profiles..." << std::endl;
    calibrateSensors(15000); // Triggers systemic calibration sequence pass across 15 seconds

    return true;
}

void AOCS::calibrateSensors(uint32_t durationMs) {
    std::cout << "[AOCS CALIBRATION] Starting global subsystem sensor calibration. Rotate vehicle smoothly..." << std::endl;
    
    // Signal the sensor suite manager layer to enter calibration configuration mode
    _attitudeSensor.requestCalibrationStart();

    auto startTime = std::chrono::steady_clock::now();
    auto totalDuration = std::chrono::milliseconds(durationMs);
    auto samplePeriod = std::chrono::milliseconds(10); // Maintain a robust 100Hz sampling cycle pass
    
    uint32_t lastLogTimeMs = 0;
    uint32_t elapsedMs = 0;

    while (elapsedMs < durationMs) {
        auto iterStart = std::chrono::steady_clock::now();

        // Compute step data collections across all linked modules inside the sensor suite
        _attitudeSensor.updateCalibration();

        auto iterEnd = std::chrono::steady_clock::now();
        elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - startTime).count();

        if (elapsedMs - lastLogTimeMs >= 1000) {
            lastLogTimeMs = elapsedMs;
            std::cout << "[AOCS CALIBRATION] Processing active sensor profiles... " 
                      << ((durationMs - elapsedMs) / 1000) << "s remaining." << std::endl;
        }

        auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart);
        if (timeSpent < samplePeriod) {
            std::this_thread::sleep_for(samplePeriod - timeSpent);
        }
    }

    // Command the sensor suite to freeze arrays and store calculated offsets
    _attitudeSensor.requestCalibrationEnd();
    std::cout << "[AOCS CALIBRATION] Global sensor calibration sequence completed successfully." << std::endl;
}

void AOCS::runIteration() {
    _attitudeSensor.update();
    
    float currentYaw = _attitudeSensor.getEstimatedYawHeading();
    float targetTorque = 0.0f; 
    float mockThrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (_attitudeSensor.isSensorHealthy()) {
        float headingError = 0.0f - currentYaw;
        targetTorque = headingError * 0.1f; 
    }

    bool telemFresh = _controller.transmitCommand(targetTorque, mockThrusts);

    std::cout << "[FLIGHT MODE] Yaw: " << currentYaw << " rad | "
              << "Torque: " << targetTorque << " Nm | "
              << "Link Status: " << (telemFresh ? "ONLINE" : "DROP ⚠️") << std::endl;
}
