#include "AOCS.h"
#include <iostream>

AOCS::AOCS() : _controller("/dev/spidev0.0", 2000000), _attitudeSensor() {}

bool AOCS::initialize() {
    // Both interfaces must return successfully to clear initialization checks
    bool spiOk = _controller.init();
    bool i2cOk = _attitudeSensor.init();
    return (spiOk && i2cOk);
}

void AOCS::runIteration() {
    // 1. Refresh internal attitude sensor states
    _attitudeSensor.update();
    float currentYaw = _attitudeSensor.getEstimatedYawHeading();

    // 2. High-Level Flight Control Logic Blueprint
    // The target torque from the Pi can now respond directly to actual physical sensor measurements!
    float targetTorque = 0.0f; 
    float mockThrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (_attitudeSensor.isSensorHealthy()) {
        // Example Proportional Control tracking strategy: try to steer the vehicle back to 0.0 radians
        float headingError = 0.0f - currentYaw;
        targetTorque = headingError * 0.1f; // Loop gain multiplier conversion factor
    }

    // 3. Command synchronization handshake pass down over SPI bus
    bool telemFresh = _controller.transmitCommand(targetTorque, mockThrusts);

    // 4. Trace comprehensive systems feedback status metrics
    std::cout << "[FLIGHT ITERATION] Yaw Angle: " << currentYaw << " rad | "
              << "Command Torque: " << targetTorque << " Nm | "
              << "Teensy Momentum Feedback: " << _controller.getLatestMomentum() << " kg*m^2/s | "
              << "Link: " << (telemFresh ? "OK" : "DROP ⚠️") << std::endl;
}
