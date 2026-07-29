#include "AOCS.h"
#include <iostream>

AOCS::AOCS() : _controller("/dev/spidev0.0", 2000000) {}

bool AOCS::initialize() {
    return _controller.init();
}

void AOCS::runIteration() {
    // 1. Core Logic Setup: Define application variables locally
    static float mockTorque = 0.0f;
    float mockThrusts[4] = {10.5f, 10.5f, 10.5f, 10.5f};
    
    mockTorque += 0.002f;
    if (mockTorque > 0.4f) mockTorque = -0.4f;

    // 2. Dispatch commands over the bus
    // This returns false if the telemetry frame is missing or corrupted,
    // but the underlying controller handles the caching internally.
    bool telemetryReceived = _controller.transmitCommand(mockTorque, mockThrusts);

    // 3. Extract and display values from the controller cache
    std::cout << "[AOCS MONITOR] Target Torque: " << mockTorque << " Nm | ";
    
    if (telemetryReceived) {
        std::cout << "Telemetry Status: FRESH ✅ | ";
    } else {
        std::cout << "Telemetry Status: STALE/MISSING 🔄 | ";
    }

    std::cout << "Momentum: " << _controller.getLatestMomentum() << " kg*m^2/s | "
              << "Fuel Remaining: " << _controller.getLatestPropellant() << " | "
              << "Bus Drops (Pi: " << _controller.getLocalRxErrors() 
              << ", Teensy: " << _controller.getTeensyRxErrorCount() << ")" << std::endl;
}
