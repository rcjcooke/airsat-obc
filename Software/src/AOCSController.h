#ifndef AOCS_CONTROLLER_H
#define AOCS_CONTROLLER_H

#include <string>
#include <cstdint>

class AOCSController {
public:
    AOCSController(const std::string& device = "/dev/spidev0.0", uint32_t speedHz = 2000000);
    ~AOCSController();

    bool init();
    void closeConnection();
    
    // Core Interface: Drive the bus using pure application variables
    bool transmitCommand(float targetTorque, const float targetThrust[4]);
    
    // Telemetry Getters: Access cached, validated data parameters on demand
    float getLatestMomentum() const;
    uint16_t getLatestPropellant() const;
    uint16_t getTeensyRxErrorCount() const;
    
    // Diagnostic Getters
    uint32_t getLocalRxErrors() const;
    bool isLastTransactionValid() const;

private:
    std::string _devicePath;
    uint32_t _speedHz;
    int _spiFd;
    
    // Cached internal state variables (persisted across un-synced frames)
    float _cachedMomentum;
    uint16_t _cachedPropellant;
    uint16_t _cachedTeensyErrors;
    
    uint32_t _localRxErrors;
    bool _lastTransactionValid;

    // Opaque helper method to hide internal Fletcher-16 logic
    uint16_t calculateFletcher16(const uint8_t* data, size_t count);
};

#endif
