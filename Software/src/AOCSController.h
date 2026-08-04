#ifndef AOCS_CONTROLLER_H
#define AOCS_CONTROLLER_H

#include <string>
#include <cstdint>
#include <chrono>

class AOCSController {
public:
    struct CommConstants {
        // Minimum frequency to request updates from Teensy (200ms = 5Hz)
        static constexpr uint32_t MIN_COMM_PERIOD_MS = 200; 
        static constexpr float COMMAND_EPSILON       = 0.00001f;
    };

    AOCSController(const std::string& device = "/dev/spidev0.0", uint32_t speedHz = 2000000, int csGpioPin = 17);
    ~AOCSController();

    bool init();
    void closeConnection();
    
    // Core Interface: Automatically determines if a write is a Command delta or a NOP telemetry poll
    bool updateActuators(float targetTorque, const float targetThrust[4]);
    
    float getLatestMomentum() const;
    uint16_t getLatestPropellant() const;
    uint16_t getTeensyRxErrorCount() const;
    uint32_t getLocalRxErrors() const;
    bool isLastTransactionValid() const;

private:
    std::string _devicePath;
    uint32_t _speedHz;
    int _spiFd;
    int _csGpioPin;
    bool _csGpioConfigured;
    
    // Cache variables
    float _cachedMomentum;
    uint16_t _cachedPropellant;
    uint16_t _cachedTeensyErrors;
    uint32_t _localRxErrors;
    bool _lastTransactionValid;

    // Change detection state engine
    float _lastSentTorque;
    float _lastSentThrust[4];
    std::chrono::steady_clock::time_point _lastCommTime;

    uint16_t calculateFletcher16(const uint8_t* data, size_t count);
    bool configureCsGpio();
    void setCsState(bool asserted);
    bool executeFullDuplexTransfer(float torque, const float thrust[4], bool isNop);
};

#endif
