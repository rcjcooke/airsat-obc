#ifndef AOCS_CONTROLLER_H
#define AOCS_CONTROLLER_H

#include <string>
#include <cstdint>
#include <chrono>
#include <array>

#include "AOCSPacketStructures.h"

class AOCSController {
public:
    struct CommConstants {
        // Minimum frequency to request updates from Teensy (200ms = 5Hz)
        static constexpr uint32_t MIN_COMM_PERIOD_MS = 200; 
        static constexpr float COMMAND_EPSILON       = 0.00001f;
        static constexpr bool DEBUG = true;
        static constexpr uint32_t DEFAULT_SPI_SPEED_HZ = 50000;
        static constexpr std::size_t RX_STREAM_BUFFER_SIZE = AOCSPacketConstants::kFrameSize * 4;
    };

    AOCSController(const std::string& device = "/dev/spidev0.0",
                   uint32_t speedHz = CommConstants::DEFAULT_SPI_SPEED_HZ,
                   int csGpioPin = 17);
    ~AOCSController();

    bool init();
    void closeConnection();
    
    // Core Interface: Automatically determines if a write is a Command delta or a NOP telemetry poll
    bool update();
    void sendNewCommand(float targetTorque, const float targetThrust[4]);
    
    float getLatestMomentum() const;
    uint16_t getLatestPropellant() const;
    uint16_t getTeensyRxErrorCount() const;
    uint32_t getLocalRxErrorCount() const;
    bool isLastTransactionValid() const;

private:
    std::string _devicePath;
    uint32_t _speedHz;
    int _spiFd;
    int _csGpioPin;
    bool _csGpioConfigured;

    // The commanded values that are sent to the AOCS Hardware
    float _commandedTorque;
    float _commandedThrust[4];
    
    // Cache variables
    float _cachedMomentum;
    uint16_t _cachedPropellant;
    uint16_t _cachedTeensyErrors;
    uint32_t _localRxErrors;
    bool _lastTransactionValid;
    std::array<uint8_t, CommConstants::RX_STREAM_BUFFER_SIZE> _rxStreamBuffer;
    std::size_t _rxStreamSize;

    // Change detection state engine
    float _lastSentTorque;
    float _lastSentThrust[4];
    std::chrono::steady_clock::time_point _lastCommTime;

    uint16_t calculateFletcher16(const uint8_t* data, size_t count);
    bool configureCsGpio();
    void setCsState(bool asserted);
    void appendRxBytes(const uint8_t* data, std::size_t count);
    bool tryConsumeTelemetryFrame(TelemetryFrame* frame);
    bool executeFullDuplexTransfer(float torque, const float thrust[4], bool isNop);
};

#endif
