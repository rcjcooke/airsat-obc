#include "AOCSController.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <cmath>

#pragma pack(push, 1)
struct CommandPayload {
    float torque;
    float thrust[4];
    uint8_t flags; // NEW: Byte index 20 holds flags (0x00 = Valid Cmd, 0xFF = NOP Telemetry Poll Only)
    uint8_t alignment_pad; // Keeps struct balanced at 22 bytes
};

struct TelemetryPayload {
    float momentum;
    uint16_t propellant;
    uint16_t error_count;
    uint8_t padding[14]; 
};

struct CommandFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    CommandPayload payload;
    uint16_t checksum;
};

struct TelemetryFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    TelemetryPayload payload;
    uint16_t checksum;
};
#pragma pack(pop)

AOCSController::AOCSController(const std::string& device, uint32_t speedHz)
    : _devicePath(device), _speedHz(speedHz), _spiFd(-1),
      _cachedMomentum(0.0f), _cachedPropellant(0), _cachedTeensyErrors(0),
      _localRxErrors(0), _lastTransactionValid(false), _lastSentTorque(-999.0f) {
          for (int i = 0; i < 4; ++i) _lastSentThrust[i] = -999.0f;
          _lastCommTime = std::chrono::steady_clock::now();
      }

AOCSController::~AOCSController() { closeConnection(); }

bool AOCSController::init() {
    _spiFd = open(_devicePath.c_str(), O_RDWR);
    if (_spiFd < 0) return false;
    uint8_t mode = SPI_MODE_0, bits = 8;
    if (ioctl(_spiFd, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_MODE, &mode) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &_speedHz) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &_speedHz) < 0) return false;
    return true;
}

void AOCSController::closeConnection() { if (_spiFd >= 0) { close(_spiFd); _spiFd = -1; } }

uint16_t AOCSController::calculateFletcher16(const uint8_t* data, size_t count) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < count; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

bool AOCSController::updateActuators(float targetTorque, const float targetThrust[4]) {
    auto now = std::chrono::steady_clock::now();
    uint32_t elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastCommTime).count();

    // 1. Evaluate Delta Changes
    bool commandChanged = false;
    if (std::abs(targetTorque - _lastSentTorque) > CommConstants::COMMAND_EPSILON) {
        commandChanged = true;
    }
    for (int i = 0; i < 4; ++i) {
        if (std::abs(targetThrust[i] - _lastSentThrust[i]) > CommConstants::COMMAND_EPSILON) {
            commandChanged = true;
        }
    }

    // 2. Decision Matrix Rule:
    if (commandChanged) {
        // Send a true Command Frame immediately
        return executeFullDuplexTransfer(targetTorque, targetThrust, false);
    } 
    else if (elapsedMs >= CommConstants::MIN_COMM_PERIOD_MS) {
        // No new command, but heartbeat interval hit: Send a NOP Telemetry Poll Frame
        return executeFullDuplexTransfer(targetTorque, targetThrust, true);
    }

    // Silence mode active: No bus transmission needed on this pass
    return false;
}

bool AOCSController::executeFullDuplexTransfer(float torque, const float thrust[4], bool isNop) {
    if (_spiFd < 0) return false;

    CommandFrame txFrame;
    txFrame.sync[0] = 0xAA; txFrame.sync[1] = 0x55;
    txFrame.payload.torque = torque;
    std::memcpy(txFrame.payload.thrust, thrust, sizeof(txFrame.payload.thrust));
    
    // Inject the flag identifier
    txFrame.payload.flags = isNop ? 0x22 : 0x11;
    txFrame.payload.alignment_pad = 0x00;
    
    txFrame.checksum = calculateFletcher16(reinterpret_cast<const uint8_t*>(&txFrame), 24);

    TelemetryFrame rxFrame;
    std::memset(&rxFrame, 0, sizeof(TelemetryFrame));

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(&txFrame);
    tr.rx_buf = reinterpret_cast<unsigned long>(&rxFrame);
    tr.len = sizeof(CommandFrame);
    tr.speed_hz = _speedHz;
    tr.bits_per_word = 8;

    _lastCommTime = std::chrono::steady_clock::now();

    if (ioctl(_spiFd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        _lastTransactionValid = false;
        return false;
    }

    if (rxFrame.sync[0] == 0xAA && rxFrame.sync[1] == 0x55) {
        uint16_t calculated = calculateFletcher16(reinterpret_cast<const uint8_t*>(&rxFrame), 24);
        if (calculated == rxFrame.checksum) {
            _cachedMomentum     = rxFrame.payload.momentum;
            _cachedPropellant   = rxFrame.payload.propellant;
            _cachedTeensyErrors = rxFrame.payload.error_count;
            _lastTransactionValid = true;

            // Only update local command caches if this wasn't a NOP frame
            if (!isNop) {
                _lastSentTorque = torque;
                std::memcpy(_lastSentThrust, thrust, sizeof(_lastSentThrust));
            }
            return true;
        }
    }

    _localRxErrors++;
    _lastTransactionValid = false;
    return false;
}

float AOCSController::getLatestMomentum() const       { return _cachedMomentum; }
uint16_t AOCSController::getLatestPropellant() const   { return _cachedPropellant; }
uint16_t AOCSController::getTeensyRxErrorCount() const { return _cachedTeensyErrors; }
uint32_t AOCSController::getLocalRxErrors() const     { return _localRxErrors; }
bool AOCSController::isLastTransactionValid() const    { return _lastTransactionValid; }
