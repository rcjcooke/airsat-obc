#include "AOCSController.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>

// Encapsulate packet structures purely inside the compilation unit scope
#pragma pack(push, 1)
struct CommandPayload {
    float torque;
    float thrust[4];
};

struct TelemetryPayload {
    float momentum;
    uint16_t propellant;
    uint16_t error_count;
    uint8_t padding[14]; 
};

struct CommandFrame {
    uint8_t sync[2]; // [0] = 0xAA, [1] = 0x55
    CommandPayload payload;
    uint16_t checksum;
};

struct TelemetryFrame {
    uint8_t sync[2]; // [0] = 0xAA, [1] = 0x55
    TelemetryPayload payload;
    uint16_t checksum;
};
#pragma pack(pop)

AOCSController::AOCSController(const std::string& device, uint32_t speedHz)
    : _devicePath(device), _speedHz(speedHz), _spiFd(-1),
      _cachedMomentum(0.0f), _cachedPropellant(0), _cachedTeensyErrors(0),
      _localRxErrors(0), _lastTransactionValid(false) {}

AOCSController::~AOCSController() {
    closeConnection();
}

bool AOCSController::init() {
    _spiFd = open(_devicePath.c_str(), O_RDWR);
    if (_spiFd < 0) {
        std::cerr << "[ERROR] Failed to open SPI device node: " << _devicePath << std::endl;
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;

    if (ioctl(_spiFd, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_MODE, &mode) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &_speedHz) < 0) return false;
    if (ioctl(_spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &_speedHz) < 0) return false;

    return true;
}

void AOCSController::closeConnection() {
    if (_spiFd >= 0) {
        close(_spiFd);
        _spiFd = -1;
    }
}

uint16_t AOCSController::calculateFletcher16(const uint8_t* data, size_t count) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < count; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

bool AOCSController::transmitCommand(float targetTorque, const float targetThrust[4]) {
    if (_spiFd < 0) return false;

    // 1. Map raw input array parameters into internal Command structures
    CommandFrame txFrame;
    txFrame.sync[0] = 0xAA;
    txFrame.sync[1] = 0x55;
    txFrame.payload.torque = targetTorque;
    std::memcpy(txFrame.payload.thrust, targetThrust, sizeof(txFrame.payload.thrust));
    txFrame.checksum = calculateFletcher16(reinterpret_cast<const uint8_t*>(&txFrame), 24);

    TelemetryFrame rxFrame;
    std::memset(&rxFrame, 0, sizeof(TelemetryFrame));

    // 2. Configure full duplex message descriptors
    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(&txFrame);
    tr.rx_buf = reinterpret_cast<unsigned long>(&rxFrame);
    tr.len = sizeof(CommandFrame); // 26 bytes
    tr.speed_hz = _speedHz;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    if (ioctl(_spiFd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        _lastTransactionValid = false;
        return false;
    }

    // 3. Process the shifted return frame
    // If the frame lacks valid headers or fails the check, we flag it but don't crash
    if (rxFrame.sync[0] == 0xAA && rxFrame.sync[1] == 0x55) {
        uint16_t calculated = calculateFletcher16(reinterpret_cast<const uint8_t*>(&rxFrame), 24);
        if (calculated == rxFrame.checksum) {
            // Update the persistent cache variables with verified fresh data
            _cachedMomentum     = rxFrame.payload.momentum;
            _cachedPropellant   = rxFrame.payload.propellant;
            _cachedTeensyErrors = rxFrame.payload.error_count;
            _lastTransactionValid = true;
            return true;
        }
    }

    // Telemetry response was invalid/missing on this command pass
    _localRxErrors++;
    _lastTransactionValid = false;
    return false;
}

float AOCSController::getLatestMomentum() const       { return _cachedMomentum; }
uint16_t AOCSController::getLatestPropellant() const   { return _cachedPropellant; }
uint16_t AOCSController::getTeensyRxErrorCount() const { return _cachedTeensyErrors; }
uint32_t AOCSController::getLocalRxErrors() const     { return _localRxErrors; }
bool AOCSController::isLastTransactionValid() const    { return _lastTransactionValid; }
