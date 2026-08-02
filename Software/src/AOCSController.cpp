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
    uint8_t flags;
    uint8_t alignment_pad;
}; // 22 bytes

struct TelemetryPayload {
    float momentum;
    uint16_t propellant;
    uint16_t error_count;
    uint8_t padding[14]; // FIXED: Match the 14-byte array footprint
}; // 22 bytes

struct CommandFrame {
    uint8_t sync[2];
    CommandPayload payload;
    uint16_t checksum;
}; // 26 bytes

struct TelemetryFrame {
    uint8_t sync[2];
    TelemetryPayload payload;
    uint16_t checksum;
}; // 26 bytes
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

    std::cout << "[ALIGNMENT CHECK] CommandFrame Size: " << sizeof(CommandFrame) 
          << " | TelemetryFrame Size: " << sizeof(TelemetryFrame) << std::endl;


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

// Explicitly uncomment this line when testing with a physical loopback cable on the Pi pins.
// Comment it out when deploying production code connected to the Teensy.
#define SPI_LOOPBACK_TEST 

bool AOCSController::executeFullDuplexTransfer(float torque, const float thrust[4], bool isNop) {
    if (_spiFd < 0) return false;

    // 1. Pack the operational command frame exactly like production
    CommandFrame txFrame;
    txFrame.sync[0] = 0xAA; 
    txFrame.sync[1] = 0x55;
    txFrame.payload.torque = torque;
    std::memcpy(txFrame.payload.thrust, thrust, sizeof(txFrame.payload.thrust));
    txFrame.payload.flags = isNop ? 0x22 : 0x11;
    txFrame.payload.alignment_pad = 0x00;
    txFrame.checksum = calculateFletcher16(reinterpret_cast<const uint8_t*>(&txFrame), 24);

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr)); 
    
    tr.tx_buf        = reinterpret_cast<unsigned long>(&txFrame);
    tr.len           = sizeof(CommandFrame); // Exactly 26 bytes
    tr.speed_hz      = _speedHz;
    tr.bits_per_word = 8; 
    tr.cs_change     = 0;  

    _lastCommTime = std::chrono::steady_clock::now();

#ifdef SPI_LOOPBACK_TEST
    // ------------------------------------------------------------------------
    // PATH A: LOOPBACK TEST MODE
    // ------------------------------------------------------------------------
    // Instantiate a structurally correct CommandFrame to capture the returned bytes
    CommandFrame rxLoopbackFrame;
    std::memset(&rxLoopbackFrame, 0, sizeof(CommandFrame));
    
    tr.rx_buf = reinterpret_cast<unsigned long>(&rxLoopbackFrame);

    if (ioctl(_spiFd, SPI_IOC_MESSAGE(1), &tr) < 0) { 
        _lastTransactionValid = false;
        return false;
    }

    // Process the loopback contents using the production sync and checksum rules
    if (rxLoopbackFrame.sync[0] == 0xAA && rxLoopbackFrame.sync[1] == 0x55) {
        uint16_t calculated = calculateFletcher16(reinterpret_cast<const uint8_t*>(&rxLoopbackFrame), 24);
        
        if (calculated == rxLoopbackFrame.checksum) {
            // Checksum matches! Map returned fields to cache so AOCS remains happy
            _cachedMomentum     = rxLoopbackFrame.payload.torque; 
            _cachedPropellant   = 999; 
            _cachedTeensyErrors = 0;
            _lastTransactionValid = true;

            if (!isNop) {
                _lastSentTorque = torque;
                std::memcpy(_lastSentThrust, thrust, sizeof(_lastSentThrust));
            }
            return true;
        }
    }
#else
    // ------------------------------------------------------------------------
    // PATH B: STANDARD PRODUCTION MODE (Connected to Teensy)
    // ------------------------------------------------------------------------
    TelemetryFrame rxFrame;
    std::memset(&rxFrame, 0, sizeof(TelemetryFrame));
    
    tr.rx_buf = reinterpret_cast<unsigned long>(&rxFrame);

    if (ioctl(_spiFd, SPI_IOC_MESSAGE(1), &tr) < 0) { 
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

            if (!isNop) {
                _lastSentTorque = torque;
                std::memcpy(_lastSentThrust, thrust, sizeof(_lastSentThrust));
            }
            return true;
        }
    }
#endif

    _localRxErrors++;
    _lastTransactionValid = false;
    return false;
}

float AOCSController::getLatestMomentum() const       { return _cachedMomentum; }
uint16_t AOCSController::getLatestPropellant() const   { return _cachedPropellant; }
uint16_t AOCSController::getTeensyRxErrorCount() const { return _cachedTeensyErrors; }
uint32_t AOCSController::getLocalRxErrors() const     { return _localRxErrors; }
bool AOCSController::isLastTransactionValid() const    { return _lastTransactionValid; }
