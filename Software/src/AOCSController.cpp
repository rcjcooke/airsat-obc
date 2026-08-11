#include "AOCSController.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "AOCSPacketStructures.h"

AOCSController::AOCSController(const std::string& device, uint32_t speedHz, int csGpioPin)
    : _devicePath(device), _speedHz(speedHz), _spiFd(-1), _csGpioPin(csGpioPin), _csGpioConfigured(false),
      _cachedMomentum(0.0f), _cachedPropellant(0), _cachedTeensyErrors(0),
      _localRxErrors(0), _lastTransactionValid(false), _rxStreamBuffer{}, _rxStreamSize(0), _lastSentTorque(0.0f) {
          for (int i = 0; i < 4; ++i) _lastSentThrust[i] = 0.0f;
          _lastCommTime = std::chrono::steady_clock::now();
      }

AOCSController::~AOCSController() { closeConnection(); }

bool AOCSController::init() {
    _spiFd = wiringPiSPISetupMode(0, _speedHz, SPI_MODE_0);
    if (_spiFd < 0) {
        std::cerr << "[SPI] Failed to open WiringPi SPI device" << std::endl;
        return false;
    }

    // Set and check the SPI parameters explicitly using ioctl to ensure they are set correctly
    uint8_t mode = SPI_MODE_0;
    if (ioctl(_spiFd, SPI_IOC_WR_MODE, &mode) < 0) { return false; }
    if (ioctl(_spiFd, SPI_IOC_RD_MODE, &mode) < 0) { return false; }

    uint8_t bits = 8;
    if (ioctl(_spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { return false; }
    if (ioctl(_spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) { return false; }

    uint32_t speed = _speedHz;
    if (ioctl(_spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) { return false; }
    if (ioctl(_spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) { return false; }

    std::cout << "[SPI] mode=" << (int)mode
    << " bits=" << (int)bits
    << " speed=" << speed << std::endl;

    return true;
}

void AOCSController::closeConnection() {
    if (_spiFd >= 0) {
        setCsState(true);
        _spiFd = -1;
    }
}

bool AOCSController::configureCsGpio() {
    if (_csGpioConfigured) return true;

    pinMode(_csGpioPin, OUTPUT);
    digitalWrite(_csGpioPin, HIGH);

    _csGpioConfigured = true;
    return true;
}

void AOCSController::setCsState(bool asserted) {
    if (!_csGpioConfigured) {
        return;
    }

    digitalWrite(_csGpioPin, asserted ? HIGH : LOW);
}

uint16_t AOCSController::calculateFletcher16(const uint8_t* data, size_t count) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < count; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

void AOCSController::appendRxBytes(const uint8_t* data, std::size_t count) {
    if (data == nullptr || count == 0) {
        return;
    }

    if (count >= _rxStreamBuffer.size()) {
        std::memcpy(_rxStreamBuffer.data(), data + count - _rxStreamBuffer.size(), _rxStreamBuffer.size());
        _rxStreamSize = _rxStreamBuffer.size();
        return;
    }

    if (_rxStreamSize + count > _rxStreamBuffer.size()) {
        const std::size_t overflow = (_rxStreamSize + count) - _rxStreamBuffer.size();
        std::memmove(_rxStreamBuffer.data(), _rxStreamBuffer.data() + overflow, _rxStreamSize - overflow);
        _rxStreamSize -= overflow;
    }

    std::memcpy(_rxStreamBuffer.data() + _rxStreamSize, data, count);
    _rxStreamSize += count;
}

bool AOCSController::tryConsumeTelemetryFrame(TelemetryFrame* frame) {
    if (frame == nullptr || _rxStreamSize < AOCSPacketConstants::kFrameSize) {
        return false;
    }

    for (std::size_t start = 0; start + AOCSPacketConstants::kFrameSize <= _rxStreamSize; ++start) {
        if (_rxStreamBuffer[start] != AOCSPacketConstants::kSyncByte0 ||
            _rxStreamBuffer[start + 1] != AOCSPacketConstants::kSyncByte1) {
            continue;
        }

        TelemetryFrame candidate;
        std::memcpy(&candidate, _rxStreamBuffer.data() + start, sizeof(candidate));
        const uint16_t calculated = calculateFletcher16(
            reinterpret_cast<const uint8_t*>(&candidate),
            AOCSPacketConstants::kFrameSize - AOCSPacketConstants::kChecksumSize);

        if (calculated == candidate.checksum) {
            *frame = candidate;

            const std::size_t consumed = start + AOCSPacketConstants::kFrameSize;
            const std::size_t remaining = _rxStreamSize - consumed;
            if (remaining > 0) {
                std::memmove(_rxStreamBuffer.data(), _rxStreamBuffer.data() + consumed, remaining);
            }
            _rxStreamSize = remaining;
            return true;
        }
    }

    if (_rxStreamSize >= AOCSPacketConstants::kFrameSize) {
        const std::size_t retained = AOCSPacketConstants::kFrameSize - 1;
        std::memmove(_rxStreamBuffer.data(), _rxStreamBuffer.data() + (_rxStreamSize - retained), retained);
        _rxStreamSize = retained;
    }

    return false;
}

bool AOCSController::update() {
    auto now = std::chrono::steady_clock::now();
    uint32_t elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastCommTime).count();

    // Work out whether the command has meaningfully changed
    bool commandChanged = false;
    if (std::abs(_commandedTorque - _lastSentTorque) > CommConstants::COMMAND_EPSILON) {
        commandChanged = true;
    }
    for (int i = 0; i < 4; ++i) {
        if (std::abs(_commandedThrust[i] - _lastSentThrust[i]) > CommConstants::COMMAND_EPSILON) {
            commandChanged = true;
        }
    }

    // Only send the command if the values have changed
    if (commandChanged) {
        return executeFullDuplexTransfer(_commandedTorque, _commandedThrust, false);
    } 
    else if (elapsedMs >= CommConstants::MIN_COMM_PERIOD_MS) {
        // No new command, but heartbeat interval hit: Send a NOP Telemetry Poll Frame
        return executeFullDuplexTransfer(_commandedTorque, _commandedThrust, true);
    }

    // Silence mode active: No bus transmission needed on this pass
    return false;
}

void AOCSController::sendNewCommand(float targetTorque, const float targetThrust[4]) {
    _commandedTorque = targetTorque;
    std::memcpy(_commandedThrust, targetThrust, sizeof(_commandedThrust));
}

// Uncomment this line when testing with a physical loopback cable on the Pi pins.
//#define SPI_LOOPBACK_TEST 

bool AOCSController::executeFullDuplexTransfer(float torque, const float thrust[4], bool isNop) {
    if (_spiFd < 0) return false;

    // 1. Pack the operational command frame exactly like production
    CommandFrame txFrame;
    txFrame.sync[0] = AOCSPacketConstants::kSyncByte0;
    txFrame.sync[1] = AOCSPacketConstants::kSyncByte1;
    txFrame.payload.torque = torque;
    std::memcpy(txFrame.payload.thrust, thrust, sizeof(txFrame.payload.thrust));
    txFrame.payload.flags = isNop ? 0x22 : 0x11;
    txFrame.payload.alignment_pad = 0x00;
    txFrame.checksum = calculateFletcher16(
        reinterpret_cast<const uint8_t*>(&txFrame),
        AOCSPacketConstants::kFrameSize - AOCSPacketConstants::kChecksumSize);

    auto logRawFrame = [](const char* label, const uint8_t* bytes, size_t count) {
        std::cout << label << " ";
        for (size_t i = 0; i < count; ++i) {
            std::cout << std::setw(2) << std::setfill('0') << std::hex
                      << std::uppercase << static_cast<int>(bytes[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    };

    if (CommConstants::DEBUG) {
        logRawFrame("[SPI TRANSMIT] Outbound Frame Hex Dump: ", reinterpret_cast<const uint8_t*>(&txFrame), sizeof(CommandFrame));
    }

    unsigned char txrxBuffer[AOCSPacketConstants::kFrameSize];
    std::memcpy(txrxBuffer, &txFrame, sizeof(txFrame));

    _lastCommTime = std::chrono::steady_clock::now();

#ifdef SPI_LOOPBACK_TEST
    // ------------------------------------------------------------------------
    // PATH A: LOOPBACK TEST MODE
    // ------------------------------------------------------------------------
    // Instantiate a structurally correct CommandFrame to capture the returned bytes
    CommandFrame rxLoopbackFrame;
    std::memset(&rxLoopbackFrame, 0, sizeof(CommandFrame));
    
    setCsState(false);
    usleep(1000);
    if (wiringPiSPIDataRW(0, txrxBuffer, sizeof(txrxBuffer)) < 0) { 
        _lastTransactionValid = false;
        return false;
    }

    if (CommConstants::DEBUG) {
        logRawFrame("[SPI RECEIVE] Loopback Frame Hex Dump:", txrxBuffer, sizeof(txrxBuffer));
    }
    usleep(1000);
    setCsState(true);

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

    // txrxBuffer gets overwritten with receive data in transfer
    if (wiringPiSPIDataRW(0, txrxBuffer, sizeof(txrxBuffer)) < 0) { 
        _lastTransactionValid = false;
        return false;
    }

    if (CommConstants::DEBUG) {
        logRawFrame("[SPI RECEIVE] Inbound Frame Hex Dump:   ", txrxBuffer, sizeof(txrxBuffer));
    }

    appendRxBytes(txrxBuffer, sizeof(txrxBuffer));

    TelemetryFrame rxFrame;
    if (tryConsumeTelemetryFrame(&rxFrame)) {
        _cachedMomentum     = rxFrame.payload.storedAngularMomentum;
        _cachedPropellant   = rxFrame.payload.propellant;
        _cachedTeensyErrors = rxFrame.payload.error_count;
        _lastTransactionValid = true;

        if (!isNop) {
            _lastSentTorque = torque;
            std::memcpy(_lastSentThrust, thrust, sizeof(_lastSentThrust));
        }
        return true;
    }
#endif

    _localRxErrors++;
    _lastTransactionValid = false;
    return false;
}

float AOCSController::getLatestMomentum() const       { return _cachedMomentum; }
uint16_t AOCSController::getLatestPropellant() const   { return _cachedPropellant; }
uint16_t AOCSController::getTeensyRxErrorCount() const { return _cachedTeensyErrors; }
uint32_t AOCSController::getLocalRxErrorCount() const     { return _localRxErrors; }
bool AOCSController::isLastTransactionValid() const    { return _lastTransactionValid; }
