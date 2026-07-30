#include "GY271Magnetometer.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>

GY271Magnetometer::GY271Magnetometer(const std::string& i2cDevice, uint8_t i2cAddress)
    : _devicePath(i2cDevice), _address(i2cAddress), _i2cFd(-1) {}

GY271Magnetometer::~GY271Magnetometer() {
    closeDevice();
}

bool GY271Magnetometer::init() {
    _i2cFd = open(_devicePath.c_str(), O_RDWR);
    if (_i2cFd < 0) {
        std::cerr << "[I2C ERROR] Failed to open bus device node: " << _devicePath << std::endl;
        return false;
    }

    if (ioctl(_i2cFd, I2C_SLAVE, _address) < 0) {
        std::cerr << "[I2C ERROR] Target device address unacknowledged: 0x" << std::hex << (int)_address << std::endl;
        return false;
    }

    // Configure the QMC5883L Control Registers
    // Register 0x0B (FBR / Period Register) -> Write 0x01 (Recommended initialization byte)
    if (!writeRegister(0x0B, 0x01)) return false;

    // Register 0x09 (Control Register 1) -> Mode Control Setup:
    // Bit 0-1: 01 (Continuous conversion mode)
    // Bit 2-3: 10 (100 Hz Output Data Rate / ODR)
    // Bit 4-5: 01 (Magnetic field full scale scaling: +/- 8 Gauss ceiling configuration)
    // Bit 6-7: 00 (Over-sampling ratio: 512)
    // Combined payload allocation: 0b00011001 = 0x19
    if (!writeRegister(0x09, 0x19)) return false;

    return true;
}

void GY271Magnetometer::closeDevice() {
    if (_i2cFd >= 0) {
        close(_i2cFd);
        _i2cFd = -1;
    }
}

bool GY271Magnetometer::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    if (write(_i2cFd, buffer, 2) != 2) {
        std::cerr << "[I2C] Failed writing byte value to target register: 0x" << std::hex << (int)reg << std::endl;
        return false;
    }
    return true;
}

bool GY271Magnetometer::readRegisters(uint8_t startReg, uint8_t* outputBuffer, size_t length) {
    // Write target pointer start address instruction byte pass
    if (write(_i2cFd, &startReg, 1) != 1) return false;
    
    // Read the sequential return data payload parameters sequence
    if (read(_i2cFd, outputBuffer, length) != static_cast<ssize_t>(length)) return false;
    
    return true;
}

bool GY271Magnetometer::readMagneticField(float& xUt, float& yUt, float& zUt) {
    if (_i2cFd < 0) return false;

    uint8_t dataBuffer[6];
    // QMC5883L data registers range sequentially across 0x00 to 0x05 (X-LSB, X-MSB, Y-LSB...)
    if (!readRegisters(0x00, dataBuffer, 6)) {
        return false;
    }

    // Assemble 16-bit signed integer values out of split low and high byte pairs
    int16_t rawX = static_cast<int16_t>((dataBuffer[1] << 8) | dataBuffer[0]);
    int16_t rawY = static_cast<int16_t>((dataBuffer[3] << 8) | dataBuffer[2]);
    int16_t rawZ = static_cast<int16_t>((dataBuffer[5] << 4) | dataBuffer[4]);

    // QMC5883L scaling resolution conversion multiplier under +/- 8 Gauss setup mode limit
    // 1 Gauss = 100 uT. Sensor scale factor is 3000 LSB per Gauss -> Multiplier = 100.0f / 3000.0f
    const float lsbToMicrotesla = 100.0f / 3000.0f;

    xUt = static_cast<float>(rawX) * lsbToMicrotesla;
    yUt = static_cast<float>(rawY) * lsbToMicrotesla;
    zUt = static_cast<float>(rawZ) * lsbToMicrotesla;

    return true;
}
