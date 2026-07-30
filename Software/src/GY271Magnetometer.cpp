#include "GY271Magnetometer.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>
#include <limits>

GY271Magnetometer::GY271Magnetometer(const std::string& i2cDevice, uint8_t i2cAddress)
    : _devicePath(i2cDevice), _address(i2cAddress), _i2cFd(-1),
      _isCalibrating(false), _isCalibrated(false),
      _offsetX(0.0f), _offsetY(0.0f), _offsetZ(0.0f) {}

GY271Magnetometer::~GY271Magnetometer() {
    closeDevice();
}

bool GY271Magnetometer::init() {
    _i2cFd = open(_devicePath.c_str(), O_RDWR);
    if (_i2cFd < 0) return false;

    if (ioctl(_i2cFd, I2C_SLAVE, _address) < 0) return false;

    if (!writeRegister(0x0B, 0x01)) return false; // FBR Register Setup
    if (!writeRegister(0x09, 0x19)) return false; // 100Hz ODR Continuous Mode

    return true;
}

void GY271Magnetometer::closeDevice() {
    if (_i2cFd >= 0) {
        close(_i2cFd);
        _i2cFd = -1;
    }
}

bool GY271Magnetometer::writeRegister(uint8_t reg, uint8_t value) {
    // FIX: Change 'buffer' to an array type of size 2
    uint8_t buffer[2] = { reg, value };
    
    if (write(_i2cFd, buffer, 2) != 2) {
        std::cerr << "[I2C] Failed writing byte value to target register: 0x" << std::hex << (int)reg << std::endl;
        return false;
    }
    return true;
}


bool GY271Magnetometer::readRegisters(uint8_t startReg, uint8_t* outputBuffer, size_t length) {
    if (write(_i2cFd, &startReg, 1) != 1) return false;
    return (read(_i2cFd, outputBuffer, length) == static_cast<ssize_t>(length));
}

bool GY271Magnetometer::readRawData(float& rx, float& ry, float& rz) {
    if (_i2cFd < 0) return false;
    uint8_t dataBuffer[6];
    if (!readRegisters(0x00, dataBuffer, 6)) return false;

    int16_t rawX = static_cast<int16_t>((dataBuffer[1] << 8) | dataBuffer[0]);
    int16_t rawY = static_cast<int16_t>((dataBuffer[3] << 8) | dataBuffer[2]);
    int16_t rawZ = static_cast<int16_t>((dataBuffer[5] << 8) | dataBuffer[4]);

    const float lsbToMicrotesla = 100.0f / 3000.0f;
    rx = static_cast<float>(rawX) * lsbToMicrotesla;
    ry = static_cast<float>(rawY) * lsbToMicrotesla;
    rz = static_cast<float>(rawZ) * lsbToMicrotesla;
    return true;
}

bool GY271Magnetometer::readMagneticField(float& xUt, float& yUt, float& zUt) {
    float rx, ry, rz;
    if (!readRawData(rx, ry, rz)) return false;

    if (_isCalibrated) {
        // Subtract calculated hard-iron offsets from live stream parameters
        xUt = rx - _offsetX;
        yUt = ry - _offsetY;
        zUt = rz - _offsetZ;
    } else {
        xUt = rx;
        yUt = ry;
        zUt = rz;
    }
    return true;
}

void GY271Magnetometer::startCalibration() {
    _isCalibrating = true;
    _isCalibrated = false;
    
    // Seed boundary variables to their maximum inverted extremes
    _minX = _minY = _minZ = std::numeric_limits<float>::max();
    _maxX = _maxY = _maxZ = -std::numeric_limits<float>::max();
    
    std::cout << "[CALIBRATION] Started. Rotate vehicle 360 degrees smoothly..." << std::endl;
}

void GY271Magnetometer::updateCalibrationValues() {
    if (!_isCalibrating) return;
    float rx, ry, rz;
    if (readRawData(rx, ry, rz)) {
        if (rx < _minX) _minX = rx;
        if (rx > _maxX) _maxX = rx;
        if (ry < _minY) _minY = ry;
        if (ry > _maxY) _maxY = ry;
        if (rz < _minZ) _minZ = rz;
        if (rz > _maxZ) _maxZ = rz;
    }
}

void GY271Magnetometer::endCalibration() {
    if (!_isCalibrating) return;
    _isCalibrating = false;

    // Calculate midpoints of peak envelopes to determine the center offset
    _offsetX = (_maxX + _minX) / 2.0f;
    _offsetY = (_maxY + _minY) / 2.0f;
    _offsetZ = (_maxZ + _minZ) / 2.0f;

    _isCalibrated = true;
    std::cout << "[CALIBRATION] Success. Offsets -> X: " << _offsetX 
              << " | Y: " << _offsetY << " | Z: " << _offsetZ << " uT" << std::endl;
}
