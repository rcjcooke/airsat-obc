#include "FusedAttitudeSensor.h"
#include <cmath>
#include <iostream>

// Define the filters

float Filters::quantizeToAccuracy(float value, float accuracy) {
    return std::round(value / accuracy) * accuracy;
}

float Filters::filterOutChangesBelowAccuracy(float newValue, float existingValue, float accuracy) {
    if (std::abs(newValue - existingValue) < accuracy / 2) {
        return existingValue; // Ignore small changes
    }
    return newValue; // Accept significant changes
}

// FusedAttitudeSensor Implementation

FusedAttitudeSensor::FusedAttitudeSensor() 
    : _magSensor("/dev/i2c-1", 0x0D), _currentAttitude(0.0f), _healthy(false),
      _faultActive(false), _faultDetectedAt(std::chrono::steady_clock::now()), _sensorFaults(0) {}

bool FusedAttitudeSensor::init() {
    _healthy = _magSensor.init();
    if (_healthy) {
        clearFaultState();
    }
    return _healthy;
}

void FusedAttitudeSensor::update() {
    auto now = std::chrono::steady_clock::now();

    if (readCurrentAttitude()) {
        _healthy = true;
        clearFaultState();
        return;
    }

    if (!_faultActive) {
        _faultActive = true;
        _faultDetectedAt = now;
        _sensorFaults++;
        if (kDebug) std::cerr << "[FusedAttitudeSensor] Magnetometer communication fault detected. Attempting immediate reconnect..." << std::endl;
    }

    if (attemptReconnect() && readCurrentAttitude()) {
        _healthy = true;
        clearFaultState();
        if (kDebug) std::cerr << "[FusedAttitudeSensor] Magnetometer communication restored." << std::endl;
        return;
    }

    const auto faultAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - _faultDetectedAt).count();
    if (faultAgeMs >= kFaultReestablishTimeoutMs) {
        if (_healthy) {
            if (kDebug) std::cerr << "[FusedAttitudeSensor] Magnetometer fault timeout exceeded. Marking sensor unhealthy." << std::endl;
        }
        _healthy = false;
    } else {
        _healthy = true;
    }
}

bool FusedAttitudeSensor::readCurrentAttitude() {
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;
    if (!_magSensor.readMagneticField(mx, my, mz)) {
        return false;
    }

    const float rawAttitude = std::atan2(my, mx);
    // Filter out sensor accuracy noise
    _currentAttitude = Filters::filterOutChangesBelowAccuracy(rawAttitude, _currentAttitude, GY271Magnetometer::kAccuracy);
    (void)mz;
    return true;
}

bool FusedAttitudeSensor::attemptReconnect() {
    _magSensor.closeDevice();
    return _magSensor.init();
}

void FusedAttitudeSensor::clearFaultState() {
    _faultActive = false;
    _faultDetectedAt = std::chrono::steady_clock::now();
}

uint32_t FusedAttitudeSensor::getSensorFaultCount() const {
    return _sensorFaults;
}

void FusedAttitudeSensor::requestCalibrationStart() {
    _magSensor.startCalibration();
    // Add additional startup triggers for future sensors here
}

void FusedAttitudeSensor::updateCalibration() {
    if (!_healthy) return;

    // 1. Service active Magnetometer profile capture allocations
    _magSensor.updateCalibrationValues();
    
    // 2. [Future Expansion]: Add IMU/Gyro offset integration accumulation steps here:
    // _gyroSensor.accumulateZeroBiasOffsets();
}

void FusedAttitudeSensor::requestCalibrationEnd() {
    _magSensor.endCalibration();
    // Add additional shutdown or flash storage commit calls for future sensors here
}

bool FusedAttitudeSensor::isSystemCalibrated() const {
    return _magSensor.isCalibrated(); // Modify to check all operational module bounds if extended
}

float FusedAttitudeSensor::getEstimatedAttitude() const { return _currentAttitude; }
bool FusedAttitudeSensor::isSensorHealthy() const { return _healthy; }
