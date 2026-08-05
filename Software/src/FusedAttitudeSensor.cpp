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
    : _magSensor("/dev/i2c-1", 0x0D), _currentAttitude(0.0f), _healthy(false) {}

bool FusedAttitudeSensor::init() {
    _healthy = _magSensor.init();
    return _healthy;
}

void FusedAttitudeSensor::update() {
    if (!_healthy) return;

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    if (_magSensor.readMagneticField(mx, my, mz)) {
        const float rawAttitude = std::atan2(my, mx);
        // Filter out sensor accuracy noise
        _currentAttitude = Filters::filterOutChangesBelowAccuracy(rawAttitude, _currentAttitude, GY271Magnetometer::kAccuracy);
    } else {
        _healthy = false;
    }
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
