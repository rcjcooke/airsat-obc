#include "FusedAttitudeSensor.h"
#include <cmath>
#include <iostream>

FusedAttitudeSensor::FusedAttitudeSensor() 
    : _magSensor("/dev/i2c-1", 0x0D), _currentYawHeading(0.0f), _healthy(false) {}

bool FusedAttitudeSensor::init() {
    _healthy = _magSensor.init();
    return _healthy;
}

void FusedAttitudeSensor::update() {
    if (!_healthy) return;

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    if (_magSensor.readMagneticField(mx, my, mz)) {
        _currentYawHeading = std::atan2(my, mx);
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

float FusedAttitudeSensor::getEstimatedYawHeading() const { return _currentYawHeading; }
bool FusedAttitudeSensor::isSensorHealthy() const { return _healthy; }
