#include "FusedAttitudeSensor.h"
#include <cmath>
#include <iostream>

FusedAttitudeSensor::FusedAttitudeSensor() 
    : _magSensor("/dev/i2c-1", 0x0D), _currentYawHeading(0.0f), _healthy(false), _calibratingMode(false) {}

bool FusedAttitudeSensor::init() {
    _healthy = _magSensor.init();
    return _healthy;
}

void FusedAttitudeSensor::update() {
    if (!_healthy) return;

    if (_calibratingMode) {
        // Sample raw values repeatedly to lock peak envelope extremes
        _magSensor.updateCalibrationValues();
    } else {
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        if (_magSensor.readMagneticField(mx, my, mz)) {
            _currentYawHeading = std::atan2(my, mx);
        } else {
            _healthy = false;
        }
    }
}

void FusedAttitudeSensor::requestCalibrationStart() {
    _calibratingMode = true;
    _magSensor.startCalibration();
}

void FusedAttitudeSensor::requestCalibrationEnd() {
    _calibratingMode = false;
    _magSensor.endCalibration();
}

bool FusedAttitudeSensor::isSystemCalibrated() const {
    return _magSensor.isCalibrated();
}

float FusedAttitudeSensor::getEstimatedYawHeading() const { return _currentYawHeading; }
bool FusedAttitudeSensor::isSensorHealthy() const { return _healthy; }
