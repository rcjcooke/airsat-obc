#include "FusedAttitudeSensor.h"
#include <cmath>
#include <iostream>

FusedAttitudeSensor::FusedAttitudeSensor() 
    : _magSensor("/dev/i2c-1", 0x0D), _currentYawHeading(0.0f), _healthy(false) {}

bool FusedAttitudeSensor::init() {
    _healthy = _magSensor.init();
    if (!_healthy) {
        std::cerr << "[AOCS CORE] Fused Attitude Engine failed to link GY-271 peripheral." << std::endl;
    }
    return _healthy;
}

void FusedAttitudeSensor::update() {
    if (!_healthy) return;

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    if (_magSensor.readMagneticField(mx, my, mz)) {
        // Calculate the horizontal heading angle reference tracking vector using arctangent
        // Yields an orientation bounds ranging across -PI to +PI radians
        _currentYawHeading = std::atan2(my, mx);
    } else {
        _healthy = false;
        std::cerr << "[AOCS WARNING] Magnetometer drop detected during active flight step." << std::endl;
    }
}

float FusedAttitudeSensor::getEstimatedYawHeading() const { return _currentYawHeading; }
bool FusedAttitudeSensor::isSensorHealthy() const { return _healthy; }
