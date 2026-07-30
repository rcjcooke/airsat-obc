#ifndef FUSED_ATTITUDE_SENSOR_H
#define FUSED_ATTITUDE_SENSOR_H

#include "GY271Magnetometer.h"

class FusedAttitudeSensor {
public:
    FusedAttitudeSensor();
    bool init();
    void update();

    // Centralized Calibration Methods
    void requestCalibrationStart();
    void updateCalibration(); // Central collection node for all embedded sensor modules
    void requestCalibrationEnd();
    bool isSystemCalibrated() const;

    float getEstimatedYawHeading() const;
    bool isSensorHealthy() const;

private:
    GY271Magnetometer _magSensor;
    // Future expansion sensors (e.g., IMU MPU6050, StarTracker, etc.) can be placed cleanly here
    
    float _currentYawHeading;
    bool _healthy;
};

#endif
