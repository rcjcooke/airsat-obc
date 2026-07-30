#ifndef FUSED_ATTITUDE_SENSOR_H
#define FUSED_ATTITUDE_SENSOR_H

#include "GY271Magnetometer.h"

class FusedAttitudeSensor {
public:
    FusedAttitudeSensor();
    bool init();
    void update();

    // Calibration Hooks
    void requestCalibrationStart();
    void requestCalibrationEnd();
    bool isSystemCalibrated() const;

    float getEstimatedYawHeading() const;
    bool isSensorHealthy() const;

private:
    GY271Magnetometer _magSensor;
    float _currentYawHeading;
    bool _healthy;
    bool _calibratingMode;
};

#endif
