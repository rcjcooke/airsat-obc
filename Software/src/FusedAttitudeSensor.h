#ifndef FUSED_ATTITUDE_SENSOR_H
#define FUSED_ATTITUDE_SENSOR_H

#include "GY271Magnetometer.h"
#include <chrono>

namespace Filters {
    // Placeholder for future sensor fusion algorithms (e.g., Kalman Filter, Complementary Filter)
    // For now, the FusedAttitudeSensor class will primarily serve as a wrapper around the GY271Magnetometer
    
    /**
     * Quantizes the given value to the specified accuracy.
     * For example, if accuracy is 0.1, the value will be rounded to the nearest 0.1.
     */
    float quantizeToAccuracy(float value, float accuracy);

    /**
     * Filters out small changes in sensor readings that are below the specified accuracy threshold.
     */
    float filterOutChangesBelowAccuracy(float newValue, float existingValue, float accuracy);
}

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

    float getEstimatedAttitude() const;
    bool isSensorHealthy() const;
    uint32_t getSensorFaultCount() const;

private:
    static constexpr bool kDebug = false;
    static constexpr uint32_t kFaultReestablishTimeoutMs = 5000;

    GY271Magnetometer _magSensor;
    // Future expansion sensors (e.g., IMU MPU6050, StarTracker, etc.) can be placed cleanly here
    
    float _currentAttitude;
    bool _healthy;
    bool _faultActive;
    std::chrono::steady_clock::time_point _faultDetectedAt;
    uint32_t _sensorFaultsCount;

    bool readCurrentAttitude();
    bool attemptReconnect();
    void clearFaultState();
};

#endif
