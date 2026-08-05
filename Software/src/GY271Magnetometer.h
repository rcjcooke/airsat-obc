#ifndef GY271_MAGNETOMETER_H
#define GY271_MAGNETOMETER_H

#include <string>
#include <cstdint>

#define ONE_DEGREE_IN_RADIANS 0.01745329252f

class GY271Magnetometer {
public:

    // Magnetometer angular accuracy (datasheet says approx 1-2 degrees)
    static constexpr float kAccuracy = ONE_DEGREE_IN_RADIANS * 1.5f;

    GY271Magnetometer(const std::string& i2cDevice = "/dev/i2c-1", uint8_t i2cAddress = 0x0D);
    ~GY271Magnetometer();

    bool init();
    void closeDevice();
    
    // Reads calibrated microtesla data if calibrated, otherwise raw data
    bool readMagneticField(float& xUt, float& yUt, float& zUt);

    // Calibration Controls
    void startCalibration();
    void updateCalibrationValues(); // Called routinely during the 360-degree rotation
    void endCalibration();
    
    bool isCalibrated() const { return _isCalibrated; }

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t startReg, uint8_t* outputBuffer, size_t length);
    bool readRawData(float& rx, float& ry, float& rz);

    std::string _devicePath;
    uint8_t _address;
    int _i2cFd;

    // Hard-Iron Offset Variables
    bool _isCalibrating;
    bool _isCalibrated;
    float _offsetX, _offsetY, _offsetZ;
    float _minX, _maxX, _minY, _maxY, _minZ, _maxZ;
};

#endif
