#ifndef GY271_MAGNETOMETER_H
#define GY271_MAGNETOMETER_H

#include <string>
#include <cstdint>

class GY271Magnetometer {
public:
    GY271Magnetometer(const std::string& i2cDevice = "/dev/i2c-1", uint8_t i2cAddress = 0x0D);
    ~GY271Magnetometer();

    bool init();
    void closeDevice();
    
    // Reads raw magnetic field strength vector (Microteslas - uT)
    bool readMagneticField(float& xUt, float& yUt, float& zUt);

private:
    // Low level I2C transaction helper abstraction layouts
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t startReg, uint8_t* outputBuffer, size_t length);

    std::string _devicePath;
    uint8_t _address;
    int _i2cFd;
};

#endif
