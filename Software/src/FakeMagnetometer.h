#ifndef FAKE_MAGNETOMETER_H
#define FAKE_MAGNETOMETER_H

class FakeMagnetometer {
public:
  static constexpr float kAmplitude = 0.5f; // Maximum angular deviation in radians
  static constexpr float kFrequency = 0.01f; // Frequency of oscillation in Hz

  float readAttitude();
};

#endif // FAKE_MAGNETOMETER_H