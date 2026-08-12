#include "FakeMagnetometer.h"
#include <cmath>
#include <chrono>

float FakeMagnetometer::readAttitude() {
  static auto startTime = std::chrono::steady_clock::now();
  // Simulate a simple oscillating attitude for testing purposes
  static float timeDiff = 0.0f;
  // The time will incrememt in real-time
  timeDiff = std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count();
  // Return a sine wave based on the elapsed time, amplitude, and frequency
  return kAmplitude * std::sin(2.0f * static_cast<float>(M_PI) * kFrequency * timeDiff);
}