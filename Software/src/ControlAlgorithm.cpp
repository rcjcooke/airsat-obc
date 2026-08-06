#include "ControlAlgorithm.h"

#include <cmath>

ControlAlgorithm::ControlCommands ControlAlgorithm::computeControlCommands(float attitudeRad, float angularVelocityRadS, float storedMomentumKgMMS, float remainingPropellantMMM) {
  (void)remainingPropellantMMM; // Parameter unused for now
  ControlCommands commands;
    
  // Control gains
  float kp = 0.8f; // Proportional gain
  float kd = 2.0f * std::sqrt(kp * AirSatConstraints::SATELLITE_INERTIA); // Derivative gain based on critical damping for a second-order system
  
  // Calculate heading error
  float headingError = std::atan2(std::sin(attitudeRad), std::cos(attitudeRad)); // Assuming target attitude is 0 wrt the magnetic field reference frame
  
  // Compute requested torque using PD control
  float requestedTorque = (kp * headingError) - (kd * angularVelocityRadS);
  
  // Saturate torque to maximum limits
  if (requestedTorque > AirSatConstraints::MAX_MOTOR_TORQUE)  requestedTorque = AirSatConstraints::MAX_MOTOR_TORQUE;
  if (requestedTorque < -AirSatConstraints::MAX_MOTOR_TORQUE) requestedTorque = -AirSatConstraints::MAX_MOTOR_TORQUE;
  
  // Check wheel momentum saturation
  if (storedMomentumKgMMS >= AirSatConstraints::MAX_WHEEL_MOMENTUM && requestedTorque < 0.0f) requestedTorque = 0.0f;
  if (storedMomentumKgMMS <= -AirSatConstraints::MAX_WHEEL_MOMENTUM && requestedTorque > 0.0f) requestedTorque = 0.0f;

  commands.torque = requestedTorque;
  
  // For this example, we assume thrust commands are zeroed out.
  for (int i = 0; i < 4; ++i) {
      commands.thrust[i] = 0.0f;
  }
  
  return commands;
}