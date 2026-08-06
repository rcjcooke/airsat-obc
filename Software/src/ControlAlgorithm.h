#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

class ControlAlgorithm {
public:
    struct AirSatConstraints {
        static constexpr float SATELLITE_INERTIA   = 0.0500f; // I_sat (kg.m^2)
        static constexpr float MAX_MOTOR_TORQUE    = 0.1500f; // Max Torque limit (N.m)
        static constexpr float MAX_WHEEL_MOMENTUM  = 0.0200f; // Wheel saturation limit (kg.m^2/s)
    };

    struct ControlCommands {
        float torque; // Torque command in N*m
        float thrust[4]; // Thrust commands for 4 thrusters (N)
    };

    static ControlCommands computeControlCommands(float attitudeRad, float angularVelocityRadS, float storedMomentumKgMMS, float remainingPropellantMMM);
};

#endif // CONTROL_ALGORITHM_H