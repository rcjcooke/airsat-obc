#ifndef PACKET_STRUCTURES_H
#define PACKET_STRUCTURES_H

#include <cstdint>

#pragma pack(push, 1)
// Inner Core Payloads
struct CommandPayload {
    float torque;
    float thrust[4];
};

struct TelemetryPayload {
    float momentum;
    uint16_t propellant;
    uint16_t error_count;
    uint8_t padding[14]; // Keeps payload frame size identical to CommandPayload (22 bytes)
};

// Full 26-byte Over-The-Wire Communication Frames
struct CommandFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    CommandPayload payload;
    uint16_t checksum;
};

struct TelemetryFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    TelemetryPayload payload;
    uint16_t checksum;
};
#pragma pack(pop)

#endif
