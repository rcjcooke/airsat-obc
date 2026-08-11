#ifndef AOCS_PACKET_STRUCTURES_H
#define AOCS_PACKET_STRUCTURES_H

#include <cstddef>
#include <cstdint>

namespace AOCSPacketConstants {
constexpr uint8_t kSyncByte0 = 0xAA;
constexpr uint8_t kSyncByte1 = 0x55;
constexpr std::size_t kSyncSize = 2;
constexpr std::size_t kChecksumSize = 2;
constexpr std::size_t kCommandPayloadSize = 22;
constexpr std::size_t kTelemetryPayloadSize = 22;
constexpr std::size_t kFrameSize = kSyncSize + kCommandPayloadSize + kChecksumSize;
}

#pragma pack(push, 1)
struct CommandPayload {
    float torque; // N.m
    float thrust[4]; // N
    uint8_t flags;
    uint8_t alignment_pad;
}; // 22 bytes

struct TelemetryPayload {
    float storedAngularMomentum; // kg.m^2/s
    uint16_t propellant; // kg
    uint16_t error_count;
    uint8_t padding[14];
}; // 22 bytes

struct CommandFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    CommandPayload payload;
    uint16_t checksum;
}; // 26 bytes

struct TelemetryFrame {
    uint8_t sync[2]; // 0xAA, 0x55
    TelemetryPayload payload;
    uint16_t checksum;
}; // 26 bytes
#pragma pack(pop)

#endif // AOCS_PACKET_STRUCTURES_H
