#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cyd {

// Hai project (CYD va ESP32 38-pin) phai dung dung cac gia tri nay.
enum PacketType : uint8_t {
  PKT_NONE = 0,
  PKT_CMD_TOGGLE = 1,
  PKT_SCAN_REQ = 2,
  PKT_SCAN_RES = 3,
  PKT_STATUS = 4,
};

enum SystemPhase : uint8_t {
  PHASE_HOMING = 0,
  PHASE_MOVING = 1,
  PHASE_SCAN_SETTLING = 2,
  PHASE_WAIT_PI = 3,
  PHASE_WAIT_IR2 = 4,
  PHASE_GATE_SETTLING = 5,
  PHASE_GATING = 6,
  PHASE_RELEASING = 7,
  PHASE_RETURNING = 8,
  PHASE_FAULT = 9,
};

// Giao thuc ESP-NOW co dinh 16 byte, tuong thich voi code CYD cua ban.
// Khong doi thu tu/kieu cua ba truong dau ma khong cap nhat ca hai bo.
#pragma pack(push, 1)
struct SystemPacket {
  uint8_t type;
  bool isRunning;
  uint8_t phase;
  uint32_t scanId;
  uint8_t resultType;
  float brix;
  int32_t moisture;
};
#pragma pack(pop)

static_assert(sizeof(bool) == 1, "This protocol requires a 1-byte bool");
static_assert(sizeof(SystemPacket) == 16, "SystemPacket must be exactly 16 bytes");
static_assert(offsetof(SystemPacket, type) == 0, "Unexpected packet layout");
static_assert(offsetof(SystemPacket, phase) == 2, "Unexpected packet layout");
static_assert(offsetof(SystemPacket, isRunning) == 1,
              "Unexpected packet layout");
static_assert(offsetof(SystemPacket, scanId) == 3, "Unexpected scanId layout");
static_assert(offsetof(SystemPacket, resultType) == 7, "Unexpected result layout");
static_assert(offsetof(SystemPacket, brix) == 8, "Unexpected brix layout");
static_assert(offsetof(SystemPacket, moisture) == 12, "Unexpected moisture layout");

inline SystemPacket makeStatus(uint8_t phase, bool isRunning) {
  SystemPacket packet{};
  packet.type = PKT_STATUS;
  packet.phase = phase;
  packet.isRunning = isRunning;
  return packet;
}

}  // namespace cyd
