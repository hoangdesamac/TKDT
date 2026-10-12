// Standalone host check: g++ -std=c++11 -Iinclude test/cyd_protocol_compat.cpp
#include "cyd_protocol.h"
#include <cassert>
#include <cstring>

int main() {
  // Little-endian 16-byte fixture using the supplied CYD wire layout.
  const uint8_t bytes[16] = {
      4, 1, 3, 0x78, 0x56, 0x34, 0x12, 2,
      0, 0, 0x48, 0x41, 64, 0, 0, 0};
  cyd::SystemPacket decoded{};
  std::memcpy(&decoded, bytes, sizeof(decoded));
  assert(decoded.type == cyd::PKT_STATUS);
  assert(decoded.isRunning && decoded.phase == cyd::PHASE_WAIT_PI);
  assert(decoded.scanId == 0x12345678 && decoded.resultType == 2);
  assert(decoded.brix == 12.5f && decoded.moisture == 64);

  const cyd::SystemPacket status = cyd::makeStatus(cyd::PHASE_WAIT_PI, true);
  const uint8_t expectedStatus[16] = {4, 1, 3};
  assert(std::memcmp(&status, expectedStatus, sizeof(status)) == 0);

  const uint8_t startBytes[16] = {1, 1, 0};
  std::memcpy(&decoded, startBytes, sizeof(decoded));
  assert(decoded.type == cyd::PKT_CMD_TOGGLE && decoded.isRunning);
  const uint8_t stopBytes[16] = {1, 0, 0};
  std::memcpy(&decoded, stopBytes, sizeof(decoded));
  assert(decoded.type == cyd::PKT_CMD_TOGGLE && !decoded.isRunning);
}
