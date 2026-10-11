#pragma once
#include <cstdint>
inline uint32_t mockRandomValue = 100;
inline uint32_t esp_random() { return mockRandomValue++; }
inline int esp_reset_reason() { return 1; }
