#pragma once
#include "Arduino.h"
#include <stdexcept>

struct ServoCommand { uint8_t channel; uint16_t pulse; };
inline std::vector<ServoCommand> servoCommands;
inline bool mockPcaPresent = true;

class Adafruit_PWMServoDriver {
public:
  explicit Adafruit_PWMServoDriver(int) {}
  bool begin() { return mockPcaPresent; }
  void setOscillatorFrequency(unsigned long) {}
  void setPWMFreq(int) {}
  void writeMicroseconds(uint8_t channel, uint16_t pulse) {
    if (pulse == 1700 && mockPins[26] != HIGH)
      throw std::runtime_error("Gate moved before conveyor was stopped");
    servoCommands.push_back({channel, pulse});
  }
};
