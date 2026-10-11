#pragma once
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int INPUT = 0;
constexpr int OUTPUT = 1;
constexpr int INPUT_PULLUP = 2;
constexpr int SERIAL_8N1 = 0;
inline unsigned long mockTime = 0;
inline int mockPins[40] = {};
inline unsigned long millis() { return mockTime; }
inline void delay(unsigned long value) { mockTime += value; }
inline void pinMode(int, int) {}
inline int digitalRead(int pin) { return mockPins[pin]; }
inline void digitalWrite(int pin, int value) { mockPins[pin] = value; }

class HardwareSerial {
public:
  explicit HardwareSerial(int = 0) {}
  std::deque<char> rx;
  std::string tx;
  void begin(unsigned long, int = 0, int = 0, int = 0) {}
  int available() { return static_cast<int>(rx.size()); }
  int read() {
    if (rx.empty()) return -1;
    char value = rx.front();
    rx.pop_front();
    return value;
  }
  void println(const char *value) { tx += std::string(value) + "\r\n"; }
  void printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    tx += buffer;
  }
  void inject(const std::string &value) {
    for (char c : value) rx.push_back(c);
  }
};
inline HardwareSerial Serial;
