#include <cstdlib>
#include <iostream>
#include "../src/main.cpp"

bool mockConveyorStopped() {
  return mockPins[RELAY_PIN] == RELAY_OFF;
}

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

int main() {
  check(IR_BELT_TEST_MODE, "default firmware selects two-IR belt test");
  mockPcaPresent = false;
  mockPins[IR1_PIN] = HIGH;
  mockPins[IR2_PIN] = HIGH;
  mockPins[34] = LOW;
  setup();
  check(mockPins[RELAY_PIN] == RELAY_ON,
        "clear sensors start belt without Pi, PCA or Start button");
  check(Serial.tx.find("FIRMWARE: IR_STOP_TEST") != std::string::npos,
        "boot identifies two-IR firmware");

  mockPins[IR1_PIN] = LOW;
  loop();
  check(mockConveyorStopped(), "IR1 obstacle stops belt on next loop");
  check(Serial.tx.find("G32=0 G33=1 BLOCKED=1 REQUEST=STOP") != std::string::npos,
        "log explains active-low detection and stop request");
  PiSerial.inject("RESULT,100,3\n");
  mockTime += PI_SCAN_TIMEOUT_MS + 1000;
  loop();
  check(mockConveyorStopped(), "held obstacle stays stopped without Pi timeout");

  mockPins[IR1_PIN] = HIGH;
  loop();
  check(mockPins[RELAY_PIN] == RELAY_ON, "clearing IR1 automatically resumes");
  mockPins[IR2_PIN] = LOW;
  loop();
  check(mockConveyorStopped(), "IR2 obstacle independently stops belt");
  check(Serial.tx.find("G32=1 G33=0 BLOCKED=1 REQUEST=STOP") != std::string::npos,
        "log identifies IR2 detection");
  mockPins[IR1_PIN] = LOW;
  loop();
  check(mockConveyorStopped(), "both sensors blocked keeps belt stopped");
  mockPins[IR2_PIN] = HIGH;
  loop();
  check(mockConveyorStopped(), "clearing only IR2 cannot override blocked IR1");
  mockPins[IR2_PIN] = LOW;
  mockPins[IR1_PIN] = HIGH;
  loop();
  check(mockConveyorStopped(), "clearing only IR1 cannot override blocked IR2");
  mockTime += PI_SCAN_TIMEOUT_MS + 1000;
  loop();
  check(mockConveyorStopped(), "held IR2 stays stopped without Pi timeout");
  mockPins[IR2_PIN] = HIGH;
  loop();
  check(mockPins[RELAY_PIN] == RELAY_ON, "both clear automatically resumes belt");

  Serial.tx.clear();
  mockPins[IR1_PIN] = LOW;
  mockPcaPresent = true;
  setup();
  check(mockConveyorStopped() && Serial.tx.find("REQUEST=RUN") == std::string::npos,
        "boot with blocked IR1 never requests belt movement");
  Serial.tx.clear();
  mockPins[IR1_PIN] = HIGH;
  mockPins[IR2_PIN] = LOW;
  setup();
  check(mockConveyorStopped() && Serial.tx.find("REQUEST=RUN") == std::string::npos,
        "boot with blocked IR2 never requests belt movement");
  check(servoCommands.empty() && !servoDriverReady,
        "test never initializes or commands servos");
  check(PiSerial.tx.empty(), "test sends no Pi UART requests");
  std::cout << "PASS: two-IR startup, independent stops, overlapping obstacles, hold, auto-resume, no Pi or servo dependency\n";
}
