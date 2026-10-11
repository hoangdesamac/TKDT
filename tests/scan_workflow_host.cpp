#include <cstdlib>
#include <iostream>
#define IR_BELT_TEST_ENABLED 0
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

void tick(unsigned long elapsed = 10, int ir1 = HIGH, int ir2 = HIGH,
          int button = HIGH) {
  mockTime += elapsed;
  mockPins[IR1_PIN] = ir1;
  mockPins[IR2_PIN] = ir2;
  mockPins[34] = button;
  loop();
}

void resetMachine(int ir1 = HIGH, int ir2 = HIGH) {
  mockTime += 20000;
  for (int &pin : mockPins) pin = HIGH;
  mockPcaPresent = true;
  servoDriverReady = false;
  isSystemRunning = false;
  currentMango = Mango();
  pendingScanId = 0;
  piLineLength = 0;
  piLineOverflow = false;
  Serial.rx.clear();
  Serial.tx.clear();
  PiSerial.rx.clear();
  PiSerial.tx.clear();
  setup();
  tick(600, ir1, ir2);
  tick(60, ir1, ir2);
  servoCommands.clear();
}

uint32_t scanAtIr1() {
  check(mockPins[RELAY_PIN] == RELAY_ON, "automatic startup runs clear belt");
  tick(10, LOW);
  check(mockPins[RELAY_PIN] == RELAY_OFF, "IR1 stops belt immediately");
  tick(250, LOW);
  check(Serial.tx.find("IR_RAW: G32=0") != std::string::npos,
        "raw log identifies IR1 detection");
  check(sortingPhase == SortingPhase::WAIT_PI && pendingScanId != 0,
        "stopped mango requests a scan");
  check(PiSerial.tx.find("SCAN," + std::to_string(pendingScanId) + "\n") !=
        std::string::npos, "request contains its ID");
  return pendingScanId;
}

void respond(uint32_t id, int type, int ir1 = LOW, int ir2 = HIGH) {
  PiSerial.inject("RESULT," + std::to_string(id) + "," +
                  std::to_string(type) + "\n");
  tick(10, ir1, ir2);
}

int main() {
  resetMachine();
  check(isSystemRunning && mockPins[RELAY_PIN] == RELAY_ON,
        "boot automatically starts after homing");
  check(Serial.tx.find("FIRMWARE: AUTO_UART_DIAG1") != std::string::npos,
        "boot identifies diagnostic automatic firmware");
  tick(100, HIGH, HIGH, LOW);
  tick(100, HIGH, HIGH, HIGH);
  check(isSystemRunning, "floating G34 cannot toggle automatic mode");
  Serial.inject("STOP\r\nSTART\r\n");
  tick();
  check(isSystemRunning && mockPins[RELAY_PIN] == RELAY_ON,
        "removed USB manual commands do not control the machine");
  PiSerial.inject("P");
  tick();
  check(PiSerial.tx.find("PONG") != std::string::npos, "ping in automatic mode");

  uint32_t id = scanAtIr1();
  tick(300, HIGH);
  check(sortingPhase == SortingPhase::WAIT_PI && mockPins[RELAY_PIN] == RELAY_OFF,
        "clearing IR1 cannot bypass Pi");
  respond(id + 1, 3);
  PiSerial.inject("3\nRESULT,100,4\nRESULT,+" + std::to_string(id) + ",3\n");
  tick(10, LOW);
  tick(10, LOW);
  check(sortingPhase == SortingPhase::WAIT_PI && servoCommands.empty(),
        "stale, bare and malformed results cannot gate");
  PiSerial.inject("RESULT," + std::to_string(id) + ",");
  tick(10, LOW);
  check(sortingPhase == SortingPhase::WAIT_PI, "partial result waits for newline");
  PiSerial.inject("3\r\n");
  tick(10, LOW);
  check(sortingPhase == SortingPhase::GATING && servoCommands.size() == 1 &&
        servoCommands.back().channel == 0 && mockPins[RELAY_PIN] == RELAY_OFF,
        "class 3 gates CH0 while stopped");
  respond(id, 3);
  check(servoCommands.size() == 1, "duplicate result does not gate twice");
  tick(900, LOW);
  check(sortingPhase == SortingPhase::RELEASING &&
        mockPins[RELAY_PIN] == RELAY_ON && servoCommands.size() == 1,
        "class 3 belt runs with gate held");
  tick(1200, LOW);
  check(servoCommands.size() == 1, "blocked mango does not re-scan or re-gate");
  tick(10);
  tick(200);
  check(sortingPhase == SortingPhase::RETURNING &&
        servoCommands.back().pulse == SERVO_REST_US && mockPins[RELAY_PIN] == RELAY_OFF,
        "clear exit returns gate with belt stopped");
  tick(600);
  tick();
  check(sortingPhase == SortingPhase::MOVING && mockPins[RELAY_PIN] == RELAY_ON,
        "resume after return wait");

  resetMachine();
  id = scanAtIr1();
  respond(id, 2);
  check(sortingPhase == SortingPhase::WAIT_IR2 && servoCommands.empty() &&
        mockPins[RELAY_PIN] == RELAY_ON, "class 2 travels without CH0 movement");
  tick(10, HIGH, LOW);
  check(mockPins[RELAY_PIN] == RELAY_OFF && servoCommands.empty(),
        "IR2 stops and settles before gate");
  tick(250, HIGH, LOW);
  check(sortingPhase == SortingPhase::GATING && servoCommands.back().channel == 1,
        "class 2 gates CH1 at IR2");
  tick(900, HIGH, LOW);
  tick(1100);
  tick(200);
  check(sortingPhase == SortingPhase::RETURNING && servoCommands.back().channel == 1,
        "class 2 returns CH1 after release");

  resetMachine();
  id = scanAtIr1();
  respond(id, 1);
  tick(10);
  tick(200);
  check(sortingPhase == SortingPhase::MOVING && servoCommands.empty(),
        "class 1 passes without servo movement");

  resetMachine();
  id = scanAtIr1();
  tick(PI_SCAN_TIMEOUT_MS, LOW);
  check(sortingPhase == SortingPhase::FAULT && mockPins[RELAY_PIN] == RELAY_OFF,
        "Pi timeout latches belt off");
  respond(id, 3);
  check(sortingPhase == SortingPhase::FAULT, "late result cannot restart fault");
  tick(30000, HIGH);
  Serial.inject("START\n");
  tick();
  check(sortingPhase == SortingPhase::FAULT && !isSystemRunning &&
        mockPins[RELAY_PIN] == RELAY_OFF, "fault never automatically restarts");

  resetMachine(LOW);
  tick(250, LOW);
  uint32_t retryId = pendingScanId;
  check(retryId != id && sortingPhase == SortingPhase::WAIT_PI,
        "reset with parked mango automatically requests a fresh scan");
  respond(id, 3);
  check(sortingPhase == SortingPhase::WAIT_PI, "old result cannot satisfy new boot");
  respond(retryId, 3);
  check(sortingPhase == SortingPhase::GATING, "fresh result works after reset");

  resetMachine();
  id = scanAtIr1();
  respond(id, 2);
  tick(MANGO_TRAVEL_TIMEOUT_MS);
  check(sortingPhase == SortingPhase::FAULT, "missing IR2 faults instead of running forever");

  resetMachine();
  id = scanAtIr1();
  respond(id, 3);
  tick(900, LOW);
  tick(MANGO_TRAVEL_TIMEOUT_MS, LOW);
  check(sortingPhase == SortingPhase::FAULT && mockPins[RELAY_PIN] == RELAY_OFF,
        "blocked exit faults");

  resetMachine();
  id = scanAtIr1();
  PiSerial.inject(std::string(80, 'X') + "\nRESULT," + std::to_string(id) + ",1\n");
  tick(10, LOW);
  tick(10, LOW);
  check(sortingPhase == SortingPhase::RELEASING, "UART recovers after overlong line");

  resetMachine();
  mockPcaPresent = false;
  servoDriverReady = false;
  setup();
  tick(600);
  check(sortingPhase == SortingPhase::FAULT && !isSystemRunning &&
        mockPins[RELAY_PIN] == RELAY_OFF, "missing PCA9685 prevents automatic startup");
  std::cout << "PASS: classes 1/2/3, UART framing/IDs, servo/belt sequence, automatic startup, reset and faults\n";
}
