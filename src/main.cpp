#include "mango.h"
#include <cstdio>
#include <cstring>
#include <esp_system.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>                                       
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
constexpr uint8_t CYD_MAC_ADDRESS[6] = {0xF4, 0x2D, 0xC9, 0x7A, 0x6A, 0xD4};
constexpr bool SENSOR_STOP_TEST_MODE = false;
#define BUTTON_PIN 34
#define RELAY_PIN 26

#define IR1_PIN 32
#define IR2_PIN 33
#define IR3_PIN 25

#define RELAY_ON LOW
#define RELAY_OFF HIGH

#define SERVO_FREQ 50
#define USMIN 500
#define USMAX 2500

#define SERVO1_CH 0
#define SERVO2_CH 1

constexpr uint16_t SERVO_REST_US = 1500;
constexpr uint16_t SERVO_GATE_US = 1700;
constexpr unsigned long BELT_SETTLE_MS = 200;
constexpr unsigned long SERVO_HOLD_MS = 800;
constexpr unsigned long SERVO_RETURN_MS = 500;

enum class TestServoPhase { IDLE, SETTLING, GATING, RETURNING };
struct TestServoState {
  TestServoPhase phase = TestServoPhase::RETURNING;
  unsigned long phaseStarted = 0;
  bool obstacleHandled = false;
};
TestServoState testServos[2];
bool servoDriverReady = false;

bool isSystemRunning = false;
Mango currentMango;

bool updateTestServos(uint8_t sensorMask) {
  const uint8_t channels[2] = {SERVO1_CH, SERVO2_CH};
  unsigned long now = millis();
  bool busy = false;

  for (uint8_t i = 0; i < 2; ++i) {
    TestServoState &state = testServos[i];
    bool blocked = (sensorMask & (1 << i)) != 0;
    if (!blocked)
      state.obstacleHandled = false;

    if (!isSystemRunning && state.phase == TestServoPhase::SETTLING) {
      state.phase = TestServoPhase::IDLE;
    } else if (state.phase == TestServoPhase::GATING &&
               (!isSystemRunning || now - state.phaseStarted >= SERVO_HOLD_MS)) {
      pwm.writeMicroseconds(channels[i], SERVO_REST_US);
      state.phase = TestServoPhase::RETURNING;
      state.phaseStarted = now;
      Serial.printf("TEST_SERVO: CH%d RETURN\n", channels[i]);
    } else if (state.phase == TestServoPhase::RETURNING &&
               now - state.phaseStarted >= SERVO_RETURN_MS) {
      state.phase = TestServoPhase::IDLE;
    }

    if (state.phase == TestServoPhase::IDLE && isSystemRunning &&
        blocked && !state.obstacleHandled) {
      state.obstacleHandled = true;
      state.phase = TestServoPhase::SETTLING;
      state.phaseStarted = now;
    }

    if (state.phase == TestServoPhase::SETTLING &&
        now - state.phaseStarted >= BELT_SETTLE_MS) {
      pwm.writeMicroseconds(channels[i], SERVO_GATE_US);
      state.phase = TestServoPhase::GATING;
      state.phaseStarted = now;
      Serial.printf("TEST_SERVO: IR%d -> CH%d GATE\n", i + 1, channels[i]);
    }
    busy |= state.phase != TestServoPhase::IDLE;
  }
  return busy;
}

void runSensorStopTest() {
  if (!servoDriverReady) {
    isSystemRunning = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
    delay(10);
    return;
  }

  static int lastButtonReading = HIGH;
  static int stableButtonState = HIGH;
  static unsigned long lastButtonChange = 0;
  static uint8_t lastSensorMask = 0xFF;
  static bool lastConveyorRunning = false;
  static unsigned long lastStatusLog = 0;

  int ir1Raw = digitalRead(IR1_PIN);
  int ir2Raw = digitalRead(IR2_PIN);
  uint8_t sensorMask = 0;
  if (ir1Raw == LOW)
    sensorMask |= 1;
  if (ir2Raw == LOW)
    sensorMask |= 2;
  bool obstacleDetected = sensorMask != 0;

  if (sensorMask != lastSensorMask) {
    Serial.printf("TEST_IR: IR1=%d IR2=%d (1=blocked)\n",
                  (sensorMask & 1) != 0, (sensorMask & 2) != 0);
    lastSensorMask = sensorMask;
  }

  int buttonReading = digitalRead(BUTTON_PIN);
  if (buttonReading != lastButtonReading) {
    lastButtonChange = millis();
    lastButtonReading = buttonReading;
  }

  // Toggle once per debounced press, without pausing sensor checks.
  if (millis() - lastButtonChange >= 50 &&
      buttonReading != stableButtonState) {
    stableButtonState = buttonReading;
    if (stableButtonState == LOW) {
      if (isSystemRunning) {
        isSystemRunning = false;
        Serial.println("SYSTEM_STOP");
      } else if (!obstacleDetected) {
        isSystemRunning = true;
        Serial.println("SYSTEM_START");
      } else {
        Serial.println("TEST_START_BLOCKED: clear IR1 and IR2 first");
      }
    }
  }

  // Stop the belt before any servo command, then wait for the return motion.
  if (!isSystemRunning || obstacleDetected)
    digitalWrite(RELAY_PIN, RELAY_OFF);
  bool servosBusy = updateTestServos(sensorMask);
  bool conveyorRunning = isSystemRunning && !obstacleDetected && !servosBusy;
  int relayCommand = conveyorRunning ? RELAY_ON : RELAY_OFF;
  digitalWrite(RELAY_PIN, relayCommand);

  if (conveyorRunning != lastConveyorRunning) {
    if (conveyorRunning) {
      Serial.println("TEST_CONVEYOR_RUNNING");
    } else if (isSystemRunning && obstacleDetected) {
      Serial.println("TEST_OBSTACLE_STOP: auto-resume when both IRs are clear and servos are home");
    }
    lastConveyorRunning = conveyorRunning;
  }

  // Report raw inputs separately from the interpreted obstacle flags.
  if (millis() - lastStatusLog >= 1000) {
    Serial.printf("TEST_RAW: IR1=%d IR2=%d BUTTON=%d ENABLED=%d RUN=%d "
                  "SERVO_BUSY=%d RELAY_CMD=%d\n",
                  ir1Raw, ir2Raw, buttonReading, isSystemRunning,
                  conveyorRunning, servosBusy, relayCommand);
    lastStatusLog = millis();
  }
  delay(10);
}

constexpr int PI_UART_RX_PIN = 16;
constexpr int PI_UART_TX_PIN = 17;
constexpr uint32_t PI_UART_BAUD = 115200;
constexpr unsigned long PI_SCAN_TIMEOUT_MS = 15000;
constexpr unsigned long MANGO_TRAVEL_TIMEOUT_MS = 10000;
constexpr unsigned long MANGO_DROP_MS = 1000;
constexpr unsigned long IR_CLEAR_MS = 150;

HardwareSerial PiSerial(2);
enum class SortingPhase {
  STOPPED, HOMING, MOVING, SCAN_SETTLING, WAIT_PI, WAIT_IR2,
  GATE_SETTLING, GATING, RELEASING, RETURNING, FAULT
};
SortingPhase sortingPhase = SortingPhase::HOMING;
unsigned long sortingPhaseStarted = 0;
unsigned long releaseClearStarted = 0;
bool releaseClearTracking = false;
bool releaseUsesServo = false;
uint8_t activeServoChannel = SERVO1_CH;
int releaseSensorPin = IR1_PIN;
uint32_t scanSequence = 0;
uint32_t pendingScanId = 0;
char piLine[64];
size_t piLineLength = 0;
bool piLineOverflow = false;

const char *sortingPhaseName() {
  switch (sortingPhase) {
  case SortingPhase::STOPPED: return "STOPPED";
  case SortingPhase::HOMING: return "HOMING";
  case SortingPhase::MOVING: return "MOVING";
  case SortingPhase::SCAN_SETTLING: return "SCAN_SETTLING";
  case SortingPhase::WAIT_PI: return "WAIT_PI";
  case SortingPhase::WAIT_IR2: return "WAIT_IR2";
  case SortingPhase::GATE_SETTLING: return "GATE_SETTLING";
  case SortingPhase::GATING: return "GATING";
  case SortingPhase::RELEASING: return "RELEASING";
  case SortingPhase::RETURNING: return "RETURNING";
  case SortingPhase::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

void setSortingPhase(SortingPhase phase) {
  sortingPhase = phase;
  sortingPhaseStarted = millis();
}

void setConveyor(bool running) {
  digitalWrite(RELAY_PIN, running ? RELAY_ON : RELAY_OFF);
}

void homeSortingServos() {
  if (servoDriverReady) {
    pwm.writeMicroseconds(SERVO1_CH, SERVO_REST_US);
    pwm.writeMicroseconds(SERVO2_CH, SERVO_REST_US);
  }
}

void sortingFault(const char *reason) {
  setConveyor(false);
  isSystemRunning = false;
  pendingScanId = 0;
  homeSortingServos();
  setSortingPhase(SortingPhase::FAULT);
  Serial.printf("SORT_ERROR: %s; conveyor stopped; press Start to retry\n", reason);
  PiSerial.printf("ERROR,%s\n", reason);
}

void beginRelease(int sensorPin, bool usesServo) {
  releaseSensorPin = sensorPin;
  releaseUsesServo = usesServo;
  releaseClearTracking = false;
  setSortingPhase(SortingPhase::RELEASING);
  setConveyor(true);
}

void beginGate(uint8_t channel) {
  setConveyor(false);
  activeServoChannel = channel;
  pwm.writeMicroseconds(channel, SERVO_GATE_US);
  setSortingPhase(SortingPhase::GATING);
  Serial.printf("SORT_SERVO: CH%d GATE\n", channel);
}

void handlePiLine(const char *line) {
  if (strcmp(line, "P") == 0) {
    PiSerial.println("PONG");
    return;
  }

  unsigned long requestId = 0;
  unsigned int result = 0;
  int consumed = 0;
  if (sscanf(line, "RESULT,%lu,%u%n", &requestId, &result, &consumed) != 2 ||
      consumed != static_cast<int>(strlen(line)) || requestId == 0 ||
      requestId > UINT32_MAX || result < 1 || result > 3) {
    Serial.printf("PI_IGNORED: %s\n", line);
    return;
  }
  char canonical[64];
  snprintf(canonical, sizeof(canonical), "RESULT,%lu,%u", requestId, result);
  if (strcmp(canonical, line) != 0) {
    Serial.println("PI_IGNORED: invalid result format");
    return;
  }
  if (!isSystemRunning || sortingPhase != SortingPhase::WAIT_PI ||
      requestId != pendingScanId ||
      millis() - sortingPhaseStarted >= PI_SCAN_TIMEOUT_MS) {
    Serial.println("PI_IGNORED: stale or unexpected result");
    return;
  }

  pendingScanId = 0;
  currentMango.detected = true;
  currentMango.type = result == 3 ? MangoType::TOP3 :
                      result == 2 ? MangoType::TOP2 : MangoType::TOP1;
  Serial.printf("PI_RESULT: ID=%lu TYPE=%u\n", requestId, result);
  if (result == 3) {
    beginGate(SERVO1_CH);
  } else if (result == 2) {
    setSortingPhase(SortingPhase::WAIT_IR2);
    setConveyor(true);
  } else {
    beginRelease(IR1_PIN, false);
  }
}

void pollPiUart() {
  // Bound work per loop so incoming UART traffic cannot starve the Stop button.
  for (uint8_t count = 0; count < 64 && PiSerial.available() > 0; ++count) {
    char c = static_cast<char>(PiSerial.read());
    if (c == 'P' && piLineLength == 0 && !piLineOverflow) {
      PiSerial.println("PONG");
    } else if (c == '\n') {
      if (!piLineOverflow && piLineLength != 0) {
        piLine[piLineLength] = '\0';
        handlePiLine(piLine);
      }
      piLineLength = 0;
      piLineOverflow = false;
    } else if (c != '\r' && !piLineOverflow) {
      if (piLineLength < sizeof(piLine) - 1)
        piLine[piLineLength++] = c;
      else
        piLineOverflow = true;
    }
  }
}

void pollSortingButton() {
  static int lastReading = HIGH;
  static int stableState = HIGH;
  static unsigned long changedAt = 0;
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    changedAt = millis();
    lastReading = reading;
  }
  if (millis() - changedAt < 50 || reading == stableState)
    return;
  stableState = reading;
  if (stableState != LOW)
    return;

  if (isSystemRunning) {
    setConveyor(false);
    isSystemRunning = false;
    pendingScanId = 0;
    currentMango = Mango();
    homeSortingServos();
    setSortingPhase(SortingPhase::HOMING);
    Serial.println("SYSTEM_STOP");
    PiSerial.println("SYSTEM_STOP");
  } else if (!servoDriverReady ||
             sortingPhase == SortingPhase::HOMING ||
             (sortingPhase == SortingPhase::FAULT &&
              millis() - sortingPhaseStarted < SERVO_RETURN_MS)) {
    Serial.println("START_BLOCKED: check PCA9685 and wait for servos to return");
  } else {
    isSystemRunning = true;
    currentMango = Mango();
    setSortingPhase(digitalRead(IR1_PIN) == LOW ?
                    SortingPhase::SCAN_SETTLING : SortingPhase::MOVING);
    Serial.println("SYSTEM_START");
    PiSerial.println("SYSTEM_START");
  }
}

void runPiSorting() {
  pollSortingButton();
  pollPiUart();
  unsigned long now = millis();

  switch (sortingPhase) {
  case SortingPhase::STOPPED:
  case SortingPhase::FAULT:
    setConveyor(false);
    break;
  case SortingPhase::HOMING:
    setConveyor(false);
    if (now - sortingPhaseStarted >= SERVO_RETURN_MS)
      setSortingPhase(SortingPhase::STOPPED);
    break;
  case SortingPhase::MOVING:
    if (digitalRead(IR1_PIN) == LOW) {
      setConveyor(false);
      setSortingPhase(SortingPhase::SCAN_SETTLING);
    } else {
      setConveyor(true);
    }
    break;
  case SortingPhase::SCAN_SETTLING:
    setConveyor(false);
    if (now - sortingPhaseStarted >= BELT_SETTLE_MS) {
      if (++scanSequence == 0)
        ++scanSequence;
      pendingScanId = scanSequence;
      setSortingPhase(SortingPhase::WAIT_PI);
      PiSerial.printf("SCAN,%lu\n", static_cast<unsigned long>(pendingScanId));
      Serial.printf("PI_SCAN: ID=%lu; conveyor stopped\n",
                    static_cast<unsigned long>(pendingScanId));
    }
    break;
  case SortingPhase::WAIT_PI:
    setConveyor(false);
    if (now - sortingPhaseStarted >= PI_SCAN_TIMEOUT_MS)
      sortingFault("PI_TIMEOUT");
    break;
  case SortingPhase::WAIT_IR2:
    if (digitalRead(IR2_PIN) == LOW) {
      setConveyor(false);
      activeServoChannel = SERVO2_CH;
      setSortingPhase(SortingPhase::GATE_SETTLING);
    } else if (now - sortingPhaseStarted >= MANGO_TRAVEL_TIMEOUT_MS) {
      sortingFault("IR2_TIMEOUT");
    } else {
      setConveyor(true);
    }
    break;
  case SortingPhase::GATE_SETTLING:
    setConveyor(false);
    if (now - sortingPhaseStarted >= BELT_SETTLE_MS)
      beginGate(activeServoChannel);
    break;
  case SortingPhase::GATING:
    setConveyor(false);
    if (now - sortingPhaseStarted >= SERVO_HOLD_MS)
      beginRelease(activeServoChannel == SERVO1_CH ? IR1_PIN : IR2_PIN, true);
    break;
  case SortingPhase::RELEASING:
    setConveyor(true);
    if (digitalRead(releaseSensorPin) == LOW) {
      releaseClearTracking = false;
    } else if (!releaseClearTracking) {
      releaseClearTracking = true;
      releaseClearStarted = now;
    }
    if (releaseClearTracking && now - releaseClearStarted >= IR_CLEAR_MS &&
        (!releaseUsesServo || now - sortingPhaseStarted >= MANGO_DROP_MS)) {
      if (releaseUsesServo) {
        setConveyor(false);
        pwm.writeMicroseconds(activeServoChannel, SERVO_REST_US);
        setSortingPhase(SortingPhase::RETURNING);
        Serial.printf("SORT_SERVO: CH%d RETURN\n", activeServoChannel);
      } else {
        currentMango = Mango();
        setSortingPhase(SortingPhase::MOVING);
      }
    } else if (now - sortingPhaseStarted >= MANGO_TRAVEL_TIMEOUT_MS) {
      sortingFault("EXIT_TIMEOUT");
    }
    break;
  case SortingPhase::RETURNING:
    setConveyor(false);
    if (now - sortingPhaseStarted >= SERVO_RETURN_MS) {
      currentMango = Mango();
      setSortingPhase(SortingPhase::MOVING);
    }
    break;
  }

  static unsigned long lastLog = 0;
  if (now - lastLog >= 1000) {
    Serial.printf("SORT_STATUS: STATE=%s IR1=%d IR2=%d ENABLED=%d RELAY_CMD=%d\n",
                  sortingPhaseName(), digitalRead(IR1_PIN), digitalRead(IR2_PIN),
                  isSystemRunning, digitalRead(RELAY_PIN));
    lastLog = now;
  }
  delay(5);
}

void setup() {
  Serial.begin(115200);
  PiSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  scanSequence = esp_random();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, RELAY_OFF);

  if (SENSOR_STOP_TEST_MODE) {
    if (!pwm.begin()) {
      Serial.println("TEST_ERROR: PCA9685 not found; conveyor stays stopped");
      return;
    }
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    Wire.setClock(400000);
    pwm.writeMicroseconds(SERVO1_CH, SERVO_REST_US);
    pwm.writeMicroseconds(SERVO2_CH, SERVO_REST_US);
    for (auto &state : testServos)
      state.phaseStarted = millis();
    servoDriverReady = true;
    Serial.println("TEST_READY: IR1 (GPIO32) and IR2 (GPIO33); clear both, then press button to start");
    Serial.println("TEST_SERVOS: IR1 -> CH0, IR2 -> CH1; one cycle per obstacle");
    Serial.println("TEST_AUTO_RESUME: conveyor restarts when both sensors are clear and servos are home");
    return;
  }

  if (!pwm.begin()) {
    sortingFault("PCA9685_NOT_FOUND");
    return;
  }
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(400000);
  servoDriverReady = true;
  homeSortingServos();
  setSortingPhase(SortingPhase::HOMING);
  Serial.println("SORT_READY: IR1 scan -> Pi UART; type 3 CH0, type 2 at IR2 CH1");
  Serial.println("SORT_READY: press Start after servos return; one mango at a time");
}

void loop() {
  if (SENSOR_STOP_TEST_MODE) {
    pollPiUart();
    runSensorStopTest();
    return;
  }
  runPiSorting();
}
