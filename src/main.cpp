#include "mango.h"
#include <cstdio>
#include <cstring>
#include <esp_system.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include "cyd_link.h"
#include "camera_sorter.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
constexpr uint8_t CYD_MAC_ADDRESS[6] = {0xF4, 0x2D, 0xC9, 0x7A, 0x6A, 0xD4};
#ifndef IR_BELT_TEST_ENABLED
#define IR_BELT_TEST_ENABLED 1
#endif
constexpr bool IR_BELT_TEST_MODE = IR_BELT_TEST_ENABLED != 0;
constexpr bool SENSOR_STOP_TEST_MODE = false;
#define RELAY_PIN 26

#define IR1_PIN 32
#define IR2_PIN 33

#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define SERVO_FREQ 50
#define USMIN 500
#define USMAX 2500

#define SERVO1_CH 0
#define SERVO2_CH 1

const int servoMin = 150;
const int servoMax = 600;
constexpr int SERVO1_HOME_ANGLE = 65;
constexpr int SERVO1_GATE_ANGLE = 20;
// Temporary servo2 angles: calibrate these two values on the actual mechanism.
constexpr int SERVO2_HOME_ANGLE = 65;
constexpr int SERVO2_GATE_ANGLE = 20;
constexpr uint8_t IR1_ACTIVE_LEVEL = LOW;
constexpr uint32_t IR1_DEBOUNCE_MS = 30;
constexpr uint8_t IR2_ACTIVE_LEVEL = LOW;
constexpr uint32_t IR2_DEBOUNCE_MS = 30;

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

namespace {
CydLink cydLink;
CameraSorter cameraSorter;
bool isSystemRunning = false;
bool servoReady = false;
bool beltOutputOn = false;
bool gateOutputExtended = false;
bool gate2OutputExtended = false;
bool ir1RawBlocked = false;
bool ir1Blocked = false;
uint32_t ir1RawChangedMs = 0;
bool ir2RawBlocked = false;
bool ir2Blocked = false;
bool ir2StatusChanged = false;
uint32_t ir2RawChangedMs = 0;
uint32_t cameraScanId = 0;
CameraSorter::State reportedState = CameraSorter::State::Stopped;
uint8_t systemPhase = cyd::PHASE_HOMING;
bool statusDirty = true;
bool hasStatusAttempt = false;
uint32_t lastStatusAttemptMs = 0;
uint32_t lastStatusQueuedMs = 0;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;
constexpr uint32_t STATUS_RETRY_MS = 250;
String usbLine;
bool discardUsbLine = false;

// Call this from the conveyor state machine whenever its real phase changes.
void setSystemPhase(uint8_t phase) {
  if (phase == systemPhase) return;
  systemPhase = phase;
  statusDirty = true;
}

const char *stateName(CameraSorter::State state) {
  switch (state) {
    case CameraSorter::State::Stopped: return "STOPPED";
    case CameraSorter::State::Moving: return "MOVING";
    case CameraSorter::State::CameraSettling: return "CAMERA_SETTLING";
    case CameraSorter::State::WaitingPi: return "WAIT_CAMERA_RESULT";
    case CameraSorter::State::GateSettling: return "SERVO1_SETTLING";
    case CameraSorter::State::Rejecting: return "REJECT_UNTIL_IR1_CLEAR";
    case CameraSorter::State::Returning: return "SERVO1_RETURNING";
    case CameraSorter::State::Passing: return "PASS_TO_DARK_CHAMBER";
    case CameraSorter::State::MovingToDark: return "MOVING_TO_IR2";
    case CameraSorter::State::WaitingDarkPi: return "WAIT_DARK_RESULT";
    case CameraSorter::State::Gate2Settling: return "SERVO2_SETTLING";
    case CameraSorter::State::Rejecting2: return "SOUR_UNTIL_IR2_CLEAR";
    case CameraSorter::State::Returning2: return "SERVO2_RETURNING";
    case CameraSorter::State::SweetPassing: return "SWEET_TO_END";
    default: return "FAULT";
  }
}

uint8_t phaseForState(CameraSorter::State state) {
  switch (state) {
    case CameraSorter::State::Stopped: return cyd::PHASE_HOMING;
    case CameraSorter::State::CameraSettling: return cyd::PHASE_SCAN_SETTLING;
    case CameraSorter::State::WaitingPi: return cyd::PHASE_WAIT_PI;
    case CameraSorter::State::WaitingDarkPi: return cyd::PHASE_WAIT_PI;
    case CameraSorter::State::Passing:
    case CameraSorter::State::MovingToDark: return cyd::PHASE_WAIT_IR2;
    case CameraSorter::State::GateSettling: return cyd::PHASE_GATE_SETTLING;
    case CameraSorter::State::Gate2Settling: return cyd::PHASE_GATE_SETTLING;
    case CameraSorter::State::Rejecting: return cyd::PHASE_GATING;
    case CameraSorter::State::Rejecting2: return cyd::PHASE_GATING;
    case CameraSorter::State::Returning: return cyd::PHASE_RETURNING;
    case CameraSorter::State::Returning2: return cyd::PHASE_RETURNING;
    case CameraSorter::State::SweetPassing: return cyd::PHASE_RELEASING;
    case CameraSorter::State::Fault: return cyd::PHASE_FAULT;
    default: return cyd::PHASE_MOVING;
  }
}

void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  const int pulse = map(angle, 0, 180, servoMin, servoMax);
  pwm.setPWM(channel, 0, pulse);
}

void syncSorterOutputs(bool force = false) {
  const bool beltOn = cameraSorter.beltOn();
  if (force || beltOn != beltOutputOn) {
    const uint8_t level = beltOn ? RELAY_ON : RELAY_OFF;
    digitalWrite(RELAY_PIN, level);
    beltOutputOn = beltOn;
    Serial.printf("RELAY_CMD PIN=%u LEVEL=%u REQUEST=%s\n",
                  RELAY_PIN, level, beltOn ? "ON" : "OFF");
  }

  const bool gateExtended = cameraSorter.gateExtended();
  if (servoReady && (force || gateExtended != gateOutputExtended)) {
    const int angle = gateExtended ? SERVO1_GATE_ANGLE : SERVO1_HOME_ANGLE;
    setServoAngle(SERVO1_CH, angle);
    gateOutputExtended = gateExtended;
    Serial.printf("SERVO1 ANGLE=%d\n", angle);
  }

  const bool gate2Extended = cameraSorter.gate2Extended();
  if (servoReady && (force || gate2Extended != gate2OutputExtended)) {
    const int angle = gate2Extended ? SERVO2_GATE_ANGLE : SERVO2_HOME_ANGLE;
    setServoAngle(SERVO2_CH, angle);
    gate2OutputExtended = gate2Extended;
    Serial.printf("SERVO2 ANGLE=%d\n", angle);
  }

  if (isSystemRunning != cameraSorter.running()) {
    isSystemRunning = cameraSorter.running();
    statusDirty = true;
  }
  setSystemPhase(phaseForState(cameraSorter.state()));

  if (reportedState != cameraSorter.state()) {
    reportedState = cameraSorter.state();
    Serial.printf("STATE %s\n", stateName(reportedState));
    if (reportedState == CameraSorter::State::Fault) {
      switch (cameraSorter.fault()) {
        case CameraSorter::Fault::PiTimeout:
          Serial.println("ERR CAMERA_TIMEOUT"); break;
        case CameraSorter::Fault::Ir1Timeout:
          Serial.println("ERR IR1_CLEAR_TIMEOUT"); break;
        case CameraSorter::Fault::DarkPiTimeout:
          Serial.println("ERR DARK_PI_TIMEOUT"); break;
        case CameraSorter::Fault::Ir2Timeout:
          Serial.println("ERR IR2_CLEAR_TIMEOUT"); break;
        case CameraSorter::Fault::Ir2ArrivalTimeout:
          Serial.println("ERR IR2_ARRIVAL_TIMEOUT"); break;
        case CameraSorter::Fault::ServoMissing:
          Serial.println("ERR PCA9685_NOT_READY"); break;
        default: break;
      }
    }
  }
}

void applyRunCommand(bool run) {
  if (run && !servoReady)
    cameraSorter.fail(CameraSorter::Fault::ServoMissing, millis());
  else
    cameraSorter.setRunning(run, millis());
  syncSorterOutputs(true);
  statusDirty = true;
}

// GUI LENH CAMERA TU ESP32 SANG PI QUA USB/COM (115200 baud).
// Pi xu ly anh va tra ve mot dong "0\n" hoac "1\n".
void sendCameraCommandToPi() {
  Serial.println("CAMERA");
}

void updateIr1(uint32_t now) {
  const bool rawBlocked = digitalRead(IR1_PIN) == IR1_ACTIVE_LEVEL;
  if (rawBlocked != ir1RawBlocked) {
    ir1RawBlocked = rawBlocked;
    ir1RawChangedMs = now;
  }
  if (now - ir1RawChangedMs >= IR1_DEBOUNCE_MS &&
      ir1Blocked != ir1RawBlocked) {
    ir1Blocked = ir1RawBlocked;
    Serial.printf("IR1 %s\n", ir1Blocked ? "BLOCKED" : "CLEAR");
  }
}

void updateCameraSequence(uint32_t now) {
  cameraSorter.tick(now, ir1Blocked, ir2Blocked);
  // Stop relay before sending either model trigger to Pi.
  syncSorterOutputs();
  if (cameraSorter.takeCameraRequest()) {
    ++cameraScanId;
    statusDirty = true;
    sendCameraCommandToPi();
  }
  const bool darkRequest = cameraSorter.takeDarkRequest();
  // Report stable sensor edges once, AFTER applying the motor stop.
  const bool blockedReported = ir2StatusChanged && ir2Blocked;
  if (ir2StatusChanged) {
    Serial.println(ir2Blocked ? "IR2 BLOCKED" : "IR2 CLEAR");
    ir2StatusChanged = false;
  }
  // Re-arm the Pi if START is pressed with a mango already blocking IR2.
  if (darkRequest && !blockedReported) Serial.println("IR2 BLOCKED");
}

void updateIr2(uint32_t now) {
  const bool rawBlocked = digitalRead(IR2_PIN) == IR2_ACTIVE_LEVEL;
  if (rawBlocked != ir2RawBlocked) {
    ir2RawBlocked = rawBlocked;
    ir2RawChangedMs = now;
  }
  if (now - ir2RawChangedMs >= IR2_DEBOUNCE_MS &&
      ir2Blocked != ir2RawBlocked) {
    ir2Blocked = ir2RawBlocked;
    ir2StatusChanged = true;
  }
}

bool sendStatusToCyd() {
  hasStatusAttempt = true;
  lastStatusAttemptMs = millis();
  cyd::SystemPacket packet =
      cyd::makeStatus(systemPhase, isSystemRunning);
  packet.scanId = cameraScanId;
  // 0=unknown, 1=sweet, 2=sour, 3=external defect.
  packet.resultType = cameraSorter.result() == 0 ? 3 :
                      cameraSorter.darkResult() == 0 ? 2 :
                      cameraSorter.darkResult() == 1 ? 1 : 0;
  if (!cydLink.send(packet)) return false;
  lastStatusQueuedMs = millis();
  statusDirty = false;
  return true;
}

// Called by CydLink::poll() in loop(), never in the Wi-Fi callback.
void onCydPacket(const cyd::SystemPacket &packet, const uint8_t *) {
  applyRunCommand(packet.isRunning);
  Serial.printf("CYD_CMD %s\n", packet.isRunning ? "START" : "STOP");
  sendStatusToCyd();
}

void printStatus() {
  Serial.printf("STATUS READY=%u RUNNING=%u PHASE=%u CH=%u ",
                cydLink.ready(), isSystemRunning, systemPhase, ESPNOW_CHANNEL);
  Serial.print("MAC=");
  Serial.print(WiFi.macAddress());
  Serial.printf(" STATE=%s BELT=%u IR1=%u IR2=%u SERVO_READY=%u CAMERA_RESULT=%d DARK_RESULT=%d",
                stateName(cameraSorter.state()), beltOutputOn, ir1Blocked,
                ir2Blocked, servoReady, cameraSorter.result(), cameraSorter.darkResult());
  Serial.printf(" PEER=%02X:%02X:%02X:%02X:%02X:%02X",
                CYD_MAC_ADDRESS[0], CYD_MAC_ADDRESS[1], CYD_MAC_ADDRESS[2],
                CYD_MAC_ADDRESS[3], CYD_MAC_ADDRESS[4], CYD_MAC_ADDRESS[5]);
  Serial.printf(" TX_OK=%lu TX_FAIL=%lu RX_INVALID=%lu RX_DROP=%lu\n",
                static_cast<unsigned long>(cydLink.sentOk()),
                static_cast<unsigned long>(cydLink.sentFailed()),
                static_cast<unsigned long>(cydLink.invalidReceived()),
                static_cast<unsigned long>(cydLink.droppedReceived()));
}

void processUsbCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;
  if (line == "0" || line == "1") {
    const uint8_t result = static_cast<uint8_t>(line[0] - '0');
    const bool dark = cameraSorter.state() == CameraSorter::State::WaitingDarkPi;
    if (cameraSorter.acceptResult(result, millis())) {
      Serial.printf("%s_RESULT %u %s\n", dark ? "DARK" : "CAMERA", result,
                    dark ? (result == 0 ? "SOUR" : "SWEET") :
                           (result == 0 ? "DEFECT" : "PASS"));
      statusDirty = true;
    } else {
      Serial.println("ERR UNEXPECTED_PI_RESULT");
    }
    syncSorterOutputs();
  } else if (line == "STATUS") {
    printStatus();
  } else if (line == "SYNC") {
    statusDirty = true;
    Serial.println(sendStatusToCyd() ? "QUEUED STATUS" : "ERR SEND_STATUS");
  } else if (line == "START" || line == "RUN 1" ||
             line == "STOP" || line == "RUN 0") {
    applyRunCommand(line == "START" || line == "RUN 1");
    sendStatusToCyd();
    printStatus();
  } else {
    Serial.println("ERR COMMANDS: STATUS SYNC START STOP RUN_0_1 PI_RESULT_0_1");
  }
}
}  // namespace

void setup() {
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  Serial.begin(115200);
  Serial.printf("RELAY_INIT PIN=%u OFF_LEVEL=%u ON_LEVEL=%u\n",
                RELAY_PIN, RELAY_OFF, RELAY_ON);
  pinMode(IR1_PIN, INPUT_PULLUP);
  ir1RawBlocked = digitalRead(IR1_PIN) == IR1_ACTIVE_LEVEL;
  ir1RawChangedMs = millis();
  pinMode(IR2_PIN, INPUT_PULLUP);
  ir2RawBlocked = digitalRead(IR2_PIN) == IR2_ACTIVE_LEVEL;
  ir2RawChangedMs = millis();
  Wire.begin();  // SDA=21, SCL=22; PCA9685 address 0x40.
  servoReady = pwm.begin();
  if (servoReady) {
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
  } else {
    cameraSorter.fail(CameraSorter::Fault::ServoMissing, millis());
  }
  syncSorterOutputs(true);
  usbLine.reserve(64);
  const bool ready = cydLink.begin(CYD_MAC_ADDRESS, ESPNOW_CHANNEL, onCydPacket);
  Serial.println(ready ? "READY CYD_ESPNOW_16_BYTES" : "ERR ESPNOW_INIT");
  printStatus();
  if (ready) sendStatusToCyd();
}

void loop() {
  updateIr1(millis());
  updateIr2(millis());
  cydLink.poll();
  // Limit serial processing so a continuous stream cannot starve ESP-NOW.
  for (uint8_t i = 0; i < 64 && Serial.available(); ++i) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!discardUsbLine) processUsbCommand(usbLine);
      usbLine = "";
      discardUsbLine = false;
    } else if (c != '\r' && !discardUsbLine) {
      if (usbLine.length() < 64) {
        usbLine += c;
      } else {
        discardUsbLine = true;
        usbLine = "";
        Serial.println("ERR LINE_TOO_LONG");
      }
    }
  }

  const uint32_t now = millis();
  updateCameraSequence(now);
  if (cydLink.ready() &&
      (statusDirty || now - lastStatusQueuedMs >= STATUS_INTERVAL_MS) &&
      (!hasStatusAttempt || now - lastStatusAttemptMs >= STATUS_RETRY_MS))
    sendStatusToCyd();
  delay(1);
}
