#include "mango.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

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

bool isSystemRunning = false;
Mango currentMango;

void gatServo(uint8_t channel, int angle = 90, int holdTimeMs = 800) {
  int usGat = map(angle, 0, 180, USMIN, USMAX);
  int usVe = map(0, 0, 180, USMIN, USMAX);

  pwm.writeMicroseconds(channel, usGat);
  delay(holdTimeMs);
  pwm.writeMicroseconds(channel, usVe);
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, RELAY_OFF);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(400000);

  int us0 = map(0, 0, 180, USMIN, USMAX);
  pwm.writeMicroseconds(SERVO1_CH, us0);
  pwm.writeMicroseconds(SERVO2_CH, us0);

  Serial.println("Hệ thống phân loại xoài sẵn sàng!");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(200); 
    isSystemRunning = !isSystemRunning;
    if (isSystemRunning) {
      Serial.println("SYSTEM_START");
      digitalWrite(RELAY_PIN, RELAY_ON);
    } else {
      Serial.println("SYSTEM_STOP");
      digitalWrite(RELAY_PIN, RELAY_OFF);
    }
  }

  if (!isSystemRunning)
    return;

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '3' || cmd == 'H') {
      currentMango.detected = true;
      currentMango.type = MangoType::TOP3;
      Serial.println("CONFIRM: BAD_MANGO (TOP3)");
    }
  }

  if (digitalRead(IR1_PIN) == LOW) {
    if (currentMango.type == MangoType::TOP3) {
      Serial.println("IR1: Gat xoai TOP3");
      digitalWrite(RELAY_PIN, RELAY_OFF);
      delay(200);

      gatServo(SERVO1_CH, 90, 800);

      currentMango = Mango();
      delay(200);
      digitalWrite(RELAY_PIN, RELAY_ON);
    }
  }

  if (digitalRead(IR2_PIN) == LOW) {
    Serial.println("IR2: Dung tai Buong Toi... Gui lenh SCAN");
    digitalWrite(RELAY_PIN, RELAY_OFF);

    Serial.println("SCAN");

    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      if (Serial.available() > 0) {
        char res = Serial.read();
        if (res == '1') {
          currentMango.type = MangoType::TOP1;
          currentMango.detected = true;
          break;
        } else if (res == '2') {
          currentMango.type = MangoType::TOP2;
          currentMango.detected = true;
          break;
        }
      }
    }

    delay(200);
    digitalWrite(RELAY_PIN, RELAY_ON);
    delay(1000);
  }

  if (digitalRead(IR3_PIN) == LOW) {
    if (currentMango.type == MangoType::TOP2) {
      Serial.println("IR3: Gat xoai TOP2");
      digitalWrite(RELAY_PIN, RELAY_OFF);
      delay(200);

      gatServo(SERVO2_CH, 90, 800);

      currentMango = Mango();
      delay(200);
      digitalWrite(RELAY_PIN, RELAY_ON);
    } else if (currentMango.type == MangoType::TOP1) {
      Serial.println("IR3: Xoai TOP1 -> Cho đi thẳng");
      currentMango = Mango();
    }
  }

  delay(30);
}