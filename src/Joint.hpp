#pragma once

#ifdef ESP32
#include <ESP32Servo.h>
#endif

#ifdef AVR
#include <Servo.h>
#endif

class Joint {
 private:
  unsigned long prevTime;
  bool Inverted;
  int angle;
  int8_t AngleOffset;
  Servo _Servo;
  uint8_t _Pin;

 public:
  // CTOR
  Joint(uint8_t pin = -1, bool inv = false) : Inverted(inv), angle(90), AngleOffset(0), _Pin(pin) {
  }

  // Methods
  void Setup(uint8_t Pin, bool I = false, int8_t Ao = 0) {
    _Pin = Pin;
    Inverted = I;
    AngleOffset = Ao;
    _Servo.attach(_Pin, 500, 2400);
    _Servo.write(angle + AngleOffset);
    prevTime = millis();
  }

  void Setup(int8_t Ao = 0) {
    AngleOffset = Ao;
    _Servo.attach(_Pin, 500, 2400);
    _Servo.write(angle + AngleOffset);
    prevTime = millis();
  }

  bool Update(int targetAngle, unsigned long __angleTimeGap = 10) {
    // Serial.println(targetAngle);
    if (angle == targetAngle) {
      return true;
    }

    if (millis() - prevTime >= __angleTimeGap) {
      prevTime = millis();
      if (targetAngle < angle) {
        angle--;
        if (angle < 0) angle = 0;
      } else {
        angle++;
        if (angle > 180) angle = 180;
      }
      if (Inverted) {
        _Servo.write(180 - angle + AngleOffset);
      } else {
        _Servo.write(angle + AngleOffset);
      }
    }
    return false;
  }
};
