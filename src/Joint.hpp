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
  Joint(uint8_t pin, int8_t Ao = 0, bool inv = false)
      : Inverted(inv), angle(90), AngleOffset(Ao), _Pin(pin) {}

  void attach() {
    _Servo.attach(_Pin, 500, 2400);
    writeAngle();
    prevTime = millis();
  }

  void setAngleOffset(int8_t Ao) { AngleOffset = Ao; }

  bool Update(int targetAngle, unsigned long __angleTimeGap = 20) {
    // Serial.println(targetAngle);
    if (angle == targetAngle) {
      return true;
    }
    if (targetAngle < 0) {
      Serial.println("target angle is not valid");
      return true;
    }
    if (millis() - prevTime >= __angleTimeGap) {
      if (targetAngle < angle) {
        angle--;
        if (angle < 0) angle = 0;
      } else {
        angle++;
        if (angle > 180) angle = 180;
      }
      //
      writeAngle();
      prevTime = millis();
    }
    return false;
  }
  private:
    inline void writeAngle() {
      if (Inverted) {
        _Servo.write(180 - (angle + AngleOffset));
      } else {
        _Servo.write(angle + AngleOffset);
      }
    }
};
