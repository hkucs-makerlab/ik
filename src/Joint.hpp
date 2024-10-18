#pragma once

#include <Arduino.h>
#include <Servo.h>

#define Console Serial

class Joint : public Servo {
  int _pin, _offset, _refAngle;
  bool _rev;
  unsigned long _prevTime;
  int _timeInterval;
  int _angleInterval;

 public:
  Joint(int pin, int offset = 0, bool rev = false)
      : _pin(pin),
        _offset(offset),
        _refAngle(90),
        _rev(rev),
        _prevTime(0),
        _timeInterval(5),
        _angleInterval(3) {}

  void setup() {
    attach(_pin, 500, 2400);
    write(_refAngle);
    // Serial.println("Joint setup " + String(_pin));
  }
  

  void setTimeInterval(int t) { _timeInterval = t; }

  void setAngleInterval(int a) { _angleInterval = a; }

  void toggleInvert() {
    _rev = !_rev;
    write(_refAngle);
  }

  inline void write(int angle) {
    angle = constrain(angle + _offset, 0, 180);
    if (_rev) angle = 180 - angle;
    Servo::write(angle);
  }

  bool interpolateMove(int targetAngle) {
    if (_refAngle == targetAngle) {
      return true;
    }

    if (millis() > _prevTime) {
      if (targetAngle < _refAngle) {
        _refAngle -= _angleInterval;
        if (_refAngle <= targetAngle) {
          _refAngle = targetAngle;
        }
      } else {
        _refAngle += _angleInterval;
        if (_refAngle >= targetAngle) {
          _refAngle = targetAngle;
        }
      }
      //
      write(_refAngle);
      _prevTime = millis() + _timeInterval;
    }
    return false;
  }
};
