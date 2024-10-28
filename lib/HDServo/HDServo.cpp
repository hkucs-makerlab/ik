#include "HDServo.h"

static bool inited = false;
static bool channels[18] = {false,false,false, false, false, false, false, false, false,
                            false,false,false, false, false, false, false, false, false};

HDServo::HDServo() : _ch(-1) {}

void HDServo::attach(int ch, int min, int max) {
  if (!inited) {
    //Serial.println("init");
    ServoSetup();
    inited = true;
  }

  if (ch == -1 || channels[ch]) {
    return;
  }

  _ch = constrain(ch, 0, 17);
  //Serial.println("attach " + String(_ch));
  channels[_ch] = true;
}

void HDServo::detach() {
  if (_ch >= 0) {
    _ch = -1;
    channels[_ch] = false;
  }
}

void HDServo::write(int angle) {
  if (_ch == -1 || channels[_ch] == false) {
    return;
  }
  angle = constrain(angle, 0, 180);
  ServoMoveAngle(_ch, angle, 10);
}