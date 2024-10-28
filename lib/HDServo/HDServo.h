#pragma once
#define __HDServo__
#include "pwm.hpp"

class HDServo {
  private:
    int _ch;

  public:
    HDServo();
    void attach(int ch, int min=0, int max=0);
    void detach();
    void write(int angle);
       
};