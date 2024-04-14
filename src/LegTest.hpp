#pragma once
#include "DataTypes.hpp"
#include "Leg.hpp"

#define DEBUG_PRINTL(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)

class LegTest {
 private:
  Leg leg;
  int step;
  DataTypes::Vector initPos;

 public:
  //GaitTest(Leg &l) : leg(l), step(0), initPos({0,-30,-20}){}
  LegTest(Leg &l) : leg(l), step(0), initPos({0, 0, 0}){}
  void setup() { leg.Setup(); }

  void stance() {
    while (!leg.CartesianMove(initPos.x, initPos.y, initPos.z));
  }

  bool loop() {
    DataTypes::Vector nextPos={40, 0, 0};
    DataTypes::Vector steps[] = {initPos, nextPos};
    DataTypes::Vector *p;
    int lastStepOfsteps;
    bool stepComplete = true;

    lastStepOfsteps = sizeof(steps) / sizeof(DataTypes::Vector) - 1;
    p = steps;
    stepComplete &= leg.CartesianMove(p[step].x, p[step].y, p[step].z);

    if (stepComplete) {
      DEBUG_PRINT("step: ");
      DEBUG_PRINTL(step);
      DEBUG_PRINT("xMove: ");
      DEBUG_PRINTL(p[step].x);
      DEBUG_PRINT("yMove: ");
      DEBUG_PRINTL(p[step].y);
      DEBUG_PRINT("zMove: ");
      DEBUG_PRINTL(p[step].z);
      DEBUG_PRINTL("");
      if (step >= lastStepOfsteps) {
        step = 0;
        //while (1);
        return true;
      } else {
        step++;
      }
    }
    return false;
  }
};