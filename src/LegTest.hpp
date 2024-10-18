#pragma once

#include "Leg.hpp"

#define DEBUG_PRINTL(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)

class LegTest {
 private:
  Leg leg;
  int step;
  DataTypes::Vector initPos;

 public:
  LegTest(Leg &l) : leg(l), step(0), initPos({0, -30, 0}){}
  void setup() { 
    leg.setup(); 
    leg.setAngleInterval(10);
    leg.setAngleStep(1);
   }

  void stance() {
    while (!leg.cartesianMove(initPos));
  }

  bool loop() {
    double x = initPos.x;
    double y = initPos.y;
    double z = initPos.z;
    int xOffset = 50;
    int zOffset = 20;
    //DataTypes::Vector steps[] = {{x,y,z},{x,y,z+zOffset}, {x+xOffset, y, z+zOffset},{x + xOffset, y, z}};
    DataTypes::Vector steps[] = {{x-xOffset,y,z},{x - xOffset,y, z + zOffset}, {x+xOffset, y, z+zOffset},{x + xOffset, y, z}};
    DataTypes::Vector *p;
    int lastStepOfsteps;
    bool stepComplete = true;

    lastStepOfsteps = sizeof(steps) / sizeof(DataTypes::Vector) - 1;
    p = steps;
    stepComplete &= leg.cartesianMove(p[step]);
    static unsigned long timeout=0;
    if (millis() > timeout) {
      timeout=millis() + 100;
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
      } //
    }
    return false;
  }
};