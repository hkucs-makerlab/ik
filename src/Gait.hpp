#pragma once

// CO-ORDINATE SYSTEM
//          x
//          ^
//          |
//          |
// y <------o
// +ve z = OUT OF SCREEN

// LEG CONFIGURATION
//
//        FRONT
// L2-------|-------L3
//   -------|-------
// L1-------|-------L4
//
// https://youtu.be/HjmIOKSp7v4
//

// Debug
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTL(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTL(x)
#define DEBUG_PRINT(x)
#endif

static const double lines[][3] = {{0.0, 0.0, 0.0},

                                  {-60.0, 40.0, 20.0}, {-60.0, 40.0, -20.0},
                                  {60.0, 40.0, -20.0}, {60.0, 60.0, 20.0},

                                  {-60.0, 30.0, 20.0}, {-60.0, 30.0, -20.0},
                                  {60.0, 30.0, -20.0}, {60.0, 50.0, 20.0},

                                  {-60.0, 20.0, 20.0}, {-60.0, 20.0, -20.0},
                                  {60.0, 20.0, -20.0}, {60.0, 40.0, 20.0},

                                  {-60.0, 10.0, 20.0}, {-60.0, 10.0, -20.0},
                                  {60.0, 10.0, -20.0}, {60.0, 30.0, 20.0},

                                  {-60.0, 0.0, 20.0},  {-60.0, 0.0, -20.0},
                                  {60.0, 0.0, -20.0},  {60.0, 20.0, 20.0},

                                  {-60.0, 40.0, 20.0}, {-60.0, 40.0, -20.0},
                                  {-60.0, 0.0, -20.0}, {-60.0, 20.0, 20.0},

                                  {0.0, 0.0, 0.0},  // line 25

                                  {-20.0, 40.0, 20.0}, {-20.0, 40.0, -20.0},
                                  {-20.0, 0.0, -20.0}, {-20.0, 20.0, 20.0},

                                  {-10.0, 40.0, 20.0}, {-10.0, 40.0, -20.0},
                                  {-10.0, 0.0, -20.0}, {-10.0, 20.0, 20.0},

                                  {0.0, 40.0, 20.0},   {0.0, 40.0, -20.0},
                                  {0.0, 0.0, -20.0},   {0.0, 20.0, 20.0},

                                  {10.0, 40.0, 20.0},  {10.0, 40.0, -20.0},
                                  {10.0, 0.0, -20.0},  {10.0, 20.0, 20.0},

                                  {20.0, 40.0, 0.0},   {20.0, 40.0, -20.0},
                                  {20.0, 0.0, -20.0},  {20.0, 20.0, 0.0},

                                  {30.0, 40.0, 0.0},   {30.0, 40.0, -20.0},
                                  {30.0, 0.0, -20.0},  {30.0, 20.0, 0.0},

                                  {40.0, 40.0, 0.0},   {40.0, 40.0, -20.0},
                                  {40.0, 0.0, -20.0},  {40.0, 20.0, 0.0},

                                  {50.0, 40.0, 0.0},   {50.0, 40.0, -20.0},
                                  {50.0, 0.0, -20.0},  {50.0, 20.0, 0.0},

                                  {60.0, 40.0, 0.0},   {60.0, 40.0, -20.0},
                                  {60.0, 0.0, -20.0},  {60.0, 20.0, 0.0},

                                  {0.0, 0.0, 0.0}};

class CreepGait {
  Leg &leftRear;
  Leg &leftFront;
  Leg &rightFront;
  Leg &rightRear;

  // Joint Variables
  double AXAct;
  double AYAct;
  double AZAct;

  uint8_t step;

 public:
  CreepGait(Leg &l1, Leg &l2, Leg &l3, Leg &l4)
      : leftRear(l1), leftFront(l2), rightFront(l3), rightRear(l4),
      AXAct(0),AYAct(0),AZAct(0),step(0) {}

  void setup() {
    // all servos at 90 degree after setup
    leftRear.Setup();
    leftFront.Setup();
    rightFront.Setup();
    rightRear.Setup();

    standUp();
  }

  void standUp() {
    bool l = false;
    while (!l) {
      l = true;
      l &= leftRear.CartesianMove(AXAct, AYAct, AZAct);
      l &= leftFront.CartesianMove(AXAct, AYAct, AZAct);
      l &= rightFront.CartesianMove(AXAct,AYAct, AZAct);
      l &= rightRear.CartesianMove(AXAct, AYAct, AZAct);
    }
  }

  void test() {
    const int lastLine = 62;
    bool stepComplete = true;

    //stepComplete &= leftRear.CartesianMove(AXAct, AYAct, AZAct);
    //stepComplete &= leftFront.CartesianMove(AXAct, AYAct, AZAct);
    stepComplete &= rightFront.CartesianMove(AXAct, AYAct, AZAct);
    //stepComplete &= rightRear.CartesianMove(AXAct, AYAct, AZAct);

    if (stepComplete) {
      if (step > lastLine) {
        step = 0;
        while (1)
          ;
        return;
      }
      AXAct = lines[step][0];
      AYAct = lines[step][1];
      AZAct = lines[step][2];
      step++;
      DEBUG_PRINT("xMove: ");
      DEBUG_PRINTL(AXAct);
      DEBUG_PRINT("yMove: ");
      DEBUG_PRINTL(AYAct);
      DEBUG_PRINT("zMove: ");
      DEBUG_PRINTL(AZAct);
      DEBUG_PRINTL("");
    }
  }
};
