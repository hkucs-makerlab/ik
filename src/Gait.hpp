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



class Gait {
  Leg &leftRear;
  Leg &leftFront;
  Leg &rightFront;
  Leg &rightRear;

 public:
  Gait(Leg &l1, Leg &l2, Leg &l3, Leg &l4)
      : leftRear(l1), leftFront(l2), rightFront(l3), rightRear(l4){}

  void setup() {
    // all servos at 90 degree after setup
    leftRear.Setup();
    leftFront.Setup();
    rightFront.Setup();
    rightRear.Setup();
  }

  void stance() {
    bool l = false;
    while (!l) {
      l = true;
      l &= leftRear.CartesianMove(0, 0, 0);
      l &= leftFront.CartesianMove(0, 0, 0);
      l &= rightFront.CartesianMove(0,0, 0);
      l &= rightRear.CartesianMove(0, 0, 0);
    }
  }

   void test() {
    static uint8_t step = 0;
    static double AXAct = 0;
    static double AYAct = 0;
    static double AZAct = 0;

    static const double steps[][3] = {{0.0, 0.0, 0.0},

                                  {-60.0, 40.0, 20.0}, 
                                  {-60.0, 40.0, -20.0},
                                  {60.0, 40.0, -20.0}, 
                                  {60.0, 60.0, 20.0},

                                  {-60.0, 30.0, 20.0}, 
                                  {-60.0, 30.0, -20.0},
                                  {60.0, 30.0, -20.0}, 
                                  {60.0, 50.0, 20.0},

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

                                  {0.0, 0.0, 0.0}}; // line 62
    //const int lastStep = 62;
    const int lastStep = 1;
    bool stepComplete = true;

    //stepComplete &= leftRear.CartesianMove(AXAct, AYAct, AZAct);
    //stepComplete &= leftFront.CartesianMove(AXAct, AYAct, AZAct);
    stepComplete &= rightFront.CartesianMove(AXAct, AYAct, AZAct);
    //stepComplete &= rightRear.CartesianMove(AXAct, AYAct, AZAct);

    if (stepComplete) {
      if (step > lastStep) {
        step = 0;
        while (1)
          ;
        return;
      }
      AXAct = steps[step][0];
      AYAct = steps[step][1];
      AZAct = steps[step][2];
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
