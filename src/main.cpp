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
// L3-------|-------L4
// L2-------|-------L5
// L1-------|-------L6
//
// https://youtu.be/HjmIOKSp7v4
//
#include <Arduino.h>
#ifdef ESP32
#include <ESP32Servo.h>
#endif

// Debug
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTL(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTL(x)
#define DEBUG_PRINT(x)
#endif

// Constants
const double J1L = 33.0;
const double J2L = 46.0;
const double J3L = 82.0;

const double Y_Rest = 40.0;
const double Z_Rest = -48.0;

const double J3_LegAngle = 10;
const double lines[][3] = {{0.0, 0.0, 0.0},

                           {-30.0, 40.0, 20.0},
                           {-30.0, 40.0, -20.0},
                           {60.0, 40.0, -20.0},
                           {60.0, 60.0, 20.0},

                           {-30.0, 30.0, 20.0},
                           {-30.0, 30.0, -20.0},
                           {60.0, 30.0, -20.0},
                           {60.0, 50.0, 20.0},

                           {-30.0, 20.0, 20.0},
                           {-30.0, 20.0, -20.0},
                           {60.0, 20.0, -20.0},
                           {60.0, 40.0, 20.0},

                           {-30.0, 10.0, 20.0},
                           {-30.0, 10.0, -20.0},
                           {60.0, 10.0, -20.0},
                           {60.0, 30.0, 20.0},

                           {-30.0, 0.0, 20.0},
                           {-30.0, 0.0, -20.0},
                           {60.0, 0.0, -20.0},
                           {60.0, 20.0, 20.0},

                           {-30.0, 40.0, 20.0},
                           {-30.0, 40.0, -20.0},
                           {-30.0, 0.0, -20.0},
                           {-30.0, 20.0, 20.0},

                           {-20.0, 40.0, 20.0},
                           {-20.0, 40.0, -20.0},
                           {-20.0, 0.0, -20.0},
                           {-20.0, 20.0, 20.0},

                           {-10.0, 40.0, 20.0},
                           {-10.0, 40.0, -20.0},
                           {-10.0, 0.0, -20.0},
                           {-10.0, 20.0, 20.0},

                           {0.0, 40.0, 20.0},
                           {0.0, 40.0, -20.0},
                           {0.0, 0.0, -20.0},
                           {0.0, 20.0, 20.0},

                           {10.0, 40.0, 20.0},
                           {10.0, 40.0, -20.0},
                           {10.0, 0.0, -20.0},
                           {10.0, 20.0, 20.0},

                           {20.0, 40.0, 0.0},
                           {20.0, 40.0, -20.0},
                           {20.0, 0.0, -20.0},
                           {20.0, 20.0, 0.0},

                           {30.0, 40.0, 0.0},
                           {30.0, 40.0, -20.0},
                           {30.0, 0.0, -20.0},
                           {30.0, 20.0, 0.0},

                           {40.0, 40.0, 0.0},
                           {40.0, 40.0, -20.0},
                           {40.0, 0.0, -20.0},
                           {40.0, 20.0, 0.0},

                           {50.0, 40.0, 0.0},
                           {50.0, 40.0, -20.0},
                           {50.0, 0.0, -20.0},
                           {50.0, 20.0, 0.0},

                           {60.0, 40.0, 0.0},
                           {60.0, 40.0, -20.0},
                           {60.0, 0.0, -20.0},
                           {60.0, 20.0, 0.0},

                           {0.0, 0.0, 0.0}};

// ##### ##### ##### ##### ##### ##### ##### ##### ##### JOINT CLASS
class Joint {
 private:
  int angle;
  unsigned long prevTime;
  bool Inverted;
  int8_t AngleOffset;
  Servo _Servo;
  uint8_t _Pin;

 public:
  // CTOR
  Joint() : angle(90), Inverted(false), AngleOffset(0) {
  }

  // Methods
  void Setup(uint8_t Pin, bool I = false, int8_t Ao = 0) {
    _Pin = Pin;
    Inverted = I;
    AngleOffset = Ao;
    _Servo.attach(_Pin, 500, 2500);
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

// ##### ##### ##### ##### ##### ##### ##### ##### ##### LEG CLASS
class Leg {
 public:
  // CTOR
  Leg() : doIK(true), _LegAngle(0) {
  }

  // THE OFFSETS ALLOW ALL THE LEGS TO 'REST'
  // AT THE SAME PLACE RELATIVE TO J1, THE ANGLE
  // ALLOWS THEM TO MOVE IN THE SAME XY PLANE
  void Setup(double Angle = 0) {
    _LegAngle = Angle;
    doIK = true;
  }

  bool CartesianMove(double X, double Y, double Z, Joint *Joint1, Joint *Joint2, Joint *Joint3) {
    if (doIK) {
      // Apply XYZ Offsets
      Y += Y_Rest;
      Z += Z_Rest;

      // Pivot by angle
      if (_LegAngle != 0) {
        double aR = _LegAngle / (180 / PI);
        double xn = ((X * cos(aR)) - (Y * sin(aR)));
        double yn = ((Y * cos(aR)) + (X * sin(aR)));

        X = xn;
        Y = yn;
      }
      // CALCULATE INVERSE KINEMATIC SOLUTION
      J1 = atan(X / Y) * (180 / PI);
      double H = sqrt((Y * Y) + (X * X));
      double L = sqrt((H * H) + (Z * Z));
      J3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
      double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
      double A = atan(Z / H) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
      J2 = (B + A);                         // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'
      doIK = false;
    }
    //
    bool l = true;
    if (!doIK) {
      l &= Joint1->Update(90 - J1);
      l &= Joint2->Update(90 - J2);
      l &= Joint3->Update(J3 + J3_LegAngle);
      if (l) doIK = true;
    }

    return l;
  }

 private:
  // Variables
  double _LegAngle;
  bool doIK;
  double J1;
  double J2;
  double J3;
};

// ##### ##### ##### ##### ##### ##### ##### ##### ##### MAIN
// Servo pins
#ifdef ESP32
#define L1J1Pin 23
#define L1J2Pin 22
#define L1J3Pin L3J3Pin

#define L2J1Pin 19
#define L2J2Pin 18
#define L2J3Pin 5

#define L3J1Pin 17
#define L3J2Pin 16
#define L3J3Pin 21

#define L4J1Pin 13
#define L4J2Pin 12
#define L4J3Pin 14

#define L5J1Pin 27
#define L5J2Pin 26
#define L5J3Pin 25

#define L6J1Pin 4
#define L6J2Pin 2
#define L6J3Pin L4J3Pin


// Legs
Leg L1;
Leg L2;
Leg L3;
Leg L4;
Leg L5;
Leg L6;

// Joints
Joint L1J1;
Joint L1J2;

Joint L2J1;
Joint L2J2;
Joint L2J3;

Joint L3J1;
Joint L3J2;
Joint L3J3;

Joint L4J1;
Joint L4J2;
Joint L4J3;

Joint L5J1;
Joint L5J2;
Joint L5J3;

Joint L6J1;
Joint L6J2;
#endif  // ESP32

// Joint Variables
double AXAct = 0.0;
double AYAct = 0.0;
double AZAct = 0.0;

uint8_t commandStep = 0;

void loop() {
  const int lastLine = 61;
  bool stepComplete = true;

  // stepComplete &= L1.CartesianMove(AXAct, AYAct, AZAct, &L1J1, &L1J2, &L4J3);
  stepComplete &= L2.CartesianMove(AXAct, AYAct, AZAct, &L2J1, &L2J2, &L2J3);
  // stepComplete &= L3.CartesianMove(AXAct, AYAct, AZAct, &L3J1, &L3J2, &L3J3);
  // stepComplete &= L4.CartesianMove(AXAct, AYAct, AZAct, &L4J1, &L4J2, &L4J3);
  stepComplete &= L5.CartesianMove(AXAct, AYAct, AZAct, &L5J1, &L5J2, &L5J3);
  // stepComplete &= L6.CartesianMove(AXAct, AYAct, AZAct, &L6J1, &L6J2, &L3J3);

  if (stepComplete) {
    commandStep++;
    if (commandStep > lastLine) {
      commandStep = 0;
      while (1)
        ;
      return;
    }
    AXAct = lines[commandStep][0];
    AYAct = lines[commandStep][1];
    AZAct = lines[commandStep][2];
    // DEBUG_PRINT("xMove: ");
    // DEBUG_PRINTL(AXAct);
    // DEBUG_PRINT("yMove: ");
    // DEBUG_PRINTL(AYAct);
    // DEBUG_PRINT("zMove: ");
    // DEBUG_PRINTL(AZAct);
    // DEBUG_PRINTL("");
  }
}

void setup() {
// Debug
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // THE FIRST VALUE IS THE PIN
  // THE SECOND VALUE IS TO INVERT THE JOINT
  // THE LAST VALUE IS AN OFFSET TO ACCOUNT FOR ASSEMBLY DIFFERENCES AND OFF CENTRE WEIGHT,
  // THIS SHOULDN'T BE NEEDED FOR STRONG SERVOS WITH ACCURATE CONSTRUCTION.

  // all servos at 90 degree after setup
  L1J1.Setup(L1J1Pin);
  L1J2.Setup(L1J2Pin);

  L2J1.Setup(L2J1Pin, true);
  L2J2.Setup(L2J2Pin, true);
  L2J3.Setup(L2J3Pin, true);

  L3J1.Setup(L3J1Pin, true);
  L3J2.Setup(L3J2Pin, true);
  L3J3.Setup(L3J3Pin, true);

  L4J1.Setup(L4J1Pin);
  L4J2.Setup(L4J2Pin);
  L4J3.Setup(L4J3Pin);

  L5J1.Setup(L5J1Pin);
  L5J2.Setup(L5J2Pin);
  L5J3.Setup(L5J3Pin);

  L6J1.Setup(L6J1Pin, true);
  L6J2.Setup(L6J2Pin, true);
  // while (1)
  //   ;

  // L1.Setup();
  // L2.Setup();
  // L3.Setup();
  // L4.Setup();
  // L5.Setup();
  // L6.Setup();

  // Stand Up
  bool l = false;
  while (!l) {
    l = true;
    l &= L1.CartesianMove(0, 0, 0, &L1J1, &L1J2, &L4J3);
    l &= L2.CartesianMove(0, 0, 0, &L2J1, &L2J2, &L2J3);
    l &= L3.CartesianMove(0, 0, 0, &L3J1, &L3J2, &L3J3);
    l &= L4.CartesianMove(0, 0, 0, &L4J1, &L4J2, &L4J3);
    l &= L5.CartesianMove(0, 0, 0, &L5J1, &L5J2, &L5J3);
    l &= L6.CartesianMove(0, 0, 0, &L6J1, &L6J2, &L3J3);
  }
  delay(2000);
#ifdef DEBUG
  Serial.println("[start]");
#endif
  // while (1)
  //   ;
}
