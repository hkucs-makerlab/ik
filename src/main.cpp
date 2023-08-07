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
#include <Arduino.h>
#ifdef ESP32
#include <ESP32Servo.h>
#endif

#ifdef AVR
#include <Servo.h>
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
    _Servo.attach(_Pin, 500, 2500);
    _Servo.write(angle + AngleOffset);
    prevTime = millis();
  }

  void Setup(int8_t Ao = 0) {
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
  Leg(Joint &coxa, Joint &fumer, Joint &tibia) : _LegAngle(0), doIK(true), _coxa(coxa), _fumer(fumer), _tibia(tibia) {
  }

  // THE OFFSETS ALLOW ALL THE LEGS TO 'REST'
  // AT THE SAME PLACE RELATIVE TO J1, THE ANGLE
  // ALLOWS THEM TO MOVE IN THE SAME XY PLANE
  void Setup(double Angle = 0, int8_t Ao = 0) {
    _LegAngle = Angle;
    _coxa.Setup(Ao);
    _fumer.Setup(Ao);
    _tibia.Setup(Ao);
    doIK = true;
  }

  bool CartesianMove(double X, double Y, double Z) {
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
      l &= _coxa.Update(90 - J1);
      l &= _fumer.Update(90 - J2);
      l &= _tibia.Update(J3 + J3_LegAngle);
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
  Joint &_coxa;
  Joint &_fumer;
  Joint &_tibia;
};

// ##### ##### ##### ##### ##### ##### ##### ##### ##### MAIN
// Servo pins
#ifdef ESP32

#define L1J1Pin 5
#define L1J2Pin 18
#define L1J3Pin 19

#define L2J1Pin 15
#define L2J2Pin 2
#define L2J3Pin 4

#define L3J1Pin 12
#define L3J2Pin 14
#define L3J3Pin 27

#define L4J1Pin 25
#define L4J2Pin 33
#define L4J3Pin 32
#endif  // ESP32

#ifdef AVR
#define L1J1Pin 5
#define L1J2Pin 13
#define L1J3Pin 12

#define L2J1Pin 10
#define L2J2Pin 11
#define L2J3Pin 7

#define L3J1Pin 9
#define L3J2Pin 8
#define L3J3Pin 6

#define L4J1Pin 4
#define L4J2Pin 3
#define L4J3Pin 2
#endif

// Joints
Joint L1J1(L1J1Pin);
Joint L1J2(L1J2Pin);
Joint L1J3(L1J3Pin);

Joint L2J1(L2J1Pin, true);
Joint L2J2(L2J2Pin, true);
Joint L2J3(L2J3Pin, true);

Joint L3J1(L3J1Pin);
Joint L3J2(L3J2Pin);
Joint L3J3(L3J3Pin);

Joint L4J1(L4J1Pin, true);
Joint L4J2(L4J2Pin, true);
Joint L4J3(L4J3Pin, true);

// Legs
Leg L1(L1J1, L1J2, L1J3);  // left rear
Leg L2(L2J1, L2J2, L2J3);  // left front
Leg L3(L3J1, L3J2, L3J3);  // right front
Leg L4(L4J1, L4J2, L4J3);  // right rear

// Joint Variables
double AXAct = 0.0;
double AYAct = 0.0;
double AZAct = 0.0;

uint8_t commandStep = 0;

void loop() {
  const int lastLine = 61;
  bool stepComplete = true;

  // stepComplete &= L1.CartesianMove(AXAct, AYAct, AZAct);
  // stepComplete &= L2.CartesianMove(AXAct, AYAct, AZAct);
  stepComplete &= L3.CartesianMove(AXAct, AYAct, AZAct);
  // stepComplete &= L4.CartesianMove(AXAct, AYAct, AZAct);

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
  L1.Setup();
  L2.Setup();
  L3.Setup();
  L4.Setup();
  // while (1)
  //   ;

  // Stand Up
  bool l = false;
  while (!l) {
    l = true;
    l &= L1.CartesianMove(0, 0, 0);
    l &= L2.CartesianMove(0, 0, 0);
    l &= L3.CartesianMove(0, 0, 0);
    l &= L4.CartesianMove(0, 0, 0);
  }
  delay(2000);
#ifdef DEBUG
  Serial.println("[start]");
#endif
  // while (1)
  //   ;
}
