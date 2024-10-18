
#include <Arduino.h>

#include "LegTest.hpp"

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

Joint L2J1(L2J1Pin,0,true);
Joint L2J2(L2J2Pin,0,true);
Joint L2J3(L2J3Pin,0,true);

Joint L3J1(L3J1Pin,0);
Joint L3J2(L3J2Pin,0);
Joint L3J3(L3J3Pin,0);

Joint L4J1(L4J1Pin, 0,true);
Joint L4J2(L4J2Pin, 0,true);
Joint L4J3(L4J3Pin, 0,true);

// Legs
Leg L1(1,L1J1, L1J2, L1J3);  // left rear
Leg L2(2,L2J1, L2J2, L2J3);  // left front
Leg L3(3,L3J1, L3J2, L3J3);  // right front
Leg L4(4,L4J1, L4J2, L4J3);  // right rear


LegTest legTest(L3);

void loop() {
  legTest.loop();
}

void setup() {

  Serial.begin(115200);
  Serial.println("[start]");

  legTest.setup();
  // while (1)
  //   ;
  legTest.stance();
  delay(2000);
}
