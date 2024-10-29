
#include <Arduino.h>

#include "LegTest.hpp"

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

#define SERVO_SETUP_PIN A0

// Joints
Joint L1J1(L1J1Pin,0,false);
Joint L1J2(L1J2Pin,0,false);
Joint L1J3(L1J3Pin,0,false);

Joint L2J1(L2J1Pin,0,true);
Joint L2J2(L2J2Pin,0,true);
Joint L2J3(L2J3Pin,0,true);

Joint L3J1(L3J1Pin,0,false);
Joint L3J2(L3J2Pin,0,false);
Joint L3J3(L3J3Pin,0,false);

Joint L4J1(L4J1Pin, 0,true);
Joint L4J2(L4J2Pin, 0,true);
Joint L4J3(L4J3Pin, 0,true);

// Legs
Leg L1(1,L1J1, L1J2, L1J3);  // left rear
Leg L2(2,L2J1, L2J2, L2J3);  // left front
Leg L3(3,L3J1, L3J2, L3J3);  // right front
Leg L4(4,L4J1, L4J2, L4J3);  // right rear

LegTest legTest(L1);

void loop() {
  legTest.swing();
  //legTest.move(0,0,0);

}

void setup() {

  Serial.begin(115200);
  legTest.setup();
  if (1) {
    pinMode(SERVO_SETUP_PIN, INPUT);
    while (digitalRead(SERVO_SETUP_PIN)) {
      Console.println("[calibrate]");
      delay(1000);
    }
  }
  legTest.stance();
  delay(2000);
  Serial.println("[started]");
}
