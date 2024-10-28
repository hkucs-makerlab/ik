#pragma once

//////////////////////////////////////////////////////////////////////////////////////////////////////
// UNO_20_Servos_Controller.ino - High definition (15 bit), low jitter, 20 servo software for Atmega328P and Arduino UNO. Version 1.2
//
// Jitter is typically 2 ns (0.3% of resolution) with jitter compensation ON, and 400 ns (640% of resolution) 
// with Jitter compensation OFF. 
// 32000 steps resolution for 0-180 degrees (15 bit resolution).
// In 18 servos mode it can receive serial servo-move commands.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                              !!!!!!!!!!!!!!!!!!!
// Copyright (c) 2013 Arvid Mortensen.  All right reserved. 
// http://www.lamja.com
// 
// This software is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                              !!!!!!!!!!!!!!!!!!!
// This software will only work with ATMEGA328P (Arduino UNO and compatible. Or that is what I have tested it with anyways....  !!!!
//                              !!!!!!!!!!!!!!!!!!!
// How it works:
// 18 or 20 pins are used for the servos. These pins are all pre configured.
// All 18/20 servos are always active and updated. To change the servo position, change the
// values in the ServoPW[] array, use the serial commands (int 18 servos mode only) or use 
// ServoMove() function. The range of the ServoPW is 8320 to 39680. 8320=520us. 39680=2480us.
//
// Some formulas:
// micro second = ServoPW value / 16
// angle = (ServoPW value - 8000) / 177.77  (or there about)
// ServoPW value = angle * 177.77 + 8000
// ServoPW value = micro second * 16
//
// Channels are locked to these pins:
// Ch0=Pin2, Ch1=Pin3, Ch2=Pin4, Ch3=Pin5, Ch4=Pin6, Ch5=Pin7, Ch6=Pin8, Ch7=Pin9, Ch8=Pin10, Ch9=Pin11
// Ch10=Pin12, Ch11=Pin13, Ch12=PinA0, Ch13=PinA1, Ch14=PinA2, Ch15=PinA3, Ch16=PinA4, Ch17=PinA5, Ch18=Pin0, Ch19=Pin1
//
// Serial commands:
// # = Servo channel
// P = Pulse width in 1 us
// T = Time in ms   
// <cr> = Carrage return. ASCII value 13. Used to end command.
//
// Examples:
// #0 P1500 T1000<cr>                        - Move Servo 0 to 1500us in 1 second.
// 18 or 20 channels mode:
// #define HDServoMode 18            - This will set 18 channels mode so you can use serial in and out. Serial command interpreter is activated.
// #define HDServoMode 20            - This will set 20 channels mode, and you can not use serial. 
//                                     A demo will run in the loop() routine . Serial command interpreter is not active.
//                                     use ServoMove(int Channel, long PulseHD, long SpeedHD, long Time) to control servos.
//                                     one of SpeedHD or Time can be set to 0 to just use the other one for speed. If both are used,
//                                     the one that takes the longest time will be used. You can also change the values in the 
//                                     ServoPW[] array directly, but take care not to go under/over 8320/39680.
// #define UseJitterCompensation NO  - No jitter compensation. Jitter will be about 400 ns.
// #define UseJitterCompensation YES - Compensating for jitter in the timer interrupt routines. Jitter will be about 2 ns.
//                                     With jitter compensating ON, there wil be a wider gap for PWM duty cycle for channel n+10 if they
//                                     are almost the same. 3.5 us with no jitter compensating, and 5.5 us with jitter compensating.
//                                     To be ensured all servos are jitter free and no gap, set servo channel 10-19 at max PW,
//                                     and then use only channel 0-9. Then you have a 10 channel rock steady servocontroller
//                                     with practically NO jitter. Resolution of 1/16 us PW also works best with jitter compensating.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

#define UseJitterCompensation YES

#define HDServoMode 18
//#define HDServoMode 20

#if HDServoMode == 18  // Serial command interpreter is acive. 18-servos mode.
void CheckSerial(); // Serial command interpreter.
#endif
//
byte getRealTime5s();
void ServoSetup(bool flag=false);
void ServoMoveAngle(int Channel, int angle,  long Time=10);