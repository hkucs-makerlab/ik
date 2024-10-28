#include <Arduino.h>
#include "pwm.hpp"
#include <avr/interrupt.h>
#define YES 1
#define NO 0


static byte Jitter;
static byte Jitter2;
static byte Jitter3;
static byte Jitter4;
static byte RealTime5s;
static unsigned int iCount;
static volatile uint8_t *OutPortTable[20] = {
    &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTB,
    &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTC, &PORTC,
    &PORTC, &PORTC, &PORTC, &PORTC, &PORTD, &PORTD};
static uint8_t OutBitTable[20] = {4,  8,  16, 32, 64, 128, 1,  2,  4, 8,
                                  16, 32, 1,  2,  4,  8,   16, 32, 1, 2};
static unsigned int ServoPW[20] = {
    24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000,
    24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000, 24000};
static byte ServoInvert[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static byte Timer2Toggle;
static volatile uint8_t *OutPort1A = &PORTD;
static volatile uint8_t *OutPort1B = &PORTB;
static uint8_t OutBit1A = 4;
static uint8_t OutBit1B = 16;
static volatile uint8_t *OutPortNext1A = &PORTD;
static volatile uint8_t *OutPortNext1B = &PORTB;
static uint8_t OutBitNext1A = 4;
static uint8_t OutBitNext1B = 16;

static long ServoStepsHD[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static long ServoLastPos[20] = {24000, 24000, 24000, 24000, 24000, 24000, 24000,
                                24000, 24000, 24000, 24000, 24000, 24000, 24000,
                                24000, 24000, 24000, 24000, 24000, 24000};
static long StepsToGo[20] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                             -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
static int ChannelCount;

static long ServoGroupStepsToGo = 0;
static long ServoGroupServoLastPos[20];
static int ServoGroupChannel[20];
static int ServoGroupNbOfChannels = 0;

static char SerialIn;
static int SerialCommand = 0;  // 0= none, 1 = '#' and so on...
static long SerialNumbers[10];
static int SerialNumbersLength = 0;
static boolean FirstSerialChannelAfterCR = 1;

static int SerialChannel = 0;
static long SerialPulseHD = 24000;
static long SerialPulseOffsetHD[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static long SerialSpeedHD = 0;
static long SerialTime = 0;
static long SerialNegative = 1;
static boolean SerialNeedToMove = 0;
//static char SerialCharToSend[50] = "]ko[";
//static int SerialNbOfCharToSend = 0;  // 0= none, 1 = [0], 2 = [1] and so on...

void ServoGroupMove(int Channel, long PulseHD, long SpeedHD,
                    long Time);  // ServoMove used by serial command interpreter
void ServoGroupMoveActivate();   // ServoMove used by serial command interpreter
void RealTime50Hz();  // Move servos every 20ms to the desired position.

long ConvertSerialNumbers();
long CheckRange(long PulseHDValue);
void ServoMove(int Channel, long PulseHD, long SpeedHD, long Time);

void ServoMoveAngle(int Channel, int angle,  long Time) {
     long pw = angle * 177.77 + 8000;
     ServoMove(Channel,pw,0,Time);

}

void ServoMove(int Channel, long PulseHD, long SpeedHD, long Time) {
  // Use ServoMove(int Channel, long PulseHD, long SpeedHD, long Time) to
  // control servos. One of th SpeedHD or Time can be set to 0 to only  use the
  // other one for speed. If both are used, the one that takes the longest time,
  // will be used
  ServoGroupMove(Channel, constrain(PulseHD,8320,39680), SpeedHD, Time);
  ServoGroupMoveActivate();
}

byte getRealTime5s() { return RealTime5s; }

void CheckSerial()  // Serial command interpreter.
{
  if (Serial.available() > 0) {
    SerialIn = Serial.read();
    if (SerialIn == '#') {
      SerialCommand = 1;
      SerialNeedToMove = 1;
      if (!FirstSerialChannelAfterCR)
        ServoGroupMove(SerialChannel,constrain(SerialPulseHD,8320,39680), SerialSpeedHD, SerialTime);
      FirstSerialChannelAfterCR = 0;
    }
    if (SerialIn == 'P') {
        SerialCommand = 5;  // 'P'
    }

    if (SerialIn == 'T') SerialCommand = 4;

    //
    if (SerialIn == ' ' || SerialIn == 13) {
      if (SerialCommand == 1) { // #
        SerialChannel = constrain(ConvertSerialNumbers(),0,HDServoMode - 1);
        SerialCommand = 0;
      }
      if (SerialCommand == 5) { // P
        SerialPulseHD =
            ConvertSerialNumbers() * 16 + SerialPulseOffsetHD[SerialChannel];
        SerialCommand = 0;
      }      
      if (SerialCommand == 4) { // T
        SerialTime = ConvertSerialNumbers();
        SerialCommand = 0;
      }
      if (SerialIn == 13) {
        if (SerialNeedToMove) {
          ServoGroupMove(SerialChannel, constrain(SerialPulseHD,8320,39680), SerialSpeedHD, SerialTime);
          ServoGroupMoveActivate();
          FirstSerialChannelAfterCR = 1;
          SerialCommand = 0;
          SerialSpeedHD = 0;
          SerialTime = 0;
          SerialNeedToMove = 0;
        }
      }
    }
    //
    if ((SerialIn >= '0') && (SerialIn <= '9')) {
      SerialNumbers[SerialNumbersLength] = SerialIn - '0';
      SerialNumbersLength++;
    }
    //
    if (SerialIn == '-') SerialNegative = -1;
  }
}

long ConvertSerialNumbers()  // Converts numbers gotten from serial line to
                             // long.
{
  int i = 0;

  long ReturnValue = 0;
  long Multiplier = 1;
  if (SerialNumbersLength > 0) {
    for (i = SerialNumbersLength - 1; i >= 0; i--) {
      ReturnValue += SerialNumbers[i] * Multiplier;
      Multiplier *= 10;
    }
    ReturnValue *= SerialNegative;
    SerialNumbersLength = 0;
    SerialNegative = 1;
    return ReturnValue;
  } else
    return 0;
}

void ServoGroupMove(int Channel, long PulseHD, long SpeedHD, long Time)  // ServoMove used by serial command interpreter
{
  long StepsToGoSpeed = 0;
  long StepsToGoTime = 0;

  ServoGroupChannel[ServoGroupNbOfChannels] = Channel;
  if (SpeedHD < 1) SpeedHD = 3200000;
  StepsToGoSpeed = abs((PulseHD - ServoPW[Channel]) / (SpeedHD / 50));
  StepsToGoTime = Time / 20;
  if (StepsToGoSpeed > ServoGroupStepsToGo)
    ServoGroupStepsToGo = StepsToGoSpeed;
  if (StepsToGoTime > ServoGroupStepsToGo) ServoGroupStepsToGo = StepsToGoTime;
  ServoGroupChannel[ServoGroupNbOfChannels] = Channel;
  ServoGroupServoLastPos[ServoGroupNbOfChannels] = PulseHD;
  ServoGroupNbOfChannels++;
}

void ServoGroupMoveActivate()  // ServoMove used by serial command interpreter
{
  int ServoCount = 0;

  for (ServoCount = 0; ServoCount < ServoGroupNbOfChannels; ServoCount++) {
    ServoStepsHD[ServoGroupChannel[ServoCount]] = (ServoGroupServoLastPos[ServoCount] - ServoPW[ServoGroupChannel[ServoCount]]) / ServoGroupStepsToGo;
    StepsToGo[ServoGroupChannel[ServoCount]] = ServoGroupStepsToGo;
    ServoLastPos[ServoGroupChannel[ServoCount]] =  ServoGroupServoLastPos[ServoCount];
  }
  ServoGroupNbOfChannels = 0;
  ServoGroupStepsToGo = 0;
}

void RealTime50Hz()  // Move servos every 20ms to the desired position.
{

  // if (SerialNbOfCharToSend) {
  //   SerialNbOfCharToSend--;
  //   Serial.print(SerialCharToSend[SerialNbOfCharToSend]);
  // }
  
  RealTime5s++;
  for (ChannelCount = 0; ChannelCount < 20; ChannelCount++) {
    if (StepsToGo[ChannelCount] > 0) {
      ServoPW[ChannelCount] += ServoStepsHD[ChannelCount];
      StepsToGo[ChannelCount]--;
    } else if (StepsToGo[ChannelCount] == 0) {
      ServoPW[ChannelCount] = ServoLastPos[ChannelCount];
      StepsToGo[ChannelCount]--;
    }
  }
}

ISR(TIMER1_COMPA_vect)  // Interrupt routine for timer 1 compare A. Used for
                        // timing each pulse width for the servo PWM.
{
#if UseJitterCompensation == YES
  Jitter = TCNT1 - OCR1A;
  if (Jitter == 32) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter == 31) {
    asm volatile("nop\n\tnop\n\tnop\n\t");
  }
  if (Jitter == 30) {
    asm volatile("nop\n\t");
  }
  if (Jitter == 29) {
    asm volatile("nop\n\t");
  }
#endif
  *OutPort1A &= ~OutBit1A;  // Pulse A finished. Set to low
}

ISR(TIMER1_COMPB_vect)  // Interrupt routine for timer 1 compare A. Used for
                        // timing each pulse width for the servo PWM.
{
#if UseJitterCompensation == YES
  Jitter2 = TCNT1 - OCR1B;
  if (Jitter2 == 32) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter2 == 31) {
    asm volatile("nop\n\tnop\n\tnop\n\t");
  }
  if (Jitter2 == 30) {
    asm volatile("nop\n\t");
  }
  if (Jitter2 == 29) {
    asm volatile("nop\n\t");
  }
#endif
  *OutPort1B &= ~OutBit1B;  // Pulse B finished. Set to low
}

ISR(TIMER2_COMPA_vect)  // Interrupt routine for timer 2 compare A. Used for
                        // timing 50Hz for each servo.
{
#if UseJitterCompensation == YES
  Jitter4 = TCNT1L - 100;
  if (Jitter4 == 118) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter4 == 117) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter4 == 116) {
    asm volatile("nop\n\tnop\n\tnop\n\t");
  }
  if (Jitter4 == 115) {
    asm volatile("nop\n\t");
  }
  if (Jitter4 == 114) {
    asm volatile("nop\n\t");
  }
#endif
  *OutPortNext1A |=
      OutBitNext1A;  // Start new pulse on next servo. Write pin HIGH
  *OutPortNext1B |=
      OutBitNext1B;  // Start new pulse on next servo. Write pin HIGH
}

ISR(TIMER2_COMPB_vect)  // Interrupt routine for timer 2 compare A. Used for
                        // timing 50Hz for each servo.
{
  TIFR1 = 255;  // Clear  pending interrupts
#if UseJitterCompensation == YES
  Jitter3 = TCNT1L - 100;
  if (Jitter3 == 137) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter3 == 136) {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
  if (Jitter3 == 135) {
    asm volatile("nop\n\tnop\n\tnop\n\t");
  }
  if (Jitter3 == 134) {
    asm volatile("nop\n\t");
  }
  if (Jitter3 == 133) {
    asm volatile("nop\n\t");
  }
#endif
  TCNT1 = 0;  // Restart counter for timer1
  TCNT2 = 0;  // Restart counter for timer2
  sei();
  *OutPort1A &= ~OutBit1A;  // Set pulse low to if not done already
  *OutPort1B &= ~OutBit1B;  // Set pulse low to if not done already
  OutPort1A = OutPortTable[Timer2Toggle];       // Temp port for COMP1A
  OutBit1A = OutBitTable[Timer2Toggle];         // Temp bitmask for COMP1A
  OutPort1B = OutPortTable[Timer2Toggle + 10];  // Temp port for COMP1B
  OutBit1B = OutBitTable[Timer2Toggle + 10];    // Temp bitmask for COMP1B
  if (ServoInvert[Timer2Toggle])
    OCR1A = 48000 - ServoPW[Timer2Toggle] -
            8025;  // Set timer1 count for pulse width.
  else
    OCR1A = ServoPW[Timer2Toggle] - 8020;
  if (ServoInvert[Timer2Toggle + 10])
    OCR1B = 48000 - ServoPW[Timer2Toggle + 10] -
            8020;  // Set timer1 count for pulse width.
  else
    OCR1B = ServoPW[Timer2Toggle + 10] - 8015;
  Timer2Toggle++;  // Next servo in line.
  if (Timer2Toggle == 10) {
    Timer2Toggle = 0;  // If next servo is grater than 9, start on 0 again.
    RealTime50Hz();    // Do servo management
  }
  OutPortNext1A = OutPortTable[Timer2Toggle];  // Next Temp port for COMP1A
  OutBitNext1A = OutBitTable[Timer2Toggle];    // Next Temp bitmask for COMP1A
  OutPortNext1B = OutPortTable[Timer2Toggle + 10];  // Next Temp port for COMP1B
  OutBitNext1B =
      OutBitTable[Timer2Toggle + 10];  // Next Temp bitmask for COMP1B
}

void ServoSetup(bool flag) {
  if (flag) {
    TIMSK0 = 0;   //TIMSK0 = 0;   // Disable timer 0. This can reduse jitter some more. But it's
                // used for millis() & delay() funtions. This will disable them!
  }
  // Timer 1 setup(16 bit):
  TCCR1A = 0;   // Normal counting mode
  TCCR1B = 1;   // Set prescaler to 1
  TCNT1 = 0;    // Clear timer count
  TIFR1 = 255;  // Clear  pending interrupts
  TIMSK1 = 6;   // Enable the output compare A and B interrupt
  // Timer 2 setup(8 bit):
  TCCR2A = 0;   // Normal counting mode
  TCCR2B = 6;   // Set prescaler to 256
  TCNT2 = 0;    // Clear timer countSerialNbOfCharToSend = strlen(SerialCharToSend);
  TIFR2 = 255;  // Clear pending interrupts
  TIMSK2 = 6;   // Enable the output compare A and B interrupt
  OCR2A = 93;   // Set counter A for about 500us before counter B below;
  OCR2B = 124;  // Set counter B for about 2000us (20ms/10, where 20ms is 50Hz);

#if HDServoMode == 18
  for (iCount = 2; iCount < 14; iCount++)
    pinMode(iCount, OUTPUT);  // Set all pins used to output:
  OutPortTable[18] = &PORTC;  // In 18 channel mode set channel 18 and 19 to a
                              // dummy pin that does not exist.
  OutPortTable[19] = &PORTC;
  OutBitTable[18] = 128;
  OutBitTable[19] = 128;

  // SerialNbOfCharToSend = strlen(SerialCharToSend);
#elif HDServoMode == 20
  for (iCount = 0; iCount < 14; iCount++)
    pinMode(iCount, OUTPUT);  // Set all pins used to output:
#endif
  DDRC = 63;  // Set analog pins A0 - A5 as digital output also.
}
