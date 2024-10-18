
#pragma once

#include "Joint.hpp"

class DataTypes {
 public:
  struct Vector {
    double x;
    double y;
    double z;
  };

  struct Rotator {
    float yaw;
    float pitch;
    float roll;
  };
};

// CO-ORDINATE SYSTEM
//          x
//          ^
//          |
//          |
// y <------o
// +ve z = OUT OF SCREEN
class Leg {
  int _id;
  Joint &_coxa, &_fumer, &_tibia;
  bool _doIK;
  double _j1, _j2, _j3;

 public:
  Leg(int id, Joint &j1, Joint &j2, Joint &j3)
      : _id(id), _coxa(j1), _fumer(j2), _tibia(j3), _doIK(true) {}

  void setup() {
    _coxa.setup();
    _fumer.setup();
    _tibia.setup();
  }

  void setAngleStep(int s) { 
    _coxa.setAngleInterval(s);
    _fumer.setAngleInterval(s);
    _tibia.setAngleInterval(s);
  }

  void setAngleInterval(uint16_t angleInterval) {
    _coxa.setAngleInterval(angleInterval);
    _fumer.setAngleInterval(angleInterval);
    _tibia.setAngleInterval(angleInterval);
  }

  int getID() {
    return _id;
  }
  
  void move(DataTypes::Vector pos) {
    bool l = true;
    do {
      l =cartesianMove(pos);
    } while (l ==false);
  }

  void toggleInvert() {
    _coxa.toggleInvert();
    _tibia.toggleInvert();
    _fumer.toggleInvert();
  }

  bool cartesianMove(DataTypes::Vector pos) {
    return cartesianMove(pos.x, pos.y, pos.z);
  }

  bool cartesianMove(double X, double Y, double Z) {
    const double J1L = 30.91;
    const double J2L = 47.0;
    const double J3L = 105.58;

    const double Y_Rest = 65.85 + J1L;
    const double Z_Rest = -67.18;

    const double J3_LegAngle = 27;

    if (_doIK) {
      // Apply YZ Offsets
      Y += Y_Rest;
      Z += Z_Rest;
      if (Y == 0) {
        Console.println("1 - Y is 0");
        return true;
      }      
      // CALCULATE INVERSE KINEMATIC SOLUTION
      _j1 = degrees(atan(X / Y));
      double H = sqrt(sq(Y) + sq(X)) - J1L;
      //
      double L = sqrt(sq(H) + sq(Z));
      double T = (sq(J2L) + sq(J3L) - sq(L)) / (2 * J2L * J3L);
      if (T > 1 || T < -1 ) {
         Console.println("2 - invalid T");
      }      
      T = constrain(T,-0.9,0.9);
      _j3 = degrees(acos(T));
      //
      T = (sq(L) + sq(J2L) - sq(J3L)) / (2 * L * J2L);
      if (T > 1 || T < -1 ) {
         Console.println("3 - invalid T");
      }
      T = constrain(T,-0.9,0.9);
      double B = acos(T);
      double A = atan(Z / H);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
      _j2 = (degrees(B) + degrees(A));  // BECAUSE 'A' IS NEGATIVE AT REST WE
                                        // NEED TO INVERT '-' TO '+'

      // if (1) {
      //   uint8_t flags = 0;

      //   flags |= isnan(_j1) ? 0x01 : 0x00;
      //   flags |= isnan(_j2) ? 0x02 : 0x00;
      //   flags |= isnan(_j3) ? 0x04 : 0x00;

      //   String msg = "";
      //   if (flags & 0x01)
      //     msg += "invalid J1, ";
      //   else
      //     msg += "coxa: " + String(90 - int(_j1));
      //   if (flags & 0x02) {
      //     msg += " invalid J2, ";
      //     Serial.println(T);
      //   } else
      //     msg += ", tibia: " + String(90 - int(_j2));
      //   if (flags & 0x04)
      //     msg += "invalid J3, ";
      //   else
      //     msg += ", femur: " + String(180-(int(_j3)+J3_LegAngle));

      //   String t = String(X) + ", " + String(Y) + ", " + String(Z);
      //   Serial.println(t);
      //   if (flags) {
      //     Serial.println(msg);              
      //   }

      //   if (flags) {
      //     return true;
      //   }
      // }
      
      _doIK = false;
    }

    bool l = true;
    if (!_doIK) {
      l &= _coxa.interpolateMove(90 - _j1);
      l &= _fumer.interpolateMove(90 - _j2);
      l &= _tibia.interpolateMove(_j3 + J3_LegAngle);
      if (l) {
        _doIK = true;
      }
    }

    return l;
  }
    bool operator==(Leg &leg) { return this == &leg; }
};