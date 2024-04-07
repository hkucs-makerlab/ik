#pragma once

// Constants
// const double J1L = 33.0;
// const double J2L = 46.0;
// const double J3L = 82.0;

// const double Y_Rest = 40.0;
// const double Z_Rest = -48.0;

// const double J3_LegAngle = 10;

const double J1L = 30.0;
const double J2L = 47.0;
const double J3L = 106;

const double Y_Rest = 58; 
const double Z_Rest = -65;

const double J3_LegAngle = 0; 

class Leg {
 public:
  // CTOR
  Leg(Joint &coxa, Joint &fumer, Joint &tibia) : _LegAngle(0), doIK(true), _coxa(coxa), _fumer(fumer), _tibia(tibia) {
  }

  // THE OFFSETS ALLOW ALL THE LEGS TO 'REST'
  // AT THE SAME PLACE RELATIVE TO J1, THE ANGLE
  // ALLOWS THEM TO MOVE IN THE SAME XY PLANE
  void Setup() {
    _coxa.attach();
    _fumer.attach();
    _tibia.attach();
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
      double H = sqrt((Y * Y) + (X * X))+J1L;
      double L = sqrt((H * H) + (Z * Z));
      J3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
      double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
      double A = atan(Z / H) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
      J2 = (B + A);                         // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'
            if (isnan(J1)) {
        Serial.println("invalid J1");
        return true;
      }
      if (isnan(J2)) {
        Serial.println("invalid J2");
        return true;
      }
      if (isnan(J3)) {
        Serial.println("invalid J3");
        return true;
      }    
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
