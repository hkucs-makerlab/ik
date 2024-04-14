#ifndef __DATA_TYPES__
#define __DATA_TYPES__

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
#endif
