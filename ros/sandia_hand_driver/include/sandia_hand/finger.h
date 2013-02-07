#ifndef SANDIA_HAND_FINGER_H
#define SANDIA_HAND_FINGER_H

#include "sandia_hand/motor_module.h"

namespace sandia_hand
{

class Finger
{
public:
  Finger();
  virtual ~Finger();

  MotorModule mm;
};

}

#endif
