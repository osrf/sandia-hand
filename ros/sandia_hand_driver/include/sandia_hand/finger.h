#ifndef SANDIA_HAND_FINGER_H
#define SANDIA_HAND_FINGER_H

#include "sandia_hand/motor_module.h"
#include "sandia_hand/proximal_phalange.h"
#include "sandia_hand/distal_phalange.h"

namespace sandia_hand
{

class Finger
{
public:
  Finger();
  virtual ~Finger();

  MotorModule mm;
  ProximalPhalange pp;
  DistalPhalange dp;
};

}

#endif
