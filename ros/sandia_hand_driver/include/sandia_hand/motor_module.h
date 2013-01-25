#ifndef SANDIA_HAND_MOTOR_MODULE_H
#define SANDIA_HAND_MOTOR_MODULE_H

#include "message_processor.h"

namespace sandia_hand
{

class MotorModule : public MessageProcessor
{
public:
  MotorModule(const uint8_t addr);
  virtual ~MotorModule();
};

}

#endif

