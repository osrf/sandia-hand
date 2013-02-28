#ifndef SANDIA_HAND_MOTOR_MODULE_H
#define SANDIA_HAND_MOTOR_MODULE_H

#include "sandia_hand/serial_message_processor.h"

namespace sandia_hand
{

class MotorModule : public SerialMessageProcessor
{
public:
  MotorModule(const uint8_t addr);
  virtual ~MotorModule();
};

}

#endif

