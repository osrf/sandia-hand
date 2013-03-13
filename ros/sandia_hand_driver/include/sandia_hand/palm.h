#ifndef SANDIA_HAND_PALM_H
#define SANDIA_HAND_PALM_H

#include "sandia_hand/serial_message_processor.h"

namespace sandia_hand
{

class Palm : public SerialMessageProcessor
{
public:
  Palm(const uint8_t addr = 1);
  virtual ~Palm();
};

}

#endif
