#ifndef SANDIA_HAND_PALM_H
#define SANDIA_HAND_PALM_H

#include "sandia_hand/message_processor.h"

namespace sandia_hand
{

class Palm : public MessageProcessor
{
public:
  Palm(const uint8_t addr);
  virtual ~Palm();
};

}

#endif
