#include "sandia_hand/palm.h"
using namespace sandia_hand;

Palm::Palm(const uint8_t addr)
: SerialMessageProcessor(addr)
{
}

Palm::~Palm()
{
}

