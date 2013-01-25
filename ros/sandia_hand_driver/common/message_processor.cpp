#include <cstdio>
#include "sandia_hand/message_processor.h"
using namespace sandia_hand;

MessageProcessor::MessageProcessor(const addr)
: addr_(addr)
{
}

MessageProcessor::~MessageProcessor()
{
}

bool MessageProcessor::ping()
{
  printf("MessageProcessor::ping()\n");
  return true;
}

