#include "sandia_hand/motor_module.h"
using namespace sandia_hand;

MotorModule::MotorModule(const uint8_t addr)
: SerialMessageProcessor(addr)
{
}

MotorModule::~MotorModule()
{
}

