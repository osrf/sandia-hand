#include <cstdio>
#include "sandia_hand/motor_module.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

MotorModule::MotorModule(const uint8_t addr)
: SerialMessageProcessor(addr)
{
  registerRxHandler(PKT_FINGER_STATUS, 
                    boost::bind(&MotorModule::rxFingerStatus, this, _1, _2));
}

MotorModule::~MotorModule()
{
}

bool MotorModule::setPhalangeBusPower(bool on)
{
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_POWER, 1))
    return false;
  return listenFor(PKT_PHALANGE_POWER, 0.5);
}

bool MotorModule::setPhalangeAutopoll(bool on)
{
  // this doesn't do anything yet :(
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_AUTOPOLL, 1))
    return false;
  return listenFor(PKT_PHALANGE_AUTOPOLL, 0.5);
}

void MotorModule::rxFingerStatus(const uint8_t *payload, 
                                 const uint16_t payload_len)
{
  printf("rx finger status\n  ");
  for (int i = 0; i < payload_len; i++)
    printf("%02x ", payload[i]);
  printf("\n");
}

