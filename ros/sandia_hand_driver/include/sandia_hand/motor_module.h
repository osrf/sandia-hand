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
  bool setPhalangeBusPower(bool on);
  bool setPhalangeAutopoll(bool on); // caveat... doesn't work yet.
private:
  static const uint8_t  PKT_PHALANGE_POWER    = 0x1e;
  static const uint8_t  PKT_PHALANGE_AUTOPOLL = 0x20; // broken
  static const uint8_t  PKT_FINGER_STATUS     = 0x21;
  void rxFingerStatus(const uint8_t *payload, const uint16_t payload_len);
};

}

#endif

