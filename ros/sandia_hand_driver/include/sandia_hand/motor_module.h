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
  bool setPhalangeAutopoll(bool on); 
  bool pollFingerStatus();
  bool phalangeTxRx(const uint8_t *data, const uint16_t data_len);
                    //const uint16_t timeout_ms);
  void addPhalangeRxFunctor(RxFunctor f);
  static const uint8_t PKT_FINGER_STATUS     = 0x21;
private:
  static const uint8_t PKT_PHALANGE_POWER    = 0x1e;
  static const uint8_t PKT_PHALANGE_TXRX     = 0x1f;
  static const uint8_t PKT_PHALANGE_AUTOPOLL = 0x20; // broken
  void rxFingerStatus(const uint8_t *payload, const uint16_t payload_len);
  void rxPhalangeTxRx(const uint8_t *data, const uint16_t data_len);
  std::vector<uint8_t> phalange_rx;
  std::vector<RxFunctor> phalange_rx_functors;
};

}

#endif

