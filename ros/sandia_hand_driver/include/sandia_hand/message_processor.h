#ifndef SANDIA_HAND_MESSAGE_PROCESSOR
#define SANDIA_HAND_MESSAGE_PROCESSOR

#include <stdint.h>
#include <boost/function.hpp>

namespace sandia_hand
{

class MessageProcessor
{
public:
  MessageProcessor(const uint8_t addr);
  virtual ~MessageProcessor();
  bool rx(uint8_t *data, uint16_t data_len);

  typedef boost::function<void(const uint8_t *, const uint16_t)> RawTxFunctor;
  inline void setRawTx(RawTxFunctor f) { raw_tx_ = f; }
  typedef boost::function<void(const uint8_t, const uint8_t *, 
                               const uint16_t)> RxMessageFunctor;
  inline void setRxMessageHandler(RxMessageFunctor f) { rx_message_ = f; }
  bool ping();

protected:
  RawTxFunctor raw_tx_;
  RxMessageFunctor rx_message_;
  uint8_t *getTxBuffer();
  bool sendTxBuffer(const uint8_t pkt_id, uint16_t payload_len);
private:
  uint8_t addr_;
  std::vector<uint8_t> outgoing_packet_;
  static const uint32_t MAX_OUTGOING_PACKET_LENGTH = 512;
  static const uint8_t PKT_PING = 0x01;
};

}

#endif
