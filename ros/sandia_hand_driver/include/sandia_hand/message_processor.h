#ifndef SANDIA_HAND_MESSAGE_PROCESSOR
#define SANDIA_HAND_MESSAGE_PROCESSOR

#include <stdint.h>
#include <boost/function.hpp>
#include <vector>
#include <map>

namespace sandia_hand
{

class MessageProcessor
{
public:
  MessageProcessor(const uint8_t addr);
  virtual ~MessageProcessor();
  bool rx(const uint8_t *data, const uint16_t data_len);

  typedef boost::function<void(const uint8_t *, const uint16_t)> RawTxFunctor;
  inline void setRawTx(RawTxFunctor f) { raw_tx_ = f; }
  typedef boost::function<void(const uint8_t *, const uint16_t)> RxFunctor;
  void registerRxHandler(uint8_t msg_id, RxFunctor f);
  bool ping();

protected:
  RawTxFunctor raw_tx_;
  uint8_t *getTxBuffer();
  bool sendTxBuffer(const uint8_t pkt_id, uint16_t payload_len);
private:
  uint8_t addr_;
  std::map<uint8_t, RxFunctor> rx_message_map_;
  std::vector<uint8_t> outgoing_packet_;
  static const uint32_t MAX_OUTGOING_PACKET_LENGTH = 512;
  static const uint8_t  PKT_PING = 0x01;
  void rxPing();
};

}

#endif
