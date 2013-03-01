#ifndef SANDIA_HAND_SERIAL_MESSAGE_PROCESSOR
#define SANDIA_HAND_SERIAL_MESSAGE_PROCESSOR

#include <stdint.h>
#include <boost/function.hpp>
#include <vector>
#include <map>

namespace sandia_hand
{

class SerialMessageProcessor
{
public:
  SerialMessageProcessor(const uint8_t addr);
  virtual ~SerialMessageProcessor();
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
  static const uint32_t MAX_PACKET_LENGTH = 512;
  uint8_t addr_;
  uint8_t rx_pkt_addr_, rx_pkt_type_;
  uint16_t rx_pkt_write_idx_, rx_pkt_len_, rx_pkt_crc_;
  enum { ST_IDLE=0, ST_ADDRESS=1, ST_LEN_1=2, ST_LEN_2=3, 
         ST_TYPE=4, ST_DATA=5, ST_CRC_1=6, ST_CRC_2=7 } rx_pkt_parser_state_;
  std::vector<uint8_t> rx_pkt_data_;
  std::map<uint8_t, RxFunctor> rx_map_;
  std::vector<uint8_t> outgoing_packet_;
  static const uint8_t  PKT_PING = 0x01;
  void rxPing(const uint8_t *payload, const uint16_t payload_len);
  void rxByte(const uint8_t b);
};

}

#endif
