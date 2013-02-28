#include <cstdio>
#include "sandia_hand/serial_message_processor.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

SerialMessageProcessor::SerialMessageProcessor(const uint8_t addr)
: addr_(addr)
{
  outgoing_packet_.resize(MAX_OUTGOING_PACKET_LENGTH);
  registerRxHandler(PKT_PING, boost::bind(&SerialMessageProcessor::rxPing, 
                                          this, _1, _2));
}

SerialMessageProcessor::~SerialMessageProcessor()
{
}

bool SerialMessageProcessor::ping()
{
  printf("SerialMessageProcessor::ping()\n");
  return sendTxBuffer(PKT_PING, 0);
}

void SerialMessageProcessor::rxPing(const uint8_t *data, 
                                    const uint16_t data_len)
{
  printf("SerialMessageProcessor::rxPing()\n");
}

uint8_t *SerialMessageProcessor::getTxBuffer()
{
  return (uint8_t *)(&outgoing_packet_[5]);
}

bool SerialMessageProcessor::sendTxBuffer(const uint8_t pkt_id, 
                                          uint16_t payload_len)
{
  if (!raw_tx_)
    return false;
  const uint32_t PAD = 20;  
  if (payload_len > MAX_OUTGOING_PACKET_LENGTH - PAD)
  {
    printf("WOAH THERE PARTNER. you asked for payload len %d, capped to %d.",
           payload_len, MAX_OUTGOING_PACKET_LENGTH - PAD);
    payload_len = MAX_OUTGOING_PACKET_LENGTH - PAD;
  }
  outgoing_packet_[0] = 0x42;
  outgoing_packet_[1] = addr_;
  *((uint16_t *)(&outgoing_packet_[2])) = payload_len;
  outgoing_packet_[4] = pkt_id;
  // i'm sure this could be done much faster, if it ever mattered.
  uint16_t crc = 0;
  uint8_t d, crc_highbit;
  for (uint32_t i = 0; i < (uint32_t)payload_len + 5; i++)
  {
    d = outgoing_packet_[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // crc-16 ccitt polynomial
      d <<= 1;
    }
  }
  *((uint16_t *)(&outgoing_packet_[5 + payload_len])) = crc;
  raw_tx_(&outgoing_packet_[0], payload_len + 7);
  return true;
}

bool SerialMessageProcessor::rx(const uint8_t *data, const uint16_t data_len)
{
  //printf("MessageProcessor::rx(0x%08x, %d)\n", (uint32_t)data, data_len);
  // todo: search rx_map_ and see if we have a handler registered for this msg
  return true;
}

void SerialMessageProcessor::registerRxHandler(uint8_t msg_id, RxFunctor f)
{
  rx_map_[msg_id] = f;
}

