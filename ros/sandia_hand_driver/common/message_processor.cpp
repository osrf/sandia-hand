#include <cstdio>
#include "sandia_hand/message_processor.h"
using namespace sandia_hand;

MessageProcessor::MessageProcessor(const uint8_t addr)
: addr_(addr)
{
  outgoing_packet_.resize(MAX_OUTGOING_PACKET_LENGTH);
}

MessageProcessor::~MessageProcessor()
{
}

bool MessageProcessor::ping()
{
  printf("MessageProcessor::ping()\n");
  prepareTxBuffer(PKT_PING, 0);
  return true;
}

uint8_t *MessageProcessor::getTxBuffer()
{
  return (uint8_t *)(&outgoing_packet_[5]);
}

bool MessageProcessor::sendTxBuffer(const uint8_t pkt_id, uint16_t payload_len)
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
  outgoing_packet_[1] = addr;
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

