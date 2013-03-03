#include <cstdio>
#include "sandia_hand/serial_message_processor.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/time.h>
using namespace sandia_hand;

SerialMessageProcessor::SerialMessageProcessor(const uint8_t addr)
: addr_(addr), rx_pkt_addr_(0), rx_pkt_type_(0), rx_pkt_write_idx_(0),
  rx_pkt_len_(0), rx_pkt_crc_(0), rx_pkt_parser_state_(ST_IDLE),
  listen_pkt_type_(0)
{
  outgoing_packet_.resize(MAX_PACKET_LENGTH);
  registerRxHandler(PKT_PING, boost::bind(&SerialMessageProcessor::rxPing, 
                                          this, _1, _2));
}

SerialMessageProcessor::~SerialMessageProcessor()
{
}

bool SerialMessageProcessor::ping()
{
  printf("SerialMessageProcessor::ping()\n");
  if (!sendTxBuffer(PKT_PING, 0))
    return false;
  if (listenFor(PKT_PING, 1.0))
    printf("SMP::ping ok\n");
  else
    printf("SMP::ping fail\n");
  return true;
}

void SerialMessageProcessor::rxPing(const uint8_t *data, 
                                    const uint16_t data_len)
{
  //printf("SerialMessageProcessor::rxPing()\n");
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
  if (payload_len > MAX_PACKET_LENGTH - PAD)
  {
    printf("WOAH THERE PARTNER. you asked for payload len %d, capped to %d.",
           payload_len, MAX_PACKET_LENGTH - PAD);
    payload_len = MAX_PACKET_LENGTH - PAD;
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
  //printf("SerialMessageProcessor::rx  %d bytes\n", data_len);
  for (int i = 0; i < data_len; i++)
    rxByte(data[i]);
  return true;
}

void SerialMessageProcessor::registerRxHandler(uint8_t msg_id, RxFunctor f)
{
  rx_map_[msg_id] = f;
}

void SerialMessageProcessor::rxByte(const uint8_t b)
{
  // todo: upon complete message reception, search rx_map_ and see if we have a
  // handler registered for this msg
  // todo: timeout to reset parser if a packet was garbled
  //printf("processing 0x%02x, rx_pkt_parser_state = %d\n", 
  //       b, (int)rx_pkt_parser_state_);
  switch (rx_pkt_parser_state_)
  {
    case ST_IDLE:
      if (b == 0x42) 
        rx_pkt_parser_state_ = ST_ADDRESS;
      break;
    case ST_ADDRESS:
      rx_pkt_addr_ = b;
      rx_pkt_parser_state_ = ST_LEN_1;
      break;
    case ST_LEN_1:
      rx_pkt_len_ = b;
      rx_pkt_parser_state_ = ST_LEN_2;
      break;
    case ST_LEN_2:
      rx_pkt_len_ |= ((uint16_t)b << 8);
      rx_pkt_parser_state_ = ST_TYPE;
      rx_pkt_data_.resize(rx_pkt_len_ > 0 ? rx_pkt_len_ : 1); // keep >=1 byte
      //printf("expected data payload: %d\n", rx_pkt_len_);
      break;
    case ST_TYPE:
      rx_pkt_type_ = b;
      rx_pkt_write_idx_ = 0;
      if (rx_pkt_len_ > 0)
        rx_pkt_parser_state_ = ST_DATA;
      else
        rx_pkt_parser_state_ = ST_CRC_1;
      break;
    case ST_DATA:
      if (rx_pkt_write_idx_ < MAX_PACKET_LENGTH &&
          rx_pkt_write_idx_ < (uint16_t)rx_pkt_data_.size())
        rx_pkt_data_[rx_pkt_write_idx_++] = b;
      if (rx_pkt_write_idx_ >= rx_pkt_len_)
        rx_pkt_parser_state_ = ST_CRC_1;
      break;
    case ST_CRC_1:
      rx_pkt_crc_ = b;
      rx_pkt_parser_state_ = ST_CRC_2;
      break;
    case ST_CRC_2:
    {
      rx_pkt_crc_ |= ((uint16_t)b << 8);
      rx_pkt_parser_state_ = ST_IDLE; // no matter what happens, reset state.
      // compare crc
      uint16_t crc = 0;
      uint8_t d, crc_highbit;
      for (int i = 0; i < (int)rx_pkt_len_ + 5; i++)
      {
        if (i == 0)
          d = 0x42;
        else if (i == 1)
          d = rx_pkt_addr_;
        else if (i == 2)
          d = rx_pkt_len_ & 0xff;
        else if (i == 3)
          d = (rx_pkt_len_ >> 8) & 0xff;
        else if (i == 4)
          d = rx_pkt_type_;
        else
          d = rx_pkt_data_[i-5];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
          crc_highbit = (crc >> 8) & 0x80;
          crc <<= 1;
          if ((d & 0x80) ^ crc_highbit)
            crc ^= 0x1021; // CRC-16 CCITT polynomial
          d <<= 1;
        }
      }
      if (rx_pkt_crc_ != crc)
      {
        // crc mismatch.
        printf("crc mismatch: 0x%04x != 0x%04x\n", crc, rx_pkt_crc_);
        break;
      }
      if (rx_pkt_addr_ != 0 && rx_pkt_addr_ != 0xff)
      {
        printf("unexpected addr: 0x%02x\n", rx_pkt_addr_);
        break;
      }
      if (rx_map_.find(rx_pkt_type_) != rx_map_.end())
        rx_map_[rx_pkt_type_](&rx_pkt_data_[0], rx_pkt_len_);
      //printf("listening for type 0x%02x\n", listen_pkt_type_);
      if (listen_pkt_type_ == rx_pkt_type_)
        stopListening();
      //printf("received packet type 0x%02x with payload length %d\n",
      //       rx_pkt_type_, rx_pkt_len_);
      break;
    }
    default:
      rx_pkt_parser_state_ = ST_IDLE;
      break;
  }
}

void SerialMessageProcessor::registerListenHandler(ListenFunctor f)
{
  listen_functor_ = f;
}

bool SerialMessageProcessor::listenFor(const uint8_t listen_pkt_type,
                                       const float max_seconds)
{
  //printf("SMP::listenFor(%d, %.6f)\n", listen_pkt_type, max_seconds);
  if (!listen_functor_)
  {
    printf("WOAH THERE PARTNER. called listenFor without listen_functor_ set");
    return false;
  }
  done_listening_ = false;
  listen_pkt_type_ = listen_pkt_type;
  ros::Time t_start(ros::Time::now());
  for (ros::Time t_start(ros::Time::now());
       (ros::Time::now() - t_start).toSec() < max_seconds;)
  {
    //printf("elapsed duration: %.6f\n", (ros::Time::now() - t_start).toSec());
    //printf("listening for %d\n", listen_pkt_type_);
    listen_functor_(0.01);
    if (done_listening_)
      return true;
  }
  return false;
}

void SerialMessageProcessor::stopListening()
{
  //printf("SMP::stopListening()\n");
  done_listening_ = true;
}

