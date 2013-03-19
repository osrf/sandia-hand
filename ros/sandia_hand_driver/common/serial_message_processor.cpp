#include <cstdio>
#include "sandia_hand/serial_message_processor.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/time.h>
using namespace sandia_hand;

SerialMessageProcessor::SerialMessageProcessor(const uint8_t addr)
: print_parser_debris_(false), 
  addr_(addr), rx_pkt_addr_(0), rx_pkt_type_(0), rx_pkt_write_idx_(0),
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
  if (!sendTxBuffer(PKT_PING))
    return false;
  return listenFor(PKT_PING, 0.2);
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

uint16_t SerialMessageProcessor::deserializeUint16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t SerialMessageProcessor::deserializeUint32(const uint8_t *p)
{
  return  (uint32_t)p[0]        |
         ((uint32_t)p[1] << 8)  |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

float SerialMessageProcessor::deserializeFloat32(const uint8_t *p)
{
  uint8_t q[4];
  q[3] = p[3];
  q[2] = p[2];
  q[1] = p[1];
  q[0] = p[0];
  return *((float *)q);
}

void SerialMessageProcessor::serializeUint16(const uint16_t x, uint8_t *p)
{
  p[0] = (uint8_t)(x & 0xff);
  p[1] = (uint8_t)((x >> 8) & 0xff);
}

void SerialMessageProcessor::serializeInt16(const  int16_t x, uint8_t *p)
{
  p[0] = (uint8_t)(x & 0xff);
  p[1] = (uint8_t)((x >> 8) & 0xff);
}

void SerialMessageProcessor::serializeUint32(const uint32_t x, uint8_t *p)
{
  p[0] = (uint8_t) (x        & 0xff);
  p[1] = (uint8_t)((x >> 8)  & 0xff);
  p[2] = (uint8_t)((x >> 16) & 0xff);
  p[3] = (uint8_t)((x >> 24) & 0xff);
}

void SerialMessageProcessor::serializeFloat32(const float x, uint8_t *p)
{
  const uint8_t *q = (uint8_t *)(&x);
  p[0] = q[0];
  p[1] = q[1];
  p[2] = q[2];
  p[3] = q[3];
}


void SerialMessageProcessor::rxByte(const uint8_t b)
{
  // todo: upon complete message reception, search rx_map_ and see if we have a
  // handler registered for this msg
  // todo: timeout to reset parser if a packet was garbled
  if (print_parser_debris_)
    printf("processing 0x%02x, rx_pkt_parser_state = %d\n", 
           b, (int)rx_pkt_parser_state_);
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
      if (print_parser_debris_)
        printf("received packet type 0x%02x with payload length %d\n",
               rx_pkt_type_, rx_pkt_len_);
      if (rx_map_.find(rx_pkt_type_) != rx_map_.end())
        rx_map_[rx_pkt_type_](&rx_pkt_data_[0], rx_pkt_len_);
      //printf("listening for type 0x%02x\n", listen_pkt_type_);
      if (listen_pkt_type_ == rx_pkt_type_)
        stopListening();
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
  resetParser();
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

bool SerialMessageProcessor::listenFor(float seconds)
{
  if (!listen_functor_)
  {
    printf("WOAH THERE PARTNER. called listenFor without listen_functor_ set");
    return false;
  }
  done_listening_ = false;
  listen_pkt_type_ = 0xff;
  ros::Time t_start(ros::Time::now());
  for (ros::Time t_start(ros::Time::now());
       (ros::Time::now() - t_start).toSec() < seconds;)
    listen_functor_(0.01);
  return true;
}

void SerialMessageProcessor::stopListening()
{
  //printf("SMP::stopListening()\n");
  done_listening_ = true;
}

bool SerialMessageProcessor::blHaltAutoboot()
{
  if (!sendTxBuffer(PKT_BL_HALT_AUTOBOOT))
    return false;
  return listenFor(PKT_BL_HALT_AUTOBOOT, 0.5);
}

bool SerialMessageProcessor::blBoot()
{
  if (!sendTxBuffer(PKT_BL_BOOT))
    return false;
  return listenFor(PKT_BL_BOOT, 0.5);
}

bool SerialMessageProcessor::reset()
{
  if (!sendTxBuffer(PKT_RESET))
    return false;
  return listenFor(PKT_RESET, 0.5);
}

bool SerialMessageProcessor::blReadFlashPage(const uint16_t page_num, 
                                             uint8_t *page_buf)
{
  serializeUint32((uint32_t)page_num, getTxBuffer());
  if (!sendTxBuffer(PKT_BL_READ_FLASH_PAGE, 4))
    return false;
  if (!listenFor(PKT_BL_READ_FLASH_PAGE, 0.5))
    return false;
  if (rx_pkt_data_.size() != 256)
  {
    printf("blReadFlashPage unexpected length: %d\n", (int)rx_pkt_data_.size());
    return false;
  }
  memcpy(page_buf, &rx_pkt_data_[0], 256);
  return true;
}

bool SerialMessageProcessor::blWriteFlashPage(const uint16_t page_num, 
                                              const uint8_t *page_buf,
                                              bool chop)
{
  listen_functor_(0.01); // flush buffer from previous hiccups...
  if (page_num > 1024 || !page_buf)
    return false; // sanity check...
  if (!chop) // try it all as one transmission
  {
    for (int attempt = 0; attempt < 50; attempt++)
    {
      printf("writing page %d, attempt %d\n", page_num, attempt);
      serializeUint32((uint32_t)page_num, getTxBuffer());
      memcpy(getTxBuffer()+4, page_buf, 256);
      if (!sendTxBuffer(PKT_BL_WRITE_FLASH_PAGE, 256+4))
        continue;
      if (!listenFor(PKT_BL_WRITE_FLASH_PAGE, 0.03))
        continue;
      // check what it sent back to us
      if (rx_pkt_data_.size() != 5)
        continue;
      uint8_t write_result = rx_pkt_data_[4];
      if (write_result != 0)
        continue;
      return true; // if we get here, all is well.
    }
    return false; // failed N times.
  }
  // if we get here, we need to do 64 individual transactions to buffer it
  for (int word_idx = 0; word_idx < 64; word_idx++)
  {
    bool done = false;
    for (int attempt = 0; !done && attempt < 20; attempt++)
    {
      printf("stuffing buffer word %d, attempt %d\n", word_idx, attempt);
      getTxBuffer()[0] = word_idx;
      for (int i = 0; i < 4; i++)
        getTxBuffer()[i+1] = page_buf[word_idx*4 + i];
      if (!sendTxBuffer(PKT_BL_SET_FLASH_BUF_WORD, 5))
        return false;
      if (!listenFor(PKT_BL_SET_FLASH_BUF_WORD, 0.2))
      {
        printf("no response\n");
        continue;
      }
      if (rx_pkt_data_.size() != 2 || rx_pkt_data_[1] != 0)
      {
        printf("incorrect rx length: %d or value: %d\n",
               (int)rx_pkt_data_.size(),
               rx_pkt_data_[1]);
        continue;
      }
      done = true;
    }
    if (!done)
    {
      printf("failed to stuff flash page buffer\n");
      return false;
    }
  }
  // if we get here, the buffer is stuffed, commit it now.
  for (int attempt = 0; attempt < 20; attempt++)
  {
    printf("committing buffer, attempt %d\n", attempt);
    serializeUint32((uint32_t)page_num, getTxBuffer());
    if (!sendTxBuffer(PKT_BL_WRITE_FLASH_BUF, 4))
      continue;
    if (!listenFor(PKT_BL_WRITE_FLASH_BUF, 0.2))
      continue;
    if (rx_pkt_data_.size() != 5 || rx_pkt_data_[5] != 0)
      continue;
    return true; // hooray, we're done
  }
  return false; 
}

void SerialMessageProcessor::resetParser()
{
  rx_pkt_parser_state_ = ST_IDLE;
}

bool SerialMessageProcessor::programAppFile(FILE *bin_file, 
                                            PowerFunctor power_off, 
                                            PowerFunctor power_on)
{
  // important! this function assumes that the FILE* is either created for
  // application image space (0x0402000) or it is already advanced via fseek() 

  // first, do a power-cycle hard reset to ensure the bootloader is running
  if (!power_off()) return false;
  if (!listenFor(1.0)) return false; // wait for power off
  if (!power_on()) return false;
  if (!listenFor(2.0)) return false; // wait for bootloader init
  if (!blHaltAutoboot())
  {
    printf("unable to halt autoboot\n");
    return false;
  }
  printf("autoboot halted.\n");
  for (int page_num = 32; !feof(bin_file) && page_num < 1024; page_num++)
  {
    bool page_written = false;
    uint8_t page_buf[256] = {0};
    size_t nread = 0;
    nread = fread(page_buf, 1, 256, bin_file);
    if (nread == 0)
    {
      printf("couldn't read a flash page from FILE: returned %d\n",
             (int)nread);
      return false;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n",
             (int)nread);
    if (blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      return false;
    }
  }
  if (!blBoot())
  {
    printf("failed to boot\n");
    return false;
  }
  printf("successfully booted after app write\n");
  return true;
}

