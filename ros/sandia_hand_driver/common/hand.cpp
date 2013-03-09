#include "sandia_hand/hand.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <boost/bind.hpp>
#include "sandia_hand/hand_packets.h"
#include <ros/time.h>
using namespace sandia_hand;

Hand::Hand()
: palm(10) // default rs485 address of palm
{
  socks[0] = &control_sock;
  socks[1] = &cam_socks[0];
  socks[2] = &cam_socks[1];
  socks[3] = &rs485_sock;
  saddrs[0] = &control_saddr;
  saddrs[1] = &cam_saddrs[0];
  saddrs[2] = &cam_saddrs[1];
  saddrs[3] = &rs485_saddr;
  for (int i = 0; i < NUM_SOCKS; i++)
    *socks[i] = 0;
  for (int i = 0; i < NUM_CAMS; i++)
  {
    img_data[i] = new uint8_t[IMG_WIDTH * IMG_HEIGHT];
    img_rows_recv[i] = new bool[IMG_HEIGHT];
    for (int j = 0; j < IMG_HEIGHT; j++)
      img_rows_recv[i][j] = false;
  }
  // this is for right hand. todo: load this dynamically one way or another.
  // depending on if we have a right or left hand.
  rx_rs485_map_[0] = 4;
  rx_rs485_map_[1] = 1;
  rx_rs485_map_[2] = 2;
  rx_rs485_map_[3] = 3;
  rx_rs485_map_[4] = 0;

  for (int i = 0; i < NUM_FINGERS; i++)
  {
    fingers[i].mm.setRawTx(boost::bind(&Hand::fingerRawTx, this, i, _1, _2));
    fingers[i].registerListenHandler(boost::bind(&Hand::listen, this, _1));
  }
  palm.setRawTx(boost::bind(&Hand::fingerRawTx, this, 4, _1, _2));
}

Hand::~Hand()
{
  for (int i = 0; i < NUM_SOCKS; i++)
    if (*socks[i])
      close(*socks[i]);
}

bool Hand::init(const char *ip)
{
  for (int i = 0; i < NUM_SOCKS; i++)
  {
    if (-1 == (*socks[i] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)))
    {
      perror("could't create udp socket");
      return false;
    }
    bzero(saddrs[i], sizeof(saddrs[i]));
    saddrs[i]->sin_family = AF_INET;
    saddrs[i]->sin_port = htons(HAND_BASE_PORT + i);
    saddrs[i]->sin_addr.s_addr = INADDR_ANY;
    if (bind(*socks[i], (struct sockaddr *)saddrs[i], sizeof(*saddrs[i])) != 0)
    {
      perror("couldn't bind udp socket");
      return false;
    }
    // recycle the saddr structs to easily send future outgoing datagrams
    bzero(saddrs[i], sizeof(*saddrs[i]));
    saddrs[i]->sin_family = AF_INET;
    if (0 == inet_aton(ip, &saddrs[i]->sin_addr))
    {
      perror("inet_aton");
      return false;
    }
    saddrs[i]->sin_port = htons(HAND_BASE_PORT + i);
  }
  return true;
}

bool Hand::setFingerPower(const uint8_t finger_idx, const FingerPowerState fps)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_POWER_STATE;
  set_finger_power_state_t *sfps = (set_finger_power_state_t *)(pkt+4);
  sfps->finger_idx = finger_idx;
  sfps->finger_power_state = (uint8_t)fps;
  if (!tx_udp(pkt, 4 + sizeof(set_finger_power_state_t)))
    return false;
  return true;
}

bool Hand::setAllFingerPowers(const FingerPowerState fps)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_ALL_FINGER_POWER_STATES;
  set_all_finger_power_states_t *p = 
                              (set_all_finger_power_states_t *)(pkt+4);
  for (int i = 0; i < 4; i++)
    p->fps[i] = fps;
  if (!tx_udp(pkt, 4 + sizeof(set_all_finger_power_states_t)))
    return false;
  return true;
}

bool Hand::setFingerControlMode(const uint8_t finger_idx,
                                const FingerControlMode fcm)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_CONTROL_MODE;
  set_finger_control_mode_t *p = (set_finger_control_mode_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  p->finger_control_mode = (uint8_t)fcm;
  if (!tx_udp(pkt, 4 + sizeof(set_finger_control_mode_t)))
    return false;
  return true;
}

bool Hand::setFingerJointPos(const uint8_t finger_idx,
                             float joint_0, float joint_1, float joint_2)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_JOINT_POS;
  set_finger_joint_pos_t *p = (set_finger_joint_pos_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  p->joint_0_radians = joint_0;
  p->joint_1_radians = joint_1;
  p->joint_2_radians = joint_2;
  if (!tx_udp(pkt, 4 + sizeof(set_finger_joint_pos_t)))
    return false;
  return true;
}

bool Hand::setCameraStreaming(const bool cam_0_streaming, 
                              const bool cam_1_streaming)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_CONFIGURE_CAMERA_STREAM;
  configure_camera_stream_t *p = (configure_camera_stream_t *)(pkt + 4);
  p->cam_0_stream = cam_0_streaming ? CAMERA_STREAM_ON : CAMERA_STREAM_OFF;
  p->cam_1_stream = cam_1_streaming ? CAMERA_STREAM_ON : CAMERA_STREAM_OFF;
  return tx_udp(pkt, 4 + sizeof(configure_camera_stream_t));
}

bool Hand::setMoboStatusHz(const uint16_t mobo_status_hz)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_MOBO_STATUS_RATE;
  set_mobo_status_rate_t *p = (set_mobo_status_rate_t *)(pkt+4);
  p->mobo_status_hz = mobo_status_hz;
  return tx_udp(pkt, 4 + sizeof(set_mobo_status_rate_t));
}

bool Hand::setFingerAutopollHz(const uint16_t autopoll_hz)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_AUTOPOLL;
  set_finger_autopoll_t *p = (set_finger_autopoll_t *)(pkt+4);
  p->finger_autopoll_hz = autopoll_hz;
  return tx_udp(pkt, 4 + sizeof(set_finger_autopoll_t));
}

bool Hand::fingerRawTx(const uint8_t finger_idx, 
                       const uint8_t *data, const uint16_t data_len)
{
  /*
  printf("Hand::fingerRawTx(%d, x, %d)\n", finger_idx, data_len);
  for (int i = 0; i < data_len; i++)
    printf("  %02d:0x%02x\n", i, data[i]);
  */
  uint8_t pkt[FINGER_RAW_TX_MAX_LEN+20];
  *((uint32_t *)pkt) = CMD_ID_FINGER_RAW_TX;
  finger_raw_tx_t *p = (finger_raw_tx_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  p->pad = 0;
  p->tx_data_len = data_len;
  for (int i = 0; i < data_len && i < FINGER_RAW_TX_MAX_LEN; i++)
    p->tx_data[i] = data[i]; // todo: speed this up someday if it ever matters
  return tx_udp(pkt, 4 + sizeof(finger_raw_tx_t) - 
                     FINGER_RAW_TX_MAX_LEN + data_len);  // ugly
}

bool Hand::tx_udp(uint8_t *pkt, uint16_t pkt_len)
{
  if (-1 == sendto(control_sock, pkt, pkt_len, 0, 
                   (sockaddr *)&control_saddr, sizeof(sockaddr)))
  {
    perror("couldn't send udp packet");
    return false;
  }
  return true;
}

bool Hand::listen(const float max_seconds)
{
  //printf("listen()\n");
  timeval timeout;
  timeout.tv_sec  = (time_t)trunc(max_seconds);
  timeout.tv_usec = (suseconds_t)((max_seconds - timeout.tv_sec) * 1e6);
  fd_set rdset;
  FD_ZERO(&rdset);
  for (int i = 0; i < NUM_SOCKS; i++)
    FD_SET(*socks[i], &rdset);
  int rv = select(*socks[NUM_SOCKS-1]+1, &rdset, NULL, NULL, &timeout);
  if (rv < 0)
    return false; // select returned some error. need better way to bubble up
  if (rv == 0)
    return true; // no errors, but nothing happened. bail.
  for (int i = 0; i < NUM_SOCKS; i++)
    if (FD_ISSET(*socks[i], &rdset)) 
    {
      //printf("%d set\n", i);
      int bytes_recv;
      sockaddr_in recv_addr;
      socklen_t addr_len = sizeof(recv_addr);
      uint8_t recv_buf[2000];
      if ((bytes_recv = recvfrom(*socks[i], recv_buf, sizeof(recv_buf), 0,
                                 (struct sockaddr *)&recv_addr, 
                                 &addr_len)) == -1) 
      {
        perror("recvfrom"); // find a better way to report this...
        return false;
      }
      if (!rx_data(i, recv_buf, bytes_recv)) // just to reduce indenting...
        return false;
    }
  return true; // no errors
}

bool Hand::rx_data(const int sock_idx, const uint8_t *data, const int data_len)
{
  //printf("received %d bytes on sock %d\n", data_len, sock_idx);
  if (sock_idx == 1 || sock_idx == 2)
  {
    const int cam_idx = sock_idx - 1;
    const uint32_t frame_count = *((uint32_t *)data); // maybe use sometime?
    const uint16_t row_count = *((uint16_t *)(data+4));
    const uint8_t *pixels = data + 8;
    if (row_count == 0)
      for (int i = 0; i < IMG_HEIGHT; i++)
        img_rows_recv[cam_idx][i] = false;
    if (row_count < IMG_HEIGHT)
    {
      memcpy(img_data[cam_idx] + row_count * IMG_WIDTH, pixels, IMG_WIDTH);
      img_rows_recv[cam_idx][row_count] = true;
    }
    if (row_count == IMG_HEIGHT - 1)
    {
      // only counts if it's a complete image...
      bool all_recv = true;
      for (int i = 0; i < IMG_HEIGHT-1 && all_recv; i++)
        if (!img_rows_recv[cam_idx][i])
          all_recv = false;
      if (all_recv && img_cb)
        img_cb(cam_idx, frame_count, img_data[cam_idx]);
    }
  }
  else if (sock_idx == 0)
  {
    const uint32_t pkt_id = *((uint32_t *)(data));
    if (rx_map_.find(pkt_id) != rx_map_.end())
      rx_map_[pkt_id](data + 4, (uint16_t)data_len - 4); // neat.
    else
      printf("unhandled packet id: %d\n", pkt_id);
  }
  else if (sock_idx == 3) // rs485 sock
  {
    //printf("rs485 sock received %d bytes\n", data_len);
    // bytes come in pairs on this socket
    for (int i = 0; i < data_len / 2; i++)
    {
      const uint8_t rs485_sender_byte = data[i * 2]; 
      if (!(rs485_sender_byte & 0x80)) // hardware sets high bit. verify.
      {
        //printf("WOAH THERE PARTNER. unexpected byte on rs485 sock: 0x%02x\n",
        //       rs485_sender_byte);
        // actually... this happens as needed to align the UDP payloads on 
        // quadword boundaries, for easier checksum generation in the fpga.
        continue;
      }
      const uint8_t rs485_sender = rs485_sender_byte & ~0x80;
      if (rx_rs485_map_.find(rs485_sender) == rx_rs485_map_.end())
        continue; // bogus sender address. maybe garbled packet somehow.
      const uint8_t remapped_sender = rx_rs485_map_[rs485_sender];
      const uint8_t rs485_byte = data[i * 2 + 1];
      // printf("rs485 sender = %d mapped to %d\n", 
      //        rs485_sender, remapped_sender);
      if (remapped_sender < 4)
        fingers[remapped_sender].mm.rx(&rs485_byte, 1); // todo: batch this
      else if (remapped_sender == 4)
        palm.rx(&rs485_byte, 1);
    }
  }
  return true;
}

void Hand::setImageCallback(ImageCallback callback)
{
  img_cb = callback;
}

bool Hand::pingFinger(const uint8_t finger_idx)
{
  return false; // todo
}

void Hand::registerRxHandler(const uint32_t msg_id, RxFunctor f)
{
  rx_map_[msg_id] = f;
}

bool Hand::programMotorModuleAppFile(const uint8_t finger_idx, FILE *bin_file)
{
  if (finger_idx >= 4 || !bin_file)
    return false; // sanity check
  Finger *finger = &fingers[finger_idx];
  return finger->mm.programAppFile(bin_file,
            boost::bind(&Hand::setFingerPower, this, finger_idx, FPS_OFF),
            boost::bind(&Hand::setFingerPower, this, finger_idx, FPS_LOW));
}

bool Hand::programDistalPhalangeAppFile(const uint8_t finger_idx, 
                                        FILE *bin_file)
{
  if (finger_idx >= 4 || !bin_file)
    return false; // sanity check
  Finger *finger = &fingers[finger_idx];
  return finger->programDistalPhalangeAppFile(bin_file);
}

bool Hand::listenForDuration(float seconds)
{
  for (ros::Time t_start(ros::Time::now()); 
       (ros::Time::now() - t_start).toSec() < seconds;)
    if (!this->listen(0.01))
      return false;
  return true;
}

