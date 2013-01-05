#include "hand.h"
#include <cstdio>
#include <cstring>
using namespace sandia_hand;

Hand::Hand(int num_fingers)
{
  if (num_fingers < 0)
    num_fingers = 0;
  else if (num_fingers > MAX_FINGERS)
    num_fingers = MAX_FINGERS;
  sock = 0;
}

Hand::~Hand()
{
}

bool Hand::init(const char *ip)
{
  if (-1 == (sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)))
  {
    perror("could't create udp socket");
    return false;
  }
  bzero(&saddr, sizeof(saddr));
  saddr.sin_family = AF_INET;
  saddr.sin_port = htons(HAND_PORT);
  saddr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock, (struct sockaddr *)&saddr, sizeof(saddr)) != 0)
  {
    perror("couldn't bind udp socket");
    return false;
  }
  // recycle the saddr struct to be used for future outgoing datagrams
  bzero(&saddr, sizeof(saddr));
  saddr.sin_family = AF_INET;
  if (0 == inet_aton(ip, &saddr.sin_addr))
  {
    perror("inet_aton");
    return false;
  }
  saddr.sin_port = htons(HAND_PORT);
  return true;
}

bool Hand::setFingerPower(uint8_t finger_idx, FingerPowerState fps)
{
  if (finger_idx >= MAX_FINGERS)
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

bool Hand::tx_udp(uint8_t *pkt, uint16_t pkt_len)
{
  if (-1 == sendto(sock, pkt, pkt_len, 0, (sockaddr *)&saddr, sizeof(sockaddr)))
  {
    perror("couldn't send udp packet");
    return false;
  }
  return true;
}

