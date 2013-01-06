#ifndef HAND_H
#define HAND_H

#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "hand_packets.h"

namespace sandia_hand
{

class Hand
{
public:
  Hand(int num_fingers = 4);
  ~Hand();
  bool init(const char *ip = "10.10.1.2");

  enum FingerPowerState { FPS_OFF  = FINGER_POWER_STATE_OFF, 
                          FPS_LOW  = FINGER_POWER_STATE_LOW,
                          FPS_FULL = FINGER_POWER_STATE_FULL }; 
  bool setFingerPower(const uint8_t finger_idx, const FingerPowerState fps);

  enum FingerControlMode { FCM_IDLE      = FINGER_CONTROL_MODE_IDLE,
                           FCM_JOINT_POS = FINGER_CONTROL_MODE_JOINT_POS };
  bool setFingerControlMode(const uint8_t finger_idx, 
                            const FingerControlMode fcm);

private:
  static const int MAX_FINGERS = 4;
  static const uint16_t HAND_PORT = 12321; // i love palindromes
  int sock;
  sockaddr_in saddr;
  bool tx_udp(uint8_t *pkt, uint16_t pkt_len);
};

}

#endif

