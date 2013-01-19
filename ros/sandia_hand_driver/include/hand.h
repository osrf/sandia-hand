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
  bool setFingerJointPos(const uint8_t finger_idx,
                         float joint_0, float joint_1, float joint_2);
  bool listen(const float max_seconds);
  bool setCameraStreaming(bool cam_0_stream, bool cam_1_stream);

private:
  static const int MAX_FINGERS = 4, NUM_SOCKS = 3;
  static const uint16_t HAND_BASE_PORT = 12321; // i love palindromes
  int control_sock, cam_socks[2];
  sockaddr_in control_saddr, cam_saddrs[2];
  int *socks[NUM_SOCKS];
  sockaddr_in *saddrs[NUM_SOCKS];
  bool tx_udp(uint8_t *pkt, uint16_t pkt_len);
  bool rx_data(const int sock_idx, const uint8_t *data, const int data_len);
};

}

#endif

