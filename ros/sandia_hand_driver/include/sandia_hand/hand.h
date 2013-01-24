#ifndef SANDIA_HAND_H
#define SANDIA_HAND_H

#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "hand_packets.h"
#include <boost/function.hpp>

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
  static const int IMG_WIDTH = 720, IMG_HEIGHT = 480, NUM_CAMS = 2;
  typedef boost::function<void(uint8_t, uint32_t, uint8_t *)> ImageCallback;
  void setImageCallback(ImageCallback callback);

private:
  static const int MAX_FINGERS = 4, NUM_SOCKS = 3;
  static const uint16_t HAND_BASE_PORT = 12321; // i love palindromes
  int control_sock, cam_socks[NUM_CAMS];
  sockaddr_in control_saddr, cam_saddrs[NUM_CAMS];
  int *socks[NUM_SOCKS];
  sockaddr_in *saddrs[NUM_SOCKS];
  bool tx_udp(uint8_t *pkt, uint16_t pkt_len);
  bool rx_data(const int sock_idx, const uint8_t *data, const int data_len);
  uint8_t *img_data[NUM_CAMS]; // camera image buffers
  bool *img_rows_recv[NUM_CAMS]; // keep track of completeness
  ImageCallback img_cb;
};

}

#endif
