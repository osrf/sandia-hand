#ifndef SANDIA_HAND_H
#define SANDIA_HAND_H

#include <vector>
#include <map>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "hand_packets.h"
#include <boost/function.hpp>
#include <sandia_hand/finger.h>
#include <sandia_hand/palm.h>

namespace sandia_hand
{

class Hand
{
public:
  static const int NUM_FINGERS = 4;
  Finger fingers[NUM_FINGERS];
  Palm palm;
  
  Hand();
  ~Hand();
  bool init(const char *ip = "10.10.1.2");

  enum FingerPowerState { FPS_OFF  = FINGER_POWER_STATE_OFF, 
                          FPS_LOW  = FINGER_POWER_STATE_LOW,
                          FPS_FULL = FINGER_POWER_STATE_FULL }; 
  bool setFingerPower(const uint8_t finger_idx, const FingerPowerState fps);
  bool setAllFingerPowers(const FingerPowerState fps);

  enum FingerControlMode { FCM_IDLE      = FINGER_CONTROL_MODE_IDLE,
                           FCM_JOINT_POS = FINGER_CONTROL_MODE_JOINT_POS };
  bool setFingerControlMode(const uint8_t finger_idx, 
                            const FingerControlMode fcm);
  bool setFingerJointPos(const uint8_t finger_idx,
                         float joint_0, float joint_1, float joint_2);
  bool listen(const float max_seconds);
  bool setCameraStreaming(const bool cam_0_streaming, 
                          const bool cam_1_streaming);
  static const int IMG_WIDTH = 720, IMG_HEIGHT = 480, NUM_CAMS = 2;
  typedef boost::function<void(const uint8_t, const uint32_t, 
                               const uint8_t *)> ImageCallback;
  typedef boost::function<void(const uint8_t *, const uint16_t)> RxFunctor;
  void registerRxHandler(const uint32_t msg_id, RxFunctor f);
  void setImageCallback(ImageCallback callback);
  bool pingFinger(const uint8_t finger_idx);
  bool setMoboStatusHz(const uint16_t mobo_status_hz);
  bool setFingerAutopollHz(const uint16_t finger_autopoll_hz);
  bool programMotorModuleAppFile(const uint8_t finger_idx, FILE *bin_file);
private:
  static const int NUM_SOCKS = 4;
  static const uint16_t HAND_BASE_PORT = 12321; // i love palindromes
  int control_sock, cam_socks[NUM_CAMS], rs485_sock;
  sockaddr_in control_saddr, cam_saddrs[NUM_CAMS], rs485_saddr;
  int *socks[NUM_SOCKS];
  sockaddr_in *saddrs[NUM_SOCKS];
  bool tx_udp(uint8_t *pkt, uint16_t pkt_len);
  bool rx_data(const int sock_idx, const uint8_t *data, const int data_len);
  uint8_t *img_data[NUM_CAMS]; // camera image buffers
  bool *img_rows_recv[NUM_CAMS]; // keep track of completeness
  ImageCallback img_cb;
  bool fingerRawTx(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len);
  std::map<uint32_t, RxFunctor> rx_map_;
  std::map<uint8_t, uint8_t> rx_rs485_map_; // changes in right vs left hands
  bool listenForDuration(float seconds);
};

}

#endif

