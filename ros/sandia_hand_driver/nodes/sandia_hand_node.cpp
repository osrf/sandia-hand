#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include "sandia_hand/hand.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sandia_hand_msgs/RawFingerInertial.h>
#include <osrf_msgs/JointCommands.h>
using namespace sandia_hand;

bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

void listenToHand(Hand &hand, const float seconds)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    hand.listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > seconds)
      break;
  }
}

void shutdownHand(Hand &hand)
{
  hand.setCameraStreaming(false, false);
  hand.setFingerAutopollHz(0);
  hand.setAllFingerPowers(Hand::FPS_OFF);
}

void jointCommandsCallback(const osrf_common::JointCommandsConstPtr &msg)
{
  printf("jcb\n");
}

// todo: have firmware and driver pull from same .h file, instead of pure haxx
typedef struct
{
  uint32_t pp_tactile_time;
  uint32_t dp_tactile_time;
  uint32_t fmcb_time;
  uint16_t pp_tactile[6];
  uint16_t dp_tactile[12];
  uint16_t pp_imu[6];
  uint16_t dp_imu[6];
  uint16_t fmcb_imu[6];
  uint16_t pp_temp[4];
  uint16_t dp_temp[4];
  uint16_t fmcb_temp[3];
  uint16_t fmcb_voltage;
  uint16_t fmcb_pb_current;
  uint32_t pp_strain;
  int32_t  fmcb_hall_tgt[3];
  int32_t  fmcb_hall_pos[3];
} finger_status_t;

sandia_hand_msgs::RawFingerInertial g_raw_finger_inertial;
ros::Publisher *g_raw_finger_inertial_pubs[Hand::NUM_FINGERS] = {NULL};

void rxFingerStatus(const uint8_t finger_idx, 
                    const uint8_t *payload, const uint16_t payload_len)
{
  //printf("rxFingerStatus %d:   %d bytes\n", finger_idx, payload_len);
  if (payload_len < sizeof(finger_status_t) || finger_idx >= Hand::NUM_FINGERS)
    return; // buh bye
  const finger_status_t *p = (const finger_status_t *)payload;
  //printf("%.6f\n", p->fmcb_time * 1.0e-6);
  for (int i = 0; i < 3; i++)
  {
    g_raw_finger_inertial.mm_accel[i] = p->fmcb_imu[i];
    g_raw_finger_inertial.mm_mag[i]   = p->fmcb_imu[i+3];
  }
  if (g_raw_finger_inertial_pubs[finger_idx])
    g_raw_finger_inertial_pubs[finger_idx]->publish(g_raw_finger_inertial);
}

static const unsigned NUM_CAMS = 2;
boost::shared_ptr<camera_info_manager::CameraInfoManager> g_cinfo[NUM_CAMS];
image_transport::CameraPublisher *g_image_pub[NUM_CAMS] = {0};
sensor_msgs::Image g_img_msg[NUM_CAMS];

void image_cb(const uint8_t cam_idx, const uint32_t frame_count, 
              const uint8_t *img_data)
{
  if (cam_idx > 1)
    return; // woah
  fillImage(g_img_msg[cam_idx], "mono8",
            Hand::IMG_HEIGHT, Hand::IMG_WIDTH, Hand::IMG_WIDTH, img_data);
  if (cam_idx == 0)
    g_img_msg[cam_idx].header.stamp = ros::Time::now();
  else
    g_img_msg[cam_idx].header.stamp = g_img_msg[0].header.stamp;
  g_img_msg[cam_idx].encoding = "mono8";
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(
                                        g_cinfo[cam_idx]->getCameraInfo()));
  ci->header.stamp = g_img_msg[0].header.stamp;
  ci->header.frame_id = "stereo_cam";
  if (g_image_pub[cam_idx])
    g_image_pub[cam_idx]->publish(g_img_msg[cam_idx], *ci);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sandia_hand_node");
  Hand hand;
  if (!hand.init())
  {
    ROS_FATAL("couldn't init hand");
    return 1;
  }

  ros::NodeHandle nh, nh_right("right"), nh_left("left");

  // be sure we can ping it
  if (!hand.pingMoboMCU())
  {
    ROS_FATAL("couldn't ping hand.");
    return 1;
  }
  ROS_INFO("successfully pinged hand.");

  signal(SIGINT, signal_handler);
  g_cinfo[0] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_left));
  g_cinfo[1] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_right));
  g_cinfo[0]->setCameraName("left");
  g_cinfo[1]->setCameraName("right");
  g_cinfo[0]->loadCameraInfo("package://sandia_hand_driver/camera_calibration/stereo/left.ini");
  g_cinfo[1]->loadCameraInfo("package://sandia_hand_driver/camera_calibration/stereo/right.ini");
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher image_pub[2];
  image_pub[0] = it.advertiseCamera("left/image_raw", 1);
  image_pub[1] = it.advertiseCamera("right/image_raw", 1);
  g_image_pub[0] = &image_pub[0];
  g_image_pub[1] = &image_pub[1];
  hand.setImageCallback(&image_cb); 

  // now, start up finger(s)
  // set finger 0 socket to low voltage
  hand.setFingerPower(0, Hand::FPS_LOW);
  listenToHand(hand, 1.0);
  if (!hand.fingers[0].mm.blBoot())
  {
    printf("couldn't boot finger 0\n");
    shutdownHand(hand);
    return 1;
  }
  listenToHand(hand, 0.5);
  if (!hand.fingers[0].mm.ping())
  {
    printf("couldn't ping finger 0\n");
    shutdownHand(hand);
    return 1;
  }
  hand.setFingerPower(0, Hand::FPS_FULL);
  printf("finger 0 booted\n");
  hand.fingers[0].mm.setPhalangeBusPower(true);
  listenToHand(hand, 1.0);
  if (!hand.fingers[0].pp.blBoot())
  {
    printf("couldn't boot finger 0 proximal phalange\n");
    shutdownHand(hand);
    return 1;
  }
  if (!hand.fingers[0].dp.blBoot())
  {
    printf("couldn't boot finger 0 distal phalange\n");
    shutdownHand(hand);
    return 1;
  }
  listenToHand(hand, 0.5);
  if (!hand.fingers[0].pp.ping())
  {
    printf("couldn't ping finger 0 proximal phalange\n");
    shutdownHand(hand);
    return 1;
  }
  if (!hand.fingers[0].dp.ping())
  {
    printf("couldn't ping finger 0 distal phalange\n");
    shutdownHand(hand);
    return 1;
  }
  printf("finger 0 phalanges booted\n");
  if (!hand.fingers[0].mm.setPhalangeAutopoll(true))
  {
    printf("couldn't start phalange autopoll on finger 0\n");
    shutdownHand(hand);
    return 1;
  }
  printf("finger 0 is autopolling its phalanges\n");
  hand.fingers[0].mm.registerRxHandler(MotorModule::PKT_FINGER_STATUS,
                                       boost::bind(rxFingerStatus, 0, _1, _2));
  if (!hand.setFingerAutopollHz(100))
  {
    printf("couldn't set finger autopoll rate for hand\n");
    shutdownHand(hand);
    return 1;
  }
  printf("hand is autopolling its fingers\n");
  ros::Publisher raw_finger_inertial_pubs[Hand::NUM_FINGERS];
  for (int i = 0; i < Hand::NUM_FINGERS; i++)
  { 
    char topic_name[100];
    snprintf(topic_name, sizeof(topic_name), "raw_finger_inertial_%d", i);
    raw_finger_inertial_pubs[i] = 
         nh.advertise<sandia_hand_msgs::RawFingerInertial>(topic_name, 1);
    g_raw_finger_inertial_pubs[i] = &raw_finger_inertial_pubs[i];
  }
  // todo: some sort of auto-home sequence. for now, the fingers assume they 
  // were powered up in (0,0,0)

  ros::Subscriber joint_commands_sub = 
          n.subscribe("joint_commands", 1, 
                      boost::bind(JointCommandsCallback, hand, _1));

  // set the joint limits for each finger
  float upper[4][3] = { { 1.5 ,  1.5,  1.7},
                        { 0.05,  1.5,  1.7},
                        { 0.05,  1.5,  1.7},
                        { 0.3 ,  1.1,  1.0} };
  float lower[4][3] = { {-0.05, -1.2, -1.2},
                        {-0.05, -1.2, -1.2},
                        {-1.5 , -1.2, -1.2},
                        {-1.5 , -1.2, -0.8} };


  //hand.setCameraStreaming(true, true);
  ros::spinOnce();
  ros::Time t_prev_spin = ros::Time::now();
  for (int i = 0; !g_done; i++)
  {
    hand.listen(0.01);
    if (i % 100 == 0) // todo: be smarter
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
    }
  }
  shutdownHand(hand);
  return 0;
}

