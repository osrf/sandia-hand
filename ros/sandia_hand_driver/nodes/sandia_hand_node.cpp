#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include "sandia_hand/hand.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sandia_hand_msgs/RawFingerStatus.h>
#include <sandia_hand_msgs/RawMoboStatus.h>
#include <osrf_msgs/JointCommands.h>
using namespace sandia_hand;

/////////////////////////////////////////////////////////////////////////
sandia_hand_msgs::RawFingerStatus g_raw_finger_status;
sandia_hand_msgs::RawMoboStatus   g_raw_mobo_status;
ros::Publisher *g_raw_finger_status_pubs[Hand::NUM_FINGERS] = {NULL};
ros::Publisher *g_raw_mobo_status_pub = NULL;
bool g_done = false;
/////////////////////////////////////////////////////////////////////////

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
  for (int i = 0; i < Hand::NUM_FINGERS; i++)
    hand.setFingerControlMode(i, Hand::FCM_IDLE);
  usleep(1000); // todo: something smarter
  hand.setCameraStreaming(false, false);
  usleep(1000); // todo: something smarter
  hand.setFingerAutopollHz(0);
  usleep(1000); // todo: something smarter
  hand.setMoboStatusHz(0);
  usleep(1000); // todo: something smarter
  hand.setAllFingerPowers(Hand::FPS_OFF);
}

int perish(const char *msg, Hand &hand)
{
  ROS_FATAL("%s", msg);
  shutdownHand(hand);
  return 1;
}

void fingerJointCommandsCallback(Hand *hand, const uint8_t finger_idx,
                                 const osrf_msgs::JointCommandsConstPtr &msg)
{
  ROS_INFO("finger %d joint command %.3f %.3f %.3f",
           finger_idx, msg->position[0], msg->position[1], msg->position[2]);
  hand->setFingerJointPos(finger_idx,
                      msg->position[0], msg->position[1], msg->position[2]);
}

void jointCommandsCallback(Hand *hand, 
                           const osrf_msgs::JointCommandsConstPtr &msg)
{
  /*
  ROS_INFO("joint command %.3f %.3f %.3f",
           msg->position[0], msg->position[1], msg->position[2]);
  hand->setFingerJointPos(0, 
         msg->position[0], msg->position[1], msg->position[2]);
  */
  uint8_t max_effort_dummy[12]; // todo: if max_effort is populated, take it.
  float joint_angles[12];
  for (int i = 0; i < 12; i++)
  {
    max_effort_dummy[i] = 50;
    joint_angles[i] = msg->position[i];
  }
  hand->setAllFingerJointPos(joint_angles, max_effort_dummy);
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
  int16_t  fmcb_effort[3];
} finger_status_t;

void rxFingerStatus(const uint8_t finger_idx, 
                    const uint8_t *payload, const uint16_t payload_len)
{
  //printf("rxFingerStatus %d:   %d bytes\n", finger_idx, payload_len);
  if (payload_len < sizeof(finger_status_t) || finger_idx >= Hand::NUM_FINGERS)
    return; // buh bye
  const finger_status_t *p = (const finger_status_t *)payload;
  sandia_hand_msgs::RawFingerStatus *rfs = &g_raw_finger_status; // save typing
  rfs->fmcb_time = p->fmcb_time;
  rfs->pp_time = p->pp_tactile_time;
  rfs->dp_time = p->dp_tactile_time;
  for (int i = 0; i < 6; i++)
    rfs->pp_tactile[i] = p->pp_tactile[i];
  for (int i = 0; i < 12; i++)
    rfs->dp_tactile[i] = p->dp_tactile[i];
  rfs->pp_strain = p->pp_strain;
  for (int i = 0; i < 3; i++)
  {
    rfs->mm_accel[i] = p->fmcb_imu[i];
    rfs->mm_mag[i]   = p->fmcb_imu[i+3];
    rfs->pp_accel[i] = p->pp_imu[i];
    rfs->pp_mag[i]   = p->pp_imu[i+3];
    rfs->dp_accel[i] = p->dp_imu[i];
    rfs->dp_mag[i]   = p->dp_imu[i+3];
  }
  for (int i = 0; i < 4; i++)
  {
    rfs->pp_temp[i] = p->pp_temp[i];
    rfs->dp_temp[i] = p->dp_temp[i];
  }
  for (int i = 0; i < 3; i++)
    rfs->fmcb_temp[i] = p->fmcb_temp[i];
  rfs->fmcb_voltage = p->fmcb_voltage;
  rfs->fmcb_pb_current = p->fmcb_pb_current;
  for (int i = 0; i < 3; i++)
  {
    rfs->hall_tgt[i] = p->fmcb_hall_tgt[i];
    rfs->hall_pos[i] = p->fmcb_hall_pos[i];
    rfs->fmcb_effort[i] = p->fmcb_effort[i];
  }
  if (g_raw_finger_status_pubs[finger_idx])
    g_raw_finger_status_pubs[finger_idx]->publish(g_raw_finger_status);
}

void rxMoboStatus(const uint8_t *data, const uint16_t data_len)
{
  if (!g_raw_mobo_status_pub)
    return;
  const mobo_status_t *p = (mobo_status_t *)data;
  g_raw_mobo_status.mobo_time = p->mobo_time_ms;
  for (int i = 0; i < 4; i++)
    g_raw_mobo_status.finger_currents[i] = p->finger_currents[i];
  for (int i = 0; i < 3; i++)
  {
    g_raw_mobo_status.logic_currents[i] = p->logic_currents[i];
    g_raw_mobo_status.mobo_temp[i] = p->mobo_raw_temperatures[i];
  }
  g_raw_mobo_status.mobo_max_effort = p->mobo_max_effort;
  g_raw_mobo_status_pub->publish(g_raw_mobo_status);
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
    ROS_FATAL("couldn't ping hand motherboard.");
    return 1;
  }
  ROS_INFO("successfully pinged hand motherboard.");

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

  ros::Publisher raw_finger_status_pubs[Hand::NUM_FINGERS];
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    if (!hand.fingers[finger_idx].mm.ping())
      hand.setFingerPower(finger_idx, Hand::FPS_LOW);
  listenToHand(hand, 1.0);
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    if (!hand.fingers[finger_idx].mm.blBoot())
      ROS_WARN("couldn't boot finger %d. is it already booted?", finger_idx);
  listenToHand(hand, 0.5);
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
  {
    if (!hand.fingers[finger_idx].mm.ping())
    {
      ROS_FATAL("couldn't ping finger %d", finger_idx);
      shutdownHand(hand);
      return 1;
    }
    hand.setFingerPower(finger_idx, Hand::FPS_FULL);
    ROS_INFO("finger %d motor module up and running.", finger_idx);
  }
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    if (!hand.fingers[finger_idx].pp.ping())
      hand.fingers[finger_idx].mm.setPhalangeBusPower(true);
  listenToHand(hand, 1.0);
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
  {
    if (!hand.fingers[finger_idx].pp.blBoot())
      ROS_WARN("couldn't boot finger %d proximal phalange", finger_idx);
    if (!hand.fingers[finger_idx].dp.blBoot())
      ROS_WARN("couldn't boot finger %d distal phalange", finger_idx);
  }
  listenToHand(hand, 0.5);
  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
  {
    if (!hand.fingers[finger_idx].pp.ping())
    {
      if (!hand.fingers[finger_idx].pp.ping())
        return perish("couldn't ping proximal phalange", hand);
      if (!hand.fingers[finger_idx].dp.ping())
        return perish("couldn't ping distal phalange", hand);
    }
    ROS_INFO("finger %d phalanges booted", finger_idx);
  }

  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
  {
    char topic_name[100];
    snprintf(topic_name, sizeof(topic_name), 
             "raw_finger_status_%d",finger_idx);
    printf("advertising topic [%s]\n", topic_name);
    raw_finger_status_pubs[finger_idx] = 
      nh.advertise<sandia_hand_msgs::RawFingerStatus>(topic_name, 1);
    g_raw_finger_status_pubs[finger_idx] = &raw_finger_status_pubs[finger_idx];
    // todo: some sort of auto-home sequence. for now, the fingers assume they 
    // were powered up in (0,0,0)

    // set the joint limits for each finger
    const float upper[4][3] = { { 1.5 ,  1.5,  1.7},
      { 0.05,  1.5,  1.7},
      { 0.05,  1.5,  1.7},
      { 0.3 ,  1.1,  1.0} };
    const float lower[4][3] = { {-0.05, -1.2, -1.2},
      {-0.05, -1.2, -1.2},
      {-1.5 , -1.2, -1.2},
      {-1.5 , -1.2, -0.8} };
    if (!hand.fingers[finger_idx].mm.setJointLimits(
                                        lower[finger_idx], upper[finger_idx]))
      return perish("couldn't set finger joint limits", hand);
    if (!hand.fingers[finger_idx].mm.setPhalangeAutopoll(true))
      return perish("couldn't start phalange autopoll", hand);
    ROS_INFO("finger %d is autopolling its phalanges", finger_idx);
    hand.fingers[finger_idx].mm.registerRxHandler(
                              MotorModule::PKT_FINGER_STATUS,
                              boost::bind(rxFingerStatus, finger_idx, _1, _2));

  }
  ros::Publisher raw_mobo_status_pub = 
    nh.advertise<sandia_hand_msgs::RawMoboStatus>("raw_mobo_status", 1);
  g_raw_mobo_status_pub = &raw_mobo_status_pub;
 
  // if we get here, all fingers are up and running. let's start everything now
  usleep(1000); // todo: something smarter
  if (!hand.setFingerAutopollHz(100))
    return perish("couldn't set finger autopoll rate for hand", hand);
  ROS_INFO("hand is autopolling its fingers");

  usleep(1000);
  hand.registerRxHandler(CMD_ID_MOBO_STATUS, rxMoboStatus);
  hand.setMoboStatusHz(100);

  for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    hand.setFingerControlMode(finger_idx, Hand::FCM_JOINT_POS);

  ros::Subscriber joint_commands_sub = 
    nh.subscribe<osrf_msgs::JointCommands>("joint_commands", 1, 
        boost::bind(jointCommandsCallback, &hand, _1));

  ros::Subscriber finger_joint_commands_subs[4];
  for (int i = 0; i < 3; i++)
  {
    char topic[100];
    snprintf(topic, sizeof(topic), "finger_%d/joint_commands", i);
    finger_joint_commands_subs[i] = 
      nh.subscribe<osrf_msgs::JointCommands>(topic, 1,
        boost::bind(fingerJointCommandsCallback, &hand, i, _1));
  }

  hand.setCameraStreaming(true, true);
  ros::spinOnce();
  ros::Time t_prev_spin = ros::Time::now();
  for (int i = 0; !g_done; i++)
  {
    hand.listen(0.01);
    if ((ros::Time::now() - t_prev_spin).toSec() > 0.01)
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
      t_prev_spin = ros::Time::now();
    }
  }
  shutdownHand(hand);
  return 0;
}

