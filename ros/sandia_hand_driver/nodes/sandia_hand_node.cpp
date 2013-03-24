#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include "sandia_hand/hand.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
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
  hand.setAllFingerPowers(Hand::FPS_OFF);
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
  if (!hand.setFingerAutopollHz(1))
  {
    printf("couldn't set finger autopoll rate for hand\n");
    shutdownHand(hand);
    return 1;
  }
  printf("hand is autopolling its fingers\n");

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

