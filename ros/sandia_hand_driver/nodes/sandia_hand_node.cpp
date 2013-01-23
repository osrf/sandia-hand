#include <cstdio>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sandia_hand/hand.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
using namespace sandia_hand;

bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

static const unsigned NUM_CAMS = 2;
boost::shared_ptr<camera_info_manager::CameraInfoManager> g_cinfo[NUM_CAMS];
image_transport::CameraPublisher *g_image_pub[NUM_CAMS] = {0};
sensor_msgs::Image g_img_msg[NUM_CAMS];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sandia_hand_node");
  Hand hand;
  if (!hand.init())
  {
    ROS_FATAL("couldn't init hand");
    return 1;
  }
  signal(SIGINT, signal_handler);
  ros::NodeHandle nh, nh_right("right"), nh_left("left");
  g_cinfo[0] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_left));
  g_cinfo[1] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_right));
  g_cinfo[0]->setCameraName("left");
  g_cinfo[1]->setCameraName("right");
  g_cinfo[0]->loadCameraInfo("package://sandia_hand_driver/calib/left.yaml");
  g_cinfo[1]->loadCameraInfo("package://sandia_hand_driver/calib/right.yaml");
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher image_pub[2];

  for (int i = 0; !g_done; i++)
  {
    hand.listen(0.01);
    if (i % 1000 == 0) // todo: be smarter
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
    }
  }
  return 0;
}
