#include <signal.h>
#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <sandia_hand_msgs/RawPalmState.h>
#include "sandia_hand/loose_right_palm.h"
#include <sandia_hand_msgs/GetParameters.h>
#include <sandia_hand_msgs/SetParameters.h>
using namespace sandia_hand;
using std::string;
using std::vector;

/////////////////////////////////////////////////////////////////////////
sandia_hand_msgs::RawPalmState g_raw_finger_state;
ros::Publisher *g_raw_palm_state_pub = NULL;
bool g_done = false;
/////////////////////////////////////////////////////////////////////////

void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}

void listenToPalm(LooseRightPalm *palm, const float seconds)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    palm->listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > seconds)
      break;
  }
}

int perish(const char *msg)
{
  ROS_FATAL("%s", msg);
  return 1;
}

bool getParametersSrv(LooseRightPalm *palm,
                      sandia_hand_msgs::GetParameters::Request &req,
                      sandia_hand_msgs::GetParameters::Response &res)
{
  ROS_INFO("get parameters");
  const vector<sandia_hand::Param> params = palm->getParams();
  res.parameters.resize(params.size());
  for (size_t i = 0; i < params.size(); i++)
  {
    res.parameters[i].name = params[i].getName();
    if (params[i].getType() == Param::PARAM_INT)
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::INTEGER;
      res.parameters[i].i_val = params[i].getIntVal();
    }
    else
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::FLOAT;
      res.parameters[i].f_val = params[i].getFloatVal();
    }
  }
  return true;
}

bool setParametersSrv(LooseRightPalm *palm,
                      sandia_hand_msgs::SetParameters::Request &req,
                      sandia_hand_msgs::SetParameters::Response &res)
{
  ROS_INFO("get parameters");
  bool all_ok = true;
  for (size_t i = 0; i < req.parameters.size(); i++)
  {
    const sandia_hand_msgs::Parameter *p = &req.parameters[i]; // save typing
    if (p->val_type == sandia_hand_msgs::Parameter::INTEGER) //Param::PARAM_INT)
      all_ok &= palm->setParamInt(p->name, (int32_t)p->i_val);
    else
      all_ok &= palm->setParamFloat(p->name, (float)p->f_val);
  }
  return all_ok;
}

#include "sandia_hand/palm_state.h"
void rxPalmState(const uint8_t *payload, const uint16_t payload_len)
{
  printf("rxPalmState\n");
}
#if 0
void rxFingerState(const uint8_t *payload, const uint16_t payload_len)
{
  //printf("rxFingerState: %d bytes\n", payload_len);
  if (payload_len < sizeof(finger_state_t))
    return; // buh bye
  const finger_state_t *p = (const finger_state_t *)payload;
  sandia_hand_msgs::RawFingerState *rfs = &g_raw_finger_state; // save typing
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
    // ugh. electronics and firmware has joint indices flipped. fix someday.
    rfs->hall_tgt[i] = p->fmcb_hall_tgt[2-i];
    rfs->hall_pos[i] = p->fmcb_hall_pos[2-i];
    rfs->fmcb_effort[i] = p->fmcb_effort[2-i];
    g_last_fmcb_hall_pos[i] = p->fmcb_hall_pos[2-i]; // gross
  }
  if (g_raw_finger_state_pub)
    g_raw_finger_state_pub->publish(g_raw_finger_state);
}
#endif
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sandia_hand_loose_palm_node");
  ros::NodeHandle nh, nh_private("~");
  LooseRightPalm palm;
  std::string serial_device;
  nh_private.param<string>("serial_device", serial_device, "/dev/ttyUSB0");
  if (!palm.init(serial_device.c_str()))
  {
    ROS_FATAL("couldn't init palm");
    return 1;
  }
  if (!palm.blBoot())
    ROS_WARN("couldn't boot palm. is it already booted?");
  else
    ROS_INFO("booted palm.");
  // be sure we can ping it
  if (!palm.ping())
  {
    ROS_FATAL("couldn't ping finger motor module");
    return 1;
  }
  ROS_INFO("successfully pinged finger motor module.");
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  ros::Publisher raw_palm_state_pub;
  if (!finger.pp.ping())
    finger.mm.setPhalangeBusPower(true);
  listenToFinger(&finger, 2.0);
  // todo: make proximal/distal phalanges ROS parameters for manufacturing
  if (use_proximal_phalange)
    if (!finger.pp.blBoot())
      ROS_WARN("couldn't boot finger proximal phalange");
  if (use_distal_phalange)
    if (!finger.dp.blBoot())
      ROS_WARN("couldn't boot finger distal phalange");
  listenToFinger(&finger, 0.5);
  if (use_proximal_phalange)
    if (!finger.pp.ping())
      return perish("couldn't ping proximal phalange", &finger);
  if (use_distal_phalange)
    if (!finger.dp.ping())
      return perish("couldn't ping distal phalange", &finger);

  raw_finger_state_pub = 
      nh.advertise<sandia_hand_msgs::RawFingerState>("raw_state", 1);
  g_raw_finger_state_pub = &raw_finger_state_pub;
  // todo: some sort of auto-home sequence. for now, the fingers assume they 
  // were powered up in (0,0,0)
  listenToFinger(&finger, 0.001);
  finger.mm.registerRxHandler(MotorModule::PKT_FINGER_STATUS, rxFingerState);
  if (!finger.mm.setPhalangeAutopoll(true))
    return perish("couldn't start phalange autopoll", &finger);
  finger.mm.setJointPosHome();
  ros::Subscriber joint_commands_sub = 
    nh.subscribe<osrf_msgs::JointCommands>("joint_commands", 1, 
                             boost::bind(jointCommandsCallback, &finger, _1));
  ros::Subscriber relative_joint_commands_sub = 
    nh.subscribe<sandia_hand_msgs::RelativeJointCommands>
       ("relative_joint_commands", 1, 
        boost::bind(relativeJointCommandsCallback, &finger, _1));
  ros::Subscriber motor_commands_sub =
    nh.subscribe<sandia_hand_msgs::RawFingerCommands>("raw_commands", 1,
                             boost::bind(rawCommandsCallback, &finger, _1));

  ros::ServiceServer home_srv = 
    nh.advertiseService<sandia_hand_msgs::SetFingerHome::Request, 
                        sandia_hand_msgs::SetFingerHome::Response>
      ("set_finger_home", boost::bind(setHomeSrv, &finger, _1, _2));

  ros::ServiceServer param_dump_srv =
    nh.advertiseService<sandia_hand_msgs::GetParameters::Request,
                        sandia_hand_msgs::GetParameters::Response>
      ("get_parameters", boost::bind(getParametersSrv, &finger, _1, _2));

  ros::ServiceServer param_set_srv =
    nh.advertiseService<sandia_hand_msgs::SetParameters::Request,
                        sandia_hand_msgs::SetParameters::Response>
      ("set_parameters", boost::bind(setParametersSrv, &finger, _1, _2));

  listenToFinger(&finger, 0.001);
  ros::spinOnce();
  ros::Time t_prev_spin = ros::Time::now();
  for (int i = 0; !g_done; i++)
  {
    finger.listen(0.01);
    if ((ros::Time::now() - t_prev_spin).toSec() > 0.01)
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
      t_prev_spin = ros::Time::now();
      finger.mm.pollFingerState();
    }
  }
  shutdownFinger(&finger);
  return 0;
}

