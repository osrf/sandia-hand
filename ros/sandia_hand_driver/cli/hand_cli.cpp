#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include "hand.h"
using namespace sandia_hand;

int b2e(bool b) // convert boolean function return values to exit codes
{
  return b ? 0 : 1;
}

bool parse_finger_idx(uint8_t &finger_idx, const char *s)
{
  finger_idx = atoi(s);
  if (finger_idx >= 4)
  {
    printf("finger_idx must be in {0,1,2,3}\n");
    return false;
  }
  return true;
}

bool verify_argc(const int argc, const int min_argc, const char *usage_text)
{
  if (argc < min_argc)
  {
    printf("%s\n", usage_text);
    exit(1);
    return false; // never gets here... but feels good to write it still
  }
  return true;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: hand_cli COMMAND [OPTIONS]\n");
    return 1;
  }
  Hand hand;
  if (!hand.init())
  {
    printf("couldn't init hand\n");
    return false;
  }
  const char *cmd = argv[1];
  uint8_t finger_idx = 0;
  if (!strcmp(cmd, "p")) // power of all finger sockets
  {
    verify_argc(argc, 3, "usage: hand_cli p POWER_STATE\n"
                         "  where POWER_STATE = {off, low, on}\n");
    const char *fp_str = argv[2];
    Hand::FingerPowerState fps;
    if (!strcmp(fp_str, "off"))
      fps = Hand::FPS_OFF;
    else if (!strcmp(fp_str, "low"))
      fps = Hand::FPS_LOW;
    else if (!strcmp(fp_str, "on"))
      fps = Hand::FPS_FULL;
    else
    {
      printf("unrecognized power state [%s]\n", fp_str);
      printf("power_state must be in {off, low, on}\n");
      return 1;
    }
    for (int i = 0; i < 4; i++)
    {
      hand.setFingerPower(i, fps);
      usleep(250000); // wait 250ms so we don't thrash power supply too much
    }
  }
  else if (!strcmp(cmd, "fp")) // set socket power for a single finger
  {
    verify_argc(argc, 4, "usage: hand_cli fp FINGER_IDX POWER_STATE\n"
                         "  where POWER_STATE = {off, low, on}\n");
    if (!parse_finger_idx(finger_idx, argv[2]))
      return 1;
    const char *fp_str = argv[3];
    Hand::FingerPowerState fps;
    if (!strcmp(fp_str, "off"))
      fps = Hand::FPS_OFF;
    else if (!strcmp(fp_str, "low"))
      fps = Hand::FPS_LOW;
    else if (!strcmp(fp_str, "on"))
      fps = Hand::FPS_FULL;
    else
    {
      printf("unrecognized power state [%s]\n", fp_str);
      printf("power_state must be in {off, low, on}\n");
      return 1;
    }
    hand.setFingerPower(finger_idx, fps);
  }
  else if (!strcmp(cmd, "fcm")) // set finger control mode
  {
    verify_argc(argc, 4, "usage: hand_cli fcm FINGER_IDX CONTROL_MODE\n"
                         "  where CONTROL_MODE = {idle, joint_pos}\n");
    if (!parse_finger_idx(finger_idx, argv[2]))
      return 1;
    const char *fcm_str = argv[3];
    Hand::FingerControlMode fcm;
    if (!strcmp(fcm_str, "idle"))
      fcm = Hand::FCM_IDLE;
    else if (!strcmp(fcm_str, "joint_pos"))
      fcm = Hand::FCM_JOINT_POS;
    else
    {
      printf("unrecognized finger control state [%s]\n", fcm_str);
      printf("finger control state must be in {idle, joint_pos}\n");
      return 1;
    }
    hand.setFingerControlMode(finger_idx, fcm);
  }
  else if (!strcmp(cmd, "jp")) // set joint position
  {
    verify_argc(argc, 6, "usage: hand_cli jp FINGER_IDX J0 J1 J2\n");
    if (!parse_finger_idx(finger_idx, argv[2]))
      return 1;
    float j0 = atof(argv[3]), j1 = atof(argv[4]), j2 = atof(argv[5]);
    hand.setFingerJointPos(finger_idx, j0, j1, j2);
  }
  else
  {
    printf("unknown command: [%s]\n", cmd);
    return 1;
  }

  return 0;
}

