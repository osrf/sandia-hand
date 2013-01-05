#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "hand.h"
using namespace sandia_hand;

int b2e(bool b) // convert boolean function return values to exit codes
{
  return b ? 0 : 1;
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
  if (!strcmp(cmd, "fp")) // set finger socket power 
  {
    if (argc < 4)
    {
      printf("usage: hand_cli fp FINGER_IDX POWER_STATE\n"
             "  where POWER_STATE = {off, low, on}\n");
      return 1;
    }
    uint8_t finger_idx = atoi(argv[2]);
    if (finger_idx >= 4)
    {
      printf("finger_idx must be in {0,1,2,3}\n");
      return 1;
    }
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
  return 0;
}

