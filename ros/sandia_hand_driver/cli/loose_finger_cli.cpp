#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <boost/function.hpp>
#include "sandia_hand/loose_finger.h"
#include "ros/time.h"
using namespace sandia_hand;

static bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

//////////////////////////////////////////////////////////////////////////////
// some helper functions to condense everything....

int b2e(bool b) // convert boolean function return values to exit codes
{
  return b ? 0 : 1;
}

void listen_loose_finger(const float duration, LooseFinger &lf)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    lf.listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > duration)
      break;
  }
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

//////////////////////////////////////////////////////////////////////////////
// command handlers

int ping(int argc, char **argv, LooseFinger &lf)
{
  if (lf.mm.ping())
    printf("   motor module ping OK\n");
  else
    printf("   motor module ping fail\n");
  return 0;
}

int status(int argc, char **argv, LooseFinger &lf)
{
  if (lf.mm.pollFingerStatus())
    printf("finger status poll ok\n");
  else
    printf("finger status poll fail\n");
  return 0;
}

int pb(int argc, char **argv, LooseFinger &lf)
{
  const char *usage = "usage: loose_finger_cli SERIAL_DEV pb { ON | OFF }\n";
  verify_argc(argc, 4, usage);
  const char *s = argv[3];
  if (!strcmp(s, "on"))
    lf.mm.setPhalangeBusPower(true);
  else if (!strcmp(s, "off"))
    lf.mm.setPhalangeBusPower(false);
  else
    printf("%s\n", usage);
  return 0;
}

int stream(int argc, char **argv, LooseFinger &lf)
{
  if (!lf.mm.setPhalangeAutopoll(true))
  {
    printf("couldn't start phalange autopoll\n");
    return 1;
  }
  while (!g_done)
  {
    listen_loose_finger(0.01, lf);
    lf.mm.pollFingerStatus();
  }
  lf.mm.setPhalangeAutopoll(false);
  return 0;
}

int pping(int argc, char **argv, LooseFinger &lf)
{
  if (lf.pp.ping())
    printf("proximal phalange ping ok\n");
  else
    printf("proximal phalange ping fail\n");
  return 0;
}

int pdump(int argc, char **argv, LooseFinger &lf)
{
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (lf.pp.blHaltAutoboot())
    printf("autoboot halted\n");
  else
    printf("couldn't halt autoboot\n");
  FILE *f = fopen("image.bin", "wb");
  for (int page_num = 0; page_num < 16; page_num++)
  {
    bool page_read = false;
    for (int attempt = 0; !page_read && attempt < 10; attempt++)
    {
      printf("reading page %d attempt %d...\n", page_num, attempt);
      uint8_t page_buf[1024] = {0};
      if (lf.pp.blReadFlashPage(page_num, page_buf))
      {
        printf("read page 32:\n");
        page_read = true;
        if (256 != fwrite(page_buf, 1, 256, f))
        {
          printf("weird fwrite result\n");
        }
      }
      /*
      for (int i = 0; i < 256; i++)
      {
        printf("%02x  ", page_buf[i]);
        if (i % 8 == 7)
          printf("\n");
      }
      printf("\n");
      */
    }
    if (!page_read)
    {
      printf("couldn't read page %d\n", page_num);
      break;
    }
  }
  if (lf.pp.blBoot())
    printf("booted proximal phalange\n");
  else
    printf("failed to boot proximal phalange\n");
  return 0;
}

int pburn(int argc, char **argv, LooseFinger &lf)
{
  verify_argc(argc, 4, "usage: pburn FILENAME");
  const char *fn = argv[3];
  printf("burning binary image %s to proximal phalange...\n", fn);
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open file %s\n", fn);
    return 1;
  }
  if (fseek(f, 32 * 256, SEEK_SET))
  {
    printf("couldn't seek to application image in %s\n", fn);
    return 1;
  }
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (!lf.pp.blHaltAutoboot())
  {
    printf("couldn't halt autoboot\n");
    return 1;
  }
  printf("autoboot halted\n");
  for (int page_num = 32; !g_done && !feof(f) && page_num < 256; page_num++)
  {
    bool page_written = false;
    uint8_t page_buf[1024] = {0};
    size_t nread = 0;
    nread = fread(page_buf, 1, 256, f);
    if (nread <= 0)
    {
      printf("couldn't read a flash page from %s: returned %d\n", 
             fn, (int)nread);
      break;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n", 
             (int)nread);
    if (lf.pp.blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      break;
    }
  }
  if (lf.pp.blBoot())
    printf("booted proximal phalange\n");
  else
    printf("failed to boot proximal phalange\n");
  return 0;
}

/*
int set_finger_control_mode(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: hand_cli fcm FINGER_IDX CONTROL_MODE\n"
                       "  where CONTROL_MODE = {idle, joint_pos}\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
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
  return 0;
}

int set_joint_position(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 6, "usage: hand_cli jp FINGER_IDX J0 J1 J2\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  float j0 = atof(argv[3]), j1 = atof(argv[4]), j2 = atof(argv[5]);
  hand.setFingerJointPos(finger_idx, j0, j1, j2);
  return 0;
}

int test_finger_stream(int argc, char **argv, Hand &hand)
{
  hand.registerRxHandler(CMD_ID_MOBO_STATUS, mobo_status_rx);
  //printf("turning on mobo status streaming...\n");
  //hand.setMoboStatusHz(100);
  //listen_hand(1.0, hand);
  printf("powering finger sockets...\n");
  hand.setAllFingerPowers(Hand::FPS_LOW);
  listen_hand(0.5, hand);
  hand.setAllFingerPowers(Hand::FPS_FULL);
  listen_hand(4.0, hand);
  printf("turning on phalange bus...\n");
  hand.fingers[0].mm.setPhalangeBusPower(true);
  listen_hand(4.0, hand);
  hand.fingers[0].mm.setPhalangeAutopoll(true);

  printf("turning on finger streaming...\n");
  hand.setFingerAutopollHz(1);
  while (!g_done)
    listen_hand(0.1, hand);
  printf("turning off finger power...\n");
  hand.setAllFingerPowers(Hand::FPS_OFF);
  //hand.setMoboStatusHz(0);
  hand.setFingerAutopollHz(0);
  usleep(200000);
  printf("bye\n");
  return 0;
}
*/

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: loose_finger_cli SERIAL_DEVICE COMMAND [OPTIONS]\n");
    return 1;
  }
  ros::Time::init();
  LooseFinger lf;
  if (!lf.init(argv[1]))
  {
    printf("couldn't init hand\n");
    return false;
  }
  signal(SIGINT, signal_handler);
  const char *cmd = argv[2];
  if (!strcmp(cmd, "ping"))
    return ping(argc, argv, lf);
  if (!strcmp(cmd, "status"))
    return status(argc, argv, lf);
  if (!strcmp(cmd, "stream"))
    return stream(argc, argv, lf);
  if (!strcmp(cmd, "pb"))
    return pb(argc, argv, lf);
  if (!strcmp(cmd, "pping"))
    return pping(argc, argv, lf);
  if (!strcmp(cmd, "pdump"))
    return pdump(argc, argv, lf);
  if (!strcmp(cmd, "pburn"))
    return pburn(argc, argv, lf);
  printf("unknown command: [%s]\n", cmd);
  return 1;
}

