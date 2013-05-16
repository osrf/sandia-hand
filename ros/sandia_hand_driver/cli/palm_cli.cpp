#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "sandia_hand/loose_right_palm.h"
#include "sandia_hand/palm_state.h"
#include "ros/time.h"
#include "ros/console.h"
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

void listen_palm(const float duration, LooseRightPalm &lrp)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    lrp.listen(0.01);
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

void rxPalmState(const uint8_t *payload, const uint16_t payload_len)
{
  palm_state_t *pst = (palm_state_t *)payload;
  printf("================================================================\n");
  printf("palm accel: %06d %06d %06d\n",
         pst->palm_accel[0], pst->palm_accel[1], pst->palm_accel[2]);
  printf("palm gyro:  %06d %06d %06d\n",
         pst->palm_gyro [0], pst->palm_gyro [1], pst->palm_gyro [2]);
  printf("palm mag:   %06d %06d %06d\n",
         pst->palm_mag  [0], pst->palm_mag  [1], pst->palm_mag  [2]);
  printf("palm tactile:\n");   
  for (int i = 0; i < 32; i++)
  {
    printf("%06d ", pst->palm_tactile[i]);
    if (i % 8 == 7)
      printf("\n");
  }
  printf("\n\n");
  /*
  printf("  motor module imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->fmcb_imu[i]);
  printf("\n  distal imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->dp_imu[i]);
  printf("\n  proximal imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->pp_imu[i]);
  printf("\n  proximal tactile: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->pp_tactile[i]);
  printf("\n\n");
  */
}


//////////////////////////////////////////////////////////////////////////////
// command handlers

int ping(int argc, char **argv, LooseRightPalm &lrp)
{
  if (lrp.ping())
    printf("   palm ping OK\n");
  else
    printf("   palm ping fail\n");
  return 0;
}

int stream(int argc, char **argv, LooseRightPalm &lrp)
{
  lrp.registerRxHandler(Palm::PKT_PALM_STATE,
                        boost::bind(rxPalmState, _1, _2));
  while (!g_done)
  {
    listen_palm(0.04, lrp);
    if (!lrp.pollState())
      ROS_ERROR("no palm state rx\n");
  }
  return 0;
}

int ver(int argc, char **argv, LooseRightPalm &lrp)
{
  const uint32_t hw_ver = (uint32_t)lrp.getHardwareVersion();
  printf("palm hardware version: %08x\n", hw_ver);
  return 0;
}

bool reset_palm(LooseRightPalm &lrp)
{
  lrp.reset();
  return true; // return true even if we couldn't ack the reset request
}

bool fake_set_power()
{
  return true;
}

int burn(int argc, char **argv, LooseRightPalm &lrp)
{
  verify_argc(argc, 3, "usage: burn PALM_BIN_FILE\n");
  const char *fn = argv[3];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open palm application image %s\n", fn);
    return 1;
  }
  if (!lrp.programAppFile(f, boost::bind(reset_palm, boost::ref(lrp)), 
                          fake_set_power))
  {
    printf("failed to program palm with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed palm with image %s\n", fn);
  return 0;
}

#if 0
  verify_argc(argc, 4, "usage: burn FILENAME");
  const char *fn = argv[3];
  printf("burning binary image %s to motor module...\n", fn);
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
  printf("resetting motor module...\n");
  lf.mm.reset();
  listen_loose_finger(2.0, lf);
  if (!lf.mm.blHaltAutoboot())
  {
    printf("couldn't halt motor module autoboot\n");
    return 1;
  }
  printf("distal autoboot halted\n");
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
    if (lf.mm.blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      break;
    }
  }
  if (lf.mm.blBoot())
    printf("booted motor module\n");
  else
    printf("failed to boot motor module\n");
  return 0;
}
#endif

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: palm_cli SERIAL_DEVICE COMMAND [OPTIONS]\n");
    return 1;
  }
  ros::Time::init();
  LooseRightPalm lrp;
  if (!lrp.init(argv[1]))
  {
    printf("couldn't init hand\n");
    return false;
  }
  signal(SIGINT, signal_handler);
  const char *cmd = argv[2];
  if (!strcmp(cmd, "ping"))
    return ping(argc, argv, lrp);
  if (!strcmp(cmd, "burn"))
    return burn(argc, argv, lrp);
  if (!strcmp(cmd, "ver"))
    return ver(argc, argv, lrp);
  if (!strcmp(cmd, "stream"))
    return stream(argc, argv, lrp);
  /*
  if (!strcmp(cmd, "status"))
    return status(argc, argv, lf);
  if (!strcmp(cmd, "pb"))
    return pb(argc, argv, lf);
  if (!strcmp(cmd, "pping"))
    return pping(argc, argv, lf);
  if (!strcmp(cmd, "pdump"))
    return pdump(argc, argv, lf);
  if (!strcmp(cmd, "pburn"))
    return pburn(argc, argv, lf);
  if (!strcmp(cmd, "dping"))
    return dping(argc, argv, lf);
  if (!strcmp(cmd, "ddump"))
    return ddump(argc, argv, lf);
  if (!strcmp(cmd, "dburn"))
    return dburn(argc, argv, lf);
  if (!strcmp(cmd, "dump"))
    return dump(argc, argv, lf);
  if (!strcmp(cmd, "jp"))
    return jp(argc, argv, lf);
  */
  printf("unknown command: [%s]\n", cmd);
  return 1;
}

