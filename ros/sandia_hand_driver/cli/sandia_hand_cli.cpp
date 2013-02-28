#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <boost/function.hpp>
#include "sandia_hand/hand.h"
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


void parse_finger_idx(uint8_t &finger_idx, const char *s)
{
  finger_idx = atoi(s);
  if (finger_idx >= 4)
  {
    printf("finger_idx must be in {0,1,2,3}\n");
    exit(1);
  }
}

void listen_hand(const float duration, Hand &hand)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    hand.listen(0.01);
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

int set_all_finger_powers(int argc, char **argv, Hand &hand)
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
  return 0;
}

int finger_ping(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: fping FINGER_IDX\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  printf("pinging finger %d:\n", finger_idx);
  printf("  motor module: ");
  if (hand.fingers[finger_idx].mm.ping())
    printf("   OK\n");
  else
    printf("   fail\n");
  listen_hand(1.0, hand);
  return 0;
}

int set_single_finger_power(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: hand_cli fp FINGER_IDX POWER_STATE\n"
                       "  where POWER_STATE = {off, low, on}\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
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
  return 0;
}

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

void cam_pgm_cb(const uint8_t cam_idx, const uint32_t frame_count, 
                const uint8_t *img_data)
{
  printf("cam_pgm_cb\n");
  FILE *f = NULL;
  char fname_buf[100];
  snprintf(fname_buf, sizeof(fname_buf), "cam_%d_img_%06d.pgm", 
           cam_idx, frame_count);
  f = fopen(fname_buf, "w");
  fprintf(f, "P5\n%d %d\n255\n", Hand::IMG_WIDTH, Hand::IMG_HEIGHT);
  fwrite(img_data, 1, Hand::IMG_WIDTH * Hand::IMG_HEIGHT, f);
  g_done = true; // got the image. bail now from spin loop
  printf("wrote %s\n", fname_buf);
  fclose(f);
}

int cam_pgm(int argc, char **argv, Hand &hand)
{
  printf("saving one pgm image from the hand...\n");
  hand.setImageCallback(&cam_pgm_cb); 
  hand.setCameraStreaming(true, true);
  ros::Time t_start(ros::Time::now());
  while (!g_done && (ros::Time::now() - t_start).toSec() < 10)
    if (!hand.listen(1.0))
      break;
  hand.setCameraStreaming(false, false);
  return 0;
}

int test_finger_currents(int argc, char **argv, Hand &hand)
{
  printf("testing finger currents during boot cycle...\n");
  hand.setMoboStatusHz(1);
  ros::Time t_start(ros::Time::now());
  bool finger_states[4] = {false, false, false, false};
  int next_finger_powerup = 0;
  while (!g_done)
  {
    hand.listen(1.0);
    if (((ros::Time::now() - t_start).toSec() > next_finger_powerup * 5) &&
        !finger_states[next_finger_powerup])
    {
      // turn on a finger to observe current ramp
    }
  }
  hand.setMoboStatusHz(0);
  return 0;
}

void mobo_status_rx(const uint8_t *data, const uint16_t data_len)
{
  static FILE *f_log = NULL;
  if (!f_log)
    f_log = fopen("current_log.txt", "w");
  //printf("mobo status\n");
  mobo_status_t *p = (mobo_status_t *)(data + 4);
  fprintf(f_log, "%d ", p->mobo_time_ms);
  for (int i = 0; i < 4; i++)
  {
    //printf("  %d current: %.4f\n", i, p->finger_currents[i]);
    fprintf(f_log, "%.6f ", p->finger_currents[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    //printf("  logic %d current: %.4f\n", i, p->logic_currents[i]);
    fprintf(f_log, "%.6f ", p->logic_currents[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    //printf("  %d raw temperature: %d\n", i, p->mobo_raw_temperatures[i]);
    fprintf(f_log, "%d ", (int16_t)p->mobo_raw_temperatures[i]);
  }
  fprintf(f_log, "\n");
}

int test_finger_stream(int argc, char **argv, Hand &hand)
{
  hand.registerRxHandler(CMD_ID_MOBO_STATUS, mobo_status_rx);
  printf("turning on mobo status streaming...\n");
  hand.setMoboStatusHz(100);
  listen_hand(1.0, hand);
  printf("powering finger sockets...\n");
  hand.setAllFingerPowers(Hand::FPS_LOW);
  listen_hand(0.5, hand);
  hand.setAllFingerPowers(Hand::FPS_FULL);
  listen_hand(4.0, hand);
  printf("turning on finger streaming...\n");
  hand.setFingerAutopollHz(1);
  while (!g_done)
    listen_hand(0.1, hand);
  printf("turning off finger power...\n");
  hand.setAllFingerPowers(Hand::FPS_OFF);
  hand.setMoboStatusHz(0);
  hand.setFingerAutopollHz(0);
  usleep(200000);
  printf("bye\n");
  return 0;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: hand_cli COMMAND [OPTIONS]\n");
    return 1;
  }
  ros::Time::init();
  Hand hand;
  if (!hand.init())
  {
    printf("couldn't init hand\n");
    return false;
  }
  signal(SIGINT, signal_handler);
  const char *cmd = argv[1];
  if (!strcmp(cmd, "p")) 
    return set_all_finger_powers(argc, argv, hand);
  else if (!strcmp(cmd, "fp")) 
    return set_single_finger_power(argc, argv, hand);
  else if (!strcmp(cmd, "fcm")) 
    return set_finger_control_mode(argc, argv, hand);
  else if (!strcmp(cmd, "jp")) // set joint position
    return set_joint_position(argc, argv, hand);
  else if (!strcmp(cmd, "cam_pgm"))
    return cam_pgm(argc, argv, hand);
  else if (!strcmp(cmd, "fping"))
    return finger_ping(argc, argv, hand);
  else if (!strcmp(cmd, "test_finger_currents"))
    return test_finger_currents(argc, argv, hand);
  else if (!strcmp(cmd, "test_finger_stream"))
    return test_finger_stream(argc, argv, hand);
  printf("unknown command: [%s]\n", cmd);
  return 1;
}

