#include <cstdio>
#include "sandia_hand/motor_module.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

MotorModule::MotorModule(const uint8_t addr)
: SerialMessageProcessor(addr)
{
  registerRxHandler(PKT_FINGER_STATUS, 
                    boost::bind(&MotorModule::rxFingerStatus, this, _1, _2));
}

MotorModule::~MotorModule()
{
}

bool MotorModule::setPhalangeBusPower(bool on)
{
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_POWER, 1))
    return false;
  return listenFor(PKT_PHALANGE_POWER, 0.5);
}

bool MotorModule::setPhalangeAutopoll(bool on)
{
  // this doesn't do anything yet :(
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_AUTOPOLL, 1))
    return false;
  return listenFor(PKT_PHALANGE_AUTOPOLL, 0.5);
}

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

static void print_uint16_array(const char *name, 
                               const uint16_t *p, const uint16_t len)
{
  printf("%s:\n  ", name);
  for (int i = 0; i < len; i++)
    printf("%06d ", p[i]);
  printf("\n");
}

void MotorModule::rxFingerStatus(const uint8_t *payload, 
                                 const uint16_t payload_len)
{
  /*
  printf("rx finger status %d bytes\n  ", payload_len);
  for (int i = 0; i < payload_len; i++)
    printf("%02x ", payload[i]);
  printf("\n");
  */
  finger_status_t *p = (finger_status_t *)payload;
  print_uint16_array("pp tactile", p->pp_tactile,  6);
  print_uint16_array("dp tactile", p->dp_tactile, 12);
  print_uint16_array("pp imu"    , p->pp_imu    ,  6);
  print_uint16_array("dp imu"    , p->dp_imu    ,  6);
  print_uint16_array("fmcb imu"  , p->fmcb_imu  ,  6);
  print_uint16_array("pp temp"   , p->pp_temp   ,  4);
  print_uint16_array("dp temp"   , p->dp_temp   ,  4);
  print_uint16_array("fmcb temp" , p->fmcb_temp ,  3);
  const float fmcb_v_adc = (float)p->fmcb_voltage / 4095.0f * 3.3f;
  const float fmcb_v_cal = (100.0f + 10.0f) / 10.0f * fmcb_v_adc;
  printf("fmcb voltage: %d -> %.4f volts\n", p->fmcb_voltage, fmcb_v_cal);
  float cal_phalange_current = (float)p->fmcb_pb_current/4095.0f*3.3f/2.7;
  printf("fmcb phalange current: %d -> %.4f amps\n", 
         p->fmcb_pb_current, cal_phalange_current);
  printf("pp strain: %d\n", p->pp_strain);
  printf("hall targets: %d %d %d\n", 
         p->fmcb_hall_tgt[0], p->fmcb_hall_tgt[1], p->fmcb_hall_tgt[2]);
  printf("hall positions: %d %d %d\n", 
         p->fmcb_hall_pos[0], p->fmcb_hall_pos[1], p->fmcb_hall_pos[2]);
  printf("fmcb time: %d\n", p->fmcb_time);
  printf("\n");
}

bool MotorModule::pollFingerStatus()
{
  if (!sendTxBuffer(PKT_FINGER_STATUS, 0))
    return false;
  return listenFor(PKT_FINGER_STATUS, 0.5);
}

