#ifndef HAND_PACKETS_H
#define HAND_PACKETS_H

#include <stdint.h>

// this file is compiled into both the C++ driver library and the C firmware
static const uint32_t CMD_ID_SET_FINGER_POWER_STATE  = 1;
static const uint32_t CMD_ID_SET_FINGER_CONTROL_MODE = 2;
static const uint32_t CMD_ID_SET_FINGER_JOINT_POS    = 3;

typedef struct 
{
  uint8_t finger_idx;
  uint8_t finger_power_state;
} __attribute__((packed)) set_finger_power_state_t;
static const uint8_t FINGER_POWER_STATE_OFF  = 0;
static const uint8_t FINGER_POWER_STATE_LOW  = 1;
static const uint8_t FINGER_POWER_STATE_FULL = 2;

typedef struct
{
  uint8_t finger_idx;
  uint8_t finger_control_mode;
} __attribute__((packed)) set_finger_control_mode_t;
static const uint8_t FINGER_CONTROL_MODE_IDLE      = 0;
static const uint8_t FINGER_CONTROL_MODE_JOINT_POS = 1;

typedef struct
{
  uint8_t finger_idx;
  uint8_t pad_0; // pad 3 bytes so the floats are aligned
  uint8_t pad_1;
  uint8_t pad_2;
  float joint_0_radians;
  float joint_1_radians;
  float joint_2_radians;
} __attribute__((packed)) set_finger_joint_pos_t;

#endif

