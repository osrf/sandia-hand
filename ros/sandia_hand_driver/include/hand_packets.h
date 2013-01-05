#ifndef HAND_PACKETS_H
#define HAND_PACKETS_H

// this file is compiled into both the C++ driver library and the C firmware
static const uint32_t CMD_ID_SET_FINGER_POWER_STATE = 1;

typedef struct 
{
  uint8_t finger_idx;
  uint8_t finger_power_state;
} __attribute__((packed)) set_finger_power_state_t;
static const uint8_t FINGER_POWER_STATE_OFF  = 0;
static const uint8_t FINGER_POWER_STATE_LOW  = 1;
static const uint8_t FINGER_POWER_STATE_FULL = 2;

#endif

