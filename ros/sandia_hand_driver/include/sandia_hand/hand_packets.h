#ifndef HAND_PACKETS_H
#define HAND_PACKETS_H

#include <stdint.h>

// this file is compiled into both the C++ driver library and the C firmware
static const uint32_t CMD_ID_SET_FINGER_POWER_STATE      =  1;
static const uint32_t CMD_ID_SET_FINGER_CONTROL_MODE     =  2;
static const uint32_t CMD_ID_SET_FINGER_JOINT_POS        =  3;
static const uint32_t CMD_ID_CONFIGURE_CAMERA_STREAM     =  4;
static const uint32_t CMD_ID_FINGER_RAW_TX               =  5; // pass-through
static const uint32_t CMD_ID_SET_MOBO_STATUS_RATE        =  6;
static const uint32_t CMD_ID_MOBO_STATUS                 =  7;
static const uint32_t CMD_ID_SET_FINGER_AUTOPOLL         =  8;
static const uint32_t CMD_ID_SET_ALL_FINGER_POWER_STATES =  9;
static const uint32_t CMD_ID_ENABLE_LOWVOLT_REGULATOR    = 10;
static const uint32_t CMD_ID_READ_FPGA_FLASH_PAGE        = 11;
static const uint32_t CMD_ID_FPGA_FLASH_PAGE             = 12;

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
  uint8_t fps[4];
} __attribute__((packed)) set_all_finger_power_states_t;

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

typedef struct
{
  uint8_t cam_0_stream;
  uint8_t cam_1_stream;
} __attribute__((packed)) configure_camera_stream_t;
static const uint8_t CAMERA_STREAM_OFF = 0;
static const uint8_t CAMERA_STREAM_ON  = 1;

#define FINGER_RAW_TX_MAX_LEN 500
typedef struct
{
  uint8_t finger_idx;
  uint8_t pad;
  uint16_t tx_data_len;
  uint8_t tx_data[FINGER_RAW_TX_MAX_LEN];
} __attribute__((packed)) finger_raw_tx_t;

typedef struct
{
  uint8_t mobo_status_hz;
} __attribute__((packed)) set_mobo_status_rate_t;

typedef struct
{
  uint32_t mobo_time_ms;
  float finger_currents[4];
  float logic_currents[3];
  uint16_t mobo_raw_temperatures[3];
} __attribute__((packed)) mobo_status_t;

typedef struct
{
  uint16_t finger_autopoll_hz;
} __attribute__((packed)) set_finger_autopoll_t;

typedef struct
{
  uint8_t enable;
} __attribute__((packed)) enable_lowvolt_regulator_t;

typedef struct
{
  uint32_t page_num;
} __attribute__((packed)) read_fpga_flash_page_t;

typedef struct
{
  uint32_t page_num;
  uint8_t page_data[256];
} __attribute__((packed)) fpga_flash_page_t;

#endif
