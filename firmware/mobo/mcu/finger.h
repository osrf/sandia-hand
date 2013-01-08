#ifndef FINGER_H
#define FINGER_H

void finger_init();
void finger_tx_raw(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len);
void finger_set_control_mode(uint8_t finger_idx, uint8_t control_mode);
void finger_set_joint_pos(uint8_t finger_idx, float j0, float j1, float j2);

#define FINGER_INDEX  0
#define FINGER_MIDDLE 1
#define FINGER_PINKIE 2
#define FINGER_THUMB  3

#endif

