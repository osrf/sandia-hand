#ifndef FINGER_H
#define FINGER_H

void finger_init();
void finger_set_control_mode(uint8_t finger_idx, uint8_t control_mode);
void finger_tx_raw(uint8_t finger_idx, uint8_t *data, uint16_t data_len);

#endif

