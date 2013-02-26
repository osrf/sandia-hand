/*  Software License Agreement (Apache License)
 *
 *  Copyright 2012 Open Source Robotics Foundation
 *  Author: Morgan Quigley
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef FINGER_H
#define FINGER_H

void finger_init();
void finger_tx_raw(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len);
void finger_set_control_mode(uint8_t finger_idx, uint8_t control_mode);
void finger_set_joint_pos(uint8_t finger_idx, float j0, float j1, float j2);
void finger_systick();
void finger_idle();
extern volatile uint8_t g_finger_autopoll_timeout;

#define FINGER_INDEX  0
#define FINGER_MIDDLE 1
#define FINGER_PINKIE 2
#define FINGER_THUMB  3

#endif

