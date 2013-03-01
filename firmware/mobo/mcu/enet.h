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

#ifndef MOBO_MCU_ENET_H
#define MOBO_MCU_ENET_H

#include <stdint.h>

void enet_init();
void enet_vector();
void enet_tx_raw(const uint8_t *pkt, uint16_t pkt_len);
uint8_t enet_tx_avail();
void enet_idle(); // handle non time-critical ethernet tasks
void enet_systick();
void enet_tx_udp(const uint8_t *payload, const uint16_t payload_len);

#endif

