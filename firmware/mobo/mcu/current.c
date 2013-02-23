/*  Software License Agreement (Apache License)
 *
 *  Copyright 2013 Open Source Robotics Foundation
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

#include "current.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>

uint8_t g_current_poll_req = 0;
static uint16_t current_read_finger_sensor_reg(const uint8_t finger_idx, 
                                               const uint8_t reg);

void current_init()
{
  printf("current init\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOB) | (1 << ID_TWI1);
  PIOB->PIO_ABSR &= ~(PIO_PB12A_TWD1 | PIO_PB13A_TWCK1);
  PIOB->PIO_PDR = PIO_PB12A_TWD1 | PIO_PB13A_TWCK1;
  TWI1->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVDIS; // disable twi for the moment
  TWI1->TWI_IDR = 0xffffffff; // no interrupts plz
  TWI1->TWI_SR; // read status register (why? atmel library does it...)
  TWI1->TWI_CR = TWI_CR_SWRST; // sotware reset
  TWI1->TWI_RHR; // empty receive register
  TWI1->TWI_CR = TWI_CR_MSEN; // enable master mode
  TWI1->TWI_CWGR = TWI_CWGR_CLDIV(77) | TWI_CWGR_CHDIV(77) |  // 50% duty cycle
                   TWI_CWGR_CKDIV(3); // 400 khz i2c / 2^3 = 50 (weak pullups)
}

void current_idle()
{
  if (g_current_poll_req)
  {
    g_current_poll_req = 0;
    printf("cpoll\r\n");
  }
}

void current_systick()
{
  static int s_current_systick_count = 0;
  if (++s_current_systick_count % 1000 == 0)
    g_current_poll_req = 1;
}

uint16_t current_read_finger_sensor_reg(uint8_t finger_idx, uint8_t reg)
{
  uint8_t i2c_addr;
  // TODO: implement this....
}

