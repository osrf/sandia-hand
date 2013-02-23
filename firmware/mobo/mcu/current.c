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
static void current_start_read_finger_sensor_reg(const uint8_t finger_idx, 
                                                 const uint8_t reg);
static enum { CURRENT_IDLE = 0, CURRENT_RX_WAIT = 1 } 
  g_current_state = CURRENT_IDLE;
static uint16_t g_current_rx_cnt = 0, g_current_rx_val = 0;
static uint8_t *g_current_rx_ptr = 0;

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
  if (g_current_state == CURRENT_RX_WAIT)
  {
#if 0
    while (rx_cnt > 0)
  {
    uint32_t status = TWI1->TWI_SR;
    if (status & TWI_SR_NACK)
    {
      printf("twi1 received nack\r\n");
      return 0;
    }
    if (rx_cnt == 1)
      TWI1->TWI_CR = TWI_CR_STOP;
    if (!(status & TWI_SR_RXRDY))
      continue; // busy-wait in this loop
    *rx_ptr++ = TWI1->TWI_RHR;
    rx_cnt--;
  }
    if (g_current_rx_cnt == 0)
    {
      //return __REV16(rx_val);
      g_current_state = CURRENT_IDLE;
    }
#endif 
  }
}

void current_systick()
{
  static int s_current_systick_count = 0;
  if (++s_current_systick_count % 1000 == 0)
    g_current_poll_req = 1;
}

void current_start_read_finger_sensor_reg(const uint8_t finger_idx, 
                                          const uint8_t reg_idx)
{
  uint8_t i2c_addr;
  // TODO: implement this....
  switch(finger_idx)
  {
    case 0: i2c_addr = 0x44; break; // see schematics & INA226 datasheet
    case 1: i2c_addr = 0x45; break;
    case 2: i2c_addr = 0x46; break;
    case 3: i2c_addr = 0x47; break;
    default: return; break; // bogus
  }
  TWI1->TWI_MMR = 0; // not sure why, but atmel library clears this first
  TWI1->TWI_MMR = TWI_MMR_MREAD | 
                  TWI_MMR_IADRSZ_1_BYTE |
                  TWI_MMR_DADR(i2c_addr);
  TWI1->TWI_IADR = reg_idx;
  TWI1->TWI_CR = TWI_CR_START;
  g_current_rx_val = 0;
  g_current_rx_cnt = 2;
  g_current_rx_ptr = (uint8_t *)&g_current_rx_val;
  g_current_state = CURRENT_RX_WAIT;
}

