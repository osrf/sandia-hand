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

#include "power.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>

// hardware connections:
//   PA21 = F0_LV
//   PC19 = F0_HV
//   PB19 = F1_LV
//   PB17 = F1_HV
//   PA2  = F2_LV
//   PA4  = F2_HV
//   PA6  = F3_LV
//   PA16 = F3_HV
//   PB16 = 9V_EN  needs to be turned on for any of the low-volt rails to work

#define POWER_NUM_FINGERS 4

typedef struct { Pio *pio; uint32_t pin_idx; } power_switch_t;
const power_switch_t power_switches[POWER_NUM_FINGERS][2] =  
  { { { PIOB, PIO_PB19 }, { PIOB, PIO_PB17 } },   // index
    { { PIOA, PIO_PA2  }, { PIOA, PIO_PA4  } },   // middle
    { { PIOA, PIO_PA6  }, { PIOA, PIO_PA16 } },   // pinkie
    { { PIOA, PIO_PA21 }, { PIOC, PIO_PC19 } } }; // thumb

uint8_t g_power_poll_req = 0;
static void power_start_read_finger_sensor_reg(const uint8_t finger_idx, 
                                               const uint8_t reg);
static enum { POWER_IDLE = 0, POWER_RX_WAIT = 1 } 
  g_power_state = POWER_IDLE;
static uint16_t g_power_rx_cnt = 0, g_power_rx_val = 0;
static uint8_t *g_power_rx_ptr = 0;



void power_init()
{
  printf("power init\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                    (1 << ID_TWI1);
  const uint32_t pioa_pins = PIO_PA2 | PIO_PA4 | PIO_PA6 | PIO_PA16 | PIO_PA21;
  const uint32_t piob_pins = PIO_PB17 | PIO_PB19;
  const uint32_t pioc_pins = PIO_PC19;
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_CODR = pioa_pins;
  PIOB->PIO_PER = PIOB->PIO_OER = PIOB->PIO_CODR = piob_pins | PIO_PB16;
  PIOC->PIO_PER = PIOC->PIO_OER = PIOC->PIO_CODR = pioc_pins;
  // turn on 9v rail
  PIOB->PIO_SODR = PIO_PB16;
  // set up TWI bus to talk to current sensors
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

void power_set(const uint8_t finger_idx, const power_state_t power_state)
{
  if (finger_idx >= POWER_NUM_FINGERS) return; // get that out
  const power_switch_t *sw = power_switches[finger_idx];
  /*
  printf("setting finger %d to power state %d\r\n", 
         finger_idx, (uint8_t)power_state);
  printf("sw[1].pio = %08x\r\n", (uint32_t)sw[1].pio);
  printf("sw[1].pin_idx = %08x\r\n", sw[1].pin_idx);
  printf("PIOB = %08x\r\n", (uint32_t)PIOB);
  */
  switch(power_state)
  {
    case POWER_OFF:
      sw[0].pio->PIO_CODR = sw[0].pin_idx; // low voltage off
      sw[1].pio->PIO_CODR = sw[1].pin_idx; // high voltage off
      break;
    case POWER_LOW:
      sw[0].pio->PIO_SODR = sw[0].pin_idx; // low voltage on
      sw[1].pio->PIO_CODR = sw[1].pin_idx; // high voltage off
      break;
    case POWER_ON: // need to wait a bit for high voltage to ramp up? not sure
      sw[1].pio->PIO_SODR = sw[1].pin_idx; // high voltage on
      sw[0].pio->PIO_CODR = sw[0].pin_idx; // low voltage off
      break;
  }
}

void power_idle()
{
  if (g_power_poll_req)
  {
    g_power_poll_req = 0;
    //printf("cpoll\r\n");
  }
  if (g_power_state == POWER_RX_WAIT)
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

void power_systick()
{
  static int s_power_systick_count = 0;
  if (++s_power_systick_count % 1000 == 0)
    g_power_poll_req = 1;
}

void power_start_read_finger_sensor_reg(const uint8_t finger_idx, 
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
  g_power_rx_val = 0;
  g_power_rx_cnt = 2;
  g_power_rx_ptr = (uint8_t *)&g_power_rx_val;
  g_power_state = POWER_RX_WAIT;
}

