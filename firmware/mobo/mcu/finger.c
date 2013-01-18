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

#include "hand_packets.h"
#include "finger.h"
#include <stdio.h>
#include "common_sam3x/sam3x.h"

// hardware connections:
// PA15 = RS485_SEL
// PA19 = MCU_RS485_MUX_A0
// PC7  = MCU_RS485_MUX_A1
// PB27 = MCU_RS485_MUX_A2
// PC28 = MCU_RS485_0_DE
// PD6  = MCU_RS485_1_DE
// PA0  = MCU_RS485_2_DE  
// PB14 = MCU_RS485_3_DE
// PD0  = MCU_RS485_4_DE
// PD4  = TXD3 = MCU_MUXED_DI
// PD5  = RXD3 = MCU_MUXED_RO

typedef struct { Pio *pio; uint32_t pin_idx; } rs485_de_t;
// TODO: map finger_idx to rs485 channel indices. have palm be channel 4.
/*
static const rs485_de_t g_finger_rs485_de[5] =
  { { PIOC, PIO_PC28 },
    { PIOD, PIO_PD6  },
    { PIOA, PIO_PA0  },
    { PIOB, PIO_PB14 },
    { PIOD, PIO_PD0  } };
*/
static const rs485_de_t g_finger_rs485_de[5] =
  { { PIOD, PIO_PD0  },
    { PIOD, PIO_PD6  },
    { PIOA, PIO_PA0  },
    { PIOB, PIO_PB14 },
    { PIOC, PIO_PC28 } };

void finger_init()
{
  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                    (1 << ID_PIOD) | (1 << ID_USART3);
  const uint32_t pioa_pins = PIO_PA15 | PIO_PA19 | PIO_PA0;
  const uint32_t piob_pins = PIO_PB14 | PIO_PB27;
  const uint32_t pioc_pins = PIO_PC7 | PIO_PC28;
  const uint32_t piod_pins = PIO_PD6 | PIO_PD0; 
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_CODR = pioa_pins;
  PIOB->PIO_PER = PIOB->PIO_OER = PIOB->PIO_CODR = piob_pins;
  PIOC->PIO_PER = PIOC->PIO_OER = PIOC->PIO_CODR = pioc_pins;
  PIOD->PIO_PER = PIOD->PIO_OER = PIOD->PIO_CODR = piod_pins;
  PIOD->PIO_PDR = PIO_PD4 | PIO_PD5; // enable peripheral control of pd4, pd5
  PIOD->PIO_ABSR |= PIO_PD4B_TXD3 | PIO_PD5B_RXD3; // select peripheral B
  // assert PA15 (RS485_SEL) so the ARM maintains control of rs485 channels
  PIOA->PIO_SODR = PIO_PA15;
  // configure USART3 for 2 megabit
  USART3->US_CR = US_CR_RSTRX | US_CR_RSTTX |
                  US_CR_RXDIS | US_CR_TXDIS |
                  US_CR_RSTSTA;
  USART3->US_MR = US_MR_USART_MODE_NORMAL  | // regular UART, nothing fancy
                  US_MR_USCLKS_MCK         | // use master clock ( F_CPU )
                  US_MR_CHRL_8_BIT         | // 8 bit characters
                  US_MR_PAR_NO             | // no parity bit
                  US_MR_NBSTOP_1_BIT       ; // 1 stop bit
  USART3->US_IDR = 0xffffffff; // no interrupts
  USART3->US_BRGR = (F_CPU / 2000000) / 16; // make it a 2 megabit channel
  USART3->US_CR = US_CR_TXEN | US_CR_RXEN;
}

static uint16_t finger_calc_crc(uint8_t *pkt)
{
  uint16_t crc = 0, pkt_len = *(uint16_t *)(pkt+2);
  uint8_t d, crc_highbit;
  for (uint32_t i = 0; i < (uint32_t)pkt_len+5; i++)
  {
    d = pkt[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  return crc;
}

void finger_set_control_mode(uint8_t finger_idx, uint8_t control_mode)
{
  if (finger_idx > 3)
    return;
  // craft finger packet and send it out
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 13; // 1 mode byte + three 32-bit floats 
  pkt[4] = 0x1d; // control mode packet id
  if (control_mode == FINGER_CONTROL_MODE_IDLE)
    pkt[5] = 0;
  else if (control_mode == FINGER_CONTROL_MODE_JOINT_POS)
    pkt[5] = 2;
  else
    return; // other modes are currently undefined
  for (int i = 6; i < 6+12; i++)
    pkt[i] = 0; // set target as (0,0,0)
  *((uint16_t *)(&pkt[18])) = finger_calc_crc(pkt);
  finger_tx_raw(finger_idx, pkt, 20);
}

void finger_set_joint_pos(uint8_t finger_idx, float j0, float j1, float j2)
{
  if (finger_idx > 3)
    return;
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 13; // 1 mode byte + three 32-bit floats 
  pkt[4] = 0x1d; // control mode packet id
  pkt[5] = 2; // joint position mode
  *((float *)&pkt[6]) = j0;
  *((float *)&pkt[10]) = j1;
  *((float *)&pkt[14]) = j2;
  *((uint16_t *)(&pkt[18])) = finger_calc_crc(pkt);
  finger_tx_raw(finger_idx, pkt, 20);
}

void finger_tx_raw(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len)
{
  if (finger_idx > 3)
    return;
  /*
  printf("finger tx raw %d bytes:\r\n");
  for (int i = 0; i < data_len; i++)
    printf("  %d: 0x%02x\r\n", i, data[i]);
  */
  // assert RS485_SEL so that the ARM has control of the rs485 transceivers
  PIOA->PIO_SODR = PIO_PA15;
  // typedef struct { Pio *pio; uint32_t pin_idx; } rs485_de_t;
  // drive the rs485 channel
  // TEMPORARY HACK: blast out on all rs485 channels
  /*
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    de->pio->PIO_SODR = de->pin_idx;
  }
  */
  /*
  if (finger_idx == FINGER_THUMB) 
    USART3->US_MR |= US_MR_INVDATA; // thumb needs to be inverted, whoops
  else
    USART3->US_MR &= ~US_MR_INVDATA;
  */
  const rs485_de_t *de = &g_finger_rs485_de[finger_idx];
  de->pio->PIO_SODR = de->pin_idx;
  for (volatile int i = 0; i < 10; i++) { } // let driver ramp up
  while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  for (uint32_t i = 0; i < data_len; i++)
  {
    USART3->US_THR = data[i];
    while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  }
  while ((USART3->US_CSR & US_CSR_TXEMPTY) == 0) { } // wait to finish tx'ing
  for (volatile int i = 0; i < 10; i++) { } // wait a bit (why?)
  // release the rs485 channel
  de->pio->PIO_CODR = de->pin_idx;
  // TEMPORARY HACK: blast out on all rs485 channels
  /*
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    //de->pio->PIO_SODR = de->pin_idx;
    de->pio->PIO_CODR = de->pin_idx;
  }
  */
  // for now, let's allow the ARM to keep control of rs485.
}
