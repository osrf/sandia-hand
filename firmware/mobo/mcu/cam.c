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

#include "cam.h"
#include "fpga.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>

// hardware connections
// PA24 = 3.8v regulator enable
// PA17 = TWID0 = sda
// PA18 = TWCK0 = scl

void cam_init()
{
  // must be run AFTER fpga_init()
  printf("hai. cam_init.\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_TWI0);
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_SODR = PIO_PA24;
  PIOA->PIO_ABSR &= ~(PIO_PA17A_TWD0 | PIO_PA18A_TWCK0);
  PIOA->PIO_PDR = PIO_PA17A_TWD0 | PIO_PA18A_TWCK0;
  // whoops, forgot to put pull-up resistors on this bus. use internal pullups
  PIOA->PIO_PUER = PIO_PA17A_TWD0 | PIO_PA18A_TWCK0;

  TWI0->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVDIS; // disable twi for the moment
  TWI0->TWI_IDR = 0xffffffff; // no interrupts plz
  TWI0->TWI_SR; // read status register (why? atmel library does it...)
  TWI0->TWI_CR = TWI_CR_SWRST; // sotware reset
  TWI0->TWI_RHR; // empty receive register
  TWI0->TWI_CR = TWI_CR_MSEN; // enable master mode
  TWI0->TWI_CWGR = TWI_CWGR_CLDIV(77) | TWI_CWGR_CHDIV(77) |  // 50% duty cycle
                   TWI_CWGR_CKDIV(3); // 400 khz i2c / 2^3 = 50 (weak pullups)

  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x0003); // enable the camera voltage regulators
  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x000f); // enable the camera sysclk's
  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x003f); // raise (de-assert) the camera reset lines
  for (volatile int j = 0; j < 200000; j++) { } // wait for camera to wake up
  fpga_spi_txrx(FPGA_SPI_REG_CAM_MAX_ROWS | FPGA_SPI_WRITE, 0x01e0); // 480 
  /*
  for (int i = 0; i < 256; i++)
  {
    printf("reg 0x%02x: ", i);
    uint16_t val_0 = cam_read_register(0, i);
    printf("0x%04x  ", val_0);
    uint16_t val_1 = cam_read_register(1, i);
    printf("0x%04x\r\n", val_1);
  }
  */
}

void cam_set_streams(const uint8_t stream_0, const uint8_t stream_1)
{
  uint16_t cam_cfg = 0x003f |
                     (stream_0 ? CAM_CAPTURE_0 : 0) |
                     (stream_1 ? CAM_CAPTURE_1 : 0);
  printf("cam_set_streams(%d, %d) --> 0x%04x\r\n", 
         stream_0, stream_1, cam_cfg);
  fpga_spi_txrx(FPGA_SPI_REG_CAM_CFG | FPGA_SPI_WRITE, cam_cfg);
}

uint16_t cam_read_register(uint8_t cam_idx, uint8_t reg_idx)
{
  uint8_t i2c_addr;
  if (cam_idx == 0)
    i2c_addr = 0x98 >> 1;
  else if (cam_idx == 1)
    i2c_addr = 0xb8 >> 1;
  else
  {
    printf("illegal cam_idx: [%x]\r\n", cam_idx);
    return 0;
  }
  TWI0->TWI_MMR = 0; // not sure why, but atmel library clears this first
  TWI0->TWI_MMR = TWI_MMR_MREAD | 
                  TWI_MMR_IADRSZ_1_BYTE |
                  TWI_MMR_DADR(i2c_addr);
  TWI0->TWI_IADR = reg_idx;
  TWI0->TWI_CR = TWI_CR_START;
  int rx_cnt = 2;
  uint16_t rx_val = 0;
  uint8_t *rx_ptr = (uint8_t *)&rx_val;
  while (rx_cnt > 0)
  {
    uint32_t status = TWI0->TWI_SR;
    if (status & TWI_SR_NACK)
    {
      printf("received nack\r\n");
      return 0;
    }
    if (rx_cnt == 1)
      TWI0->TWI_CR = TWI_CR_STOP;
    if (!(status & TWI_SR_RXRDY))
      continue; // busy-wait in this loop
    *rx_ptr++ = TWI0->TWI_RHR;
    rx_cnt--;
  }
  return __REV16(rx_val);
}


