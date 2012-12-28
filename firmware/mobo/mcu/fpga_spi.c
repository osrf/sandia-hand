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

#include "fpga_spi.h"
#include "common_sam3x/sam3x.h"

// hardware connections:
//  PA27 = SCK
//  PA26 = MOSI
//  PA25 = MISO
//  PC24 = FPGA CS
//  PA29 = FLASH CS

void fpga_spi_init()
{
  PMC->PMC_PCER0 |= (1 << ID_SPI0) | (1 << ID_PIOA) | (1 << ID_PIOC);
  PIOA->PIO_OER = PIO_PA29 | PIO_PA25 | PIO_PA26 | PIO_PA27; // output enables
  PIOA->PIO_SODR = PIO_PA29; // drive flash chip CS line high
  PIOC->PIO_OER = PIO_PC24; // output enables
  PIOC->PIO_SODR = PIO_PC24; // drive FPGA CS line high
  PIOA->PIO_PDR = PIO_PA25A_SPI0_MISO |
                  PIO_PA26A_SPI0_MOSI |
                  PIO_PA27A_SPI0_SPCK; // disable pio pin control
  PIOA->PIO_ABSR &= ~(PIO_PA25A_SPI0_MISO |
                      PIO_PA26A_SPI0_MOSI |
                      PIO_PA27A_SPI0_SPCK); // select peripheral A for SPI
  SPI0->SPI_CR = SPI_CR_SPIDIS; // turn it off while we mess with it
  SPI0->SPI_CR = SPI_CR_SWRST;  // reset
  SPI0->SPI_CR = SPI_CR_SWRST;  // no idea why i did it twice before
  SPI0->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS; // master mode, no fault detect
  SPI0->SPI_IDR = 0xffffffff; // no interrupts plz
  SPI0->SPI_CSR[0] = SPI_CSR_CPOL |       // SCK idles high
                     SPI_CSR_NCPHA |      // capture on rising edge
                     SPI_CSR_BITS_8_BIT | // blast out 8 bits at a time
                     SPI_CSR_SCBR(16);    // F_SPI = F_CPU / 16 = 4 MHz
  SPI0->SPI_CR = SPI_CR_SPIEN; // turn it back on
  // not sure why, but i saw this init nonsense once in an EVK sample code...
  // todo: figure out if i can get rid of it
  volatile uint32_t dummy;
  for (dummy = 0; dummy < 100000; dummy++) { } // why? atmel does it
  dummy = SPI0->SPI_SR;
  dummy = SPI0->SPI_RDR;
}

uint16_t fpga_spi_txrx(uint8_t reg, uint16_t tx_data)
{
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // ensure peripheral is clear
  volatile uint16_t rx;
  PIOC->PIO_CODR = PIO_PC24; // drive FPGA CS line low
  SPI0->SPI_TDR = reg; // start TX of register value (first 8 bits)
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // wait for it to shift out
  rx = SPI0->SPI_RDR; // garbage
  SPI0->SPI_TDR = (tx_data >> 8); // start sending MSB
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // wait for it to shift out
  rx = (SPI0->SPI_RDR) << 8; // copy out rx MSB 
  SPI0->SPI_TDR = (tx_data & 0xff); // start sending LSB
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // wait for it to shift out
  rx |= SPI0->SPI_RDR; // copy out rx LSB 
  PIOC->PIO_SODR = PIO_PC24; // drive FPGA CS line high
  return rx;
}

