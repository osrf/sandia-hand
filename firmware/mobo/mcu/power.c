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
#include "sandia_hand/hand_packets.h"
#include <stdio.h>
#include "enet.h"

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

#define POWER_NUM_FINGERS    4
#define POWER_VDD_SENSOR_IDX 4

typedef struct { Pio *pio; uint32_t pin_idx; } power_switch_t;
const power_switch_t power_switches[POWER_NUM_FINGERS][2] =  
  { { { PIOB, PIO_PB19 }, { PIOB, PIO_PB17 } },   // index
    { { PIOA, PIO_PA2  }, { PIOA, PIO_PA4  } },   // middle
    { { PIOA, PIO_PA6  }, { PIOA, PIO_PA16 } },   // pinkie
    { { PIOA, PIO_PA21 }, { PIOC, PIO_PC19 } } }; // thumb
static power_state_t g_finger_power_states[4] = { POWER_OFF };

static volatile uint8_t g_power_poll_req = 0, g_power_status_send_req = 0;
static void power_start_read_finger_sensor_reg(const uint8_t finger_idx, 
                                               const uint8_t reg);
static void power_start_poll();
static void power_i2c_rx_complete(const uint16_t rx_data);
static void power_i2c_write_sync(const uint8_t  sensor_idx,
                                 const uint8_t  reg_idx,
                                 const uint16_t reg_val);
static uint16_t power_i2c_read_sync(const uint8_t sensor_idx,
                                    const uint8_t reg_idx);
static uint8_t power_sensor_idx_to_i2c_addr(const uint8_t finger_idx);
static void power_ina3221_measure(const uint8_t ch_idx, float *current, 
                                  float *voltage);

static volatile uint16_t g_power_i2c_rx_val = 0;
static volatile uint8_t g_power_i2c_rx_cnt = 0, *g_power_i2c_rx_ptr = 0;
static volatile enum { POWER_IDLE=0, POWER_RX_WAIT=1, POWER_RX_COMPLETE=2 } 
  g_power_state = POWER_IDLE;
static volatile uint8_t g_power_autopoll_sensor_idx = 0;
volatile int16_t g_power_finger_currents[4] = {0};
volatile uint16_t g_power_autosend_timeout = 0;
volatile float g_power_logic_currents[3] = {0}, 
               g_power_logic_voltages[3] = {0};
volatile uint16_t g_power_adc_readings[3] = {0};
#define POWER_ADC_CH_TEMP_0       1
#define POWER_ADC_CH_TEMP_1      11
#define POWER_ADC_CH_ONCHIP_TEMP 15
static volatile uint32_t g_power_txcomp_time = 0;
static volatile uint32_t g_power_systick_value = 0;

void power_init()
{
  printf("power init\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                    (1 << ID_TWI1);
  const uint32_t pioa_pins = PIO_PA2 | PIO_PA4 | PIO_PA6 | 
                             PIO_PA16 | PIO_PA21;
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
  // for finger sockets, assume max expected current = 3A
  // ina226 datasheet p.14:  set current_lsb = 0.1 mA
  // giving CAL = 0.00512 / (0.0001 amp * 0.01 ohm) = 5120
  for (int i = 0; i < 4; i++)
    power_i2c_write_sync(i, 5, __REV16(5120));
  /*
  printf("querying ina3221...\r\n");
  printf("ina3221 man ID: 0x%04x\r\n", power_i2c_read_sync(4, 0xfe));
  printf("ina3221 die ID: 0x%04x\r\n", power_i2c_read_sync(4, 0xff));
  */
  power_i2c_write_sync(POWER_VDD_SENSOR_IDX, 0, __REV16(0x7127)); 
  float i, v;
  /*
  for (int j = 0; j < 10; j++)
  {
    power_ina3221_measure(0, &i, &v);
    power_ina3221_measure(1, &i, &v);
    power_ina3221_measure(2, &i, &v);
  }
  */
  // turn on ADC
  PMC->PMC_PCER1 |= (1 << (ID_ADC - 32));
  ADC->ADC_CR = ADC_CR_SWRST; // reset it...
  ADC->ADC_MR = 0;
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
  ADC->ADC_RCR = 0;
  ADC->ADC_RNCR = 0;
  ADC->ADC_MR = ADC_MR_TRANSFER(1) |
                ADC_MR_TRACKTIM(1) |
                ADC_MR_SETTLING_AST17 | // long settling time... no hurry.
                ADC_MR_PRESCAL(4)  | // adc clock = 64 mhz / ((4+1)*2) = 6.4 M
                ADC_MR_FREERUN     |
                ADC_MR_STARTUP_SUT512;
  ADC->ADC_CHER = (1 << POWER_ADC_CH_TEMP_0) |
                  (1 << POWER_ADC_CH_TEMP_1) |
                  (1 << POWER_ADC_CH_ONCHIP_TEMP);
  ADC->ADC_IDR = 0xffffffff; // no interrupts plz
  ADC->ADC_ACR = ADC_ACR_TSON;
  ADC->ADC_CR = ADC_CR_START; // start a conversion
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY)) { }
  ADC->ADC_LCDR; // dummy read
  ADC->ADC_CR = ADC_CR_START; // start another
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY)) { }
  ADC->ADC_LCDR; // dummy read
  ADC->ADC_IER = ADC_IER_DRDY;
  for (int i = 0; i < 3; i++)
    g_power_adc_readings[i] = 0;
  NVIC_SetPriority(ADC_IRQn, 11);
  NVIC_EnableIRQ(ADC_IRQn);
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
  g_finger_power_states[finger_idx] = power_state;
}

void power_idle()
{
  if (g_power_poll_req)
  {
    g_power_poll_req = 0;
    power_start_poll();
    return;
  }
  const uint32_t status = TWI1->TWI_SR;
  if (g_power_state == POWER_RX_WAIT)
  {
    if (status & TWI_SR_NACK)
    {
      g_power_state = POWER_IDLE;
      //printf("twi1 received nack\r\n");
      return;
    }
    if (g_power_i2c_rx_cnt == 1) // one more byte to go.
      TWI1->TWI_CR = TWI_CR_STOP;
    if (!(status & TWI_SR_RXRDY))
      return; // nothing exciting has happened since last time. bail for now.
    // if we get here, there is a new byte waiting for us in TWI_RHR
    if (!g_power_i2c_rx_ptr) // sanity check...
    {
      //printf("woah there partner! g_power_i2c_rx_ptr is null!\r\n");
      return;
    }
    *g_power_i2c_rx_ptr++ = TWI1->TWI_RHR;
    if (--g_power_i2c_rx_cnt == 0)
    {
      g_power_state = POWER_RX_COMPLETE;
      //power_i2c_rx_complete(__REV16(g_power_i2c_rx_val));
    }
  }
  else if (g_power_state == POWER_RX_COMPLETE)
  {
    power_i2c_rx_complete(__REV16(g_power_i2c_rx_val));
    /*
    if ((!g_power_txcomp_time) && (status & TWI_SR_TXCOMP))
      g_power_txcomp_time = g_power_systick_value;
    */
  }
}

void power_systick()
{
  g_power_systick_value++;
  if (g_power_systick_value % 20 == 0) // 50 hz
    g_power_poll_req = 1;
  if (g_power_autosend_timeout && 
      (g_power_systick_value % g_power_autosend_timeout == 0)) 
    g_power_status_send_req = 1;
  /*
  if (g_power_txcomp_time && 
      g_power_systick_value >= g_power_txcomp_time + 2)
  {
    //printf("txct = %d stv = %d\r\n", g_power_txcomp_time, g_power_systick_value);
    if (g_power_autopoll_sensor_idx < 4)
      power_start_read_finger_sensor_reg(g_power_autopoll_sensor_idx, 0x04);
    else if (g_power_autopoll_sensor_idx < 7)
      power_start_read_finger_sensor_reg(4, 
                             0x01 + (2 * (g_power_autopoll_sensor_idx - 4)));
    g_power_txcomp_time = 0;
  }
  */
}

void power_start_read_finger_sensor_reg(const uint8_t finger_idx, 
                                        const uint8_t reg_idx)
{
  //printf("starting read of register %d on finger %d\r\n",
  //       reg_idx, finger_idx);
  const uint8_t i2c_addr = power_sensor_idx_to_i2c_addr(finger_idx);
  if (!i2c_addr)
    return; // bogus
  TWI1->TWI_MMR = 0; // not sure why, but atmel library clears this first
  TWI1->TWI_MMR = TWI_MMR_MREAD | 
                  TWI_MMR_IADRSZ_1_BYTE |
                  TWI_MMR_DADR(i2c_addr);
  TWI1->TWI_IADR = reg_idx;
  TWI1->TWI_CR = TWI_CR_START;
  g_power_i2c_rx_val = 0;
  g_power_i2c_rx_cnt = 2;
  g_power_i2c_rx_ptr = (uint8_t *)&g_power_i2c_rx_val;
  g_power_state = POWER_RX_WAIT;
}

void power_start_poll()
{
  /*
  printf("adc: %04d %04d %04d\r\n", 
         g_power_adc_readings[0],
         g_power_adc_readings[1],
         g_power_adc_readings[2]);
  */
  g_power_autopoll_sensor_idx = 0;
  power_start_read_finger_sensor_reg(0, 0x04);
}

void power_i2c_rx_complete(const uint16_t rx_data)
{
  /*
  printf("%d: sensor %d: %d\r\n",
         g_power_systick_value,
         g_power_autopoll_sensor_idx, 
         (int16_t)rx_data);
  */
  if (g_power_autopoll_sensor_idx < 4)
    g_power_finger_currents[g_power_autopoll_sensor_idx] = (int16_t)rx_data;
  else if (g_power_autopoll_sensor_idx < 7)
    g_power_logic_currents[g_power_autopoll_sensor_idx - 4] = 
      ((float)(rx_data >> 3)) * 0.00004f / 0.01f; // ina3221 datasheet, p.23
  if (g_power_autopoll_sensor_idx == 6)
  {
    if (g_power_status_send_req)
    {
      g_power_status_send_req = 0;
      uint8_t status_buf[sizeof(mobo_status_t) + 4];
      *((uint32_t *)(status_buf)) = CMD_ID_MOBO_STATUS;
      mobo_status_t *p = (mobo_status_t *)(status_buf + 4);
      for (int i = 0; i < 4; i++)
        p->finger_milliamps[i] = g_power_finger_currents[i];
      for (int i = 0; i < 3; i++)
        p->logic_milliamps[i] = g_power_logic_currents[i] * 1000.0f;
      for (int i = 0; i < 3; i++)
        p->mobo_raw_temperatures[i] = g_power_adc_readings[i];
      enet_tx_udp(status_buf, sizeof(mobo_status_t) + 4);
    }
  }

  if (g_power_autopoll_sensor_idx < 4)
    power_start_read_finger_sensor_reg(g_power_autopoll_sensor_idx, 0x04);
  else if (g_power_autopoll_sensor_idx < 7)
    power_start_read_finger_sensor_reg(4, 
                           0x01 + (2 * (g_power_autopoll_sensor_idx - 4)));

  g_power_autopoll_sensor_idx++;
}

uint16_t power_i2c_read_sync(const uint8_t sensor_idx,
                             const uint8_t reg_idx)
{
  const uint8_t i2c_addr = power_sensor_idx_to_i2c_addr(sensor_idx);
  if (!i2c_addr)
    return 0; // bogus
  TWI1->TWI_MMR = 0; // not sure why, but atmel library clears this first
  TWI1->TWI_MMR = TWI_MMR_MREAD | 
                  TWI_MMR_IADRSZ_1_BYTE |
                  TWI_MMR_DADR(i2c_addr);
  TWI1->TWI_IADR = reg_idx;
  TWI1->TWI_CR = TWI_CR_START;
  int rx_cnt = 2;
  uint16_t rx_val = 0;
  uint8_t *rx_ptr = (uint8_t *)&rx_val;
  //printf("starting i2c busy wait...\r\n");
  while (rx_cnt > 0)
  {
    uint32_t status = TWI1->TWI_SR;
    if (status & TWI_SR_NACK)
    {
      printf("received nack\r\n");
      return 0;
    }
    if (rx_cnt == 1)
      TWI1->TWI_CR = TWI_CR_STOP;
    if (!(status & TWI_SR_RXRDY))
      continue; // busy-wait in this loop
    //printf("i2c byte transfer complete\r\n");
    *rx_ptr++ = TWI1->TWI_RHR;
    rx_cnt--;
  }
  return __REV16(rx_val);
}

void power_i2c_write_sync(const uint8_t  sensor_idx,
                          const uint8_t  reg_idx,
                          const uint16_t reg_val)
{
  const uint8_t i2c_addr = power_sensor_idx_to_i2c_addr(sensor_idx);
  /*
  printf("writing 0x%04x to register %d at addr %d\r\n", 
         reg_val, reg_idx, i2c_addr);
  */
  if (!i2c_addr)
    return; // bogus
  TWI1->TWI_MMR = 0; // not sure why, but atmel library clears this first
  TWI1->TWI_MMR = TWI_MMR_IADRSZ_1_BYTE |
                  TWI_MMR_DADR(i2c_addr);
  TWI1->TWI_IADR = reg_idx;
  uint32_t tx_cnt = 2;
  uint8_t *tx_ptr = (uint8_t *)&reg_val;
  while (tx_cnt > 0)
  {
    uint32_t status = TWI1->TWI_SR;
    if (status & TWI_SR_NACK)
    {
      //printf("twi1 nack\r\n");
      return;
    }
    if (!(status & TWI_SR_TXRDY))
      continue; // busy wait...
    TWI1->TWI_THR = *tx_ptr++;
    tx_cnt--;
  }
  TWI1->TWI_CR = TWI_CR_STOP;
  while (!(TWI1->TWI_SR & TWI_SR_TXCOMP)) { } // busy wait...
  TWI1->TWI_SR; // why do another read? atmel library does it.
}

uint8_t power_sensor_idx_to_i2c_addr(const uint8_t sensor_idx)
{
  switch(sensor_idx)
  {
    case 0:  return 0x44; break; // see schematics & INA226 datasheet
    case 1:  return 0x45; break;
    case 2:  return 0x46; break;
    case 3:  return 0x47; break;
    case 4:  return 0x43; break;
    default: return 0;    break; // bogus
  }
}

void power_ina3221_measure(const uint8_t ch_idx, float *current, 
                           float *voltage)
{
  if (!current || !voltage)
    return; // buh bye
  uint16_t shunt = power_i2c_read_sync(POWER_VDD_SENSOR_IDX, 1+ch_idx*2) >> 3;
  *current = (float)shunt * 0.00004f / 0.01f; // ina3221 datasheet, p.23
  printf("ch %d shunt: %d = %d\r\n", 
         ch_idx, shunt, (int)((*current) * 1000.0f));
  uint16_t bus = power_i2c_read_sync(POWER_VDD_SENSOR_IDX, 2+ch_idx*2) >> 3;
  *voltage = (float)bus * 0.008f; // ina3221 datasheet, p.23
  printf("bus: %d = %d\r\n", bus, (int)((*voltage) * 1000));
}

void power_adc_vector()
{
  // TODO: filter, etc.
  g_power_adc_readings[0] = ADC->ADC_CDR[POWER_ADC_CH_TEMP_0];
  g_power_adc_readings[1] = ADC->ADC_CDR[POWER_ADC_CH_TEMP_1];
  g_power_adc_readings[2] = ADC->ADC_CDR[POWER_ADC_CH_ONCHIP_TEMP];
  ADC->ADC_LCDR; // dummy read to clear interrupt flag
}

