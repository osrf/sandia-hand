#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "status.h"
#include "tactile.h"
#include <stdbool.h>

static volatile uint32_t g_status_tc0_ovf_count = 0;
static bool g_status_build_packet = false;
palm_status_t g_status;
void status_init()
{
  for (int i = 0; i < sizeof(palm_status_t); i++)
    ((uint8_t *)(&g_status))[i] = 0; // ugly
  PMC_EnablePeripheral(ID_TC0);
  TC0->TC_QIDR = 0xffffffff; // no quadrature interrupts plz
  TC0->TC_CHANNEL[0].TC_IDR = 0xffffffff; // no timer interrupts
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_COVFS; // enable overflow interrupt
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | // 64 / 128 = 500khz
                              TC_CMR_WAVE; // waveform generation mode
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
  NVIC_SetPriority(TC0_IRQn, 4);
  NVIC_EnableIRQ(TC0_IRQn);
}

void status_tc0_irq()
{
  TC0->TC_CHANNEL[0].TC_SR; // dummy read to clear IRQ flag
}

void status_systick()
{
  static volatile int s_status_systick_count = 0;
  if (s_status_systick_count++ % (1000 / 100) == 0) // 100 hz
    g_status_build_packet = true;
}

void status_idle()
{
  if (!g_status_build_packet)
    return;
  g_status_build_packet = false;
  // stuff the status buffer, so that it's ready for pickup by mobo autopoll
  for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
    g_status.palm_tactile[i] = g_tactile_last_scan[i];
  for (int i = 0; i < 7; i++)
    g_status.palm_temps[i] = 0; // todo
  for (int i = 0; i < 3; i++)
    g_status.palm_accel[i] = g_status.palm_gyro[i] = g_status.palm_mag[i] = 0;
  g_status.palm_time = (g_status_tc0_ovf_count << 17) +
                       (TC0->TC_CHANNEL[0].TC_CV << 1); // make it microseconds
}

