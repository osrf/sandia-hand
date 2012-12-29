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

// hardware connections:
//   PA21 = F0_LV
//   PC19 = F0_HV
//   PB19 = F1_LV
//   PB17 = F1_HV
//   PA2  = F2_LV
//   PA4  = F2_HV
//   PA6  = F3_LV
//   PA16 = F3_HV

#define POWER_NUM_FINGERS 4

typedef struct { Pio *pio; uint32_t pin_idx; } power_switch_t;
const power_switch_t power_switches[POWER_NUM_FINGERS][2] =  
  { { { PIOA, PIO_PA21 }, { PIOC, PIO_PC19 } },
    { { PIOB, PIO_PB19 }, { PIOB, PIO_PB17 } },
    { { PIOA, PIO_PA2  }, { PIOA, PIO_PA4  } },
    { { PIOA, PIO_PA6  }, { PIOA, PIO_PA16 } } };

void power_init()
{
  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC);
  const uint32_t pioa_pins = PIO_PA2 | PIO_PA4 | PIO_PA6 | PIO_PA16 | PIO_PA21;
  const uint32_t piob_pins = PIO_PB17 | PIO_PB19;
  const uint32_t pioc_pins = PIO_PC19;
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_CODR = pioa_pins;
  PIOB->PIO_PER = PIOB->PIO_OER = PIOB->PIO_CODR = piob_pins;
  PIOC->PIO_PER = PIOC->PIO_OER = PIOC->PIO_CODR = pioc_pins;
}

void power_set(const uint8_t finger_idx, const power_state_t power_state)
{
  if (finger_idx >= POWER_NUM_FINGERS) return; // get that out
  const power_switch_t *sw = power_switches[finger_idx];
  switch(power_state)
  {
    POWER_OFF:
      sw[0].pio->PIO_CODR = sw[0].pin_idx; // low voltage off
      sw[1].pio->PIO_CODR = sw[1].pin_idx; // high voltage off
      break;
    POWER_LOW:
      sw[0].pio->PIO_SODR = sw[0].pin_idx; // low voltage on
      sw[1].pio->PIO_CODR = sw[1].pin_idx; // high voltage off
      break;
    POWER_ON: // need to wait a bit for high voltage to ramp up? not sure
      sw[1].pio->PIO_SODR = sw[1].pin_idx; // high voltage on
      sw[0].pio->PIO_CODR = sw[0].pin_idx; // low voltage off
      break;
  }
}

