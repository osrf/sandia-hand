#ifndef DP0_PINS_H
#define DP0_PINS_H

#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

#define PINS_NUM_CS_ADC 2
#define PINS_NUM_LEDS 20
#define PINS_NUM_MUX 5
#define PINS_MUX_ADDR_BITS 3

extern const Pin pin_led;
extern const Pin pin_rs485_de, pin_rs485_di, pin_rs485_ro;
extern const Pin pin_cs_adc[PINS_NUM_CS_ADC];
extern const Pin pin_spi_sck, pin_spi_mosi, pin_spi_miso;
extern const Pin pin_i2c_sda, pin_i2c_scl;
extern const Pin pin_leds[PINS_NUM_LEDS];
extern const Pin pin_mux[PINS_NUM_MUX][PINS_MUX_ADDR_BITS];

#endif
