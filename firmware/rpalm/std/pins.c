#include "pins.h"

const Pin pin_led = { PIO_PC19, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT };

const Pin pin_rs485_de = { PIO_PC9, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT };
const Pin pin_rs485_di = { PIO_PA6, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_rs485_ro = { PIO_PA5, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_spi_mosi = { PIO_PA13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_spi_miso = { PIO_PA12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_spi_sck = { PIO_PA14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_cs_adc[2] ={{PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC23, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}};

const Pin pin_i2c_scl = { PIO_PA4, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_i2c_sda = { PIO_PA3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

// mux0_a0  PC27
// mux0_a1  PA21
// mux0_a2  PC26

// mux1_a0  PB0
// mux1_a1  PC30
// mux1_a2  PB1

// mux2_a0  PA16
// mux2_a1  PC13
// mux2_a2  PC0

// mux3_a0  PC16
// mux3_a1  PA1
// mux3_a2  PC17

// mux4_a0  PA7
// mux4_a1  PA8
// mux4_a2  PA9

const Pin pin_mux[5][3] = {{{PIO_PC27,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PA21,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PC26,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                           {{PIO_PB0 ,PIOB,ID_PIOB,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PC30,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PB1 ,PIOB,ID_PIOB,PIO_OUTPUT_0,PIO_DEFAULT}},
                           {{PIO_PA16,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PC13,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PC0 ,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                           {{PIO_PC16,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PA1 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PC17,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                           {{PIO_PA7 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PA8 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                            {PIO_PA9 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT}}};

// 0  led_0     PC15
// 1  led_1     PA23
// 2  led_2     PA18
// 3  led_3     PC31
// 4  led_4     PA19
// 5  led_5     PA17
// 6  led_6     PA22
// 7  led_7     PA20
// 8  led_8     PC6
// 9  led_9     PC12
// 10 led_10    PA10
// 11 led_11_0  PC3
// 12 led_11_1  PC2
// 13 led_12_0  PA15
// 14 led_12_1  PA31
// 15 led_13_0  PA0
// 16 led_13_1  PC29
// 17 led_14    PC21
// 18 led_15_0  PC18
// 19 led_15_1  PC5

const Pin pin_leds[20] = {{PIO_PC15, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC31, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA17, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA22, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA20, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC6 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC12, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
              /* 10 */    {PIO_PA10, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC3 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC2 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA15, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PA31, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
              /* 13_0 */  {PIO_PA0 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC29, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC21, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC18, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                          {PIO_PC5 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}};
