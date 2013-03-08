#include "bl_stubs.h"

const int __attribute__((section (".rs485_addr"))) g_rs485_addr = 1;

#define BL_LED_PIO           PIOC
#define BL_LED_PIN           PIO_PC18

#define BL_USART_IDX         1
#define BL_RS485_DE_PIO      PIOA
#define BL_RS485_DE_PIN      PIO_PA19

#include "bl_init.c"
#include "bl_led.c"
#include "bl_uart.c"
