#ifndef BL_STUBS_H
#define BL_STUBS_H

extern const int __attribute__((section (".rs485_addr"))) g_rs485_addr;

typedef enum { BL_LED_OFF = 0, BL_LED_ON = 1, BL_LED_TOGGLE = 2} bl_led_state_t;
void bl_led(bl_led_state_t state);

void bl_init();
int bl_uart_byte_ready();
uint8_t bl_uart_get_byte();
void bl_uart_tx(const uint8_t *data, const uint16_t data_len);

#endif

