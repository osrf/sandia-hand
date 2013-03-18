#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// hardware connections:
//  PD0 = rxd
//  PD1 = txd

#define PORTC_LED_PIN   0x04
#define PORTC_SSR_PIN   0x08
#define PORTD_TX_PIN    0x02
#define INA226_I2C_ADDR 0x80
#define I2C_TIMEOUT_MS    50

inline void toggle_led() { PORTC ^= PORTC_LED_PIN; }
static uint8_t g_timer_flag = 0;
#define SWAP_16(x) (((x >> 8) & 0xff) | ((x << 8) & 0xff00))

ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;
  g_timer_flag = 1;
}

int usart0_putc(char c, FILE *f)
{
  while (!(UCSR0A & (_BV(UDRE0)))) { }
  UDR0 = c;
  return 0;
}

inline void i2c_start()
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
}

inline void i2c_stop()
{
  TWCR = _BV(TWSTO);
}

inline void i2c_start_tx(uint8_t b)
{
  TWDR = b;
  TWCR = _BV(TWINT) | _BV(TWEN);
}

inline void i2c_start_rx(uint8_t ack)
{
  if (ack)
    TWCR = (TWCR & 0xf) | _BV(TWINT) | _BV(TWEA);
  else
    TWCR = (TWCR & 0xf) | _BV(TWINT);
}

inline uint8_t i2c_get_rx()
{
  return TWDR;
}

uint8_t i2c_sync_wait()
{
  for (uint16_t ms_elapsed = 0; 
       !(TWCR & _BV(TWINT)) && ms_elapsed < I2C_TIMEOUT_MS; 
       ms_elapsed++)
    _delay_ms(1);
  return (TWCR & _BV(TWINT)) ? 1 : 0;
}

#define I2C_WAIT() do { if (!i2c_sync_wait()) { i2c_stop(); return 0; } } while (0)

uint8_t i2c_sync_read(const uint8_t dev_addr, const uint8_t dev_reg_addr, 
                      uint8_t *read_data, const uint8_t read_data_len)
{
  i2c_start();
  I2C_WAIT();
  i2c_start_tx(dev_addr);
  I2C_WAIT();
  i2c_start_tx(dev_reg_addr);
  I2C_WAIT();
  i2c_start_tx(dev_addr | 0x01); // send read addr
  I2C_WAIT();
  for (uint8_t read_idx = 0; read_idx < read_data_len; read_idx++)
  {
    i2c_start_rx(read_idx < read_data_len - 1 ? 1 : 0);
    I2C_WAIT();
    if ((TWSR & 0xf8) != 0x50) // check twi status code
    {
      i2c_stop();
      printf("lost i2c arb\r\n");
      return 0;
    }
    read_data[read_idx] = i2c_get_rx();
  }
  i2c_stop();
  return 1;
}

uint8_t i2c_sync_write(const uint8_t dev_addr, const uint8_t dev_reg_addr,
                       const uint8_t *write_data, const uint8_t write_data_len)
{
  i2c_start();
  I2C_WAIT();
  i2c_start_tx(dev_addr);
  I2C_WAIT();
  i2c_start_tx(dev_reg_addr);
  I2C_WAIT();
  for (uint8_t write_idx = 0; write_idx < write_data_len; write_idx++)
  {
    i2c_start_tx(write_data[write_idx]);
    I2C_WAIT();
  }
  i2c_stop();
  return 1;
}

uint8_t ina226_write_reg(const uint8_t reg, const uint16_t val)
{
  uint16_t val_rev = SWAP_16(val);
  return i2c_sync_write(INA226_I2C_ADDR, reg, (uint8_t *)&val_rev, 2);
}

uint8_t ina226_read_reg(const uint8_t reg, uint16_t *val)
{
  uint16_t val_rev = 0;
  if (!i2c_sync_read(INA226_I2C_ADDR, reg, (uint8_t *)&val_rev, 2))
    return 0;
  *val = SWAP_16(val_rev);
  return 1;
}

uint16_t ina226_read_current()
{
  uint16_t raw_val = 0;
  if (!ina226_read_reg(4, &raw_val))
    return 0;
  return raw_val;
}

int main()
{
  wdt_disable(); // todo: revisit this...
  DDRC = PORTC_SSR_PIN | PORTC_LED_PIN;
  DDRD = PORTD_TX_PIN;
  UCSR0A = _BV(U2X0);
  UCSR0B = _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8N1 format
  UBRR0H = 0;
  UBRR0L = 0; // with U2X0, this will give 115200 baud with 8.5% error
  fdevopen(usart0_putc, NULL);
  printf("hello, world!\r\n");
  TWSR = 0; // twi prescalar = 1
  TWBR = 0; // twi bit rate = 1000000/16 = 62.5 kHz
  uint16_t man_id = 0, die_id = 0;
  ina226_read_reg(0xfe, &man_id);
  ina226_read_reg(0xff, &die_id);
  printf("ina226 man id: 0x%04x   die id: 0x%04x\r\n", man_id, die_id);
  ina226_write_reg(5, 5120); // set ina226 to 0.1 mA resolution

  // todo: setup analog temperature read
  TCCR1B = 0x02; // timer 1 = sysclk / 8 = 125 kHz
  OCR1A = 62500; // set to 2 Hz for testing
  TCNT1 = 0;
  TIMSK1 = _BV(OCIE1A);
  sei(); // turn on interrupts
  while (1) 
  { 
    if (g_timer_flag)
    {
      g_timer_flag = 0;
      toggle_led();
      uint16_t raw_reading = ina226_read_current();
      printf("ina226 raw = %.04x\r\n", raw_reading);
    }
  }
  return 0;
}

