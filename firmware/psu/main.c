#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

// hardware connections:
//  PC3 = ssr anode
//  PC2 = red LED

int main()
{
  wdt_disable(); // todo: revisit this...
  DDRC = 0x0c;
  for (uint32_t i = 0; ; i++)
  {
    _delay_ms(250);
    PORTC ^= 0x04;
    if (i % 20 == 0)
      PORTC ^= 0x08; // toggle ssr output
  }
  return 0;
}

