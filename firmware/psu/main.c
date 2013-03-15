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
  while (1)
  {
    _delay_ms(250);
    PORTC ^= 0x04;
  }
  return 0;
}

