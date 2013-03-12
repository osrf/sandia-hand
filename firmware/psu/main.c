#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

int main()
{
  wdt_disable(); // todo: revisit this...
  while (1)
  {
  }
  return 0;
}

