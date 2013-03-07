#include "sam3s/chip.h"
#include "bl_stubs.h"

const int __attribute__((section (".rs485_addr"))) g_rs485_addr = 1;

void bl_init()
{
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) |
                   (1 << ID_USART0);
  PIOA->PIO_CODR = PIOA->PIO_OER = PIOA->PIO_PER = PIO_PA22; // led
  PIOC->PIO_CODR = PIOC->PIO_OER = PIOC->PIO_PER = PIO_PC9;  // rs485 DE
  PIOA->PIO_OER  =  PIO_PA22;
  PIOA->PIO_PDR  = PIO_PA5A_RXD0 | PIO_PA6A_TXD0;
  PIOA->PIO_ABCDSR[0] &= ~(PIO_PA5A_RXD0 | PIO_PA6A_TXD0);
  PIOA->PIO_ABCDSR[1] &= ~(PIO_PA5A_RXD0 | PIO_PA6A_TXD0);
  USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX |
                  US_CR_RXDIS | US_CR_TXDIS; // reset uart
  USART0->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO |
                  US_MR_MAN | US_MR_OVER | US_MR_MODSYNC;
                  USART0->US_BRGR = 64000000 / 2000000 / 16;
  USART0->US_MAN = US_MAN_TX_PL(1) | US_MAN_TX_PP_ALL_ONE |
                   US_MAN_RX_PL(1) | US_MAN_RX_PP_ALL_ONE |
                   US_MAN_TX_MPOL | US_MAN_RX_MPOL |
                   US_MAN_DRIFT | US_MAN_STUCKTO1;
  USART0->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS; // disable DMA
  USART0->US_CR = US_CR_TXEN | US_CR_RXEN; // enable TX and RX
}

void bl_led(bl_led_state_t state)
{   
  switch (state)
  {
    case BL_LED_OFF: PIOA->PIO_CODR = PIO_PA22; break;
    case BL_LED_ON : PIOA->PIO_SODR = PIO_PA22; break;
    case BL_LED_TOGGLE:
      if (PIOA->PIO_ODSR & PIO_PA22)
        PIOA->PIO_CODR = PIO_PA22; 
      else
        PIOA->PIO_SODR = PIO_PA22;
      break;
    default: break;
  }
} 

int bl_uart_byte_ready()
{
  return (USART0->US_CSR & US_CSR_RXRDY);
}

uint8_t bl_uart_get_byte()
{
  return (uint8_t)USART0->US_RHR;
}

void bl_uart_tx(const uint8_t *data, const uint16_t data_len)
{
  USART0->US_CR |= US_CR_RXDIS;
  PIOC->PIO_SODR = PIO_PC9;
  while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (volatile uint32_t i = 0; i < (7+data_len); i++)
  {
    USART0->US_THR = data[i];
    while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  } 
  PIOC->PIO_CODR = PIO_PC9; // release the rs485 bus
  USART0->US_CR &= ~US_CR_RXDIS;
  USART0->US_CR |= US_CR_RXEN;
}

