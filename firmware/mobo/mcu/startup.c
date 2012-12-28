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

// lots of this originally snarfed from an Atmel sam3s distro, and much changed
#include "stdint.h"
#include "common_sam3x/sam3x.h"

/* Stack Configuration */  
#define STACK_SIZE       0x1000     /** Stack size (in DWords) */
__attribute__ ((aligned(8),section(".stack")))
uint32_t pdwStack[STACK_SIZE] ;     
#define IRAM_ADDR 0x20000000

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;

extern void main( void ) ;
extern void systick_vector(void);

void reset_vector() ;
extern void __libc_init_array( void ) ;

void unmapped_vector()
{
  while (1) { } // spin here to allow JTAG trap
}

typedef void (*IntFunc)(void);

__attribute__((section(".vectors")))
IntFunc exception_table[] = {
    /* Configure Initial Stack Pointer, using linker-generated symbols */
    (IntFunc)(&pdwStack[STACK_SIZE-1]),
    reset_vector,  //0x004000d1,     //_bootloader_start,
    unmapped_vector, // NMI_Handler,
    unmapped_vector, // HardFault_Handler,
    unmapped_vector, // MemManage_Handler,
    unmapped_vector, // BusFault_Handler,
    unmapped_vector, // UsageFault_Handler,
    0, 0, 0, 0,      // Reserved
    unmapped_vector, // SVC_Handler,
    unmapped_vector, // DebugMon_Handler,
    0,               // Reserved
    unmapped_vector, // PendSV_Handler,
    systick_vector,  // SysTick_Handler,

    // add 16 to these numbers to get the ARM ISR number...
    /* Configurable interrupts  */
    unmapped_vector, // 0  Supply Controller 
    unmapped_vector, // 1  Reset Controller 
    unmapped_vector, // 2  Real Time Clock 
    unmapped_vector, // 3  Real Time Timer 
    unmapped_vector, // 4  Watchdog Timer 
    unmapped_vector, // 5  PMC 
    unmapped_vector, // 6  EEFC 
    unmapped_vector, // 7  Reserved 
    unmapped_vector, // 8  UART0 
    unmapped_vector, // 9  UART1 
    unmapped_vector, // 10 SMC 
    unmapped_vector, // 11 PIOA
    unmapped_vector, // 12 PIOB
    unmapped_vector, // 13 PIOC
    unmapped_vector, //USART0_IrqHandler,  /* 14 USART 0 */
    unmapped_vector, //USART1_IrqHandler,  /* 15 USART 1 */
    unmapped_vector, //IrqHandlerNotUsed,  /* 16 Reserved */
    unmapped_vector, //IrqHandlerNotUsed,  /* 17 Reserved */
    unmapped_vector, //MCI_IrqHandler,     /* 18 MCI */
    unmapped_vector, //TWI0_IrqHandler,    /* 19 TWI 0 */
    unmapped_vector, //TWI1_IrqHandler,    /* 20 TWI 1 */
    unmapped_vector, //SPI_IrqHandler,     /* 21 SPI */
    unmapped_vector, //SSC_IrqHandler,     /* 22 SSC */
    unmapped_vector, //TC0_IrqHandler,     /* 23 Timer Counter 0 */
    unmapped_vector, //TC1_IrqHandler,     /* 24 Timer Counter 1 */
    unmapped_vector, //TC2_IrqHandler,     /* 25 Timer Counter 2 */
    unmapped_vector, //TC3_IrqHandler,     /* 26 Timer Counter 3 */
    unmapped_vector, //TC4_IrqHandler,     /* 27 Timer Counter 4 */
    unmapped_vector, //TC5_IrqHandler,     /* 28 Timer Counter 5 */
    unmapped_vector, //ADC_IrqHandler,     /* 29 ADC controller */
    unmapped_vector, //DAC_IrqHandler,     /* 30 DAC controller */
    unmapped_vector, //PWM_IrqHandler,     /* 31 PWM */
    unmapped_vector, //CRCCU_IrqHandler,   /* 32 CRC Calculation Unit */
    unmapped_vector, //ACC_IrqHandler,     /* 33 Analog Comparator */
    unmapped_vector, //USBD_IrqHandler,    /* 34 USB Device Port */
    unmapped_vector, //IrqHandlerNotUsed   /* 35 not used */
};

void reset_vector( void )
{
  uint32_t *pSrc, *pDest ;
  //LowLevelInit() ;
  EFC0->EEFC_FMR = EEFC_FMR_FWS(3);
  EFC1->EEFC_FMR = EEFC_FMR_FWS(3);
  // TODO: power up PLL, switch to main oscillator

  pSrc = &_etext ;
  pDest = &_srelocate ;

  // set up data
  if ( pSrc != pDest )
    for ( ; pDest < &_erelocate ; )
      *pDest++ = *pSrc++ ;

  // set up bss
  for ( pDest = &_szero ; pDest < &_ezero ; )
    *pDest++ = 0;

  // set vector table base address
  pSrc = (uint32_t *)&_sfixed;
  SCB->VTOR = ( (uint32_t)pSrc & SCB_VTOR_TBLOFF_Msk ) ;

  if ( ((uint32_t)pSrc >= IRAM_ADDR) && ((uint32_t)pSrc < IRAM_ADDR+IRAM_SIZE) )
    SCB->VTOR |= 1 << SCB_VTOR_TBLBASE_Pos ;

  __libc_init_array() ;
  main() ;
  while (1) { } // never gets here...
}

