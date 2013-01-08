#include "cam.h"
#include "fpga_spi.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>

// hardware connections
// PA24 = 3.8v regulator enable

void cam_init()
{
  // must be run AFTER fpga_spi_init()
  printf("cam_init\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOA);
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_SODR = PIO_PA24;
  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x0003); // enable the camera voltage regulators
  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x000f); // enable the camera sysclk's
  for (volatile int j = 0; j < 200000; j++) { } // let 3v8 rail come up
  fpga_spi_txrx(0x83, 0x003f); // raise (de-assert) the camera reset lines
}

