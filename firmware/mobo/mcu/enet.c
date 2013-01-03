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

#include "enet.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>

// hardware connections:
//   PB0 = EREFCK
//   PB1 = ETX_EN
//   PB2 = ETX_0
//   PB3 = ETX_1
//   PB4 = ERX_DV
//   PB5 = ERX_0
//   PB6 = ERX_1
//   PB7 = ERX_ER

// structure definitions taken from ASF 3.5.1,  /sam/drivers/emac/emac.h
typedef struct emac_rx_descriptor {
  union emac_rx_addr {
    uint32_t val;
    struct emac_rx_addr_bm {
      uint32_t b_ownership:1, // user clear, EMAC sets this to 1 after rx
      b_wrap:1,   // marks last descriptor in receive buffer 
      addr_dw:30; // address in number of DW
    } bm;
  } addr; // address, wrap & ownership 
  union emac_rx_status {
    uint32_t val;
    struct emac_rx_status_bm {
      uint32_t len:12,       // Length of frame including FCS 
      offset:2,              // rx buf offset, 13:12 of frame len (jumbo frame)
      b_sof:1,               // Start of frame 
      b_eof:1,               // End of frame 
      b_cfi:1,               // Concatenation Format Indicator 
      vlan_priority:3,       // VLAN priority (if VLAN detected) 
      b_priority_detected:1, // Priority tag detected 
      b_vlan_detected:1,     // VLAN tag detected 
      b_type_id_match:1,     // Type ID match
      b_addr4match:1,        // Address register 4 match
      b_addr3match:1,        // Address register 3 match
      b_addr2match:1,        // Address register 2 match
      b_addr1match:1,        // Address register 1 match
      reserved:1,
      b_ext_addr_match:1,    // External address match
      b_uni_hash_match:1,    // Unicast hash match
      b_multi_hash_match:1,  // Multicast hash match
      b_boardcast_detect:1;  // Global broadcast address detected
    } bm;
  } status;
} __attribute__ ((packed, aligned(8))) emac_rx_descriptor_t; 

typedef struct emac_tx_descriptor {
  uint32_t addr;
  union emac_tx_status {
    uint32_t val;
    struct emac_tx_status_bm {
      uint32_t len:11, // length of frame
      reserved:4,
      b_last_buffer:1, // is last buffer in frame?
      b_no_crc:1,      // no crc
      reserved1:10,
      b_exhausted:1,   // buffer exhausted mid frame
      b_underrun:1,    // tx underrun
      b_error:1,       // retry fail... error
      b_wrap:1,        // ring buffer wraparound bit
      b_used:1;        // user clear, EMAC sets this to 1 after tx
    } bm;
  } status;
} __attribute__ ((packed, aligned(8))) emac_tx_descriptor_t;

#define ENET_RX_BUFFERS 16
#define ENET_RX_UNITSIZE 128
volatile static emac_rx_descriptor_t __attribute__((aligned(8)))
                   g_enet_rx_desc[ENET_RX_BUFFERS];
volatile static uint8_t __attribute__((aligned(8))) 
                   g_enet_rx_buf[ENET_RX_BUFFERS * ENET_RX_UNITSIZE];

// keep the TX path simple. single big buffer.
#define ENET_TX_BUFFERS 1
#define ENET_TX_UNITSIZE 1550
volatile static emac_tx_descriptor_t __attribute__((aligned(8)))
                   g_enet_tx_desc[ENET_TX_BUFFERS];
volatile static uint8_t __attribute__((aligned(8)))
                   g_enet_tx_buf[ENET_TX_BUFFERS * ENET_TX_UNITSIZE];

void enet_init()
{
  printf("enet init\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOB);
  PMC->PMC_PCER1 |= (1 << (ID_EMAC - 32));
  PIOB->PIO_ABSR &= ~(PIO_PB0A_ETXCK | PIO_PB1A_ETXEN | 
                      PIO_PB2A_ETX0  | PIO_PB3A_ETX1  |
                      PIO_PB4A_ERXDV | PIO_PB7A_ERXER |
                      PIO_PB5A_ERX0  | PIO_PB6A_ERX1) ; // select peripheral A
  PIOB->PIO_PDR = PIO_PB0A_ETXCK | PIO_PB1A_ETXEN | 
                  PIO_PB2A_ETX0  | PIO_PB3A_ETX1  |
                  PIO_PB4A_ERXDV | PIO_PB7A_ERXER |
                  PIO_PB5A_ERX0  | PIO_PB6A_ERX1  ; // set peripheral control
  EMAC->EMAC_NCR = 0; // disable everything
  EMAC->EMAC_IDR = 0xffffffff; // disable all interrupts
  EMAC->EMAC_NCR |= EMAC_NCR_CLRSTAT; // reset statistics
  EMAC->EMAC_USRIO = EMAC_USRIO_RMII | EMAC_USRIO_CLKEN; // select RMII mode
  EMAC->EMAC_RSR = EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA; // clear rx flags
  EMAC->EMAC_TSR = EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLES |
                   EMAC_TSR_BEX | EMAC_TSR_COMP | EMAC_TSR_UND; // and tx flags
  EMAC->EMAC_ISR; // drain interrupts
  EMAC->EMAC_NCFGR = EMAC_NCFGR_DRFCS     |  // drop FCS from rx packets
                     EMAC_NCFGR_PAE       |  // obey pause frames
                     EMAC_NCFGR_CAF       |  // promiscuous mode
                     EMAC_NCFGR_SPD       |  // 100 megabit
                     EMAC_NCFGR_FD        |  // full duplex
                     EMAC_NCFGR_CLK_MCK_64;  // mdio clock = mdc / 64

  for (int i = 0; i < ENET_RX_BUFFERS; i++)
  {
    g_enet_rx_desc[i].addr.val = (uint32_t)&g_enet_rx_buf[i * ENET_RX_UNITSIZE] 
                                 & 0xfffffffc; // make sure it's 8-byte aligned
    g_enet_rx_desc[i].status.val = 0; 
  }
  g_enet_rx_desc[ENET_RX_BUFFERS-1].addr.bm.b_wrap = 1; // end of ring buffer
  EMAC->EMAC_RBQP = (uint32_t)g_enet_rx_desc & 0xfffffffc;

  for (int i = 0; i < ENET_TX_BUFFERS; i++)
  {
    g_enet_tx_desc[i].addr = (uint32_t)g_enet_tx_desc;
    g_enet_tx_desc[i].status.val = 0; // clear all flags
    g_enet_tx_desc[i].status.bm.b_used = 1; // no need to send this guy
  }
  g_enet_tx_desc[ENET_TX_BUFFERS-1].status.bm.b_wrap = 1; // end of ring 
  EMAC->EMAC_TBQP = (uint32_t)g_enet_tx_desc;

  EMAC->EMAC_NCR |= EMAC_NCR_RE     | // enable receiver
                    EMAC_NCR_WESTAT | // enable stats
                    EMAC_NCR_TE;      // enable transmitter
  EMAC->EMAC_IER = EMAC_IER_RXUBR | // receive used bit read (overrun?)
                   EMAC_IER_ROVR  | // receive overrun
                   EMAC_IER_RCOMP ; // receive complete
  NVIC_EnableIRQ(EMAC_IRQn);
}

void enet_vector()
{
  //return; // haxx just to try to observe status flag as non-zero
  // read the flags to reset the interrupt 
  volatile uint32_t enet_isr = EMAC->EMAC_ISR;
  volatile uint32_t enet_rsr = EMAC->EMAC_RSR;
  volatile uint32_t enet_tsr = EMAC->EMAC_TSR;
  if ((enet_isr & EMAC_ISR_RCOMP) || (enet_rsr & EMAC_RSR_REC))
  {
    volatile uint32_t rsr_clear_flag = EMAC_RSR_REC;
    if (enet_rsr & EMAC_RSR_OVR)
      rsr_clear_flag |= EMAC_RSR_OVR;
    if (enet_rsr & EMAC_RSR_BNA)
      rsr_clear_flag |= EMAC_RSR_BNA;
    EMAC->EMAC_RSR = rsr_clear_flag;
    // TODO: fire callbacks.
    // spin through buffers and mark them as unowned
    for (int i = 0; i < ENET_RX_BUFFERS; i++)
    {
      volatile emac_rx_descriptor_t *desc = &g_enet_rx_desc[i];
      volatile uint8_t *buf = &g_enet_rx_buf[i*ENET_RX_UNITSIZE];
      if (desc->addr.bm.b_ownership)
      {
        printf("%d owned, size %d\r\n", 
               i, desc->status.bm.len);
        desc->addr.bm.b_ownership = 0; // clear buffer
        for (int j = 0; j < desc->status.bm.len; j++)
        {
          printf("%d: 0x%02x\r\n", j, buf[j]);
        }
      }
      //printf("rx\r\n");
      // TODO: do something with data
    }
  }
}

void enet_tx_raw(const uint8_t *pkt, uint16_t pkt_len)
{
  g_enet_tx_desc[0].status.bm.b_used = 0; // we're monkeying with it
  // for now, we just crush whatever is in the tx buffer.
  for (int i = 0; i < pkt_len && i < ENET_TX_UNITSIZE; i++)
    g_enet_tx_buf[i] = pkt[i]; // todo: replace with memcpy someday
  g_enet_tx_desc[0].status.bm.b_last_buffer = 1;
  g_enet_tx_desc[0].status.bm.len = pkt_len;
  EMAC->EMAC_NCR |= EMAC_NCR_TSTART; // kick off TX
}

uint8_t enet_tx_avail()
{
  return (g_enet_tx_desc[0].status.bm.b_last_buffer != 0);
}
