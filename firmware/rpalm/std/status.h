#ifndef STATUS_H
#define STATUS_H

#include "palm_status.h"

extern palm_status_t g_status;

void status_init();
void status_idle();
void status_systick();
void status_tc0_irq();

#endif

