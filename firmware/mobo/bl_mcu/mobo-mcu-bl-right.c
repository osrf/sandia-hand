#include "stdint.h"
const uint32_t __attribute__((section(".bl_hw_version")))
                                g_bl_hw_version = 0xbeef5241; // 0x52 = 'R'
