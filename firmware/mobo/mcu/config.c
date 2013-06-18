#include "config.h"
static uint32_t g_bl_hw_version = 0; // bogus, will overwrite at startup

uint8_t config_init()
{
  g_bl_hw_version = *((uint32_t *)(0x0401ff8));
}

uint8_t config_is_right_hand()
{

}

