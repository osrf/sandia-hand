#ifndef FLASH_H
#define FLASH_H

void flash_init();
void flash_read_page(const uint32_t page_num, uint8_t *page_data); // 256 bytes

#endif

