#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

void BoardCriticalSectionBegin(uint32_t *mask);
void BoardCriticalSectionEnd(uint32_t *mask);
void BoardResetMcu(void);
bool NvmmWrite(uint8_t *src, uint16_t size, uint16_t offset);
bool NvmmRead(uint8_t *dest, uint16_t size, uint16_t offset);
uint32_t NvmmCrc32Check(uint16_t size, uint16_t offset);

#endif // __BOARD_H__