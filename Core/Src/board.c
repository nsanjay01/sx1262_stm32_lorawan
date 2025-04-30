#include "board.h"
#include "stm32f4xx.h"
#include <string.h>

// RAM buffer for NVM (volatile, for testing)
static uint8_t nvm_data[1024] __attribute__((section(".bss")));  // 1 KB

// Critical section functions
void BoardCriticalSectionBegin(uint32_t *mask) {
    *mask = __get_PRIMASK();
    __disable_irq();
}

void BoardCriticalSectionEnd(uint32_t *mask) {
    __enable_irq();  // Fallback due to __set_PRIMASK issue
    // If fixed, use: __set_PRIMASK(*mask);
}

// Reset the MCU
void BoardResetMcu(void) {
    NVIC_SystemReset();
}

// NVM functions (using RAM for testing)
#define NVM_SIZE (1024)  // 1 KB

bool NvmmWrite(uint8_t *src, uint16_t size, uint16_t offset) {
    if (offset + size > NVM_SIZE) {
        return false;
    }
    memcpy(&nvm_data[offset], src, size);
    return true;
}

bool NvmmRead(uint8_t *dest, uint16_t size, uint16_t offset) {
    if (offset + size > NVM_SIZE) {
        return false;
    }
    memcpy(dest, &nvm_data[offset], size);
    return true;
}

uint32_t NvmmCrc32Check(uint16_t size, uint16_t offset) {
    if (offset + size > NVM_SIZE) {
        return 0;
    }
    uint32_t crc = 0xFFFFFFFF;
    for (uint16_t i = 0; i < size; i++) {
        uint8_t data = nvm_data[offset + i];
        crc ^= data;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}