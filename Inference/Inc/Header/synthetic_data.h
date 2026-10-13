#ifndef __SYNTHETIC_DATA_H
#define __SYNTHETIC_DATA_H
#include "stm32f446xx.h"
void store_be_i16(uint8_t *dst, int16_t value);
void prepare_synthetic_window(uint32_t window_id);
#endif