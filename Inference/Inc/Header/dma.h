#ifndef __DMA_H
#define __DMA_H
#include "stdint.h"

#define SAMPLES_PER_BUFFER 128U
#define DMA_Buffer_size (SAMPLES_PER_BUFFER * 6U)
void dma_init(void);
void oled_dma_send(const uint8_t *buf, uint16_t len);

#endif
