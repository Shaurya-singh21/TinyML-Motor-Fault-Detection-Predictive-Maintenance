#ifndef __DMA_H
#define __DMA_H

#define SAMPLES_PER_BUFFER 128U
#define DMA_Buffer_size (SAMPLES_PER_BUFFER * 6U)
void dma_init(void);

#endif
