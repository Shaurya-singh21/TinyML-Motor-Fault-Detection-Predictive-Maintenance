#include "FreeRTOS.h"
#include "task.h"
#include "stm32f446xx.h"
#include "dma.h"
#include "main.h"
#include "queue.h"

extern uint8_t bufferA[DMA_Buffer_size];
extern uint8_t bufferB[DMA_Buffer_size];

extern volatile uint8_t *current_dma_buffer;
extern volatile uint16_t sample_count;
extern QueueHandle_t xBufferPtrQueueHandle;

void dma_init(void) {
	// dma_1_stream0_channel1 for i2c1_rx
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream0->CR & DMA_SxCR_EN)
		;
	DMA1_Stream0->PAR = (uint32_t) &I2C1->DR;
	DMA1_Stream0->CR &=
			~((7U << DMA_SxCR_CHSEL_Pos) | (3U << DMA_SxCR_DIR_Pos));
	DMA1_Stream0->CR |= (1U << DMA_SxCR_CHSEL_Pos); // Channel 1

	DMA1_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE; // Mem increment, Transfer Complete Interrupt
	DMA1_Stream0->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
	NVIC_SetPriority(DMA1_Stream0_IRQn, DMA1_STREAM0_CHANNEL1_PRIORITY);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);

	// dma_1_stream7_channel7 for i2c2_tx for oled
//	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
//	while (DMA1_Stream7->CR & DMA_SxCR_EN)
//		;
//	DMA1_Stream7->PAR = (uint32_t) &I2C2->DR;
//	DMA1_Stream7->CR &=
//			~((7U << DMA_SxCR_CHSEL_Pos) | (3U << DMA_SxCR_DIR_Pos));
//	DMA1_Stream7->CR |= (7U << DMA_SxCR_CHSEL_Pos); // Channel 7
//	DMA1_Stream7->CR |= (1U << DMA_SxCR_DIR_Pos);	// Memory to Peripheral
//
//	DMA1_Stream7->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE; // Mem increment, Transfer Complete Interrupt
//	DMA1_Stream7->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
//
//	NVIC_SetPriority(DMA1_Stream7_IRQn, DMA1_STREAM7_CHANNEL7_PRIORITY);
//	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	// dma_1_stream6_channel4 for uart2_tx
	DMA1_Stream6->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream6->CR & DMA_SxCR_EN)
		;
	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream6->CR &= ~((7U << DMA_SxCR_CHSEL_Pos) | (3U << DMA_SxCR_DIR_Pos)
			| (1U << DMA_SxCR_CIRC_Pos));
	DMA1_Stream6->CR |= (4U << DMA_SxCR_CHSEL_Pos); // Channel 4
	DMA1_Stream6->CR |= (1U << DMA_SxCR_DIR_Pos);	// Memory to Peripheral

	DMA1_Stream6->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE; // Mem increment, Transfer Complete Interrupt
	DMA1_Stream6->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);

	NVIC_SetPriority(DMA1_Stream6_IRQn, DMA1_STREAM6_CHANNEL4_PRIORITY);
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}
//uart_dma
void DMA1_Stream6_IRQHandler(void) {
	if (DMA1->HISR & (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) {
		DMA1->HIFCR |= DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
	}
	if (DMA1->HISR & DMA_HISR_TCIF6) {
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
		DMA1_Stream6->CR &= ~DMA_SxCR_EN;
	}
}
//i2c_rx_dma
void DMA1_Stream0_IRQHandler(void) {
	if (DMA1->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
		DMA1->LIFCR |= DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
	}
	if (DMA1->LISR & DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;

		I2C1->CR1 |= I2C_CR1_STOP;
		I2C1->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		sample_count++;

		// Software double-buffer management
		if (sample_count >= SAMPLES_PER_BUFFER) {
			BaseType_t xHighPriority = pdFALSE;
			sample_count = 0;
			if (current_dma_buffer == bufferA) {
				uint8_t *ptrA = bufferA;
				xQueueSendToBackFromISR(xBufferPtrQueueHandle, &ptrA,
						&xHighPriority);
				current_dma_buffer = bufferB;
				GPIOA->BSRR = GPIO_BSRR_BS5;
			} else {
				uint8_t *ptrB = bufferB;
				// 2. Pass the address OF the variable
				xQueueSendToBackFromISR(xBufferPtrQueueHandle, &ptrB,
						&xHighPriority);
				current_dma_buffer = bufferA;
				GPIOA->BSRR = GPIO_BSRR_BR5;
			}
		}
	}
}
//void oled_dma_send(const uint8_t *buf, uint16_t len) {
//	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
//	while (DMA1_Stream7->CR & DMA_SxCR_EN)
//		;
//
//	DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7 |
//	DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 |
//	DMA_HIFCR_CHTIF7;
//
//	DMA1_Stream7->M0AR = (uint32_t) buf;
//	DMA1_Stream7->NDTR = len;
//
//	I2C2->CR2 |= I2C_CR2_DMAEN;
//	DMA1_Stream7->CR |= DMA_SxCR_EN;
//}
//
//void DMA1_Stream7_IRQHandler(void) {
//	if (DMA1->HISR & DMA_HISR_TCIF7) {
//		DMA1->HIFCR = DMA_HIFCR_CTCIF7;

//		uint32_t t = 20000;
//		while (!(I2C2->SR1 & I2C_SR1_BTF) && --t)
//			;
//
//		I2C2->CR1 |= I2C_CR1_STOP;
//		I2C2->CR2 &= ~I2C_CR2_DMAEN;
//
//		extern void oled_dma_complete(void);
//		oled_dma_complete();
//	}
//}
