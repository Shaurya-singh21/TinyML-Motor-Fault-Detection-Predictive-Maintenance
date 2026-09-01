#include "stm32f446xx.h"
#include "adc.h"
#include "string.h"
typedef enum {
	SAMPLING_START = 1 << 0,
	BUFFER_A_SEND = 1 << 2,
	BUFFER_B_SEND = 1 << 3,
	SAMPLING_STOP = 1 << 4,

} sys_state;

uint8_t bufferA[DMA_Buffer_size];
uint8_t bufferB[DMA_Buffer_size];
volatile uint8_t busy = 0;
volatile uint16_t bytes_left;
volatile uint8_t *ptr;
volatile uint8_t flag;
volatile uint8_t power;

volatile uint8_t *current_dma_buffer = bufferA;
volatile uint16_t sample_count = 0;

void start_data_acq(void) {
	sample_count = 0;
	current_dma_buffer = bufferA;
	GPIOA->BSRR = GPIO_BSRR_BS8;
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void stop_sampling(void) {
	GPIOA->BSRR = GPIO_BSRR_BR8;
	GPIOA->BSRR = GPIO_BSRR_BR5;
	NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR5) {
		EXTI->PR |= EXTI_PR_PR5;

		// 1. Manual I2C Setup Phase
		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB))
			;
		I2C1->DR = 0xD0; // Write Address
		while (!(I2C1->SR1 & I2C_SR1_ADDR))
			;
		(void) I2C1->SR1;
		(void) I2C1->SR2;

		I2C1->DR = 0x3B; // Target register (ACCEL_XOUT_H)
		while (!(I2C1->SR1 & I2C_SR1_TXE))
			;

		I2C1->CR1 |= I2C_CR1_START; // Repeated Start
		while (!(I2C1->SR1 & I2C_SR1_SB))
			;
		I2C1->DR = 0xD1; // Read Address
		while (!(I2C1->SR1 & I2C_SR1_ADDR))
			;
		I2C1->CR1 |= I2C_CR1_ACK;
		// 2. Hand over to DMA for the 6-byte payload
		DMA1_Stream0->M0AR = (uint32_t) &current_dma_buffer[sample_count * 6];
		DMA1_Stream0->NDTR = 6;
		I2C1->CR2 |= I2C_CR2_LAST; // Generate NACK on last byte

		DMA1_Stream0->CR |= DMA_SxCR_EN;
		I2C1->CR2 |= I2C_CR2_DMAEN;

		(void) I2C1->SR1;
		(void) I2C1->SR2;
	}
}
void DMA1_Stream0_IRQHandler(void) {
	if (DMA1->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
		DMA1->LIFCR |= DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
		// clear I2C, reset state, flag error — at minimum don't leave it silently dead
	}
	if (DMA1->LISR & DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;

		I2C1->CR1 |= I2C_CR1_STOP;
		I2C1->CR2 &= ~I2C_CR2_DMAEN;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		sample_count++;

		// Software double-buffer management
		if (sample_count >= SAMPLES_PER_BUFFER) {
			sample_count = 0;
			if (current_dma_buffer == bufferA) {
				current_dma_buffer = bufferB;
				flag |= BUFFER_A_SEND;
			} else {
				current_dma_buffer = bufferA;
				flag |= BUFFER_B_SEND;
			}
		}
	}
}

void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR13) {
		EXTI->PR |= EXTI_PR_PR13;
		if (!power) {
			//means sys stop so run it
			power = 1;
			flag |= SAMPLING_START;
		} else {
			power = 0;
			flag |= SAMPLING_STOP;
		}
	}
}

void USART2_IRQHandler(void) {
	//transmit
	if ((USART2->SR & (USART_SR_TXE)) && (USART2->CR1 & (USART_CR1_TXEIE))) {
		if (bytes_left == 0) {
			USART2->CR1 &= ~(1U << USART_CR1_TXEIE_Pos);
			busy = 0;
		} else {
			USART2->DR = *ptr++;
			--bytes_left;
		}
	}
}

static const uint8_t sync_marker[2] = { 0xAA, 0x55 };

void send_buf(uint8_t *buffer) {
	if (busy)
		return;
	busy = 1;
	// send sync marker first via blocking write, then hand buffer to interrupt-driven send
	while (!(USART2->SR & USART_SR_TXE))
		;
	USART2->DR = sync_marker[0];
	while (!(USART2->SR & USART_SR_TXE))
		;
	USART2->DR = sync_marker[1];
	ptr = buffer;
	bytes_left = DMA_Buffer_size;
	USART2->CR1 |= (USART_CR1_TXEIE);
}

int main(void) {
	clk_init();
	I2C_init();
	MPU6050_Init();
	DMA_Init();
	uart_init();
	gpio_init();
	for (;;) {
		if (flag & SAMPLING_START) {
			flag &= ~SAMPLING_START;
			start_data_acq();
		}
		if (flag & BUFFER_A_SEND) {
			GPIOA->BSRR = GPIO_BSRR_BS5;
			flag &= ~BUFFER_A_SEND;
			send_buf((uint8_t*) bufferA);
		}
		if (flag & BUFFER_B_SEND) {
			GPIOA->BSRR = GPIO_BSRR_BR5;
			flag &= ~BUFFER_B_SEND;
			send_buf((uint8_t*) bufferB);
		}
		if (flag & SAMPLING_STOP) {
			flag &= ~SAMPLING_STOP;
			stop_sampling();
		}
	}
}
