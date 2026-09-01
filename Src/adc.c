#include "stm32f446xx.h"
#include "adc.h"
volatile uint8_t stp = 0;
void clk_init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN
			| RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}
void gpio_init(void) {
	//pa5 led
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= 1U << GPIO_MODER_MODE5_Pos;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOA->OSPEEDR |= (2U << GPIO_OSPEEDR_OSPEED5_Pos);

	GPIOA->MODER &= ~GPIO_MODER_MODE8;
	GPIOA->MODER |= 1U << GPIO_MODER_MODE8_Pos;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8;
	GPIOA->OSPEEDR |= (2U << GPIO_OSPEEDR_OSPEED8_Pos);

	//pc13 for button
	GPIOC->MODER &= ~GPIO_MODER_MODE13;
	SYSCFG->EXTICR[3] |= 2U << SYSCFG_EXTICR4_EXTI13_Pos;
	EXTI->IMR |= EXTI_IMR_IM13;
	EXTI->FTSR |= EXTI_FTSR_TR13;
	NVIC_SetPriority(EXTI15_10_IRQn, 1);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// MPU6050 INT pin on PB5 (Input, Pull-down)
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOB->PUPDR |= (2U << GPIO_PUPDR_PUPD5_Pos);
	SYSCFG->EXTICR[1] |= (1U << SYSCFG_EXTICR2_EXTI5_Pos);
	EXTI->IMR |= EXTI_IMR_IM5;
	EXTI->RTSR |= EXTI_RTSR_TR5;
	NVIC_SetPriority(EXTI9_5_IRQn, 0);

}
void I2C_init(void) {
	// 1. Enable Clocks for GPIOB and I2C1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	// 2. Ensure I2C1 is disabled
	I2C1->CR1 &= ~I2C_CR1_PE;

	// 3. Configure PB8 (SCL) & PB9 (SDA) as Output Open-Drain
	GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOB->MODER |= (1U << GPIO_MODER_MODER8_Pos)
			| (1U << GPIO_MODER_MODER9_Pos);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);

	// 4. Manual STOP condition toggle
	GPIOB->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS9;
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BR9;  // SDA Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BR8;  // SCL Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BS8;  // SCL High
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BS9;  // SDA High
	for (volatile int i = 0; i < 1000; i++)
		;

	// 5. Configure PB8 and PB9 to Alternate Function 4 (AF4)
	GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOB->MODER |= (2U << GPIO_MODER_MODER8_Pos)
			| (2U << GPIO_MODER_MODER9_Pos);
	GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPD8_Pos) | (1U << GPIO_PUPDR_PUPD9_Pos);

	// Note: PB8 and PB9 are in AFR[1] (AFRH)
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos)
			| (4U << GPIO_AFRH_AFSEL9_Pos);

	// 6. SWRST Toggle
	I2C1->CR1 |= I2C_CR1_SWRST;
	for (volatile int i = 0; i < 100; i++)
		;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	// 7. Configure Timings (Assuming 16MHz APB1)
	I2C1->CR2 = (16U << I2C_CR2_FREQ_Pos);
	I2C1->CCR = I2C_CCR_FS | (13U << I2C_CCR_CCR_Pos);
	I2C1->TRISE = (6U << I2C_TRISE_TRISE_Pos);

	// 8. Enable I2C1
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;

	if (I2C2->SR2 & I2C_SR2_BUSY) {
		GPIOA->BSRR = GPIO_BSRR_BS5;
	}
}
//void I2C_init(void) {
////	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
////	GPIOB->MODER |= (1U << GPIO_MODER_MODER10_Pos) | (1U << GPIO_MODER_MODER11_Pos); // Output
////	GPIOB->OTYPER |= (GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11); // Open drain
////	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);
////	GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPD10_Pos) | (1U << GPIO_PUPDR_PUPD11_Pos); // Pull-ups
////
////	// 2. Manually clock SCL up to 9 times to free the slave
////	for (int i = 0; i < 9; i++) {
////	    GPIOB->BSRR = GPIO_BSRR_BR10; // SCL Low
////	    for(volatile int d = 0; d < 1000; d++); // Arbitrary delay
////	    GPIOB->BSRR = GPIO_BSRR_BS10; // SCL High
////	    for(volatile int d = 0; d < 1000; d++);
////
////	    // If SDA goes high, the bus is free
////	    if (GPIOB->IDR & GPIO_IDR_ID11) {
////	        break;
////	    }
////	}
////
////	// 3. Generate a manual STOP condition
////	GPIOB->BSRR = GPIO_BSRR_BR11; // SDA Low
////	for(volatile int d = 0; d < 1000; d++);
////	GPIOB->BSRR = GPIO_BSRR_BS10; // SCL High
////	for(volatile int d = 0; d < 1000; d++);
////	GPIOB->BSRR = GPIO_BSRR_BS11; // SDA High
////	for(volatile int d = 0; d < 1000; d++);
//	//PB10 (SCL) and PB11 (SDA)
//
//	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
//	GPIOB->MODER |= (2U << GPIO_MODER_MODER10_Pos)
//			| (2U << GPIO_MODER_MODER11_Pos);
//	GPIOB->OTYPER |= (GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11);
//	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11);
//	// Clear PUPDR bits, then set to Pull-Up (01)
//	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);
//	GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPD10_Pos)
//			| (1U << GPIO_PUPDR_PUPD11_Pos);
//	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
//	GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL10_Pos)
//			| (4U << GPIO_AFRH_AFSEL11_Pos);
//
//	//I2C2
//	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
//	I2C2->CR1 |= I2C_CR1_SWRST;
//	I2C2->CR1 &= ~I2C_CR1_SWRST;
//	I2C2->CR2 |= (16U << I2C_CR2_FREQ_Pos);
//	I2C2->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);
//	// Set Fast Mode (FS) and write the calculated CCR value (13)
//	I2C2->CCR |= I2C_CCR_FS | (13U << I2C_CCR_CCR_Pos);
//	I2C2->TRISE &= ~I2C_TRISE_TRISE;
//	I2C2->TRISE |= (6U << I2C_TRISE_TRISE_Pos);
//	I2C2->CR1 |= I2C_CR1_PE;
//}
void I2C_WriteReg(uint8_t reg, uint8_t data) {
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB))
		;

	I2C1->DR = 0xD0; // Write Address
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		;

	(void) I2C1->SR1;
	(void) I2C1->SR2;

	I2C1->DR = reg;
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		;

	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_BTF))
		;

	I2C1->CR1 |= I2C_CR1_STOP;
}
void MPU6050_Init(void) {
	I2C_WriteReg(0x6B, 0x00); // Wake up
	stp++;
	I2C_WriteReg(0x19, 0x00); // SMPLRT_DIV = 0 (1kHz sample rate)
	stp++;
	I2C_WriteReg(0x1A, 0x03); // CONFIG: DLPF = 42Hz, sets internal clock to 1kHz
	stp++;
	I2C_WriteReg(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g for maximum sensitivity on tiny vibrations
	stp++;
	I2C_WriteReg(0x38, 0x01); // INT_ENABLE: Enable Data Ready Interrupt
	stp++;
}

void DMA_Init(void) {
	// DMA1 Stream 0 Channel 1 (I2C1_RX)
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	DMA1_Stream0->PAR = (uint32_t) &I2C1->DR;

	// Clear channel and direction bits, then set to Channel 1
	DMA1_Stream0->CR &=
			~((7U << DMA_SxCR_CHSEL_Pos) | (3U << DMA_SxCR_DIR_Pos));
	DMA1_Stream0->CR |= (1U << DMA_SxCR_CHSEL_Pos); // Channel 1

	DMA1_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE; // Mem increment, Transfer Complete Interrupt
	DMA1_Stream0->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE); // 8-bit memory and peripheral

	NVIC_SetPriority(DMA1_Stream0_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	//dma2_stream0_channel0
//	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
//	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
//	DMA2_Stream0->M0AR = (uint32_t) &bufferA[0];
//	DMA2_Stream0->M1AR = (uint32_t) &bufferB[0];
//	DMA2_Stream0->NDTR = (uint16_t) DMA_Buffer_size;
//	DMA2_Stream0->CR &= ~((7U << DMA_SxCR_CHSEL_Pos) | (3U << DMA_SxCR_DIR_Pos)
//			| (1U << DMA_SxCR_CT_Pos));
	DMA2_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE
			| DMA_SxCR_CIRC;
//
//	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;   // 16-bit memory
//	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;
//	//double buffer mode
//	DMA2_Stream0->CR |= DMA_SxCR_DBM;
//	DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 |
//	DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 |
//	DMA_LIFCR_CHTIF0;
//	NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
//	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void uart_init(void) {
	//usart2 for data logging (pa2 for tx)
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);
	GPIOA->MODER |= (2U << GPIO_MODER_MODE2_Pos);
	GPIOA->AFR[0] |= (7U << GPIO_AFRL_AFSEL2_Pos);
	GPIOA->OTYPER &= ~((1U << 2));
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED2_Pos);

	//usart registers
	USART2->BRR = UART2_BAUD_RATE;
	USART2->CR1 |= (1U << USART_CR1_TE_Pos);
	NVIC_SetPriority(USART2_IRQn, 3);
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1 |= (1U << USART_CR1_UE_Pos);
}
