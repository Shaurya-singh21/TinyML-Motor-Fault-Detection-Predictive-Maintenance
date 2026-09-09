#include "stm32f446xx.h"
#include "main.h"
#include "i2c.h"

void i2c2_oled_init(void)
{
	// 1. Manual Bus Recovery (Frees the stuck OLED)
	// Configure PB10/PB11 as standard GPIO outputs temporarily
	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	GPIOB->MODER |= (1U << GPIO_MODER_MODER10_Pos) | (1U << GPIO_MODER_MODER11_Pos);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11); // Open Drain

	// Toggle SCL (PB10) and SDA (PB11) to generate a manual STOP condition
	GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BS11;
	for (volatile int i = 0; i < 1000; i++)
		;
	GPIOB->BSRR = GPIO_BSRR_BR11; // SDA Low
	for (volatile int i = 0; i < 1000; i++)
		;
	GPIOB->BSRR = GPIO_BSRR_BR10; // SCL Low
	for (volatile int i = 0; i < 1000; i++)
		;
	GPIOB->BSRR = GPIO_BSRR_BS10; // SCL High
	for (volatile int i = 0; i < 1000; i++)
		;
	GPIOB->BSRR = GPIO_BSRR_BS11; // SDA High
	for (volatile int i = 0; i < 1000; i++)
		;

	// 2. Switch to Alternate Function (Hardware I2C control)
	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	GPIOB->MODER |= (2U << GPIO_MODER_MODER10_Pos) | (2U << GPIO_MODER_MODER11_Pos);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
	GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL10_Pos) | (4U << GPIO_AFRH_AFSEL11_Pos);

	// 3. CRITICAL: Hardware Software Reset for I2C2 (Fixed copy-paste error)
	I2C2->CR1 |= I2C_CR1_SWRST;
	for (volatile int i = 0; i < 1000; i++)
		;
	I2C2->CR1 &= ~I2C_CR1_SWRST;

	// 4. Configure Timings & Enable
	I2C2->CR1 &= ~I2C_CR1_PE;
	I2C2->CR2 = (16U << I2C_CR2_FREQ_Pos);
	I2C2->CCR = I2C_CCR_FS | (13U << I2C_CCR_CCR_Pos);
	I2C2->TRISE = (6U << I2C_TRISE_TRISE_Pos);

	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR2 |= I2C_CR2_DMAEN;
}
volatile uint8_t stp = 0;

void i2c_init(void)
{
	// 1. Enable Clocks for GPIOB and I2C1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	// 2. Ensure I2C1 is disabled
	I2C1->CR1 &= ~I2C_CR1_PE;

	// 3. Configure PB8 (SCL) & PB9 (SDA) as Output Open-Drain
	GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOB->MODER |= (1U << GPIO_MODER_MODER8_Pos) | (1U << GPIO_MODER_MODER9_Pos);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);

	// 4. Manual STOP condition toggle
	GPIOB->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS9;
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BR9; // SDA Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BR8; // SCL Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BS8; // SCL High
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOB->BSRR = GPIO_BSRR_BS9; // SDA High
	for (volatile int i = 0; i < 1000; i++)
		;

	// 5. Configure PB8 and PB9 to Alternate Function 4 (AF4)
	GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOB->MODER |= (2U << GPIO_MODER_MODER8_Pos) | (2U << GPIO_MODER_MODER9_Pos);
	GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPD8_Pos) | (1U << GPIO_PUPDR_PUPD9_Pos);

	// Note: PB8 and PB9 are in AFR[1] (AFRH)
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos) | (4U << GPIO_AFRH_AFSEL9_Pos);

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
}
extern volatile uint8_t *current_dma_buffer;
extern volatile uint16_t sample_count;

void I2C_WriteReg(uint8_t reg, uint8_t data)
{
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB))
		;

	I2C1->DR = 0xD0; // Write Address
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		;

	(void)I2C1->SR1;
	(void)I2C1->SR2;

	I2C1->DR = reg;
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		;

	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_BTF))
		;

	I2C1->CR1 |= I2C_CR1_STOP;
	while (I2C1->CR1 & I2C_CR1_STOP)
		;
}
void MPU6050_Init(void)
{
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

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & EXTI_PR_PR5)
	{
		EXTI->PR = EXTI_PR_PR5;
		if (I2C1->SR2 & I2C_SR2_BUSY)
		{
			return; // bus not idle yet, drop this sample cycle
		}
		// 1. Manual I2C Setup Phase
		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB))
			;
		I2C1->DR = 0xD0; // Write Address
		while (!(I2C1->SR1 & I2C_SR1_ADDR))
			;
		(void)I2C1->SR1;
		(void)I2C1->SR2;

		I2C1->DR = 0x3B; // Target register (ACCEL_XOUT_H)
		while (!(I2C1->SR1 & I2C_SR1_TXE))
			;
		while (!(I2C1->SR1 & I2C_SR1_BTF))
			;
		I2C1->CR1 |= I2C_CR1_START; // Repeated Start
		while (!(I2C1->SR1 & I2C_SR1_SB))
			;
		I2C1->DR = 0xD1; // Read Address
		while (!(I2C1->SR1 & I2C_SR1_ADDR))
			;
		// 2. Hand over to DMA for the 6-byte payload
		DMA1_Stream0->M0AR = (uint32_t)&current_dma_buffer[sample_count * 6];
		DMA1_Stream0->NDTR = 6;
		I2C1->CR2 |= I2C_CR2_LAST; // Generate NACK on last byte

		DMA1_Stream0->CR |= DMA_SxCR_EN;
		I2C1->CR2 |= I2C_CR2_DMAEN;

		(void)I2C1->SR1;
		(void)I2C1->SR2;
	}
}

// void EXTI9_5_IRQHandler(void) {
//     if (EXTI->PR & EXTI_PR_PR5) {
//         EXTI->PR = EXTI_PR_PR5;
//
//         // --- Phase 1: START + Write address (0xD0) ---
//         I2C1->CR1 |= I2C_CR1_START;
//         while (!(I2C1->SR1 & I2C_SR1_SB));
//
//         I2C1->DR = 0xD0;
//         while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)));
//         if (I2C1->SR1 & I2C_SR1_AF) {
//             I2C1->SR1 &= ~I2C_SR1_AF;
//             I2C1->CR1 |= I2C_CR1_STOP;
//             return;
//         }
//         (void)I2C1->SR1;
//         (void)I2C1->SR2;
//
//         // --- Phase 2: Write target register (0x3B) ---
//         I2C1->DR = 0x3B;
//         while (!(I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_AF)));
//         if (I2C1->SR1 & I2C_SR1_AF) {
//             I2C1->SR1 &= ~I2C_SR1_AF;
//             I2C1->CR1 |= I2C_CR1_STOP;
//             return;
//         }
//
//         while (!(I2C1->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)));
//         if (I2C1->SR1 & I2C_SR1_AF) {
//             I2C1->SR1 &= ~I2C_SR1_AF;
//             I2C1->CR1 |= I2C_CR1_STOP;
//             return;
//         }
//
//         // --- Phase 3: Repeated START + Read address (0xD1) ---
//         I2C1->CR1 |= I2C_CR1_START;
//         while (!(I2C1->SR1 & I2C_SR1_SB));
//
//         I2C1->DR = 0xD1;
//         while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)));
//         if (I2C1->SR1 & I2C_SR1_AF) {
//             I2C1->SR1 &= ~I2C_SR1_AF;
//             I2C1->CR1 |= I2C_CR1_STOP;
//             return;
//         }
//
//         // --- Phase 4: Hand off to DMA for the 6-byte payload ---
//         // ADDR must be cleared (SR1 then SR2 read) *after* CR2 is armed for DMA,
//         // per RM0390: clearing ADDR is what releases the clock stretch and lets
//         // the first data byte start clocking in, so DMAEN/LAST must already be set.
//         DMA1_Stream0->M0AR = (uint32_t)&current_dma_buffer[sample_count * 6];
//         DMA1_Stream0->NDTR = 6;
//         I2C1->CR2 |= I2C_CR2_LAST;
//         DMA1_Stream0->CR |= DMA_SxCR_EN;
//         I2C1->CR2 |= I2C_CR2_DMAEN;
//
//         (void)I2C1->SR1;
//         (void)I2C1->SR2;
//     }
// }
