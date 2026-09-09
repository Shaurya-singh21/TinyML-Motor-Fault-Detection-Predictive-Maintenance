#include "stm32f446xx.h"
#include "main.h"
#include "i2c.h"

static inline int i2c_wait_sr1(uint32_t mask, uint32_t timeout)
{
	while (!(I2C1->SR1 & mask))
	{
		if (--timeout == 0U)
			return -1;
	}
	return 0;
}

void i2c3_oled_init(void)
{
	I2C3->CR1 &= ~I2C_CR1_PE;

	// 2. Drive PA8 (SCL) / PC9 (SDA) as plain open-drain outputs first, so we
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= (1U << GPIO_MODER_MODER8_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_8;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;

	GPIOC->MODER &= ~GPIO_MODER_MODER9;
	GPIOC->MODER |= (1U << GPIO_MODER_MODER9_Pos);
	GPIOC->OTYPER |= GPIO_OTYPER_OT_9;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;

	// 3. Manual STOP condition toggle (same recovery sequence as i2c_init)
	GPIOA->BSRR = GPIO_BSRR_BS8; // SCL High
	GPIOC->BSRR = GPIO_BSRR_BS9; // SDA High
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOC->BSRR = GPIO_BSRR_BR9; // SDA Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOA->BSRR = GPIO_BSRR_BR8; // SCL Low
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOA->BSRR = GPIO_BSRR_BS8; // SCL High
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOC->BSRR = GPIO_BSRR_BS9; // SDA High
	for (volatile int i = 0; i < 1000; i++)
		;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= (2U << GPIO_MODER_MODER8_Pos);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;
	GPIOA->PUPDR |= (1U << GPIO_PUPDR_PUPD8_Pos);
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
	GPIOA->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos);

	GPIOC->MODER &= ~GPIO_MODER_MODER9;
	GPIOC->MODER |= (2U << GPIO_MODER_MODER9_Pos);
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9;
	GPIOC->PUPDR |= (1U << GPIO_PUPDR_PUPD9_Pos);
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL9;
	GPIOC->AFR[1] |= (4U << GPIO_AFRH_AFSEL9_Pos);

	// 5. SWRST Toggle
	I2C3->CR1 |= I2C_CR1_SWRST;
	for (volatile int i = 0; i < 100; i++)
		;
	I2C3->CR1 &= ~I2C_CR1_SWRST;

	// 6. Configure Timings (APB1 is 16MHz: no PLL, running on HSI)
	//    Fast mode 400kHz: CCR = 16MHz / (400kHz * 3) = 13, TRISE = 300ns*16MHz+1 = 6
	I2C3->CR2 = (16U << I2C_CR2_FREQ_Pos);
	I2C3->CCR = I2C_CCR_FS | (13U << I2C_CCR_CCR_Pos);
	I2C3->TRISE = (6U << I2C_TRISE_TRISE_Pos);
	I2C3->CR2 |= I2C_CR2_ITERREN;
	NVIC_SetPriority(I2C3_ER_IRQn, I2C3_ERROR_PRIORITY);
	NVIC_EnableIRQ(I2C3_ER_IRQn);

	I2C3->CR1 |= I2C_CR1_PE;
}

void I2C3_ER_IRQHandler(void)
{
	uint32_t sr1 = I2C3->SR1;
	if (sr1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR))
	{
		I2C3->SR1 &= ~(I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR);
		I2C3->CR2 &= ~I2C_CR2_DMAEN;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
		I2C3->CR1 |= I2C_CR1_STOP;
		oled_dma_complete(); // un-wedges dma_busy — this is the fix your symptom needs
	}
}

// volatile uint8_t stp = 0;

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

int I2C_WriteReg(uint8_t reg, uint8_t data)
{
	I2C1->CR1 |= I2C_CR1_START;
	if (i2c_wait_sr1(I2C_SR1_SB, I2C_FLAG_TIMEOUT) < 0)
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return -1;
	}

	I2C1->DR = 0xD0; // Write Address
	if (i2c_wait_sr1(I2C_SR1_ADDR, I2C_FLAG_TIMEOUT) < 0)
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return -1;
	}

	(void)I2C1->SR1;
	(void)I2C1->SR2;

	I2C1->DR = reg;
	if (i2c_wait_sr1(I2C_SR1_TXE, I2C_FLAG_TIMEOUT) < 0)
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return -1;
	}

	I2C1->DR = data;
	if (i2c_wait_sr1(I2C_SR1_BTF, I2C_FLAG_TIMEOUT) < 0)
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return -1;
	}

	I2C1->CR1 |= I2C_CR1_STOP;
	uint32_t stop_timeout = I2C_FLAG_TIMEOUT;
	while (I2C1->CR1 & I2C_CR1_STOP)
	{
		if (--stop_timeout == 0U)
			break;
	}
	return 0;
}
volatile uint8_t mpu6050_ok = 0;

void MPU6050_Init(void)
{
	int status = 0;
	status |= I2C_WriteReg(0x6B, 0x00); // Wake up
	status |= I2C_WriteReg(0x19, 0x00); // SMPLRT_DIV = 0 (1kHz sample rate)
	status |= I2C_WriteReg(0x1A, 0x03); // CONFIG: DLPF = 42Hz, sets internal clock to 1kHz
	status |= I2C_WriteReg(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g for maximum sensitivity on tiny vibrations
	status |= I2C_WriteReg(0x38, 0x01); // INT_ENABLE: Enable Data Ready Interrupt

	mpu6050_ok = (status == 0) ? 1U : 0U;
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
		if (i2c_wait_sr1(I2C_SR1_SB, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP; // release the bus, drop this cycle
			return;
		}
		I2C1->DR = 0xD0; // Write Adress
		if (i2c_wait_sr1(I2C_SR1_ADDR, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return;
		}
		(void)I2C1->SR1;
		(void)I2C1->SR2;

		I2C1->DR = 0x3B; // Target register (ACCEL_XOUT_H)
		if (i2c_wait_sr1(I2C_SR1_TXE, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return;
		}
		if (i2c_wait_sr1(I2C_SR1_BTF, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return;
		}
		I2C1->CR1 |= I2C_CR1_START; // Repeated Start
		if (i2c_wait_sr1(I2C_SR1_SB, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return;
		}
		I2C1->DR = 0xD1; // Read Address
		if (i2c_wait_sr1(I2C_SR1_ADDR, I2C_FLAG_TIMEOUT) < 0)
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return;
		}
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
