#ifndef __ADC_H
#define __ADC_H

#define SAMPLES_PER_BUFFER 1000U
#define DMA_Buffer_size (SAMPLES_PER_BUFFER * 6U) // 6000 bytes
#define UART2_BAUD_RATE 0x008B

extern uint8_t bufferA[DMA_Buffer_size];
extern uint8_t bufferB[DMA_Buffer_size];

void clk_init();
void gpio_init();
void I2C_init();
void DMA_Init();
void MPU6050_Init();
void uart_init();

#endif
