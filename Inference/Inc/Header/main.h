#ifndef __MAIN_H
#define __MAIN_H

typedef enum 
{
    SYS_START_BUTTON_PRIORITY = 6, //sys start button
    I2C_RX_PRIORITY = 7, //pb5 from mpu6050
//    I2C_TX_PRIORITY = 9,
    DMA1_STREAM0_CHANNEL1_PRIORITY = 11,
    DMA1_STREAM6_CHANNEL4_PRIORITY = 12,
//    DMA1_STREAM7_CHANNEL7_PRIORITY = 13,
} InterruptPriorityLevels;

typedef enum {
	SYS_START_TASK = 9,
	PROCESS_DATA_TASK = 8,
	INFERENCE_TASK = 7,
	UART_TASK = 5
} TaskPriorityLevels;

#define FEATURE_SIZE 15U
#define OUTPUT_SIZE 4U

#endif
