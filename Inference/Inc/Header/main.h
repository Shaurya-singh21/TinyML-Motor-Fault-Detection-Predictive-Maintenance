#ifndef __MAIN_H
#define __MAIN_H

typedef enum
{
    SYS_START_BUTTON_PRIORITY        = 15,
    I2C_RX_PRIORITY                  = 5,
    DMA1_STREAM0_CHANNEL1_PRIORITY   = 6,
    DMA1_STREAM6_CHANNEL4_PRIORITY   = 4,
    DMA1_STREAM4_CHANNEL3_PRIORITY   = 6,
    I2C3_ERROR_PRIORITY              = 6

} InterruptPriorityLevels;


typedef enum {
	SYS_START_TASK = 9,
	PROCESS_DATA_TASK = 8,
	INFERENCE_TASK = 7,
	UART_TASK = 6,  
    DISPLAY_TASK = 5
} TaskPriorityLevels;

#define FEATURE_SIZE 15U
#define OUTPUT_SIZE 4U

#endif
