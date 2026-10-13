#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f446xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "clk.h"
#include "i2c.h"
#include "dma.h"
#include "gpio.h"
#include "uart.h"
#include "queue.h"
#include "arm_math.h"
#include "string.h"
#include "ssd1306.h"
#include "math.h"
#include "stdio.h"
#include "synthetic_data.h"

typedef enum {
	SYS_START_BUTTON_PRIORITY = 15,
	I2C_RX_PRIORITY = 5,
	DMA1_STREAM0_CHANNEL1_PRIORITY = 6,
	DMA1_STREAM6_CHANNEL4_PRIORITY = 4,
	DMA1_STREAM4_CHANNEL3_PRIORITY = 6,
	I2C3_ERROR_PRIORITY = 6

} InterruptPriorityLevels;

typedef enum {
	SYS_START_TASK = 9,
	PROCESS_DATA_TASK = 8,
	INFERENCE_TASK = 7,
	UART_TASK = 6,
	DISPLAY_TASK = 5
} TaskPriorityLevels;

typedef enum {
	PROCESS_STACK_SIZE = 256 ,
	INFERENCE_STACK_SIZE = 256 ,  
	DISPLAY_STACK_SIZE = 256 ,
	UART_STACK_SIZE = 128  ,
	START_STOP_STACK_SIZE = 128,
	SYNTHETIC_STACK_SIZE = 256
} StackSizes;


#define USE_SYNTHETIC_DATA 1
#define FEATURE_SIZE 15U
#define OUTPUT_SIZE 4U
#define CORE_CLOCK_HZ          16000000UL
#define WINDOW_BUDGET_MS        128UL
#define WINDOW_BUDGET_CYCLES    \
    ((uint64_t)CORE_CLOCK_HZ * WINDOW_BUDGET_MS / 1000ULL)
#define TIMING_AVERAGE_WINDOWS  10
typedef struct __attribute__((packed)) {
    float features[FEATURE_SIZE];
    int32_t prediction;
    int32_t maj_vote_prediction;
    uint32_t feature_cycles;
    uint32_t rf_cycles;
    uint32_t window_cost_pct_x100;  // 12.34% is sent as 1234
} FinalInference_t;

typedef struct __attribute__((packed)) {
	float confidence;
	int32_t idx;
	float PTP_Z;
	float RMS_Z;
} DisplayPacket_t;

typedef struct {
	float features[FEATURE_SIZE];
	uint32_t feature_cycles;
} FeaturePacket_t;

typedef struct __attribute__((packed)) {
	uint32_t magic_header;      // e.g., 0xDEADBEEF to identify packet type
	size_t min_free_heap;
	UBaseType_t hw_sys;
	UBaseType_t hw_process;
	UBaseType_t hw_inference;
	UBaseType_t hw_uart;
	UBaseType_t hw_display;
} MemoryMetrics_t;
#endif
