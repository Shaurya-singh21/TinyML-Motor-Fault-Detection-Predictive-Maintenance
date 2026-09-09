#include "stm32f446xx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rf_model.h"
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
const char *class_labels[4] = {"Bearing Fault", "Healthy", "Imbalance",
							   "Transient Shock"};
const float freq_resolution = 1000.0f / (float)SAMPLES_PER_BUFFER;
typedef struct __attribute__((packed))
{
	float features[FEATURE_SIZE];
	int32_t prediction;
	int32_t maj_vote_prediction;
} FinalInference_t;

typedef struct __attribute__((packed))
{
	float confidence;
	int32_t idx;
	float PTP_Z;
	float RMS_Z;
} DisplayPacket_t;

void welcome_message(void)
{
	oled_clear();
	oled_print(0, 0, "====================");
	oled_print(0, 1, "     MOTOR FAULT    ");
	oled_print(0, 2, "  DETECTION SYSTEM  ");
	oled_print(0, 3, "====================");
	oled_print(0, 4, "Press Button To Start");
	oled_print(0, 6, "     OR Stop System     ");
	oled_flush();
}

uint8_t bufferA[DMA_Buffer_size];
uint8_t bufferB[DMA_Buffer_size];
volatile uint8_t busy = 0;
volatile uint16_t bytes_left;
volatile uint8_t *ptr;
volatile char *uart_ptr = NULL;
volatile uint8_t flag;
volatile uint8_t power;

volatile uint8_t *current_dma_buffer = bufferA;
volatile uint16_t sample_count = 0;

TaskHandle_t xStartStopSysHandle = NULL;
TaskHandle_t xProcessDataHandle = NULL;

QueueHandle_t xLoggingQueue = NULL;
QueueHandle_t xBufferPtrQueueHandle = NULL;
QueueHandle_t xFeatureQueueHandle = NULL;
QueueHandle_t xDisplayQueueHandle = NULL;

void UART_DMA_Send(const void *data, uint16_t length)
{
	while (DMA1_Stream6->CR & DMA_SxCR_EN)
		;

	DMA1->HIFCR =
		DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 | DMA_HIFCR_CHTIF6;
	DMA1_Stream6->M0AR = (uint32_t)data;
	DMA1_Stream6->NDTR = length;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void vStartStopSys(void *pvParameters)
{
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (power)
		{
			power = 0;
			GPIOA->BSRR = GPIO_BSRR_BR3;
			GPIOA->BSRR = GPIO_BSRR_BR5;
			GPIOA->BSRR = GPIO_BSRR_BR9;
			NVIC_DisableIRQ(EXTI9_5_IRQn);
			welcome_message();
			while (DMA1_Stream4->CR & DMA_SxCR_EN)
				;
		}
		else
		{
			power = 1;
			sample_count = 0;
			current_dma_buffer = bufferA;
			GPIOA->BSRR = GPIO_BSRR_BS3;
			GPIOA->BSRR = GPIO_BSRR_BS9;
			oled_clear();
			oled_print(0, 0, "MOTOR FAULT DETECTION");
			oled_print(0, 2, "   Collecting Data...   ");
			oled_flush();
			while (DMA1_Stream4->CR & DMA_SxCR_EN)
				;
			NVIC_EnableIRQ(EXTI9_5_IRQn);
		}
	}
}
arm_rfft_fast_instance_f32 fft_instance;
static float fft_x[SAMPLES_PER_BUFFER], fft_y[SAMPLES_PER_BUFFER],
	fft_z[SAMPLES_PER_BUFFER];

static float features[FEATURE_SIZE] = {0};

static float mag_x[SAMPLES_PER_BUFFER / 2 + 1],
	mag_y[SAMPLES_PER_BUFFER / 2 + 1], mag_z[SAMPLES_PER_BUFFER / 2 + 1];

static float accl_x_buffer[SAMPLES_PER_BUFFER],
	accl_y_buffer[SAMPLES_PER_BUFFER], accl_z_buffer[SAMPLES_PER_BUFFER];

void vProcessData(void *pvParameters)
{
	arm_rfft_fast_init_f32(&fft_instance,
						   SAMPLES_PER_BUFFER);

	for (;;)
	{
		uint32_t *buffer_ptr;
		xQueueReceive(xBufferPtrQueueHandle, &buffer_ptr, portMAX_DELAY);

		const uint8_t *data_ptr = (const uint8_t *)buffer_ptr;
		for (volatile uint8_t i = 0; i < SAMPLES_PER_BUFFER; i++)
		{
			uint16_t idx = i * 6U;

			int16_t accl_x_int = (int16_t)(((uint16_t)data_ptr[idx] << 8) | (uint16_t)data_ptr[idx + 1]);
			int16_t accl_y_int = (int16_t)(((uint16_t)data_ptr[idx + 2] << 8) | (uint16_t)data_ptr[idx + 3]);
			int16_t accl_z_int = (int16_t)(((uint16_t)data_ptr[idx + 4] << 8) | (uint16_t)data_ptr[idx + 5]);

			accl_x_buffer[i] = (float)accl_x_int / 16384.0f;
			accl_y_buffer[i] = (float)accl_y_int / 16384.0f;
			accl_z_buffer[i] = (float)accl_z_int / 16384.0f;
		}
		// find mean
		float mean_x, mean_y, mean_z;
		arm_mean_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &mean_x);
		arm_mean_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &mean_y);
		arm_mean_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &mean_z);
		// subtract mean from each sample to remove DC offset
		arm_offset_f32(accl_x_buffer, -mean_x, accl_x_buffer,
					   SAMPLES_PER_BUFFER);
		arm_offset_f32(accl_y_buffer, -mean_y, accl_y_buffer,
					   SAMPLES_PER_BUFFER);
		arm_offset_f32(accl_z_buffer, -mean_z, accl_z_buffer,
					   SAMPLES_PER_BUFFER);

		// find variance
		float var_x, var_y, var_z;
		arm_var_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &var_x);
		arm_var_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &var_y);
		arm_var_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &var_z);
		const float variance_correction = (float)(SAMPLES_PER_BUFFER - 1) / (float)SAMPLES_PER_BUFFER;
		features[0] = var_x * variance_correction;
		features[1] = var_y * variance_correction;
		features[2] = var_z * variance_correction;

		// find RMS
		float rms_x, rms_y, rms_z;
		arm_rms_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &rms_x);
		arm_rms_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &rms_y);
		arm_rms_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &rms_z);
		features[3] = rms_x;
		features[4] = rms_y;
		features[5] = rms_z;

		// find peak to peak
		float max_x, max_y, max_z, min_x, min_y, min_z;
		uint32_t max_index, min_index;
		arm_max_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &max_x, &max_index);
		arm_max_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &max_y, &max_index);
		arm_max_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &max_z, &max_index);
		arm_min_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &min_x, &min_index);
		arm_min_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &min_y, &min_index);
		arm_min_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &min_z, &min_index);
		features[6] = max_x - min_x;
		features[7] = max_y - min_y;
		features[8] = max_z - min_z;

		// find fft
		arm_rfft_fast_f32(&fft_instance, accl_x_buffer, fft_x, 0);
		arm_rfft_fast_f32(&fft_instance, accl_y_buffer, fft_y, 0);
		arm_rfft_fast_f32(&fft_instance, accl_z_buffer, fft_z, 0);

		mag_x[0] = fabsf(fft_x[0]);

		mag_x[SAMPLES_PER_BUFFER / 2] = fabsf(fft_x[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++)
		{
			float real = fft_x[2U * k];
			float imag = fft_x[2U * k + 1U];

			mag_x[k] = sqrtf((real * real) + (imag * imag));
		}

		mag_y[0] = fabsf(fft_y[0]);

		mag_y[SAMPLES_PER_BUFFER / 2] = fabsf(fft_y[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++)
		{
			float real = fft_y[2U * k];
			float imag = fft_y[2U * k + 1U];

			mag_y[k] = sqrtf((real * real) + (imag * imag));
		}

		mag_z[0] = fabsf(fft_z[0]);

		mag_z[SAMPLES_PER_BUFFER / 2] = fabsf(fft_z[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++)
		{
			float real = fft_z[2U * k];
			float imag = fft_z[2U * k + 1U];

			mag_z[k] = sqrtf((real * real) + (imag * imag));
		}

		// find max of fft
		uint32_t dom_x, dom_y, dom_z;
		float fft_max_x, fft_max_y, fft_max_z;
		arm_max_f32(mag_x, SAMPLES_PER_BUFFER / 2 + 1, &fft_max_x, &dom_x);
		arm_max_f32(mag_y, SAMPLES_PER_BUFFER / 2 + 1, &fft_max_y, &dom_y);
		arm_max_f32(mag_z, SAMPLES_PER_BUFFER / 2 + 1, &fft_max_z, &dom_z);
		features[9] = (float)(dom_x);
		features[10] = (float)(dom_y);
		features[11] = (float)(dom_z);

		// find spectraL_crest
		float spectral_crest_x, spectral_crest_y, spectral_crest_z;
		arm_mean_f32(mag_x, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_x);
		arm_mean_f32(mag_y, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_y);
		arm_mean_f32(mag_z, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_z);
		features[12] = fft_max_x / (spectral_crest_x + 1e-8f);

		features[13] = fft_max_y / (spectral_crest_y + 1e-8f);

		features[14] = fft_max_z / (spectral_crest_z + 1e-8f);

		xQueueSendToBack(xFeatureQueueHandle, features, 0);
	}
}

// static int32_t final_pred[10] = { 0 };
static uint8_t final_pred_sum[4] = {0};
uint8_t cnt = 0;

static FinalInference_t packets_to_send;
static DisplayPacket_t packets_to_display;
void vRunInference(void *pvParameters)
{
	for (;;)
	{
		float received_feature_buffer[FEATURE_SIZE];
		if (xQueueReceive(xFeatureQueueHandle, received_feature_buffer,
						  portMAX_DELAY) == pdTRUE)
		{
			// run inference and find prediction
			int32_t pred = rf_model_predict(received_feature_buffer,
											FEATURE_SIZE);
			packets_to_send.prediction = pred;
			memcpy(packets_to_send.features, received_feature_buffer,
				   sizeof(packets_to_send.features));
			if (pred >= 0 && pred < 4)
			{
				final_pred_sum[pred]++;
			}
			cnt++;
			uint8_t max_index = 0;
			for (volatile uint8_t i = 0; i < OUTPUT_SIZE; i++)
			{
				if (final_pred_sum[i] > final_pred_sum[max_index])
				{
					max_index = i;
				}
			}
			packets_to_send.maj_vote_prediction = (int32_t)max_index;
			// Send to logging queue for UART task
			xQueueSendToBack(xLoggingQueue, &packets_to_send, 0);
			if (cnt >= 10)
			{
				packets_to_display.idx = (int32_t)max_index;
				packets_to_display.PTP_Z = received_feature_buffer[8];
				packets_to_display.RMS_Z = received_feature_buffer[5];
				packets_to_display.confidence = (final_pred_sum[max_index] * 100) / 10.0f;
				memset(final_pred_sum, 0, sizeof(final_pred_sum));
				cnt = 0;
				xQueueSendToBack(xDisplayQueueHandle, &packets_to_display, 0);
			}
		}
	}
}

static FinalInference_t rx_packet;
void vSend_via_UART(void *pvParameters)
{
	for (;;)
	{
		if (xQueueReceive(xLoggingQueue, &rx_packet, portMAX_DELAY) == pdTRUE)
		{
			UART_DMA_Send(&rx_packet, sizeof(rx_packet));
		}
	}
}

static DisplayPacket_t disp_rx_packet;
void vDisplayTask(void *pvParameters)
{
	for (;;)
	{
		if (xQueueReceive(xDisplayQueueHandle, &disp_rx_packet, portMAX_DELAY) == pdTRUE)
		{
			char line[30];
			oled_clear();
			oled_print(0, 0, "MOTOR FAULT DETECTION");
			snprintf(line, sizeof(line), "STATE: %s", class_labels[disp_rx_packet.idx]);
			oled_print(0, 2, line);
			snprintf(line, sizeof(line), "Conf: %.3f%%", (disp_rx_packet.confidence));
			oled_print(0, 3, line);
			snprintf(line, sizeof(line), "PTP_Z: %.3f", (disp_rx_packet.PTP_Z));
			oled_print(0, 5, line);
			snprintf(line, sizeof(line), "RMS_Z: %.3f", (disp_rx_packet.RMS_Z));
			oled_print(0, 6, line);
			oled_flush();
		}
	}
}

int main(void)
{
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
	clk_init();
	gpio_init();
	uart_init();
	i2c_init();
	MPU6050_Init();
	i2c3_oled_init();
	dma_init();
	oled_init();
	oled_clear();
	welcome_message();
	xBufferPtrQueueHandle = xQueueCreate(2, sizeof(uint32_t *));
	xFeatureQueueHandle = xQueueCreate(2, sizeof(features));
	xLoggingQueue = xQueueCreate(2, sizeof(FinalInference_t));
	xDisplayQueueHandle = xQueueCreate(2, sizeof(DisplayPacket_t));
	xTaskCreate(vStartStopSys, "StartStopSys", 64, NULL, SYS_START_TASK,
				&xStartStopSysHandle);

	if (xLoggingQueue != NULL)
	{
		xTaskCreate(vSend_via_UART, "UART_Task", 128, NULL, UART_TASK, NULL);
	}
	if (xBufferPtrQueueHandle != NULL)
	{
		xTaskCreate(vProcessData, "ProcessData", 512, NULL, PROCESS_DATA_TASK,
					&xProcessDataHandle);
	}
	if (xFeatureQueueHandle != NULL)
	{
		xTaskCreate(vRunInference, "FindPred", 512, NULL, INFERENCE_TASK, NULL);
	}
	if (xDisplayQueueHandle != NULL)
	{
		xTaskCreate(vDisplayTask, "Display_Task", 512, NULL, DISPLAY_TASK, NULL);
	}
	vTaskStartScheduler();

	for (;;)
		;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}
