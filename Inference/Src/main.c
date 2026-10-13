#include "main.h"
#include "rf_model.h"
const float freq_resolution = 1000.0f / (float) SAMPLES_PER_BUFFER;
const char *class_labels[4] = { "Bearing Fault", "Healthy", "Imbalance",
		"Transient Shock" };
void welcome_message(void) {
	oled_clear();
	oled_print(0, 0, "====================");
	oled_print(0, 1, "     MOTOR FAULT    ");
	oled_print(0, 2, "  DETECTION SYSTEM  ");
	oled_print(0, 3, "====================");
	oled_print(0, 4, "Press Button To Start");
	oled_print(0, 6, "    Or Stop System   ");
	oled_flush();
}
uint8_t bufferA[DMA_Buffer_size];
uint8_t bufferB[DMA_Buffer_size];
volatile uint8_t power;

volatile uint8_t *current_dma_buffer = bufferA;
uint8_t synthetic_buffer[DMA_Buffer_size] __attribute__((aligned(4)));
volatile uint16_t sample_count = 0;
volatile uint32_t id = 0;

QueueHandle_t xLoggingQueue = NULL;
QueueHandle_t xBufferPtrQueueHandle = NULL;
QueueHandle_t xFeatureQueueHandle = NULL;
QueueHandle_t xDisplayQueueHandle = NULL;

static StackType_t xStartStopStack[START_STOP_STACK_SIZE];
static StaticTask_t xStartStopTCB;
TaskHandle_t xStartStopSysHandle = NULL;

static StackType_t xProcessStack[PROCESS_STACK_SIZE];
static StaticTask_t xProcessTCB;
TaskHandle_t xProcessDataHandle = NULL;

static StackType_t xInferenceStack[INFERENCE_STACK_SIZE];
static StaticTask_t xInferenceTCB;
TaskHandle_t xInferenceHandle = NULL;

static StackType_t xUartStack[UART_STACK_SIZE];
static StaticTask_t xUartTCB;
TaskHandle_t xUARTTaskHandle = NULL;

static StackType_t xDisplayStack[DISPLAY_STACK_SIZE];
static StaticTask_t xDisplayTCB;
TaskHandle_t xDisplayTaskHandle = NULL;

#if USE_SYNTHETIC_DATA == 1
static StackType_t xSyntheticStack[SYNTHETIC_STACK_SIZE];
static StaticTask_t xSyntheticTCB;
TaskHandle_t xSyntheticTaskHandle = NULL;
#endif

void UART_DMA_Send(const void *data, uint16_t length) {
	while (DMA1_Stream6->CR & DMA_SxCR_EN)
		;
	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6
			| DMA_HIFCR_CFEIF6 | DMA_HIFCR_CHTIF6;
	DMA1_Stream6->M0AR = (uint32_t) data;
	DMA1_Stream6->NDTR = length;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
}
void vStartStopSys(void *pvParameters) {
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (power) {
			power = 0;
			GPIOA->BSRR = GPIO_BSRR_BR3;
			GPIOA->BSRR = GPIO_BSRR_BR5;
			GPIOA->BSRR = GPIO_BSRR_BR9;
#if USE_SYNTHETIC_DATA == 0
			NVIC_DisableIRQ(EXTI9_5_IRQn);
#endif
			welcome_message();
			MemoryMetrics_t mem_stats;
			mem_stats.magic_header = 0xDEADBEEF;
			mem_stats.min_free_heap = xPortGetMinimumEverFreeHeapSize();
			mem_stats.hw_sys = uxTaskGetStackHighWaterMark(xStartStopSysHandle);
			mem_stats.hw_process = uxTaskGetStackHighWaterMark(
					xProcessDataHandle);
			mem_stats.hw_inference = uxTaskGetStackHighWaterMark(
					xInferenceHandle);
			mem_stats.hw_uart = uxTaskGetStackHighWaterMark(xUARTTaskHandle);
			mem_stats.hw_display = uxTaskGetStackHighWaterMark(
					xDisplayTaskHandle);
			UART_DMA_Send(&mem_stats, sizeof(MemoryMetrics_t));
			while (DMA1_Stream6->CR & DMA_SxCR_EN)
				;
			while (DMA1_Stream4->CR & DMA_SxCR_EN)
				;
		} else {
			power = 1;
			sample_count = 0;
			current_dma_buffer = bufferA;
			GPIOA->BSRR = GPIO_BSRR_BS3 | GPIO_BSRR_BS9;
			oled_clear();
			oled_print(0, 0, "MOTOR FAULT DETECTION");
			oled_print(0, 2, "   Collecting Data...   ");
			oled_flush();
			while (DMA1_Stream4->CR & DMA_SxCR_EN)
				;
#if USE_SYNTHETIC_DATA == 0
			NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
		}
	}
}
arm_rfft_fast_instance_f32 fft_instance;
static float fft_x[SAMPLES_PER_BUFFER], fft_y[SAMPLES_PER_BUFFER],
		fft_z[SAMPLES_PER_BUFFER];

static float mag_x[SAMPLES_PER_BUFFER / 2 + 1],
		mag_y[SAMPLES_PER_BUFFER / 2 + 1], mag_z[SAMPLES_PER_BUFFER / 2 + 1];

static float accl_x_buffer[SAMPLES_PER_BUFFER],
		accl_y_buffer[SAMPLES_PER_BUFFER], accl_z_buffer[SAMPLES_PER_BUFFER];

#if USE_SYNTHETIC_DATA == 1
void vDataPrep(void *pvParameters) {
	for (;;) {
		if (power) {  // Only generate and send data if the system is ON
			prepare_synthetic_window(id);
			uint32_t *buffer_ptr = (uint32_t*) (void*) synthetic_buffer;
			id++;
			xQueueSendToBack(xBufferPtrQueueHandle, &buffer_ptr, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(128)); // Prevent task from infinitely starving the system
	}
}
#endif

void vProcessData(void *pvParameters) {
	static FeaturePacket_t feature_packet;
	arm_rfft_fast_init_f32(&fft_instance,
	SAMPLES_PER_BUFFER);
	for (;;) {
		uint32_t *buffer_ptr;
		xQueueReceive(xBufferPtrQueueHandle, &buffer_ptr, portMAX_DELAY);
		uint32_t feature_start_cycles = DWT->CYCCNT;
		const uint8_t *data_ptr = (const uint8_t*) buffer_ptr;
		for (volatile uint8_t i = 0; i < SAMPLES_PER_BUFFER; i++) {
			uint16_t idx = i * 6U;
			int16_t accl_x_int = (int16_t) (((uint16_t) data_ptr[idx] << 8)
					| (uint16_t) data_ptr[idx + 1]);
			int16_t accl_y_int = (int16_t) (((uint16_t) data_ptr[idx + 2] << 8)
					| (uint16_t) data_ptr[idx + 3]);
			int16_t accl_z_int = (int16_t) (((uint16_t) data_ptr[idx + 4] << 8)
					| (uint16_t) data_ptr[idx + 5]);

			accl_x_buffer[i] = (float) accl_x_int / 16384.0f;
			accl_y_buffer[i] = (float) accl_y_int / 16384.0f;
			accl_z_buffer[i] = (float) accl_z_int / 16384.0f;
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
		const float variance_correction = (float) (SAMPLES_PER_BUFFER - 1)
				/ (float) SAMPLES_PER_BUFFER;
		feature_packet.features[0] = var_x * variance_correction;
		feature_packet.features[1] = var_y * variance_correction;
		feature_packet.features[2] = var_z * variance_correction;

		// find RMS
		float rms_x, rms_y, rms_z;
		arm_rms_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &rms_x);
		arm_rms_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &rms_y);
		arm_rms_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &rms_z);
		feature_packet.features[3] = rms_x;
		feature_packet.features[4] = rms_y;
		feature_packet.features[5] = rms_z;

		// find peak to peak
		float max_x, max_y, max_z, min_x, min_y, min_z;
		uint32_t max_index, min_index;
		arm_max_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &max_x, &max_index);
		arm_max_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &max_y, &max_index);
		arm_max_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &max_z, &max_index);
		arm_min_f32(accl_x_buffer, SAMPLES_PER_BUFFER, &min_x, &min_index);
		arm_min_f32(accl_y_buffer, SAMPLES_PER_BUFFER, &min_y, &min_index);
		arm_min_f32(accl_z_buffer, SAMPLES_PER_BUFFER, &min_z, &min_index);
		feature_packet.features[6] = max_x - min_x;
		feature_packet.features[7] = max_y - min_y;
		feature_packet.features[8] = max_z - min_z;

		// find fft
		arm_rfft_fast_f32(&fft_instance, accl_x_buffer, fft_x, 0);
		arm_rfft_fast_f32(&fft_instance, accl_y_buffer, fft_y, 0);
		arm_rfft_fast_f32(&fft_instance, accl_z_buffer, fft_z, 0);

		mag_x[0] = fabsf(fft_x[0]);

		mag_x[SAMPLES_PER_BUFFER / 2] = fabsf(fft_x[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++) {
			float real = fft_x[2U * k];
			float imag = fft_x[2U * k + 1U];

			mag_x[k] = sqrtf((real * real) + (imag * imag));
		}

		mag_y[0] = fabsf(fft_y[0]);

		mag_y[SAMPLES_PER_BUFFER / 2] = fabsf(fft_y[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++) {
			float real = fft_y[2U * k];
			float imag = fft_y[2U * k + 1U];

			mag_y[k] = sqrtf((real * real) + (imag * imag));
		}

		mag_z[0] = fabsf(fft_z[0]);

		mag_z[SAMPLES_PER_BUFFER / 2] = fabsf(fft_z[1]);

		for (uint16_t k = 1; k < SAMPLES_PER_BUFFER / 2; k++) {
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
		feature_packet.features[9] = (float) (dom_x);
		feature_packet.features[10] = (float) (dom_y);
		feature_packet.features[11] = (float) (dom_z);

		// find spectraL_crest
		float spectral_crest_x, spectral_crest_y, spectral_crest_z;
		arm_mean_f32(mag_x, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_x);
		arm_mean_f32(mag_y, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_y);
		arm_mean_f32(mag_z, SAMPLES_PER_BUFFER / 2 + 1, &spectral_crest_z);
		feature_packet.features[12] = fft_max_x / (spectral_crest_x + 1e-8f);

		feature_packet.features[13] = fft_max_y / (spectral_crest_y + 1e-8f);

		feature_packet.features[14] = fft_max_z / (spectral_crest_z + 1e-8f);
		uint32_t feature_elapsed_cycles = DWT->CYCCNT - feature_start_cycles;
		feature_packet.feature_cycles = feature_elapsed_cycles;
		xQueueSendToBack(xFeatureQueueHandle, &feature_packet, 0);
	}
}

void vRunInference(void *pvParameters) {
	static FeaturePacket_t received_features;
	static uint8_t final_pred_sum[4] = { 0 };
	uint8_t cnt = 0;
	static FinalInference_t packets_to_send;
	static DisplayPacket_t packets_to_display;
	for (;;) {
		if (xQueueReceive(xFeatureQueueHandle, &received_features,
		portMAX_DELAY) == pdTRUE) {

			// run inference and find prediction
			uint32_t inference_start_cycles = DWT->CYCCNT;

			int32_t pred = rf_model_predict(received_features.features,
			FEATURE_SIZE);
			uint32_t inference_elapsed_cycles = DWT->CYCCNT
					- inference_start_cycles;

			uint32_t total_cycles = received_features.feature_cycles
					+ inference_elapsed_cycles;

			/* Convert percentage to hundredths, avoiding float printf. */
			uint32_t pct_x100 = (uint32_t) (((uint64_t) total_cycles * 10000ULL)
					/ 2048000ULL);
			packets_to_send.prediction = pred;
			packets_to_send.feature_cycles = received_features.feature_cycles;
			packets_to_send.rf_cycles = inference_elapsed_cycles;
			packets_to_send.window_cost_pct_x100 = pct_x100;
			packets_to_send.prediction = pred;

			memcpy(packets_to_send.features, received_features.features,
					sizeof(packets_to_send.features));
			if (pred >= 0 && pred < 4) {
				final_pred_sum[pred]++;
			}
			cnt++;
			uint8_t max_index = 0;
			for (volatile uint8_t i = 0; i < OUTPUT_SIZE; i++) {
				if (final_pred_sum[i] > final_pred_sum[max_index]) {
					max_index = i;
				}
			}
			packets_to_send.maj_vote_prediction = (int32_t) max_index;
			// Send to logging queue for UART task
			xQueueSendToBack(xLoggingQueue, &packets_to_send, 0);
			if (cnt >= TIMING_AVERAGE_WINDOWS) {
				packets_to_display.idx = (int32_t) max_index;
				packets_to_display.PTP_Z = received_features.features[8];
				packets_to_display.RMS_Z = received_features.features[5];
				packets_to_display.confidence =
						(final_pred_sum[max_index] * 100) / 10.0f;
				memset(final_pred_sum, 0, sizeof(final_pred_sum));
				cnt = 0;
				xQueueSendToBack(xDisplayQueueHandle, &packets_to_display, 0);
			}
		}
	}
}

static FinalInference_t rx_packet;
void vSend_via_UART(void *pvParameters) {
	for (;;) {
		if (xQueueReceive(xLoggingQueue, &rx_packet, portMAX_DELAY) == pdTRUE) {
			UART_DMA_Send(&rx_packet, sizeof(rx_packet));
		}
	}
}

static DisplayPacket_t disp_rx_packet;
void vDisplayTask(void *pvParameters) {
	for (;;) {
		if (xQueueReceive(xDisplayQueueHandle, &disp_rx_packet,
		portMAX_DELAY) == pdTRUE) {
			char line[30];
			oled_clear();
			oled_print(0, 0, "MOTOR FAULT DETECTION");
			snprintf(line, sizeof(line), "STATE: %s",
					class_labels[disp_rx_packet.idx]);
			oled_print(0, 2, line);
			snprintf(line, sizeof(line), "Conf: %.3f%%",
					(disp_rx_packet.confidence));
			oled_print(0, 3, line);
			snprintf(line, sizeof(line), "PTP_Z: %.3f", (disp_rx_packet.PTP_Z));
			oled_print(0, 5, line);
			snprintf(line, sizeof(line), "RMS_Z: %.3f", (disp_rx_packet.RMS_Z));
			oled_print(0, 6, line);
			oled_flush();
		}
	}
}

int main(void) {

	clk_init();
	dwt_init();
	gpio_init();
	uart_init();
#if USE_SYNTHETIC_DATA == 0
	i2c_init();
	MPU6050_Init();
	i2c3_oled_init();
#endif
	dma_init();
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);

	/* Stop sensor RX DMA: Stream 0. */
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream0->CR & DMA_SxCR_EN) {
	}

	/* Stop OLED DMA: Stream 4. */
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream4->CR & DMA_SxCR_EN) {
	}
	oled_init();
	oled_clear();
	welcome_message();

	xBufferPtrQueueHandle = xQueueCreate(2, sizeof(uint32_t*));
	xFeatureQueueHandle = xQueueCreate(2, sizeof(FeaturePacket_t));
	xLoggingQueue = xQueueCreate(2, sizeof(FinalInference_t));
	xDisplayQueueHandle = xQueueCreate(2, sizeof(DisplayPacket_t));

	xStartStopSysHandle = xTaskCreateStatic(vStartStopSys, "StartStopSys",
			START_STOP_STACK_SIZE, NULL, SYS_START_TASK, xStartStopStack,
			&xStartStopTCB);
	xUARTTaskHandle = xTaskCreateStatic(vSend_via_UART, "UART_Task",
			UART_STACK_SIZE, NULL, UART_TASK, xUartStack, &xUartTCB);
	xProcessDataHandle = xTaskCreateStatic(vProcessData, "ProcessData",
			PROCESS_STACK_SIZE, NULL, PROCESS_DATA_TASK, xProcessStack,
			&xProcessTCB);
	xInferenceHandle = xTaskCreateStatic(vRunInference, "FindPred",
			INFERENCE_STACK_SIZE, NULL, INFERENCE_TASK, xInferenceStack,
			&xInferenceTCB);
	xDisplayTaskHandle = xTaskCreateStatic(vDisplayTask, "Display_Task",
			DISPLAY_STACK_SIZE, NULL, DISPLAY_TASK, xDisplayStack,
			&xDisplayTCB);

#if USE_SYNTHETIC_DATA == 1
	xSyntheticTaskHandle = xTaskCreateStatic(vDataPrep, "SyntheticInput",
			SYNTHETIC_STACK_SIZE, NULL, UART_TASK, xSyntheticStack,
			&xSyntheticTCB);
#endif

	vTaskStartScheduler();

	for (;;)
		;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	(void) xTask;
	(void) pcTaskName;
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
