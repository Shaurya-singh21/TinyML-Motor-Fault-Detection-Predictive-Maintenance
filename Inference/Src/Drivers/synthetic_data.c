#include "main.h"
extern uint8_t synthetic_buffer[DMA_Buffer_size] __attribute__((aligned(4)));

//static const int16_t wave16[16] = { 0, 306, 566, 739, 800, 739, 566, 306, 0,
//		-306, -566, -739, -800, -739, -566, -306 };

void store_be_i16(uint8_t *dst, int16_t value) {
	uint16_t u = (uint16_t) value;
	dst[0] = (uint8_t) (u >> 8);
	dst[1] = (uint8_t) u;
}
void prepare_synthetic_window(uint32_t window_id) {
	const float two_pi = 6.28318530718f;
	const float n = (float) SAMPLES_PER_BUFFER;

	uint32_t max_freq = (SAMPLES_PER_BUFFER / 2U) - 1U;

	uint32_t fx = 1U + (window_id * 3U) % max_freq;
	uint32_t fy = 1U + (window_id * 5U + 7U) % max_freq;
	uint32_t fz = 1U + (window_id * 7U + 13U) % max_freq;

	float phase = (float) window_id * 0.37f;

	for (uint32_t i = 0; i < SAMPLES_PER_BUFFER; i++) {
		float t = (float) i / n;

		int16_t x = (int16_t) ((1200.0f + 50.0f * window_id)
				* sinf(two_pi * fx * t + phase)
				+ 250.0f * sinf(two_pi * 2.0f * fx * t));

		int16_t y = (int16_t) ((700.0f + 35.0f * window_id)
				* sinf(two_pi * fy * t + phase)
				+ 180.0f * cosf(two_pi * 2.0f * fy * t));

		int16_t z = (int16_t) (16384.0f
				+ (500.0f + 25.0f * window_id) * sinf(two_pi * fz * t + phase)
				+ 150.0f * cosf(two_pi * 2.0f * fz * t));

		uint32_t idx = i * 6U;

		store_be_i16(&synthetic_buffer[idx], x);
		store_be_i16(&synthetic_buffer[idx + 2], y);
		store_be_i16(&synthetic_buffer[idx + 4], z);
	}
}
