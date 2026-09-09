#include "stm32f446xx.h"
#include "gpio.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"


extern TaskHandle_t xStartStopSysHandle;

void gpio_init(void) {
	//PA3 for sys led
	GPIOA->MODER &= ~GPIO_MODER_MODER3;
	GPIOA->MODER |= (1U << GPIO_MODER_MODER3_Pos);
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_3;
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED3_Pos);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR3;

	//PA5 for led
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= (1U << GPIO_MODER_MODER5_Pos);
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED5_Pos);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;

	//PA8 for fan start and stop
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= (1U << GPIO_MODER_MODER8_Pos);
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED8_Pos);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;

	//PB5 for MPU6050 int pin
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOB->PUPDR |= (2U << GPIO_PUPDR_PUPD5_Pos);
	SYSCFG->EXTICR[1] |= (1U << SYSCFG_EXTICR2_EXTI5_Pos);
	EXTI->IMR |= EXTI_IMR_IM5;
	EXTI->RTSR |= EXTI_RTSR_TR5;
	NVIC_SetPriority(EXTI9_5_IRQn, I2C_RX_PRIORITY);

	//PC13 for button
	GPIOC->MODER &= ~GPIO_MODER_MODE13;
	SYSCFG->EXTICR[3] |= 2U << SYSCFG_EXTICR4_EXTI13_Pos;
	EXTI->IMR |= EXTI_IMR_IM13;
	EXTI->FTSR |= EXTI_FTSR_TR13;
	NVIC_SetPriority(EXTI15_10_IRQn, SYS_START_BUTTON_PRIORITY);

}

void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR13) {
		EXTI->PR = EXTI_PR_PR13;
		BaseType_t higher_priority_sys_task_woken = pdFALSE;
		vTaskNotifyGiveFromISR(xStartStopSysHandle,
				&higher_priority_sys_task_woken);
		portYIELD_FROM_ISR(higher_priority_sys_task_woken);
	}
}
