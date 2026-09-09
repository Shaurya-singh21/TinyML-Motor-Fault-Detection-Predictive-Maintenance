#include "stm32f446xx.h"
#include "uart.h"

void uart_init(void){
	//usart2 for data logging (pa2 for tx)
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);
	GPIOA->MODER |= (2U << GPIO_MODER_MODE2_Pos);
	GPIOA->AFR[0] |= (7U << GPIO_AFRL_AFSEL2_Pos);
	GPIOA->OTYPER &= ~((1U << 2));
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED2_Pos);

	//usart registers
	USART2->BRR = UART2_BAUD_RATE;
	USART2->CR1 |= (1U << USART_CR1_TE_Pos);
	USART2->CR3 |= USART_CR3_DMAT;
	NVIC_SetPriority(USART2_IRQn, 3);
	USART2->CR1 |= (1U << USART_CR1_UE_Pos);
}


