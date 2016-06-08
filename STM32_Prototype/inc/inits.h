#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/

/*STD*/
//#include "math.h"

/*STM32*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

/* Nvics  --------------------------------------------------------------------*/
void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);

/* Functions -----------------------------------------------------------------*/
void USART_putn(USART_TypeDef* USARTx, volatile char *s, int size);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);

/* Setups --------------------------------------------------------------------*/
void setup_ADC1_with_DMA2(void);
void setup_USART1_with_NVIC(void);
void setup_GPIO(void);
void setup_Timer2(void);
void setup_PWM(void);

#endif /* __MAIN_H */
