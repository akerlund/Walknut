#ifndef __MAIN_H
#define __MAIN_H

#include <stdbool.h>
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
//#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

void setLeftMotor(int32_t m_speed);
void setRightMotor(int32_t m_speed);
void drive_go_forward(int32_t motorSpeedL,int32_t motorSpeedR);
void drive_go_reverse(int32_t m_speed);
void drive_turn_left(int32_t m_speed);
void drive_turn_right(int32_t m_speed);
void drive_rotate_left(int32_t m_speed);
void drive_rotate_right(int32_t m_speed);
void drive_total_stop(void);
void drive_go(int32_t motorSpeed, int32_t err);
void setMotorsForwards(int32_t left, int32_t right);
void setMotorsBackwards(int32_t left, int32_t right);

bool allWhite( );
bool allBlack( );
bool theLineIsLost(void);

void setLED(void);
void sendUART(void);
void regulator(void);
void pid(void);
void inits(void);

void calibrate(uint32_t val);
void readAdc(void);
uint16_t interpolateMiddle(void);
float findMiddle(void);
void AdcToPercetage(void);
void usartHandler(void);
void spokesCounter(void);
/* Nvics  --------------------------------------------------------------------*/
//void TIM2_IRQHandler(void);
/*PWM*/
void TIM4_IRQHandler(void);
/*USART Stream*/
void USART6_IRQHandler(void);
/*LED Stream*/
void DMA1_Stream4_IRQHandler(void);
/*ADC*/
void DMA2_Stream0_IRQHandler(void);
//void USART6_IRQHandler(void);
/* Functions -----------------------------------------------------------------*/
void USART_putn(USART_TypeDef* USARTx, volatile char *s, int size);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
/* Setups --------------------------------------------------------------------*/
void setup_ADC1_with_DMA2_NVIC(void);
//void setup_USART6_with_NVIC(void);
void setup_GPIO(void);
void setup_PWM_with_TIM4_NVIC(void);
void setup_SPI2_with_DMA1_NVIC(void);


void setup_USART6_with_DMA2_NVIC(void);
void DMA2_Stream1_IRQHandler(void);

#endif /* __MAIN_H */
