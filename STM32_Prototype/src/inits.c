#include "main.h" 

/* Global --------------------------------------------------------------------*/
uint16_t ADC_RawBuffer[16];
uint32_t ADCreadBuf[16];
uint16_t usartBuffert[2];
uint16_t transfers = 0;

void delay(uint32_t ms) {
	ms *= 168000;
	while(ms--) {
	    __NOP();
	}
}

/* Main ----------------------------------------------------------------------*/
int main(void){

 // Setups.
	setup_ADC1_with_DMA2( );	
	setup_USART1_with_NVIC( );
	setup_GPIO( );
	GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	//setup_Timer2( );
	setup_PWM( );


	//uint32_t motorBaseA = 100;
	//uint32_t motorBaseB = 100;

	while(1){

		// Skickar iväg ADC buffern som är 16 lång. 
		for(int i=0; i<16; i++){
			USART_putn(USART1, (char*)&ADC_RawBuffer[i], 2);    	
		}
		//USART_putn(USART1, (char*)&ADC_RawBuffer[2], 2);

		delay(50); // Vänte lite med och skicka igen.


		// LÄSA ADCn

		// REGULATOR

		// REGLERA PWM FÖR MOTORERNA

 
	}
}
void setMotorsForwards(uint32_t left, uint32_t right){

	if(left>1000){left=1000;}
	if(right>1000){right=1000;}
	TIM_SetCompare1(TIM3, 0); 		// PB4
	TIM_SetCompare2(TIM3, 0); 		// PB5
	TIM_SetCompare3(TIM3, left); 	// PB0
	TIM_SetCompare4(TIM3, right); 	// PB1
}
void setMotorsBackwards(uint32_t left, uint32_t right){

	if(left>1000){left=1000;}
	if(right>1000){right=1000;}
	TIM_SetCompare1(TIM3, left); 	// PB4
	TIM_SetCompare2(TIM3, right); 	// PB5
	TIM_SetCompare3(TIM3, 0); 		// PB0
	TIM_SetCompare4(TIM3, 0); 		// PB1
}
void readAdc( ){

	int nrOfSamples = 8;
	int nrOfADC = 12;

	// Sätt på NFET
	GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_SET);

	// Läs in och spara.
	for(int i=0; i<nrOfSamples; i++){
		for(int i=0; i<nrOfADC; i++){

			ADCreadBuf[i] += ADC_RawBuffer[i];
		}
	}

	// Stäng av NFET.
	GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);

	// Ta ut medelvärdet.
	for(int i=0; i<nrOfADC; i++){

		ADCreadBuf[i] = ADCreadBuf[i]/nrOfSamples;
	}
}
void calibrateSensors( ){

	uint16_t nrOfSensors = 12;
	uint16_t nrOfCalibrations = 12;

	uint16_t ADC_MAX[nrOfSensors];
	uint16_t ADC_MIN[nrOfSensors];
	uint16_t ADC_DIF[nrOfSensors];

	for(int i=0;i<nrOfSensors;i++){
		ADC_MAX[i]=0;
		ADC_MIN[i]=4095;
		ADC_DIF[i]=0;
	}

	uint16_t tempADC = 0;

 // Sätt på NFET.
	GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_SET);

 // Hitta max, då är den på vit.
	for(int i=0;i<nrOfCalibrations;i++){
		for(int i=0;i<nrOfSensors;i++){
			tempADC = ADC_RawBuffer[i];
			if(tempADC>ADC_MAX[i]){ADC_MAX[i]=tempADC;}
		}		
	}

 // Nu ska roboten rulla över den svarta linjen.
 // När den gör det startar den sampla.
 // Vi kan anta den är långt under säg 2000.
	while((ADC_RawBuffer[0]>2000)&&(ADC_RawBuffer[nrOfSensors-1]>2000)){;;}

 // Hitta min, då är den på svart.
	for(int i=0;i<nrOfCalibrations;i++){
		for(int i=0;i<nrOfSensors;i++){
			tempADC = ADC_RawBuffer[i];
			if(tempADC<ADC_MIN[i]){ADC_MIN[i]=tempADC;}
		}		
	}	

 // Stäng av NFET.
	GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);

 // Räknar ut differensen.
	for(int i=0;i<nrOfSensors;i++){
		ADC_DIF[i]=ADC_MAX[i]-ADC_MIN[i];
	}
}
void findAverageSample( ){



}
/* Nvics  --------------------------------------------------------------------*/
void TIM2_IRQHandler( ){
    
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
}
void USART1_IRQHandler( ){

    if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
/* Functions -----------------------------------------------------------------*/
void USART_putn(USART_TypeDef* USARTx, volatile char *s, int size){
    
    if (!size) return;
    while(size--){
        while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        *s++;
    }
}
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

    while(*s){
        // wait until data register is empty
        while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        *s++;
    }
}
/* Setups --------------------------------------------------------------------*/
void setup_ADC1_with_DMA2( ){

 // RC.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE); 	// Clock for the ADC port!! Do not forget about this one ;)

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   

 // GPIO
 	GPIO_InitTypeDef GPIO_InitStruct; 

	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	// PA : Pin 0-7.
	GPIO_InitStruct.GPIO_Pin  = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
								 GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB : 0-1.
	GPIO_InitStruct.GPIO_Pin  = (GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PC : 0-5.
	GPIO_InitStruct.GPIO_Pin  = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | 
								 GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
	GPIO_Init(GPIOC, &GPIO_InitStruct);

 // DMA
	DMA_InitTypeDef DMA_InitStruct; // Skapar en struct av typ DMA_InitTypeDef som heter DMA_InitStruct.
	DMA_DeInit(DMA2_Stream0); // Nollar.

	DMA_InitStruct.DMA_Channel            = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)&ADC_RawBuffer[0];
	DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize         = 16;
	DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream0, &DMA_InitStruct);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

 // NVIC.
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);


	DMA_Cmd(DMA2_Stream0, ENABLE);

 // ADC
	ADC_DeInit( );
	ADC_InitTypeDef ADC_InitStruct; 

	ADC_InitStruct.ADC_Resolution           = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode         = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode   = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_DataAlign            = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion      = 16;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_CommonInitTypeDef ADC_CommonStruct;

	ADC_CommonStruct.ADC_Mode 			  = ADC_Mode_Independent;
	ADC_CommonStruct.ADC_Prescaler		  = ADC_Prescaler_Div2;
	ADC_CommonStruct.ADC_DMAAccessMode	  = ADC_DMAAccessMode_Disabled;
	ADC_CommonStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
 	ADC_CommonInit(&ADC_CommonStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 13, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 15, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 16, ADC_SampleTime_480Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);   	
	ADC_SoftwareStartConv(ADC1);
}
void DMA2_Stream0_IRQHandler(void){

	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
		transfers++;
		if(transfers >= 100){
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			transfers = 0;
		}		
	}
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
	}
}
void setup_USART1_with_NVIC( ){

 // GPIO
	GPIO_InitTypeDef  GPIO_InitStruct; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   
 // USART1 TX on PB6 Grey, RX on PB7 White.
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;//
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

 // NVIC
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;//NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

 // USART1
	USART_InitTypeDef USART_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStruct.USART_BaudRate   = 9600;//115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_Mode       = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStruct);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);  
}
void setup_GPIO( ){

    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

 // LEDS.
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

 // NFET.
 	// GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_11;
	// GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	// GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void setup_Timer2( ){

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStruct; 
 
    TIM_InitStruct.TIM_Prescaler = 84-1;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = 1000-1;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);

 // NVIC.
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}
void setup_PWM( ){

 // GPIO.
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

 // Timer3.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	//u32 PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 21000000) - 1;
	//SystemCoreClock = 168000000
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 // PWM.
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* PWM1 Mode configuration: Channel3 (GPIOB Pin 0)*/
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 (GPIOB Pin 1)*/
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel1 (GPIOB Pin 4)*/
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 (GPIOB Pin 5)*/
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_Cmd(TIM3, ENABLE);

	// 4 PWM för 2st bakåt och 2st framåt till 2 motorer.
	TIM_SetCompare1(TIM3, 100); // Dutycycle 100/1000 = 1/10
	TIM_SetCompare2(TIM3, 250); // DC = 1/4
	TIM_SetCompare3(TIM3, 500); // DC = 1/2
	TIM_SetCompare4(TIM3, 750); // DC = 3/4
}