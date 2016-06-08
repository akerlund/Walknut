/**
  *	@brief   Linefollower code for STM32F40x
  *	@author  Fredrik Åkerlund
  * @version 1.0
  * @date    February-May 2015
  *
  */
/* ---------------------------------------------------------------------------*/
#include "main.h"
//#include <stdio.h>  
/* ---------------------------------------------------------------------------*/
#define NFETPORT GPIOB
#define NFETPIN GPIO_Pin_13

#define SPI2_CLK_PIN GPIO_Pin_13
#define SPI2_MOSI_PIN GPIO_Pin_15
#define SPI2_ GPIOB

#define LATCH_ENABLE_PIN GPIO_Pin_11
#define LATCH_ENABLE_PORT GPIOB

#define PWM_AIN1_L_FOWR_PIN GPIO_Pin_6
#define PWM_AIN2_L_BACK_PIN GPIO_Pin_7
#define PWM_BIN1_R_FOWR_PIN GPIO_Pin_9
#define PWM_BIN2_R_BACK_PIN GPIO_Pin_8
#define PWM_PORT GPIOB

/* Global --------------------------------------------------------------------*/

// ADC data.

volatile uint16_t 	ADC_rawBuff[16];
volatile uint32_t 	ADC_avgBuff[16];
volatile float 		ADC_percentage[12];
volatile uint32_t 	ADC_MAX[12];
volatile uint32_t 	ADC_MIN[12];
volatile uint32_t 	ADC_DIF[12];
volatile uint8_t 	LED_data[2];
const 	 uint32_t 	nrOfSensors = 12;
volatile uint32_t 	nrOfSamples = 8;

/* ---------------------------------------------------------------------------*/
// USART Data.

volatile float 		usartBuffert[16];
volatile char 		usartIncoming[64];
//volatile uint32_t 	nrOfBytes = 0;;
volatile bool 		usartReceiving = false;
volatile int32_t 	nrOfUsartCalls = 0;
volatile int32_t 	usartCnt = 0;

/* ---------------------------------------------------------------------------*/
// Motor Data.

volatile bool 		motorsIsOn = true;
const 	 uint32_t 	maxPWM = 2000;
volatile uint32_t 	pololuPWM = 120;
volatile uint32_t 	edf27PWM = 120;
volatile int32_t 	pololuLeftPWM  = 0;
volatile int32_t 	pololuRightPWM = 110;

/* ---------------------------------------------------------------------------*/
// Wheel And Tychometric Data.

const 	 float 		circum = 2*(3.14159265359)*(0.032/2);
const 	 float 		nrOfSpokes = 36.0;
const 	 uint32_t	spokesThreshold = 1950;
const 	 uint32_t 	spokesLimit = 100;
volatile int32_t 	leftSpokeCnt  = 0;
volatile int32_t 	rightSpokeCnt = 0;
volatile uint32_t 	leftSpokeType = 0;
volatile uint32_t 	rightSpokeType = 0;
volatile int32_t 	leftLapCnt = 0;
volatile int32_t 	rightLapCnt = 0;
/*volatile float leftSpeed = 0.0;
volatile float rightSpeed = 0.0;*/

/* ---------------------------------------------------------------------------*/
// Regulator Data (PID).

const 	 float 		reference = 0.0;
volatile float 		position = 0.0;
volatile float 		error = 0.0;
volatile float 		previousError = 0.0;
volatile int32_t 	motorSpeed = 0;
volatile float 		Kp = 7.5;
volatile float 		Ki = 1.0;
volatile float 		Kd = 6.0;
volatile float 		integral = 0.0;
const 	 float 		integralMAX = 10.0;
volatile float 		derivative = 0.0;
const 	 uint16_t 	dt = 1;
volatile float 		iFilter = 0.0;
volatile float 		dFilter = 0.0;
volatile float 		whitePercent = 0.0;
volatile float 		blackPercent = 0.0;

/* ---------------------------------------------------------------------------*/
// Line Data.

volatile uint32_t 	middle = 0;
volatile float 		weight1 = 0;
volatile float 		weight2 = 0;
volatile float 		weight3 = 0;
volatile float 		weight4 = 0;
volatile float 		weight5 = 0;
volatile float 		weight6 = 0;
volatile float 		weight7 = 0;

/* ---------------------------------------------------------------------------*/
// Flags.

volatile bool 		adcDMA = false;
volatile bool 		handleUsart = false;
volatile bool 		usartRequested = false;

volatile bool 		runState = false;
volatile bool 		sendUsart = false;
volatile bool 		runRegulator = false;

/* ---------------------------------------------------------------------------*/
// States.

volatile bool  		isFindingLine = false;
volatile float 		virtualSensor = 0.0;
volatile float 		stopTimer = 0.0;
volatile float 		stopTimerLimit = 3000;

/* ---------------------------------------------------------------------------*/
// Send-Data.

volatile float 		values[24];
volatile float 		*adcValues[14];
volatile float 		*errorValue;
volatile float 		*integralValue;
volatile float 		*derivativeValue;
volatile float 		*leftLapCntValue;
volatile float 		*rightLapCntValue;
volatile float 		*leftRegValue;
volatile float 		*rightRegValue;
volatile float 		*virtualSensorValue;
volatile float 		*isFindingLineValue;
volatile float 		*stopTimerValue;


/* ---------------------------------------------------------------------------*/
void setLeftMotor(int32_t m_speed){
	if(m_speed >= 0){
		TIM_SetCompare1(TIM4, m_speed);
		TIM_SetCompare2(TIM4, 0);		
	}else{
		TIM_SetCompare1(TIM4, 0);
		TIM_SetCompare2(TIM4, -m_speed);
	}
}
void setRightMotor(int32_t m_speed){

	if(m_speed >= 0){
		TIM_SetCompare3(TIM4, m_speed);
		TIM_SetCompare4(TIM4, 0);	
	}else{
		TIM_SetCompare3(TIM4, 0);
		TIM_SetCompare4(TIM4, -m_speed);
	}
}
void drive_go_forward(int32_t left,int32_t right){
	setLeftMotor(left);
	setRightMotor(right);
}
void drive_go_reverse(int32_t m_speed){
	setLeftMotor(m_speed);
	setRightMotor(m_speed);
}
void drive_turn_left(int32_t m_speed){
	
	setRightMotor(m_speed);
	setLeftMotor(0);
}
void drive_turn_right(int32_t m_speed){
	
	setLeftMotor(m_speed);
	setRightMotor(0);
}
void drive_rotate_left(int32_t m_speed){
	setLeftMotor(-m_speed);
	setRightMotor(m_speed);
}
void drive_rotate_right(int32_t m_speed){
	setLeftMotor(m_speed);
	setRightMotor(-m_speed);
}
void drive_total_stop( ){
	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, 0);
}
void drive_go(int32_t m_speed, int32_t err){

	if(err == 0.0){
		drive_go_forward(pololuLeftPWM,pololuRightPWM);
	}
	
	if(err < 0.0){
		
		if(err < -100.0){
			drive_rotate_left(pololuPWM/2);
		}else{
			if(err < -50.0){
				drive_turn_left(pololuLeftPWM);
			}else{
				//if(err < -20.0){
					drive_go_forward(pololuLeftPWM,pololuRightPWM);
				//}
			}
		}
	}else{

		if(err < 20.0){
			drive_go_forward(pololuLeftPWM,pololuRightPWM);
		}else{
			if (err < 50.0){
				drive_turn_right(pololuRightPWM);
			}else{
				//if (err < 100.0) {
					drive_rotate_right(pololuPWM/2);
				//}
			}
		}
	}
}
void setMotorsForwards(int32_t left, int32_t right){

	if(left>maxPWM){left=maxPWM;}
	if(right>maxPWM){right=maxPWM;}
	TIM_SetCompare1(TIM4, right);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, left);
	TIM_SetCompare4(TIM4, 0);
}
void setMotorsBackwards(int32_t left, int32_t right){

	if(left>maxPWM){left=maxPWM;}
	if(right>maxPWM){right=maxPWM;}
	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, right);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, left);
}
/* ---------------------------------------------------------------------------*/
void inits( ){
	
	setup_GPIO( );
	setup_ADC1_with_DMA2_NVIC( );
	setup_USART6_with_DMA2_NVIC( );
	setup_SPI2_with_DMA1_NVIC( );
	setup_PWM_with_TIM4_NVIC( );

 // Turn off NFET.
	GPIO_WriteBit(NFETPORT, NFETPIN, Bit_RESET);

 // Init the LEDs to all full.
	LED_data[0] = LED_data[1] = 0xFF;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
	DMA_Cmd(DMA1_Stream4, ENABLE);

 // Init the ADC difference to max.	
	for(uint32_t i = 0; i<nrOfSensors; i++){
		ADC_MAX[i] = 4095;
		ADC_MIN[i] = 0;
		ADC_DIF[i] = 4095;
	}

 // Init pointers.
	for(uint32_t i = 0; i<14; i++){
		values[i] = 0.0;
		adcValues[i] = &values[i];
		*adcValues[i] = 100.0;
	}

	errorValue 		   = &values[14];
	integralValue 	   = &values[15];
	derivativeValue    = &values[16];
	leftLapCntValue    = &values[17];
	rightLapCntValue   = &values[18];
	leftRegValue 	   = &values[19];
	rightRegValue 	   = &values[20];
	virtualSensorValue = &values[21];
	isFindingLineValue = &values[22];
	stopTimerValue	   = &values[23];

	*adcValues[12]		= 0.0;
	*adcValues[13]		= 0.0;
	*errorValue 		= 0.0;
	*integralValue 		= 0.0;
	*derivativeValue 	= 0.0;
	*leftLapCntValue 	= 0.0;
	*rightRegValue 		= 0.0;
	*rightLapCntValue 	= 0.0;
	*leftRegValue 		= 0.0;
	*virtualSensorValue = 0.0;
	*isFindingLineValue = 0.0;

 // Init all variables.
 	weight1 = 78.0;
	weight2 = 66.0;
	weight3 = 54.0;
	weight4 = 42.0;
	weight5 = 30.0;
	weight6 = 18.0;
	weight7 = 6.0;

	iFilter = 0.0001;
	dFilter = 0.1;

	whitePercent = 0.25;
	blackPercent = 0.5;
}
void calibrate(uint32_t val){

	uint32_t nrOfCalibrations = 16;

	float percent = whitePercent;
	whitePercent = 0.3;

	if(allWhite( )){

		for(uint16_t i=0;i<nrOfSensors;i++){
			ADC_MIN[i]=0;
		}

		for(uint32_t i = 0; i < nrOfCalibrations; i++){
			readAdc( );
			for(uint32_t j = 0; j < nrOfSensors; j++){
				ADC_MIN[j] += ADC_avgBuff[j];
			}
		}

		for(uint16_t i=0;i<nrOfSensors;i++){
			ADC_MIN[i]=ADC_MIN[i]/nrOfCalibrations;
			ADC_DIF[i] = ADC_MAX[i] - ADC_MIN[i];
		}

	}else{

		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);

		for(uint16_t i=0;i<nrOfSensors;i++){
			ADC_MAX[i]=0;
		}

		for(uint32_t i = 0; i < nrOfCalibrations; i++){
			readAdc( );
			for(uint32_t j = 0; j < nrOfSensors; j++){
				ADC_MAX[j] += ADC_avgBuff[j];
			}
		}

		for(uint16_t i=0;i<nrOfSensors;i++){
			ADC_MAX[i]=ADC_MAX[i]/nrOfCalibrations;
			ADC_DIF[i] = ADC_MAX[i] - ADC_MIN[i];
		}	
	}

	whitePercent = percent;
}
void setLED( ){

	LED_data[0] = 0;
	LED_data[1] = 0;

	for(uint16_t i=0; i<8; i++){
		if(ADC_avgBuff[i]>2500){
			LED_data[1] += (1<<i);
		}
	}
	for(uint16_t i=0; i<4; i++){
		if(ADC_avgBuff[i+8]>2500){
			LED_data[0] += (1<<i);
		}
	}
	//if white
	if(ADC_rawBuff[13]>2000){
		LED_data[0] += (1<<4);
		LED_data[0] += (1<<5);
	}		
	if(ADC_rawBuff[12]>2000){
		LED_data[0] += (1<<6);
		LED_data[0] += (1<<7);
	}

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
	DMA_Cmd(DMA1_Stream4, ENABLE);
}
void sendUART( ){

	for(uint32_t i = 0; i<nrOfSensors; i++){

		USART_putn(USART6, (char*)&ADC_percentage[i], 4);
	}		

	USART_putn(USART6, (char*)&Kp, 4);			// f
	USART_putn(USART6, (char*)&Ki, 4);			// f
	USART_putn(USART6, (char*)&Kd, 4);			// f
	USART_putn(USART6, (char*)&(*virtualSensorValue), 4);	// i
	USART_putn(USART6, (char*)&(*errorValue), 4);		// i
	USART_putn(USART6, (char*)&(*integralValue), 4);	// f
	USART_putn(USART6, (char*)&(*derivativeValue), 4);	// f////
	USART_putn(USART6, (char*)&motorSpeed, 4);	// i
	USART_putn(USART6, (char*)&(*leftLapCntValue), 4);	// i
	USART_putn(USART6, (char*)&(*rightLapCntValue), 4);	// i
	USART_putn(USART6, (char*)&pololuPWM, 4);	// i
	USART_putn(USART6, (char*)&pololuPWM, 4);	// i
	USART_putn(USART6, (char*)&(*leftRegValue), 4);		// i
	USART_putn(USART6, (char*)&(*rightRegValue), 4);	// i
}
void sendSettings( ){

	USART_putn(USART6, (char*)&Kp, 4);				// f
	USART_putn(USART6, (char*)&Ki, 4);				// f
	USART_putn(USART6, (char*)&Kd, 4);				// f
	USART_putn(USART6, (char*)&iFilter, 4);			// f
	USART_putn(USART6, (char*)&dFilter, 4);			// f
	USART_putn(USART6, (char*)&integralMAX, 4);		// f
	USART_putn(USART6, (char*)&weight1, 4);			// f
	USART_putn(USART6, (char*)&weight2, 4);			// f
	USART_putn(USART6, (char*)&weight3, 4);			// f
	USART_putn(USART6, (char*)&weight4, 4);			// f
	USART_putn(USART6, (char*)&weight5, 4);			// f
	USART_putn(USART6, (char*)&weight6, 4);			// f
	USART_putn(USART6, (char*)&weight7, 4);			// f
	USART_putn(USART6, (char*)&whitePercent, 4);	// f
	USART_putn(USART6, (char*)&blackPercent, 4);	// f
	USART_putn(USART6, (char*)&nrOfSamples, 4);		// i
	USART_putn(USART6, (char*)&stopTimerLimit, 4);	// f
	USART_putn(USART6, (char*)&pololuPWM, 4);		// i
	USART_putn(USART6, (char*)&edf27PWM, 4);		// i	
}
void regulator( ){

 // Read ADC.
	readAdc( );
	AdcToPercetage( );
	setLED( );

 // Was the line found?
	if(theLineIsLost( )){
		*isFindingLineValue = 1.0;
		isFindingLine = true; //GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}else{
		*isFindingLineValue = -1.0;
		isFindingLine = false; //GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	}

 // Decide upon former.
	if(isFindingLine){
	
		stopTimer++;	
		if(*virtualSensorValue < 0.0){ drive_turn_right(80); }
		else{ drive_turn_left(90); }
	}
	else{
		pid( );
	}
}
bool allWhite( ){

	int32_t cnt = 0;

	while(true){
		
		if(cnt==nrOfSensors){
			return true;
		}

		if((ADC_percentage[cnt] > whitePercent)){
			
			return false;
		}
		cnt++;
	}	
}
bool allBlack( ){

	int32_t cnt = 0;
	float critical = blackPercent*nrOfSensors;
	float sum = 0.0;

	while(sum<critical){
				
		if(cnt==nrOfSensors){
			return false;
		}

		sum += ADC_percentage[cnt];
		cnt++;		
	}
	return true;
}
bool theLineIsLost( ){

	if(allWhite( )){
		return true;
	}else{
		// if(allBlack( )){
		// 	return true;
		// }else{
			return false;
		// }
	}
}
void pid( ){

 // PID Regulation.
 	//position = findMiddle( );
 	position = (float)interpolateMiddle( );

	//*errorValue = reference-position;
	*errorValue = 110.5-position;
	if(*errorValue == 0.0){*virtualSensorValue = 0.0;}
		else{if(*errorValue < 0.0){*virtualSensorValue = -1.0;}
			else{*virtualSensorValue = 1.0;}}

	*integralValue = (1-iFilter)*(*integralValue) + iFilter*(*errorValue)*dt;
	if(*integralValue>=integralMAX){*integralValue=integralMAX;}
	if(*integralValue<=-integralMAX){*integralValue=-integralMAX;}

	*derivativeValue = (1-dFilter)*(*derivativeValue) + dFilter*(*errorValue-previousError)/dt;
	previousError = *errorValue;

	motorSpeed = (Kp*(*errorValue) + Ki*(*integralValue) + Kd*(*derivativeValue));

 // Adjust the PWM.
	pololuLeftPWM  = pololuPWM - motorSpeed; 
	pololuRightPWM = pololuPWM + motorSpeed;
	*leftRegValue = (float)pololuLeftPWM;
	*rightRegValue = (float)pololuRightPWM;

	if(pololuLeftPWM>=125){
		pololuLeftPWM = 125;
	}

	if(pololuRightPWM>=125){
		pololuRightPWM = 125;
	}

	if(motorsIsOn){
		//drive_go(motorSpeed, error);
		//setMotorsForwards(pololuLeftPWM,pololuRightPWM);
		setLeftMotor(pololuLeftPWM);
		setRightMotor(pololuRightPWM);
	}
}
void readAdc( ){

 // Erase old buffer.
	for(uint16_t i=0; i<nrOfSensors; i++){
		ADC_avgBuff[i] = 0;
	}

 // Turn on NFET.
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);

 // Theoretical wait for ADC to stable.
 	for(uint32_t j=0; j < 10; j++){
		__NOP( );
	}	

 // Read the ADC (nrOfSamples) times.
	for(uint16_t i = 0; i<nrOfSamples; i++){

		for(uint16_t j=0; j<nrOfSensors; j++){
			ADC_avgBuff[j] += ADC_rawBuff[j];
		}
	}
 // THIS MUST GO WHEN DISCOBOARD GOES!!!
	uint32_t ADC_avgBuff_cp = 0;
	uint32_t ADC_avgBuff_cp1 = 0;
	for(uint16_t i=0; i<nrOfSamples; i++){
			ADC_avgBuff_cp += ADC_rawBuff[14];
			ADC_avgBuff_cp1 += ADC_rawBuff[15];
			for(uint16_t j = 0; j<11; j++){
				__NOP( );
			}
	}
	ADC_avgBuff_cp = ADC_avgBuff_cp/nrOfSamples;
	ADC_avgBuff_cp1 = ADC_avgBuff_cp1/nrOfSamples;	
 //////////////////////////////////////////////////

 // Turn off NFET.
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);

 // Divide and get the average.
	for(uint16_t i=0; i<nrOfSensors; i++){
		
		ADC_avgBuff[i] = (ADC_avgBuff[i]/nrOfSamples);
	}

 // THIS MUST GO WHEN DISCOBOARD GOES!!!
 	ADC_avgBuff[10] = ADC_avgBuff_cp;
 	ADC_avgBuff[4] = ADC_avgBuff_cp1;
 //////////////////////////////////////////////////
}
void AdcToPercetage( ){
 // Adjust with the calibration data to percentage.
 	float percent = 0.0;
	for(uint16_t i=0; i<nrOfSensors; i++){
		
		percent = ((float)ADC_avgBuff[i]-(float)ADC_MIN[i]);
		
		if(percent <= 0){
			ADC_percentage[i] = 0.0;
		}else{
			percent = percent/((float)ADC_DIF[i]);
			if(percent>1){
				ADC_percentage[i] = 1;
			}else{
				ADC_percentage[i] = percent;
			}			
		}	
		*adcValues[i]=	ADC_percentage[i];
	}
}
float findMiddle( ){

	float value = 0.0;

	value -= ADC_percentage[0]*weight1;
	value -= ADC_percentage[1]*weight2;
	value -= ADC_percentage[2]*weight3;
	value -= ADC_percentage[3]*weight4;
	value -= ADC_percentage[4]*weight5;
	value -= ADC_percentage[5]*weight6;
	value += ADC_percentage[6]*weight6;
	value += ADC_percentage[7]*weight5;
	value += ADC_percentage[8]*weight4;
	value += ADC_percentage[9]*weight3;
	value += ADC_percentage[10]*weight2;
	value += ADC_percentage[11]*weight1;

	return value;
}
uint16_t interpolateMiddle( ){

	uint32_t n = 20;
	uint32_t nrOf = nrOfSensors;
	uint32_t s0 = 0;
	uint32_t s1 = 0;

	uint32_t totalSteps = (nrOf-1)*n;
	uint32_t values[totalSteps];
	uint32_t sum = 0;

 // Sum.
	for(uint16_t i=0; i<nrOf-1; i++){
		
		s0 = ADC_avgBuff[i];
		s1 = ADC_avgBuff[i+1];

		for(uint16_t j=0; j<n; j++){
			values[i*n+j] = (s0*(n-j)+s1*j)/n;
			sum += values[i*n+j];
		}
	}

	uint32_t halfSum = sum/2;
	sum = 0;
	uint16_t step = 0;

 // Integrate to half.
	while(sum<halfSum){
		sum += values[step++];
	}

	//uint16_t position = step/totalSteps*4095;
	return step;

	/*
		V0 = (S1*(n-0)+S2*0)/n
		V1 = (S1*(n-1)+S2*1)/n
		V2 = (S1*(n-2)+S2*2)/n
		...
		Vn = (Sn*(n-n-1)+S2*n)/n
	*/
}
void usartHandler( ){
	
	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	
 // Ping.
	if(usartBuffert[0] == 1.0){

		USART_putn(USART6, (char*)&dt, 4);
	}

 // Usart send requested.
	if(usartBuffert[0] == 2.0){		
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		
		if(usartBuffert[1]==0.0){
			while( !(USART6->SR & 0x00000040) );
			DMA_Cmd(DMA2_Stream6, ENABLE);//GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		}
		else{
			if(usartBuffert[1]==1.0){
				sendSettings( );
			}
		}
	}

 // Auto Send Usart.
 	if(usartBuffert[0] == 3.0){		

		if(usartBuffert[1]==0.0){
			sendUsart=false; //GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		}
		if(usartBuffert[1]==1.0){
			sendUsart=true; //GPIO_SetBits(GPIOD, GPIO_Pin_15);
		}
	}

 // Start.
	if(usartBuffert[0] == 11.0){

		runState = true; //GPIO_ToggleBits(GPIOD, GPIO_Pin_12);GPIO_SetBits(GPIOD, GPIO_Pin_12);		
	}

 // Stop.
	if(usartBuffert[0] == 12.0){

		runState = false;
		drive_total_stop( ); //GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	}

 // Set PID.
	if(usartBuffert[0] == 20.0){

		Kp = usartBuffert[1];
		Ki = usartBuffert[2];
		Kd = usartBuffert[3];
	}

 // Set PID-filter.
	if(usartBuffert[0] == 21.0){

		iFilter = usartBuffert[1];
		dFilter = usartBuffert[2];
	}

 // Reset PID.
	if(usartBuffert[0] == 22.0){

		position = 0; 	*errorValue = 0; *integralValue = 0;
		derivative = 0; previousError = 0; 	motorSpeed = 0;
	}

 // Set Pololu PWM And EDF.
	if(usartBuffert[0] == 30.0){

		pololuPWM = (uint32_t)usartBuffert[1];		
		edf27PWM = (uint32_t)usartBuffert[2];
		//setMotorsForwards(pololuPWM,pololuPWM);
		//setMotorsBackwards(pololuPWM,pololuPWM);
		//setMotorsBackwards(pololuPWM,0);
	}

 // Stop or Start Motors.
	if(usartBuffert[0] == 31.0){

		if(usartBuffert[1]==0){ motorsIsOn = false; drive_total_stop( );}
		else{ motorsIsOn = true; }		
	}

 // Calibrate. 
	if(usartBuffert[0] == 40.0){		

		if(usartBuffert[1]==0.0){ calibrate(0); } // Minimum (White).
		if(usartBuffert[1]==1.0){ calibrate(1); } // Maximum (Black).
		if(usartBuffert[1]==2.0){ calibrate(2); } // Difference.
	}

 // Sensor Weights.
	if(usartBuffert[0] == 41.0){

		weight1 = usartBuffert[1];
		weight2 = usartBuffert[2];
		weight3 = usartBuffert[3];
	}

 // Sensor Weights.
	if(usartBuffert[0] == 42.0){

		weight4 = usartBuffert[1];
		weight5 = usartBuffert[2];
		weight6 = usartBuffert[3];
	}

 // Sensor Weights.
	if(usartBuffert[0] == 43.0){
		
		weight7 = usartBuffert[1];
		nrOfSamples = (uint32_t)usartBuffert[2];
	}

 // Track Settings.
	if(usartBuffert[0] == 44.0){

		whitePercent = usartBuffert[1];
		blackPercent = usartBuffert[2];
		stopTimerLimit = usartBuffert[3];
	}
}
void spokesCounter( ){

 // Counting left spokes.
	if(leftSpokeType==0){			// If current is 0 == White.
		if(ADC_rawBuff[12] > (spokesThreshold+spokesLimit)){ // If now reading a black.
			leftSpokeType = 1;
			leftSpokeCnt++;
		}	
	}else{							// Current is 1 == Black.
		if(ADC_rawBuff[12] < (spokesThreshold-spokesLimit)){ // If now reading a black.
			leftSpokeType = 0;
			leftSpokeCnt++;
		}
	}
	if(leftSpokeCnt >= nrOfSpokes){
		leftSpokeCnt=0;
		*leftLapCntValue += 1.0;
	}

 // Counting right spokes.
	if(rightSpokeType==0){			// If current is 0 == White.
		if(ADC_rawBuff[13] > (spokesThreshold+spokesLimit)){ // If now reading a black.
			rightSpokeType = 1;
			rightSpokeCnt++;
		}	
	}else{							// Current is 1 == Black.
		if(ADC_rawBuff[13] < (spokesThreshold-spokesLimit)){ // If now reading a black.
			rightSpokeType = 0;
			rightSpokeCnt++;
		}
	}
	if(rightSpokeCnt >= nrOfSpokes){
		rightSpokeCnt=0;
		*rightLapCntValue += 1.0;
	}
}
/* Main ----------------------------------------------------------------------*/
int main(void){

	inits( );

	//while(1){;;}
	while(1){

	 // Runs in 1000Hz.
		if(runRegulator){
			runRegulator = false;
			regulator( );			
		}
		
	 // Reset if line is lost.
 /*		if(stopTimer >= 3000){
			stopTimer = 0;
			isFindingLine = false;
			runState = false;
			drive_total_stop( );
		}*/	

	 // Runs when all ADC are read, to calculate speed.
		if(adcDMA){
			adcDMA = false;
			spokesCounter( );
		}

	 // Runs when data has been received.
		if(handleUsart){
			handleUsart = false;
			usartHandler( );
		}

	 // Runs when data has been requested.	
		if(usartRequested){
			usartRequested = false;
			sendUART( );
		}
	}
}
/* Nvics  --------------------------------------------------------------------*/
void TIM4_IRQHandler( ){

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		usartCnt++;	

		if(isFindingLine){ stopTimer++; }
		else{ stopTimer = 0; }		

		if(runState){ runRegulator = true; }// Flag for running the robot.
		else{ /*setMotorsForwards(0,0);*/ }
	}
}
/*ADC Stream*/
void DMA2_Stream0_IRQHandler( ){

	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)){
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		adcDMA = true;
	}
}
/*USART Stream*/
void DMA2_Stream1_IRQHandler(void){

	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{		
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);

		//nrOfUsartCalls++;	
		
		//if(nrOfUsartCalls==3){
			//sendUsart=true;
		//}	
		//if(nrOfUsartCalls>=3){
			
			handleUsart = true;
		//}
	}
}
/*USART RX - Stream*/
void DMA2_Stream6_IRQHandler(void){

	if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) == SET)
	{		
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
		//DMA_Cmd(DMA2_Stream6,DISABLE);
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
        
        while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        *s++;
    }
}
/* Setups --------------------------------------------------------------------*/
/*REFLEXSENSORS*/
void setup_ADC1_with_DMA2_NVIC( ){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE); 	// Clock for the ADC port!! Do not forget about this one ;)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   

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

	DMA_InitTypeDef DMA_InitStruct;
	DMA_DeInit(DMA2_Stream0);

	DMA_InitStruct.DMA_Channel            = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)&ADC_rawBuff[0];
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

	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_Cmd(DMA2_Stream0, ENABLE);

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

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 13, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 15, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 16, ADC_SampleTime_28Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);   	
	ADC_SoftwareStartConv(ADC1);
}
/*DB-LEDs AND LE,NFET*/
void setup_GPIO( ){

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

 // LEDS.
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

 // Setup the nFET = 13 and LE = 14.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_13 | LATCH_ENABLE_PIN);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/*MOTORS*/
void setup_PWM_with_TIM4_NVIC( ){

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 500-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* PWM1 Mode configuration: Channel3 (GPIOB Pin 9)*/
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	/* PWM1 Mode configuration: Channel4 (GPIOB Pin 8)*/
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	/* PWM1 Mode configuration: Channel1 (GPIOB Pin 6)*/
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	/* PWM1 Mode configuration: Channel2 (GPIOB Pin 7)*/
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0); 
	TIM_SetCompare4(TIM4, 0);

	NVIC_InitTypeDef NVIC_InitStruct;

	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
/*LEDs*/
void setup_SPI2_with_DMA1_NVIC( ){

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// SPI2_MOSI = PB15, SPI2_CLK  = PB10.
	GPIO_InitStruct.GPIO_Pin   = (SPI2_CLK_PIN | SPI2_MOSI_PIN);
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);  

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2); // SCK  = PB10.
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); // SCK  = PB13.
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); // MOSI = PB15.

	DMA_InitTypeDef DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_InitStruct.DMA_Channel			  = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr	  = (uint32_t)&LED_data;
	DMA_InitStruct.DMA_DIR				  = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize		  = 2;
	DMA_InitStruct.DMA_PeripheralInc	  = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc		  = DMA_MemoryInc_Enable;//
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize	  = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode			   	  = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority		   	  = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode		   	  = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_MemoryBurst		  = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst	  = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream4, &DMA_InitStruct);
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	SPI_InitTypeDef SPI_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPI2);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode	     = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize  = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL	     = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA	     = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS	     = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_FirstBit  = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 0;

	SPI_Init(SPI2, &SPI_InitStruct);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	//SPI_SSOutputCmd(SPI2, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
}
/*LED Stream*/
void DMA1_Stream4_IRQHandler( ){
	
	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) == SET){
		
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);

		GPIO_SetBits(GPIOB, LATCH_ENABLE_PIN);
		GPIO_ResetBits(GPIOB, LATCH_ENABLE_PIN);
	}
	// if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_HTIF4)){
	// 	//GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
	// 	DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_HTIF4);
	// }
}
/*SERIAL-COM*/
void setup_USART6_with_DMA2_NVIC( ){

 // USART6 TX = PC6 Grey
 //		   RX = PC7 White

 // GPIO
	GPIO_InitTypeDef  GPIO_InitStruct; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;//
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_InitTypeDef USART_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	USART_InitStruct.USART_BaudRate   = 115200;//9600//115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_Mode       = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStruct);

	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART6, ENABLE);  

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);


	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

 // DMA2 for receiving, Stream 1 Channel 5
	DMA_InitTypeDef DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);


	DMA_DeInit(DMA2_Stream1);

	DMA_InitStruct.DMA_Channel 			  = DMA_Channel_5;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr 	  = (uint32_t)&usartBuffert;
	DMA_InitStruct.DMA_DIR 				  = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize 		  = 16;
	DMA_InitStruct.DMA_PeripheralInc 	  = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc 		  = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize 	  = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode 			  = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority 		  = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode 		  = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold 	  = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst 		  = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst	  = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream1, &DMA_InitStruct);

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);


	DMA_DeInit(DMA2_Stream6);

 // DMA2 for transmitting, Stream 6 Channel 5
	DMA_InitStruct.DMA_Channel 			  = DMA_Channel_5;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr 	  = (uint32_t)&values;
	DMA_InitStruct.DMA_DIR 				  = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize 		  = 96;
	DMA_InitStruct.DMA_PeripheralInc 	  = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc 		  = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize 	  = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode 			  = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority 		  = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode 		  = DMA_FIFOMode_Disable;
	//DMA_InitStruct.DMA_FIFOThreshold 	  = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst 		  = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst	  = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream6, &DMA_InitStruct);

	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA2_Stream6, ENABLE);


	//USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
