#include "functions.h"
#include "bldc_headers.h"

void GENERAL_Pins_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOD, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
}



void PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    //////////////-------TIMER 4-----------////////////////////////
    /* ENable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 	//T2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);	//T4
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);	//T6

    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    ///---Configure Timer---///
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    /* Enable clock for TIM4 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* Set parameters */
    TIM_BaseStruct.TIM_Prescaler = 1; //timer_tick_freq=timer_clock/(prescaler +1)
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;	//counting up
    TIM_BaseStruct.TIM_Period = TIMER_PERIOD; //APB 42 MHZ / 8400 = 5 kHz || 0..8399
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;

    /* Initialize TIM4 */
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);

    /* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);

    ///---Configure OCMode---///
    TIM_OCInitTypeDef TIM_OCStruct;

    /* Common settings */
    /* PWM mode 2 = Clear on compare match */
    /* PWM mode 1 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 0; /* 25% duty cycle */
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCStruct.TIM_Pulse = 0; /* 50% duty cycle */
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCStruct.TIM_Pulse = 0; /* 75% duty cycle */
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}


void TIM2_OCM_IT_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	///////////////----TIMER 2 Used to refresh Alphanumeric display//////
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM2);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	///---Configure Timer---///
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//ABP1 42MHz x 2 = 84MHz
	TIM_BaseStruct.TIM_Prescaler = 16799; //timer_tick_freq=timer_clock/(prescaler +1) //10 khz freq
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;	//counting up
	//timer is 16 bit -> 0..65535, TIM_Period=timer_tick_freq/PWM_freq-1
	TIM_BaseStruct.TIM_Period = 0xFFFF;//ca³y zakres licznika
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM2 */
	TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
	/* Start count on TIM2 */
	TIM_Cmd(TIM2, ENABLE);

	///---Configure OCMode---///
	TIM2->CCER &= ~(0x0003); 	//write in Capture/Compare Enable Register CC1E and CC1P to 0
								//output disabled
	TIM2->CCMR1 &= ~(0x0003);	//Capture Compare Mode Register 1 set CC1S to 00
								//OC1 channel configure as output
	TIM2->CCMR1 &= ~(0x0070);   //Output Compare Mode set to FROZEN
	TIM2->CCR1 = 5000;			//Capture Compare Register 1, 16bit (Time = 1/timclk*ccr1)
	TIM2->DIER |= 0x0002;		//DMA Interupt Enable Register - Enable Interupt on channel 1

	///---Configure Interrupts---///
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

//WYSWIETLANIE NA LCD
void TIM2_IRQHandler(void){
	unsigned char buff[6];

	TIM2->CNT &= 0x0000;
	if (TIM_GetFlagStatus(TIM2,TIM_FLAG_CC1)){
		//WYSWIETL POTRZEBNE INFO NA LCD

		/// PRAD
		adcVal=scaledCurrentSens*((currentLCD/4095.0*3.3)-(offset_value/4095.0*3.3));
		snprintf(buff, sizeof buff, "%lf", adcVal);
		TM_HD44780_Puts(0, 0, buff);

		///V BAT
		adcVal = (double)adcValMean[1]*0.0089709013;
		snprintf(buff, sizeof buff, "%lf", adcVal);
		TM_HD44780_Puts(0,1,buff);

		///MANETKA
		//adcVal=scaledCurrentSens*((adcValMean[2]/4095.0*2.958)); //-scaledZeroCurrentVol);;
		snprintf(buff, sizeof buff, "%d", adcValMean[2]);
		TM_HD44780_Puts(8, 0, buff);

		///INNE
		adcVal = 60*10000/3/(2*PARY_BIEGUN*RPM);
		snprintf(buff, sizeof buff, "%lf", adcVal);
		TM_HD44780_Puts(8, 1, buff);

	}
	TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
}


void Counting_TIMER_Init(void){
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = 8399; //timer_tick_freq=timer_clock/(prescaler +1) //10 khz freq
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;	//counting up
	//timer is 16 bit -> 0..65535, TIM_Period=timer_tick_freq/PWM_freq-1
	TIM_BaseStruct.TIM_Period = 0xFFFF;//ca³y zakres licznika
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM3 */
	TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
	/* Start count on TIM3 */
	TIM_Cmd(TIM3, ENABLE);
}





void PWM_OUT_Set_Value(uint8_t channel, uint32_t pulse){
	//channels 1, 2, 3 = T2, T4, T6
	if (pulse>TIMER_PERIOD) pulse=TIMER_PERIOD;
	switch(channel){
		case 1:
			TIM4->CCR1=pulse;
		break;
		case 2:
			TIM4->CCR2=pulse;
		break;
		case 3:
			TIM4->CCR3=pulse;
		break;
		default:
		break;
	}
}

void PWM_OUT_Clear(uint8_t channel){
	switch(channel){
			case 1:
				TIM4->CCR1=0;
			break;
			case 2:
				TIM4->CCR2=0;
			break;
			case 3:
				TIM4->CCR3=0;
			break;
			default:
			break;
	}
}


void EXTI_Pins_Init(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

    /* PD5 is connected to EXTI_Line5 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/*
void EXTI0_IRQHandler(void) {					//czujnika HALLA A
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){	//sprawdzenie flagi o przerwaniu
		//RPM = 400;
		RPM = 60*10000/(2*3*TIM_GetCounter(TIM3));
		TIM_SetCounter(TIM3,0);


		EXTI_ClearITPendingBit(EXTI_Line0);		//wyczyszczenie flagi
    }
}
*/


void BLDC_komutacja(uint8_t number){
	switch(number){
	case (0):
		GPIO_ResetBits(GPIOD, T1 | T3 | T5);
		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T4);
		PWM_OUT_Clear(T6);
	break;
	case (1):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T1 | T5);
		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T4);
		//Set new
		GPIO_SetBits(GPIOD, T3);
		PWM_OUT_Set_Value(T6,pwm_duty);
	break;
	case (2):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T1 | T5);
		PWM_OUT_Clear(T4);
		PWM_OUT_Clear(T6);
		//Set new
		GPIO_SetBits(GPIOD, T3);
		PWM_OUT_Set_Value(T2,pwm_duty);
	break;
	case (3):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T1 | T3);
		PWM_OUT_Clear(T4);
		PWM_OUT_Clear(T6);
		//Set new
		GPIO_SetBits(GPIOD, T5);
		PWM_OUT_Set_Value(T2,pwm_duty);
	break;
	case (4):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T1 | T3);
		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T6);
		//Set new
		GPIO_SetBits(GPIOD, T5);
		PWM_OUT_Set_Value(T4,pwm_duty);
	break;
	case (5):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T3 | T5);
		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T6);
		//Set new
		GPIO_SetBits(GPIOD, T1);
		PWM_OUT_Set_Value(T4,pwm_duty);
	break;
	case (6):
		//Clear transistors
		GPIO_ResetBits(GPIOD, T3 | T5);
		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T4);
		//Set new
		GPIO_SetBits(GPIOD, T1);
		PWM_OUT_Set_Value(T6,pwm_duty);
	break;
	}
}


void BLDC_ruch(uint8_t kierunek, uint8_t poprzedni_kierunek){ //1-prawo, 0-lewo
	uint8_t hA = GPIO_ReadInputDataBit(GPIOD, hallA);
	uint8_t hB = GPIO_ReadInputDataBit(GPIOD, hallB);
	uint8_t hC = GPIO_ReadInputDataBit(GPIOD, hallC);
	uint8_t halls = ((hC<<2)|(hB<<1)|(hA)); //C B A

	if (kierunek!=poprzedni_kierunek) {
		Delayms(250);
	}
	//kierunek w prawo
	if (kierunek){
		switch (halls) {
		case 0:
			BLDC_komutacja(0);
		break;
		case 1:
			BLDC_komutacja(1);
		break;
		case 2:
			BLDC_komutacja(3);
		break;
		case 3:
			BLDC_komutacja(2);
		break;
		case 4:
			BLDC_komutacja(5);
		break;
		case 5:
			BLDC_komutacja(6);
		break;
		case 6:
			BLDC_komutacja(4);
		break;
		case 7:
		break;
		}
	}
	else {
		switch (halls){
		case 0:
			BLDC_komutacja(0);
			break;
		case 1:
			BLDC_komutacja(4);
			break;
		case 2:
			BLDC_komutacja(6);
			break;
		case 3:
			BLDC_komutacja(5);
			break;
		case 4:
			BLDC_komutacja(2);
			break;
		case 5:
			BLDC_komutacja(3);
			break;
		case 6:
			BLDC_komutacja(1);
			break;
		case 7:
			break;
		}
	}

}


void DMA_Initialization(){
	 DMA_InitTypeDef DMA_InitStructure;

	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	 DMA_StructInit(&DMA_InitStructure);

	 DMA_DeInit(DMA2_Stream0);  //Set DMA registers to default values
	   DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //adress zmiennej Ÿród³owej
	   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adcValue[0]; //adress pierwszego elementu tablicy docelowej
	   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	   DMA_InitStructure.DMA_BufferSize = ADC_CHANNELS; //Buffer size //ilosc zmiennych w tablicy docelowej
	   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
	   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
	   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	   DMA_Init(DMA2_Stream0, &DMA_InitStructure); //Initialize the DMA
	   DMA_Cmd(DMA2_Stream0, ENABLE); //Enable the DMA2 - Stream 4

}


void ADC_Initialization(){

	    GPIO_InitTypeDef GPIO_InitStruct;

	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);

	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	    ADC_InitTypeDef ADC_InitStruct;
	    ADC_StructInit(&ADC_InitStruct);
	    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	    ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	    ADC_InitStruct.ADC_NbrOfConversion = ADC_CHANNELS;
	    ADC_Init(ADC1, &ADC_InitStruct);

	    ADC_CommonInitTypeDef ADC_CommonInitStruct;
	    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	    ADC_CommonInit(&ADC_CommonInitStruct);

	    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);
	    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_15Cycles);

	    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	    ADC_DMACmd(ADC1, ENABLE);
	    ADC_Cmd(ADC1, ENABLE);
	    ADC_SoftwareStartConv(ADC1);
}





double PIi_reg(double setCurrent, double measuredCurrent){
	double i_uchyb;
	static double i_uchyb_suma = 0;
	double u_value;

	if (setCurrent > maxCurrent) setCurrent = maxCurrent;
	if (setCurrent <-maxCurrent) setCurrent = -maxCurrent;

	i_uchyb = setCurrent - measuredCurrent;
	i_uchyb_suma += i_uchyb;
	u_value = (K0i*i_uchyb+K1i*i_uchyb_suma)/0.0234;

	if (u_value > maxVolt) u_value = maxVolt;
	if (u_value < -maxVolt) u_value = -maxVolt;

	return u_value;
}

double CurrentMeasureAutoCalib(){
		uint16_t adcVal1;
		double adcValMean1 = 0;
		uint8_t x;

		PWM_OUT_Clear(T2);
		PWM_OUT_Clear(T4);
		PWM_OUT_Clear(T6);
		GPIO_ResetBits(GPIOD, T1|T3|T5);


		for (x=0; x<60; x++){
		    adcVal1 = ADC1_GetConvValue();
		    adcValMean1 += adcVal1;
		}

		adcValMean1 = adcValMean1/60.0;
		return  (adcValMean1*ADCfactor - scaledZeroCurrentVol);
}

void USART_Initialization(){
	GPIO_InitTypeDef     GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	///---PINS INIT---///
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	// Initialize pins as alternating function
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);


	///---USART and NVIC---///
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode =USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_2;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART6, &USART_InitStruct);
	USART_Cmd(USART6, ENABLE);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
}



void USART6_IRQHandler(void){
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET){
		usartData = USART_ReceiveData(USART6);
		//usartDataArray[byteCounter++] = USART_ReceiveData(USART6);
		//if (byteCounter>7) byteCounter = 0;
	}
}


/// NOWE FUNKCJE///

void BLDC_Step(void){
	Delay(150);
	uint8_t hA = GPIO_ReadInputDataBit(GPIOD, hallA);
	uint8_t hB = GPIO_ReadInputDataBit(GPIOD, hallB);
	uint8_t hC = GPIO_ReadInputDataBit(GPIOD, hallC);
	uint8_t halls = ((hC<<2)|(hB<<1)|(hA)); //C B A

		switch (halls) {
		case 0:
		break;
		case 1:
			BLDC_Com(1);
		break;
		case 2:
			BLDC_Com(3);
		break;
		case 3:
			BLDC_Com(2);
		break;
		case 4:
			BLDC_Com(5);
		break;
		case 5:
			BLDC_Com(6);
		break;
		case 6:
			BLDC_Com(4);
		break;
		case 7:
		break;
		}

}



void BLDC_Com(uint8_t number){
	switch(number){
	case (1):
		//Wy³¹cz stare
		GPIO_ResetBits(GPIOD, T1 | T5);
		TIM4->CCR1 = 0;
		T2_mask = 0;
		TIM4->CCR2 = 0;
		T4_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T3);
		TIM4->CCR3 = pwm_duty;
		T6_mask = 0xFFFF;
	break;
	case (2):
		//Wy³¹cz stare
		GPIO_ResetBits(GPIOD, T1 | T5);
		TIM4->CCR2 = 0;
		T4_mask = 0;
		TIM4->CCR3 = 0;
		T6_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T3);
		TIM4->CCR1 = pwm_duty;
		T2_mask = 0xFFFF;
	break;
	case (3):
		//Wy³¹cz stare
		GPIO_ResetBits(GPIOD, T1 | T3);
		TIM4->CCR2 = 0;
		T4_mask = 0;
		TIM4->CCR3 = 0;
		T6_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T5);
		TIM4->CCR1 = pwm_duty;
		T2_mask = 0xFFFF;
	break;
	case (4):
		//Wy³¹cz stare
		GPIO_ResetBits(GPIOD, T1 | T3);
		TIM4->CCR1 = 0;
		T2_mask = 0;
		TIM4->CCR3 = 0;
		T6_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T5);
		TIM4->CCR2 = pwm_duty;
		T4_mask = 0xFFFF;
	break;
	case (5):
		//Wy³¹cz stare
		GPIO_ResetBits(GPIOD, T3 | T5);
		TIM4->CCR1 = 0;
		T2_mask = 0;
		TIM4->CCR3 = 0;
		T6_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T1);
		TIM4->CCR2 = pwm_duty;
		T4_mask = 0xFFFF;
	break;
	case (6):
		//Wy³acz stare
		GPIO_ResetBits(GPIOD, T3 | T5);
		TIM4->CCR1 = 0;
		T2_mask = 0;
		TIM4->CCR2 = 0;
		T4_mask = 0;

		//Ustaw nowe
		GPIO_SetBits(GPIOD, T1);
		TIM4->CCR3 = pwm_duty;
		T6_mask = 0xFFFF;
	break;
	}
}

void EXTI0_IRQHandler(void) {					//czujnika HALLA A
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){	//sprawdzenie flagi o przerwaniu
		RPM=TIM_GetCounter(TIM3);
		TIM_SetCounter(TIM3,0);

		BLDC_Step();

        EXTI_ClearITPendingBit(EXTI_Line0);		//wyczyszczenie flagi
    }
}

void EXTI1_IRQHandler(void) {					//czujnik HALLA B
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){	//sprawdzenie flagi o przerwaniu
		RPM=TIM_GetCounter(TIM3);
		TIM_SetCounter(TIM3,0);

		BLDC_Step();

        EXTI_ClearITPendingBit(EXTI_Line1);		//wyczyszczenie flagi
    }
}

void EXTI2_IRQHandler(void) {					//czujnik HALLA C
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){	//sprawdzenie flagi o przerwaniu
		RPM=TIM_GetCounter(TIM3);
		TIM_SetCounter(TIM3,0);

		BLDC_Step();

        EXTI_ClearITPendingBit(EXTI_Line2);		//wyczyszczenie flagi
    }
}

