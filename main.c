#include "bldc_headers.h"
#include "functions.h"
int main(void){

	//VARIABLES
	unsigned char buff[6];

	double pre_u_sat = 0;
	double post_u_sat = 0;
	double anti_windup = 0;
	double wartosc_zadana = 0;

	adcVal = 0;
	RPM = 0;

	direction=1;
	prev_direction=direction;
	byteCounter=0;
	pwm_duty=0;
	currentAvg = 0;
	counterAvg = 0;
	uint8_t k=1;

	usartData = 0;


	///REGULATOR
	i_uchyb_suma = 0;


	//GENERAL//
	GENERAL_Pins_Init();
	GPIO_ResetBits(GPIOD, T1|T3|T5);

	//EXTI HALLS//
	EXTI_Pins_Init();

	//PWM//
	PWM_Init();

	//TM_HD44780//
	SystemInit();
	TM_HD44780_Init(16, 2);
	TM_HD44780_Clear();

	//DMA//
	DMA_Initialization();

	//ADC//
	ADC_Initialization();

	//TIM3 counter//
	Counting_TIMER_Init();

	//USART
	USART_Initialization();

	//Wyswietlanie
    TIM2_OCM_IT_Init();

    ///ZEROWANIE PRZETWORNIKA PRADU - OFFSET
	uint8_t hA = 0;
	uint8_t hB = 0;
	uint8_t hC = 0;
	uint8_t halls = ((hC<<2)|(hB<<1)|(hA)); //C B A
	GPIO_ResetBits(GPIOD, T1|T3|T5);
	Delayms(500);
			adcValMean[0]=0;
	    	adcValMean[1]=0;
	    	adcValMean[2]=0;
	    	for (indexx = 0; indexx<8; indexx++){
	    		ADC_SoftwareStartConv(ADC1);
	    		//Delayms(1);
	    		adcValMean[0]+=adcValue[0];
	    		adcValMean[1]+=adcValue[1];
	    		adcValMean[2]+=adcValue[2];
	    	}
	    	adcValMean[0]/=8;
	    	adcValMean[1]/=8;
	    	adcValMean[2]/=8;

	    	offset_value = (double)adcValMean[0];

    while(1)
    {

    	///KONWERSJA PRZETWRONIKOW ADC
    	adcValMean[0]=0;	// PRAD
    	adcValMean[1]=0;	// V BAT
    	adcValMean[2]=0;	// MANETKA
    	for (indexx = 0; indexx<8; indexx++){
    		ADC_SoftwareStartConv(ADC1);
    		adcValMean[0]+=adcValue[0];
    		adcValMean[1]+=adcValue[1];
    		adcValMean[2]+=adcValue[2];
    	}
    	adcValMean[0]/=8;
    	adcValMean[1]/=8;
    	adcValMean[2]/=8;

    	currentAvg += adcValMean[0];
    	counterAvg++;
    	if (counterAvg == 250){

    		currentLCD = currentAvg/counterAvg;
    		currentAvg = 0;
    		counterAvg=0;
    	}

    	///MARTWA STREFA MANETKI
    	if ((adcValMean[2])>750) {


    		///ALGORYTM PI
    		wartosc_zadana = ((double)adcValMean[2]-660)/2;
    		//wartosc_zadana = 500;

    		i_uchyb = wartosc_zadana - ((double)adcValMean[0] - offset_value);
    		i_uchyb_suma += i_uchyb - anti_windup;
    		u_value = (K0i*i_uchyb + K1i*i_uchyb_suma);

    		pre_u_sat = u_value;	//antiwindup parameter 1

    		///SATURATION
    		if (u_value>8000) u_value = 8000;
    		if(u_value<0) u_value = 0;

    		///ANTIWINDUP
    		post_u_sat = u_value;	//antiwindup parameter 2
    		anti_windup = (pre_u_sat - post_u_sat)*K1i; //antiwindup value

    		u_value_int = u_value;
    		pwm_duty = u_value_int;


    		//pwm_duty = adcValMean[2]*2;	//only voltage regulation

    		///ROZRUCH DLA MALYCH PREDKOSCI
    		if (RPM<10) {
    			BLDC_Step();
    		}
    	}
    	else{
    		pwm_duty = 0;
    	}

    	///WPISANIE WARTOSCI DO REJESTRU PWM
    	TIM4->CCR1 = pwm_duty&T2_mask;
    	TIM4->CCR2 = pwm_duty&T4_mask;
    	TIM4->CCR3 = pwm_duty&T6_mask;
    	Delayms(1);

    }
}
