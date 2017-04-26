//---INCLUDE FILES---//
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "defines.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_hd44780.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_dma.h"

/// x*2,296 = 25,23
/// y = ax + b
///p1 (0,001 , 43) - bedzie ofset
///p2 (25,28 , 2818) VBAT

//---DEFINES MAKROS---//

#define ADC_CHANNELS 3 //0 - prad, 1 - napieciebaterii, 2 - manetka

#define dzielnikR 0.67 //(10/14.7) //dzielnik napiecia za prztwornikiem R1=4.7k R2=10k 	(0,68027)
#define currentSens (30.0/2) //wspó³czynnik pradu na napiecie [mA/mV]			(15mA/mV)
#define zeroCurrentVol 2.5 //napiecie dla jakiego prad jest zero na charaktersytyce czujnika [mV]

#define scaledCurrentSens currentSens/dzielnikR //przeskalowany wspo³czynnik na dzielniku	(22,0501 mA/mV)
#define scaledZeroCurrentVol zeroCurrentVol*dzielnikR // przeskalowane zero current voltage o dzielnik (1700,675mV)

#define ADCresolution 4096 //rozdzielczosc przetwornika ADC
#define ADCrefVol 3000 //napiecie referenycjne przetowrnika ADC
#define ADCfactor 1.0*ADCrefVol/(ADCresolution-1) //wspolczynnik przeskalowania ADC


uint8_t direction;
uint8_t prev_direction;
uint16_t pwm_duty; //(0..TIMER_PERIOD)
volatile uint16_t T2_mask;
volatile uint16_t T4_mask;
volatile uint16_t T6_mask;



volatile uint16_t RPM;

volatile uint8_t usartData;
volatile uint8_t byteCounter;	//numeracja kolejnych odebranych bajtow

volatile uint16_t adcValue[ADC_CHANNELS];
volatile uint32_t adcValMean[ADC_CHANNELS];
volatile double currentAvg;
volatile double currentLCD;
volatile uint16_t counterAvg;

volatile double adcVal;
volatile uint8_t indexx;
volatile double currentMeasured;
volatile double u_value;
volatile uint16_t u_value_int;


volatile double i_uchyb;
volatile double i_uchyb_suma;
volatile double offset_value;


//---PROTOTYPES---//
