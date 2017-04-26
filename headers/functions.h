#include "bldc_headers.h"

/* tranzystory gornej galezi (bez PWM) sa na porcie GPIOD Piny 5,6,7, odpowiednio tranzystor T1 T3 T5
 * tranzystory dolnej galezi (kluczowane PWM), port GPIOD Piny 12,13,14 odpowiednio tranzystor T2 T4 T6
 * czujniki halla rowniez na porcie GPIOD Piny 0,1,2
 * pomiar pradu na porcie GPIOA Pin 0
 * komunikacja RS jeszcze nieustalone
 */


#define TIMER_PERIOD 8399 // TIMER_PERIOD=timer_tick_freq/PWM_freq-1 //std 10kHz
#define GPIOD_PIN_READ(x) (GPIOD->IDR & (0x0001<<x))

#define K0i 3.2464 			//wspó³czynnik cz³onu proporcojonalnego reg. pradu
#define Tii 0.001077687 	//czas zdwojenia
#define Td 0.0005			//czas dzia³ania algorytmu
#define K1i (K0i*Td/Tii) 		//wspó³czynnik cz³onu calkujacego reg. pradu

#define maxCurrent 25	//maksymalny prad w silniku
#define maxVolt 24//maksymalne napeicie
#define PARY_BIEGUN 8	//ilosc par biegunow w silniku

#define hallA GPIO_Pin_0
#define hallB GPIO_Pin_1
#define hallC GPIO_Pin_2

#define T1 GPIO_Pin_5
#define T3 GPIO_Pin_6
#define T5 GPIO_Pin_7
#define T2 1
#define T4 2
#define T6 3
#define ON 1
#define OFF 0

void PWM_Init(void);
void PWM_OUT_Set_Value(uint8_t, uint32_t);
void PWM_OUT_Clear(uint8_t);
void EXTI_Pins_Init(void);
void GENERAL_Pins_Init(void);
void BLDC_komutacja(uint8_t);
void BLDC_ruch(uint8_t kierunek, uint8_t poprzedni_kierunek);
void ADC_Initialization(void);
uint16_t ADC1_GetConvValue(void);
void Counting_TIMER_Init(void);
double PIi_reg(double setCurrent, double measuredCurrent);
double CurrentMeasureAutoCalib(void);
void TIM2_OCM_IT_Init(void);
void USART_Initialization(void);
void DMA_Initialization(void);
void BLDC_Com(uint8_t);
void BLDC_Step(void);

