/*Timer4 was used to control pwm signals*/
/*
PB6 - CH1 
PB7	- CH2 
PB8	- CH3
PB9	- CH4
*/
/*
PB6 - PB7 are used to control both motors
*/
#include "MOTOR_DRIVER.h"
void motorSignalsMc(int period, int pwm1, int pwm2){
	/*CLKEN FOR AFIO-PORT A-PORT B*/
	RCC->APB2ENR |= (1U << 0) | (1U << 3);
 	RCC->APB1ENR |= (1U << 2);
	/*INIT GPIO PIN AS A_PP*/
	InitGPIO(PB, 6, OUT50, AF_PP);
	InitGPIO(PB, 7, OUT50, AF_PP);
	/*Timer4 Registers*/
	TIM4->PSC = (72 - 1);
	TIM4->CNT = 0;
	TIM4->CCMR1 |= (0x60); // CH1 PWM MODE
	TIM4->CCMR1 |= (0x6000); // CH2 PWM MODE
	TIM4->CCER |= (1U << 0) | (1U << 4); // CH1 and CH2 ENABLE 
	TIM4->BDTR |= (0x8000); 
	TIM4->CCR1 = pwm1;
	TIM4->CCR2 = pwm2;
	TIM4->ARR = period;
	TIM4->CR1 |= (1U << 0);
}

void motorSignals(int period, int pwm1, int pwm2){
	/*CLKEN FOR AFIO-PORT A-PORT B*/
	RCC->APB2ENR |= (1U << 0) | (1U << 3);
	RCC->APB1ENR |= (1U << 2);
	/*INIT GPIO PIN AS A_PP*/
	InitGPIO(PB, 6, OUT50, AF_PP);
	InitGPIO(PB, 7, OUT50, AF_PP);
	/*Timer4 Registers*/
	TIM4->PSC = (7200 - 1);
	TIM4->CNT = 0;
	TIM4->CCMR1 |= (0x60); // CH1 PWM MODE
	TIM4->CCMR1 |= (0x6000); // CH2 PWM MODE
	TIM4->CCER |= (1U << 0) | (1U << 4); // CH1 and CH2 ENABLE 
	TIM4->BDTR |= (0x8000); 
	TIM4->CCR1 = pwm1*10;
	TIM4->CCR2 = pwm2*10;
	TIM4->ARR = period*10;
	TIM4->CR1 |= (1U << 0);
}
/*FOR THE POSITION FORWARD BACKWARD PC6 to PC9*/
/*PC6 and PC7 for M1*/ //LEFT
/*PC8 and PC9 for M2*/ //RIGHT
void direction(unsigned short map, unsigned short man, unsigned short mbp, unsigned short mbn){
	InitGPIO(PC, 6, OUT50, G_PP);
	InitGPIO(PC, 7, OUT50, G_PP);
	InitGPIO(PC, 8, OUT50, G_PP);
	InitGPIO(PC, 9, OUT50, G_PP);
	if((map == P) & (man == N) & (mbp== P) & (mbn == N)){
		WRITE_GPIO(PC, 6, HIGH);
		WRITE_GPIO(PC, 7, LOW);
		
		WRITE_GPIO(PC, 8, HIGH);
		WRITE_GPIO(PC, 9, LOW);
	}else if((map == N) & (man == P) & (mbp == N) & (mbn == P)){
		WRITE_GPIO(PC, 6, LOW);
		WRITE_GPIO(PC, 7, HIGH);
		
		WRITE_GPIO(PC, 8, LOW);
		WRITE_GPIO(PC, 9, HIGH);
	}else if((map == N) & (man == P) & (mbp == P) & (mbn == N)){
		WRITE_GPIO(PC, 6, LOW);
		WRITE_GPIO(PC, 7, HIGH);
		
		WRITE_GPIO(PC, 8, HIGH);
		WRITE_GPIO(PC, 9, LOW);
	}else if((map == P) & (man == N) & (mbp == N) & (mbn == P)){
		WRITE_GPIO(PC, 6, LOW);
		WRITE_GPIO(PC, 7, HIGH);
		
		WRITE_GPIO(PC, 8, HIGH);
		WRITE_GPIO(PC, 9, LOW);
	}
}
