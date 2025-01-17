#include "ADC_DRIVER.h"
#include "MOTOR_DRIVER.h"
#include "TIMER1_DRIVER.h"
#include "PID.h"
#include "OLED DRIVER.h"

int distance;

char adcChannel[17] = {PA0, PA1, PA4, PB0, PC1};

int analogData[5];

char numofchannel = 5 ;
char myint[3];
char display[3];

int e = 0;


int main(void){
	systickTimer_init();
	adcMultiInit(adc1, numofchannel, adcChannel);

	InitGPIO(PA, 8, OUT50, AF_PP); /*4 Lines are for reading distance STAYS*/
	InitGPIO(PA, 9, IN, I_PP);
	timer1PWM_micros(61000, 10);
	timer1Capture_micro(61000);
//  AFIO->MAPR |= (1U << 1);
//	gpio_init(PB, 8, OUT50, AF_OD);
//	gpio_init(PB, 9, OUT50, AF_OD);

//	  lcd_i2c_init(1);
//		SysTick_DelayMs(100);
//	  lcd_i2c_msg(1, 1, 0, "Distance:			cm");
//		lcd_i2c_msg(1, 2, 2, "38 TEKNOF 38");

	while(1){
			distance = (timerCatch() - 2205) / 58; /*STAYS*/
			adcMultiRead(adc1, numofchannel, adcChannel, analogData);
			e = positionFind(analogData);
			if(distance > 20){
				PIDControl(e);
			}else if(distance <= 20){
				direction(P,N, P, N);
				motorSignalsMc(700, 0, 0);
			}
//		int2char(myDistance, myint);
//		
//		if(myDistance < 10){
//			display[0] = ' ';
//			display[1] = ' ';
//			display[2] = myint[0];
//			lcd_i2c_msg(1, 1, 9, display);
//		}else if (myDistance < 100){
//			display[0] = ' ';
//			display[1] = myint[0];
//			display[2] = myint[1];
//			lcd_i2c_msg(1, 1, 9, display);
//		}else{
//			display[0] = myint[0];
//			display[1] = myint[1];
//			display[2] = myint[2];
//			lcd_i2c_msg(1, 1, 9, display);
//		}
//			str_empty(myint);
  }
}