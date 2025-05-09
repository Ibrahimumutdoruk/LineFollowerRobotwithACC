#ifndef ADC_H_
#define ADC_H_
#include "GPIO_DRIVER.h" 
#include "SYSTICK_DRIVER.h"
#include "math.h"
#define adc1          1 
#define adc2          2

enum channels
{
	PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5,temp_sensor
};


char adcInit(char adc, short port, short pin);
char adcFlag(char adc);
int adc_rx(char adc, short port, short pin);
void adc_irq(char adc, char port, char pin);
void adc_wd(char adc, char port, char pin, short htr, short ltr);
void adcMultiInit(char adc, char channels, char * adc_channels);
void adcMultiRead(char adc, char channels, char * adc_channels, int * analog_rx);
#endif