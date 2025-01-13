#ifndef MOTOR_H_
#define MOTOR_H_
#include <stm32f10x.h>
#include "GPIO_DRIVER.h"
void motorSignalsMc(int period, int pwm1, int pwm2);
void motorSignals(int period, int pwm1, int pwm2);
void direction(unsigned short map, unsigned short man, unsigned short mbp, unsigned short mbn);
#define P 1 
#define N 0
#endif