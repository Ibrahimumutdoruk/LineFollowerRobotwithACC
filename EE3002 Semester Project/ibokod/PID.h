#ifndef PID_H_
#define PID_H_
#include "MOTOR_DRIVER.h"
#include <stdint.h>
#include <math.h>
void PIDControl(int error);
int positionFind(int  sensorData[5]);
#endif