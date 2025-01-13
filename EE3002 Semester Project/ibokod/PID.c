#include "PID.h"

/*1140*/
int  diffrence, sum, lastError, PID, error;
float biasSpeed =250;
float Kp =5;
float Ki = 1/50;
float Kd = 120;
int leftDuty, rightDuty;
double Ts= 300;

int positionFind(int  sensorData[5]){
	int dataSum = 0;
	int locationSum = 0; 

	for(int i = 0; i <= 4; i++){
		dataSum += sensorData[i];
		locationSum += (sensorData[i] * i * 1.2);
		}
	int where = (locationSum / dataSum) * 20;
	int error = 40 - where;
	return error;
}

void PIDControl(int error){

    sum = sum + error;
    diffrence= (error - lastError);
    

		PID= Kp*error+Ki*sum*Ts*pow(10,-6)+Kd*diffrence/(Ts*pow(10,-6));
    lastError = error;

	leftDuty = biasSpeed - PID;
	rightDuty = biasSpeed + PID;

		
			direction(P, N, P, N);
		  motorSignalsMc(1300, leftDuty, rightDuty);
}		
		

