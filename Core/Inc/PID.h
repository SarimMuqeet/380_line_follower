#ifndef __PID_H
#define __PID_H_


double Kp = 0.3; //set up the constants value
double Ki = 0.001;
double Kd = 0;
//double Kr = 0;
float P, I, D;
uint32_t lastError = 0;
uint32_t errors[10] = {0,0,0,0,0,0,0,0,0,0};
//uint32_t base1 = 25000;
//uint32_t base2 = 19136;
double base1 = 0.55; //0.7; 25000
double base2 = 0.55; //0.7; 19136

double motorSpeed;

int prevError = 0;

uint32_t maxspeedL = 1; //65536
uint32_t maxspeedR = 1;

void pd_control(int16_t dist1, int16_t dist3);
void add_to_errors (int error);

//	void add_to_prevEuc (int euc)
//			{
//			  for (int i = 9; i > 0; i--)
//				  prevEuc[i] = prevEuc[i-1];
//			  prevEuc[0] = euc;
//			}

#endif
