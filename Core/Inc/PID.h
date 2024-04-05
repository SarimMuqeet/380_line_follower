#ifndef __PID_H
#define __PID_H_

//0.29 -> 0.14 -> 0.08 -> 0.05 -> 0.0098 -> 0.011 only P control
// 0.02 -> 0.05 (3 turns, miss 1)
double Kp = 0.02; //0.0055 start 0.035
//double Ki = 0.001;
double Kd = 0.3; //worked well with 0.1 -- // 0.0055 start

//double Kr = 0;
float P, I, D;
uint32_t lastError = 0;
uint32_t errors[10] = {0,0,0,0,0,0,0,0,0,0};
//uint32_t base1 = 25000;
//uint32_t base2 = 19136;
double base = 0.29; //0.7; //0.29 0.27
//double base2 = 0.25; //0.7; 19136
double calibrateLeft = 1;
double calibrateRight = 1.025;

//double calibrateLeftTurn = 1.05;
//double calibrateRightTurn = 1;
double baseTurn = 0.2;
double baseTurnLeft = baseTurn*calibrateLeft;
double baseTurnRight = baseTurn*calibrateLeft;

double baseLeft = base*calibrateLeft;
double baseRight = base*calibrateLeft;

double motorSpeed;

float prevError = 0;

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
