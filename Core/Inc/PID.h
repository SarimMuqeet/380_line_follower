#ifndef __PID_H
#define __PID_H_

double Kp = 0.075;
double Ki = 0;
double Kd = 0.22;
float P, I, D;
uint32_t lastError = 0;
double base = 0.265; //0.7; //0.29 0.27
double calibrateLeft = 1;
double calibrateRight = 1.025;
double baseTurn = 0.235;
double baseTurnLeft = baseTurn*calibrateLeft;
double baseTurnRight = baseTurn*calibrateLeft;

double baseLeft = base*calibrateLeft;
double baseRight = base*calibrateLeft;

double motorSpeed;

float prevError = 0;

uint32_t maxspeedL = 0.35; //65536
uint32_t maxspeedR = 0.35;

void pd_control(int16_t dist1, int16_t dist3);
void pd_control_bw(int16_t dist1, int16_t dist3);


#endif
