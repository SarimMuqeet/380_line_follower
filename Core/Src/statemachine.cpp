///* Includes ------------------------------------------------------------------*/
////#include "main.h"
//#include "statemachine.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "Adafruit_TCS34725.h"
//#include "helpers.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <iostream>
//#include <cstring>
//#include <cmath>
//
//
//using namespace std;
//
//extern Adafruit_TCS34725 tcsFL;
//extern Adafruit_TCS34725 tcsFC;
//extern Adafruit_TCS34725 tcsFR;
//
//
//state_c currState = IDLE;
//uint32_t dutyCycle = (uint32_t)(COUNTER_PERIOD*DEFAULT_MOTOR);
//
//void runStateMachine(){
//	switch(currState) {
//
//	case IDLE:
//		idle();
//		break;
//
//	case SEARCH_LEGO:
//		search_lego();
//		break;
//
//	case SECURE_LEGO:
//		stop();
//		grab();
//		currState = SEARCH_SAFE;
//		break;
//
//	case SEARCH_SAFE:
//		search_safe();
//		break;
//
//	case DROPOFF_LEGO:
//		stop();
//		release();
//		currState = RETURN_TO_START;
//		break;
//
//	case RETURN_TO_START:
//		line_follow_bw();
//		break;
//
//	}
//
//
//}
//
//void idle() {
//	int state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	if(state == 1) {
//		currState = SEARCH_LEGO;
//	}
//}
//
//void search_lego() {
//	line_follow_fw();
//
//	//detect blue, check bullseye
//	if(b3 > 150) {
//		currState = SECURE_LEGO;
//	}
//}
//
//void search_safe() {
//	line_follow_bw();
//
//	//detect green
//	if(g1 > 100 || g2 > 100){
//		currState = DROPOFF_LEGO;
//	}
//}
//
//void line_follow_fw() {
//	moveForward(&dutyCycle);
//
//	getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
//	getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
//	getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);
//
//	//TODO: PID, euclidean distance
//	if(r1 > 120){
//		//only until FC sees red again
//		//vary DC based on PID
//		moveRight(&dutyCycle);
//	} else if(r2 > 120) {
//		moveLeft(&dutyCycle);
//	}
//}
//
//void line_follow_bw() {
//	moveBackward(&dutyCycle);
//
//	getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
//	getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
//	getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);
//
//	//TODO: PID, euclidean distance
//	if(r1 > 120){
//		//opposite of fw
//		moveLeft(&dutyCycle);
//	} else if(r2 > 120) {
//		moveRight(&dutyCycle);
//	}
//}
