#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H_


typedef enum{
	IDLE = 0,
	SEARCH_LEGO = 1,
	SECURE_LEGO = 2,
	SEARCH_SAFE = 3,
	DROPOFF_LEGO = 4,
	RETURN_TO_START = 5,
} state_c;


void runStateMachine();
void idle();
void search_lego();
void search_safe();
void line_follow_fw();
void line_follow_bw();


#endif

