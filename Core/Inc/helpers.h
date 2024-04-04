#ifndef __HELPERS_H
#define __HELPERS_H_

#define COUNTER_PERIOD 65535
#define RIGHT_FW 0.73*0.5 //try 0.67 //really the right wheel
#define LEFT_FW 0.67*0.5   //try 0.67

#define LEFT_BW 0.73*0.5 //try 0.67
#define RIGHT_BW 0.73*0.5   //try 0.67


uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;
uint16_t r3, g3, b3, c3;


const uint16_t REDLINE_LEFT[3] = {153, 51, 102};
const uint16_t REDLINE_RIGHT[3] = {153, 51, 51};
const uint16_t REDLINE_MIDDLE[3] = {0, 0, 0};
const uint16_t GREENZONE[3] = {43, 88, 125};
const uint16_t BULLSEYE_BLUE[3] = {19, 58, 156};
const uint16_t WOOD[3] = {69, 80, 92};


void getRawData_noDelay(Adafruit_TCS34725 *tcs, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

int16_t euclideanDistance(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *RGB1[3], uint16_t *RGB2[3]);
// Movement Functions
void moveForward(uint32_t *dutyCycleL, uint32_t *dutyCycleR);

void moveBackward(uint32_t *dutyCycleL, uint32_t *dutyCycleR);

void moveLeft(uint32_t *dutyCycle);

void moveRight(uint32_t *dutyCycle);

void stop();

// Claw Functions
void grab();

void release();

void print(char *str);

#endif



