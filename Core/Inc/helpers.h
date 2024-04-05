#ifndef __HELPERS_H
#define __HELPERS_H_

#define COUNTER_PERIOD 65535
#define RIGHT_FW 0.73*0.4 //try 0.67
#define LEFT_FW 0.73*0.4   //prev 0.67

#define LEFT_BW 0.73*0.4 //try 0.67
#define RIGHT_BW 0.73*0.4   //try 0.67


uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;
uint16_t r3, g3, b3, c3;


const uint16_t REDLINE_LEFT[3] = {153, 51, 102};
const uint16_t REDLINE_RIGHT[3] = {153, 51, 51};
const uint16_t REDLINE_MIDDLE[3] = {0, 0, 0};
const uint16_t GREENZONE[3] = {43, 88, 125};
const uint16_t GREENZONE_LEFT[3] = {36, 91, 109};
const uint16_t GREENZONE_RIGHT[3] = {39, 98, 98};
const uint16_t BULLSEYE_BLUE[3] = {19, 58, 156};
const uint16_t WOOD[3] = {69, 80, 92};
const uint16_t WOOD_LEFT[3] = {77, 88, 89};
const uint16_t WOOD_RIGHT[3] = {66, 77, 89};


void getRawData_noDelay(Adafruit_TCS34725 *tcs, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

int16_t euclideanDistance(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *RGB1[3], uint16_t *RGB2[3]);
// Movement Functions
//void moveForward(uint32_t *dutyCycleL, uint32_t *dutyCycleR);
void moveForward(uint32_t *dutyCycleL, uint32_t *dutyCycleR);

void moveBackward(uint32_t *dutyCycleL, uint32_t *dutyCycleR);

void moveLeft(uint32_t *dutyCycle);

void moveRight(uint32_t *dutyCycle);

void moveRightBw(uint32_t *dutyCycle);

void moveLeftBw(uint32_t *dutyCycle);

void turn_180();

void stop();

// Claw Functions
void grab();

void release();

#endif



