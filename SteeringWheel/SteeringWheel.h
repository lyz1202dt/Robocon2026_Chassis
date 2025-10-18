#ifndef __STEERINGWHEEL_H__
#define __STEERINGWHEEL_H__

#include <math.h>




#define RADIUS 130.0f


void SteeringWheel_Deal(SteeringWheel *pSteWhe, float vel, float direction, float omega);

void LimitAngle(float *angle);
float AngleDiffer(float angle1, float angle2);
void CalcWheelSpeed1(SteeringWheel *pSteWhe, float vel, float direction, float omega);
void CalcWheelSpeed2(SteeringWheel *pSteWhe, float x, float y, float omega);
void MinorArcDeal2(SteeringWheel *motor);
uint8_t SteeringWheelReady(SteeringWheel *StrWhe);

#endif
