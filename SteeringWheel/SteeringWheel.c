#include "SteeringWheel.h"







void CalcWheelSpeed2(SteeringWheel* pSteWhe,float x, float y,float omega)
{
	float sum_x=0.0f,sum_y=0.0f;
	float Vel_N=ANGLE2RAD(omega) * RADIUS;
	
	sum_x = x + Vel_N * sinf(ANGLE2RAD(pSteWhe->postureAngle));
	sum_y = y + Vel_N * cosf(ANGLE2RAD(pSteWhe->postureAngle));
	
	pSteWhe->expextVelocity = sqrtf(sum_x*sum_x+sum_y*sum_y);

	if(sum_y>0.01f)   //防止除0
		pSteWhe->expectDirection=RAD2ANGLE(atanf(sum_x/sum_y));
	else if(sum_y<0.01f)
	{
		if(sum_x>0.0f)
			pSteWhe->expectDirection=180.0f-RAD2ANGLE(atanf(-sum_x/sum_y));
		else
			pSteWhe->expectDirection=-180.0f+RAD2ANGLE(atanf(sum_x/sum_y));
	}
	else
		pSteWhe->expectDirection=90.0f;
}

void SteeringWheel_Deal(SteeringWheel *pSteWhe, float vel, float direction, float omega)
{
	if (pSteWhe->ready_edge_flag >> 7)
	{
		CalcWheelSpeed1(pSteWhe, vel, direction, omega);
		MinorArcDeal2(pSteWhe);
	}
	else
	{
		
	}
	PID_Control2(pSteWhe->currentDirection, pSteWhe->putoutDirection, &pSteWhe->Steering_Dir_PID); 												//角度环
  	PID_Control2(pSteWhe->SteeringMotor.Speed, pSteWhe->Steering_Dir_PID.pid_out, &pSteWhe->Steering_Vel_PID);                         //速度环
}
