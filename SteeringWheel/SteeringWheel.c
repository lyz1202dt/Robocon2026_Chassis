#include "SteeringWheel.h"

void CalcWheelSpeed1(SteeringWheel* pSteWhe,float vel, float direction, float omega)
{
	float Vel_X, Vel_Y;
	float SumVelX, SumVelY;
	float Dir_N, Vel_N;
	
	Vel_X = vel * sin(ANGLE2RAD(direction));
	Vel_Y = vel * cos(ANGLE2RAD(direction));
	
	Vel_N = ANGLE2RAD(omega) * pSteWhe->postureRadius;
	Dir_N = pSteWhe->postureAngle;
	LimitAngle(&Dir_N);
	SumVelX = Vel_X + Vel_N * sinf(ANGLE2RAD(Dir_N));
	SumVelY = Vel_Y + Vel_N * cosf(ANGLE2RAD(Dir_N));
	
	pSteWhe->expextVelocity = sqrtf(SumVelX * SumVelX + SumVelY * SumVelY);
	
	if(pSteWhe->expextVelocity > 0)
		pSteWhe->expectDirection = RAD2ANGLE(atan2f(SumVelX, SumVelY));
	else 
		pSteWhe->expectDirection = pSteWhe->last_expDirection;
	pSteWhe->last_expDirection=pSteWhe->expectDirection;
}

void LimitAngle(float* angle)
{
	while (*angle<-180.0f||*angle>180.0f)
	{
		if (*angle > 180.0f)
		{
			*angle -= 360.0f;
		}
		else if (*angle < -180.0f)
		{
			*angle += 360.0f;
		}
	}
}


uint8_t SteeringWheelReady(SteeringWheel *StrWhe)
{
    return StrWhe->ready_edge_flag>>7;
}

float AngleDiffer(float angle1,float angle2)    //求angle2到angle1旋转所需的角度
{
	float temp=angle1-angle2;
	if(temp>180.0f)
		return temp-360.0f;
	else if(temp<-180.0f)
		return temp+360.0f;
	return temp;
}

void MinorArcDeal2(SteeringWheel *motor)
{
	float currentAngle = (float)(motor->SteeringMotor.Angle - motor->addoffsetangle - motor->offset) * 10 / 4.0f / 8192.0f;	//2006减速比为36/1，这里的10其实是(360/36)
	float actualTargetAngle = (float)(180 * ((int8_t)(currentAngle / 180.0f))) + motor->expectDirection;
	float D_angle = AngleDiffer(actualTargetAngle, currentAngle); // 在同一周期内求旋转角

	motor->currentDirection= currentAngle; // 更新当前角度
	if (currentAngle < motor->floatRotateAngle && currentAngle > -motor->floatRotateAngle && (fabsf(D_angle) < 2.0f || fabsf((fabsf(D_angle) - 180.0f)) < 2.0f)) // 远离机械限位，并且与目标值误差足够小，清零edge保证正常情况下最优的结果
		motor->ready_edge_flag = motor->ready_edge_flag & 0xF0;

	if (D_angle < 90.0f && D_angle > -90.0f) // 不需要执行优化，直接对准方向即可
	{
		motor->putoutDirection = currentAngle + D_angle; // 设置输出角度为当前角度加增量

		if (motor->putoutDirection > motor->maxRotateAngle || (currentAngle > motor->floatRotateAngle && currentAngle < motor->maxRotateAngle && motor->ready_edge_flag & 0x01)) // 如果(当前目标值超出最大机械限制范围)或着(当前位于柔性区间且上一次在该区间发生了震荡)
		{
			motor->putoutDirection = motor->putoutDirection - 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x01;	  // 发生反转，第一位置1
		}
		else if (motor->putoutDirection < -motor->maxRotateAngle || (currentAngle < -motor->floatRotateAngle && currentAngle > -motor->maxRotateAngle && motor->ready_edge_flag & 0x02))
		{
			motor->putoutDirection = motor->putoutDirection + 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x02;	  // 发生反转，第二位置1
		}
	}
	else // 期望角度和当前角度之间的角为钝角，可以通过反转电机实现优化
	{
		if (D_angle > 0.0f)
		{
			motor->putoutDirection = currentAngle + D_angle - 180.0f;
		}
		else
		{
			motor->putoutDirection = currentAngle + D_angle + 180.0f;
		}
		if (motor->putoutDirection > motor->maxRotateAngle || (currentAngle > motor->floatRotateAngle && currentAngle < motor->maxRotateAngle && motor->ready_edge_flag & 0x04)) // 如果(当前目标值超出最大机械限制范围)或着(当前位于柔性区间且上一次在该区间发生了震荡)
		{
			motor->putoutDirection = motor->putoutDirection - 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x04;	  // 发生反转，第一位置1
		}
		else if (motor->putoutDirection < -motor->maxRotateAngle || (currentAngle < -motor->floatRotateAngle && currentAngle > -motor->maxRotateAngle && motor->ready_edge_flag & 0x08))
		{
			motor->putoutDirection = motor->putoutDirection + 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x08;	  // 发生反转，第二位置1
		}
	}
	float temp = motor->putoutDirection - motor->expectDirection;
	LimitAngle(&temp);
	/*if (fabsf(temp) > 90.0f)
		motor->putoutVelocity = -motor->expextVelocity;
	else
		motor->putoutVelocity = motor->expextVelocity;*/
	motor->putoutVelocity=motor->expextVelocity*cos(ANGLE2RAD(temp));
}

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
		if(!(pSteWhe->ready_edge_flag&0x20))		//第一次执行时，记录角度，如果尚未进行复位，则标记进行一次复位
		{
			pSteWhe->ready_edge_flag=pSteWhe->ready_edge_flag|0x20;
			pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4);
		}
		if (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin))
		{
			pSteWhe->putoutDirection = pSteWhe->putoutDirection - 0.03f;
		}
		else
		{
			pSteWhe->putoutDirection = pSteWhe->putoutDirection + 0.03f;
		}

		if ((HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4) != (pSteWhe->ready_edge_flag & 0x10))	//如果上一次的IO电平与此次不同，那么认为复位成功
		{
			pSteWhe->offset = pSteWhe->SteeringMotor.Angle;
			pSteWhe->putoutDirection = 0.0f;
			pSteWhe->addoffsetangle=0.0f;
			pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | 0x80;
		}
		pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag & (~0x10);
		pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4);
	}
	PID_Control2(pSteWhe->currentDirection, pSteWhe->putoutDirection, &pSteWhe->Steering_Dir_PID); 												//角度环
  	PID_Control2(pSteWhe->SteeringMotor.Speed, pSteWhe->Steering_Dir_PID.pid_out, &pSteWhe->Steering_Vel_PID);                         //速度环
}
