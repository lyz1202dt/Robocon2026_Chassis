#include "drive_callback.h"
#include "semphr.h"

extern SemaphoreHandle_t reset_semaphore;

void SetWheelTarget_Callback(Wheel_t *_this, float rad, float velocity, float force)
{
    SteeringWheel *steeringwheel = (SteeringWheel *)_this->user_data;
    steeringwheel->expectDirection = RAD2ANGLE(rad);
    steeringwheel->expextVelocity = velocity;
		steeringwheel->expextForce  = force * n * wheel_radius * (2.0f * PI * KV / 60.0f) * cosf(ANGLE2RAD(steeringwheel->putoutDirection - steeringwheel->currentDirection));
}

void WheelReset_Callback(Wheel_t *_this)
{
	SteeringWheel *steeringwheel = (SteeringWheel *)_this->user_data;
	Reset_Function(steeringwheel);
}

WheelState WheelState_Callback(Wheel_t *_this)
{
    SteeringWheel *pSteWhe = (SteeringWheel *)_this->user_data;

		if(pSteWhe->ready_edge_flag >> 7)
		{
			MinorArcDeal(pSteWhe);
			return WHEEL_HEALTH;
		}else {
			return WHEEL_IDEL;
		}
}

Vector2D GetWheelVelocity_Callback(Wheel_t *_this)
{
    SteeringWheel *pSteWhe = (SteeringWheel *)_this->user_data;

    Vector2D velocity;
    velocity.x = pSteWhe->DriveMotor.posVelEstimateGet.velocity * VEL_TRANSFORM * 2.0f * 3.1415926f * n * wheel_radius * cosf(ANGLE2RAD(pSteWhe->currentDirection));
    velocity.y = pSteWhe->DriveMotor.posVelEstimateGet.velocity * VEL_TRANSFORM * 2.0f * 3.1415926f * n * wheel_radius * sinf(ANGLE2RAD(pSteWhe->currentDirection));
    // velocity.x = pSteWhe->DriveMotor.rpm * 60.0f * 2.0f * 3.1415926f * n * wheel_radius * cosf(ANGLE2RAD(pSteWhe->currentDirection));
	// velocity.y = pSteWhe->DriveMotor.rpm * 60.0f * 2.0f * 3.1415926f * n * wheel_radius * sinf(ANGLE2RAD(pSteWhe->currentDirection));
	return velocity;
}

void WheelError_Callback(Chassis_t *_this, Wheel_t *wheel)
{
    SteeringWheel *pSteWhe = (SteeringWheel *)wheel->user_data;
    HAL_GPIO_WritePin(pSteWhe->error_GPIO_Port, pSteWhe->error_GPIO_Pin, GPIO_PIN_SET);
}

uint8_t SteeringWheelReady(SteeringWheel *StrWhe)
{
    return StrWhe->ready_edge_flag >> 7;
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

float AngleDiffer(float angle1,float angle2)
{
	float temp=angle1-angle2;
	if(temp>180.0f)
		return temp-360.0f;
	else if(temp<-180.0f)
		return temp+360.0f;
	return temp;
}

// MinorArcDeal是一个智能的舵轮转向优化算法，它：
// 路径最优: 总是选择最短的旋转路径
// 机械保护: 智能处理机械限位，防止硬件损坏
// 防震荡: 记录历史状态，避免在限位附近震荡
// 速度平滑: 根据转向角度智能调整速度，运动更平稳
// 自适应: 适应各种角度场景，从微小调整到大角度旋转
void MinorArcDeal(SteeringWheel *motor)
{
	if (motor->currentDirection < motor->floatRotateAngle && motor->currentDirection > -motor->floatRotateAngle && (fabsf(motor->D_angle) < 2.0f || fabsf((fabsf(motor->D_angle ) - 180.0f)) < 2.0f)) // 远离机械限位，并且与目标值误差足够小，清零edge保证正常情况下最优的结果
		motor->ready_edge_flag = motor->ready_edge_flag & 0xF0;

	if (motor->D_angle  < 90.0f && motor->D_angle  > -90.0f) // 不需要执行优化，直接对准方向即可
	{
		motor->putoutDirection = motor->currentDirection + motor->D_angle ; // 设置输出角度为当前角度加增量

		if (motor->putoutDirection > motor->maxRotateAngle || (motor->currentDirection > motor->floatRotateAngle && motor->currentDirection < motor->maxRotateAngle && motor->ready_edge_flag & 0x01)) // 如果(当前目标值超出最大机械限制范围)或着(当前位于柔性区间且上一次在该区间发生了震荡)
		{
			motor->putoutDirection = motor->putoutDirection - 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x01;	  // 发生反转，第一位置1
		}
		else if (motor->putoutDirection < -motor->maxRotateAngle || (motor->currentDirection < -motor->floatRotateAngle && motor->currentDirection > -motor->maxRotateAngle && motor->ready_edge_flag & 0x02))
		{
			motor->putoutDirection = motor->putoutDirection + 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x02;	  // 发生反转，第二位置1
		}
	}
	else // 期望角度和当前角度之间的角为钝角，可以通过反转电机实现优化
	{
		if (motor->D_angle  > 0.0f)
		{
			motor->putoutDirection = motor->currentDirection + motor->D_angle  - 180.0f;
		}
		else
		{
			motor->putoutDirection = motor->currentDirection + motor->D_angle  + 180.0f;
		}
		if (motor->putoutDirection > motor->maxRotateAngle || (motor->currentDirection > motor->floatRotateAngle && motor->currentDirection < motor->maxRotateAngle && motor->ready_edge_flag & 0x04)) // 如果(当前目标值超出最大机械限制范围)或着(当前位于柔性区间且上一次在该区间发生了震荡)
		{
			motor->putoutDirection = motor->putoutDirection - 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x04;	  // 发生反转，第一位置1
		}
		else if (motor->putoutDirection < -motor->maxRotateAngle || (motor->currentDirection < -motor->floatRotateAngle && motor->currentDirection > -motor->maxRotateAngle && motor->ready_edge_flag & 0x08))
		{
			motor->putoutDirection = motor->putoutDirection + 180.0f; // 将目标值设为当前角的负角，并反转电机
			motor->ready_edge_flag = motor->ready_edge_flag | 0x08;	  // 发生反转，第二位置1
		}
	}
	float temp = motor->putoutDirection - motor->expectDirection;
	LimitAngle(&temp);
	motor->putoutVelocity=motor->expextVelocity*cos(ANGLE2RAD(temp));
}
void UpdateAngle(SteeringWheel *motor)
{
//float currentAngle = (motor->encoder.angle_deg);
	float currentAngle = (float)(motor->SteeringMotor.Angle - motor->offset) * 10 / 4.0f / 8192.0f;	//2006减速比为36/1，这里的10其实是(360/36)
	float actualTargetAngle = (float)(180 * ((int8_t)(currentAngle / 180.0f))) + motor->expectDirection;
	float D_angle = AngleDiffer(actualTargetAngle, currentAngle); // 在同一周期内求旋转角

	motor->D_angle = D_angle;
	motor->currentDirection = currentAngle; // 更新当前角度
}
//复位函数
void Reset_Function(SteeringWheel *pSteWhe)
{
  if(!(pSteWhe->ready_edge_flag&0x20))		//第一次执行时，记录角度，如果尚未进行复位，则标记进行一次复位
	{
		pSteWhe->ready_edge_flag=pSteWhe->ready_edge_flag|0x20;
		pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4);
	}
	if (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin))
	{
		pSteWhe->putoutDirection = pSteWhe->putoutDirection + 0.3f;
	}
	else
	{
		pSteWhe->putoutDirection = pSteWhe->putoutDirection - 0.3f;
	}

	if ((HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4) != (pSteWhe->ready_edge_flag & 0x10))	//如果上一次的IO电平与此次不同，那么认为复位成功
	{
		taskENTER_CRITICAL();
		pSteWhe->offset = pSteWhe->SteeringMotor.Angle;
		pSteWhe->putoutDirection = 0;
		taskEXIT_CRITICAL();
		pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | 0x80;
	}
	pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag & (~0x10);
	pSteWhe->ready_edge_flag = pSteWhe->ready_edge_flag | (HAL_GPIO_ReadPin(pSteWhe->Key_GPIO_Port, pSteWhe->Key_GPIO_Pin) << 4);
}
