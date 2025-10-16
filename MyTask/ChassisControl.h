#ifndef __CHASSISCONTROL_H__
#define __CHASSISCONTROL_H__


#ifdef __cplusplus
extern "C" {
#endif

	
#include "stm32f4xx_hal.h"

#include "motor.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "CANDrive.h"
#include "SteeringWheel.h"
#include "WatchDog2.h"
#include "RemoteControl.h"
#include "pid_old.h"
#include "usart.h"
#include "arm_math.h"

//正方形边长385 mm
//减速比10：3
//轮子直径96mm

#define  CHASSIS_L		0.385f
#define VEL_TRANSFORM (10.0f/(3.0f*3.14159265f*0.096f)) //将m/s速度转换为电机r/s

void ChassisControl(void* param);

#ifdef __cplusplus
}
#endif

#endif
