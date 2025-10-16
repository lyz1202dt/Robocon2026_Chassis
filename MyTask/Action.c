#include "action.h"
#include "tim.h"
#include "RemoteControl.h"
#include "ChassisControl.h"
#include "pid_old.h"

extern QueueHandle_t launch_action_semphr;
extern QueueHandle_t move_action_semphr;
extern QueueHandle_t jump_action_semphr;

extern float dir1,dir2,dir3,dir4,vel;
extern uint8_t forced_set_dir;

extern M2006_TypeDef lift_motor1;
extern M2006_TypeDef lift_motor2;
extern PID2 lift_motor1_vel_pid;
extern PID2 lift_motor2_vel_pid;
extern float lift_motor1_target_vel;
extern float lift_motor2_target_vel;

float balance_Kp=0.03f;

int16_t lift_motor_move_pos=400;
int16_t lift_motor_launch_pos=200;
int16_t lift_motor_jump_pos=0; 

//主桅杆锁定舵机
float left_lock=240.0f;
float left_unlock=80.0f;
float right_lock=270.0f;
float right_unlock=150.0f;

float lift_motor_vel=20000.0f;

extern uint8_t state;

void SwitchMoveMode(void* param)
{
	while(1)
	{
		xSemaphoreTake(move_action_semphr,portMAX_DELAY);
		forced_set_dir=0;
		state=0;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,SetSteeringEngineRAD270(left_lock*3.14159265f/180.0f));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SetSteeringEngineRAD270(right_lock*3.14159265f/180.0f));
		vTaskDelay(400);
		if(lift_motor1.r>lift_motor_move_pos)
		{
			while(lift_motor1.r>lift_motor_move_pos)
			{
				lift_motor1_target_vel=-lift_motor_vel-balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				lift_motor2_target_vel=-lift_motor_vel+balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				vTaskDelay(5);
			}
		}
		else
		{
			while(lift_motor1.r<lift_motor_move_pos)
			{
				lift_motor1_target_vel=lift_motor_vel-balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				lift_motor2_target_vel=lift_motor_vel+balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				vTaskDelay(5);
			}
		}
		lift_motor1_target_vel=0.0f;
		lift_motor2_target_vel=0.0f;
		lift_motor1_vel_pid.error_inter=0.0f;
		lift_motor2_vel_pid.error_inter=0.0f;
		state=2;
	}
}

void SwitchJumpMode(void* param)
{
	while(1)
	{
		xSemaphoreTake(jump_action_semphr,portMAX_DELAY);
		state=0;
		forced_set_dir=0;
		if(lift_motor1.r>lift_motor_jump_pos)
		{
			while(lift_motor1.r>lift_motor_jump_pos)
			{
				lift_motor1_target_vel=-lift_motor_vel-balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				lift_motor2_target_vel=-lift_motor_vel+balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				vTaskDelay(5);
			}
		}
		else
		{
			while(lift_motor1.r<lift_motor_jump_pos)
			{
				lift_motor1_target_vel=lift_motor_vel-balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				lift_motor2_target_vel=lift_motor_vel+balance_Kp*(lift_motor1.Angle-lift_motor2.Angle);
				vTaskDelay(5);
			}
		}
		lift_motor1_target_vel=0.0f;
		lift_motor2_target_vel=0.0f;
		lift_motor1_vel_pid.error_inter=0.0f;
		lift_motor2_vel_pid.error_inter=0.0f;
		forced_set_dir=0;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,SetSteeringEngineRAD270(left_unlock*3.14159265f/180.0f));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SetSteeringEngineRAD270(right_unlock*3.14159265f/180.0f));
		state=1;
	}
}

void SwitchLaunchMode(void* param)
{
	while(1)
	{
		xSemaphoreTake(launch_action_semphr,portMAX_DELAY);
		forced_set_dir=1;
		vel=0.001f;
		dir1=45;
		dir2=135;
		dir3=-45;
		dir4=-135;	//给一个很小的速度并且给四个轮子不同的方向以锁定轮子

		state=0;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,SetSteeringEngineRAD270(left_lock*3.14159265f/180.0f));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SetSteeringEngineRAD270(right_lock*3.14159265f/180.0f));
		vTaskDelay(300);
		vel=0.0f;
	}
}

uint16_t SetSteeringEngineRAD270(float rad)
{
    const uint16_t base = 500;
    const float k = 1900 / (1.5f*PI);
    if (rad > PI*1.5f)
        rad = PI*1.5f;
    else if (rad < 0.0f)
        rad = 0.0f;
    return base + (uint16_t)(rad * k);
}
