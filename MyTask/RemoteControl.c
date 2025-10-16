#include "RemoteControl.h"
#include "ChassisControl.h"
#include "ODrive.h"
#include "tim.h"
#include "semphr.h"
#include "Action.h"
#include "STP-23L.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

extern SteeringWheel steering1, steering2, steering3, steering4;

extern QueueHandle_t launch_action_semphr;
extern QueueHandle_t move_action_semphr;
extern QueueHandle_t jump_action_semphr;

extern float velocity,direction,omega;
extern ChassisState_t chassis_state;
uint8_t chassis_ctrl_recv[sizeof(ChassisCtrl_t)];
char vl53_recv_buf[64];
uint8_t STP_Data[195];
STP_23L_Data STP_t;
float cur_height;
ChassisCtrl_t chassis_ctrl,last_chassis_ctrl;

RadarPack_Typedef Radar_Rxdata,
                  Radar_data_offest;

SemaphoreHandle_t upboard_semaphore;

//上下底盘锁定舵机
float unlock_angle1=198.0f;
float unlock_angle2=228.0f;
float lock_angle1=180.0f;
float lock_angle2=210.0f;

extern float left_lock;
extern float left_unlock;
extern float right_lock;
extern float right_unlock;

float debug1=175.0f;
float debug2=210.0f;

void RadarDog_Cb(void* param);

uint8_t state=1;
extern uint8_t forced_set_dir;

uint32_t radar_watchdog;
uint8_t wheel_reset=1;

void UartRecvTask(void* parma)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chassis_ctrl_recv,sizeof(chassis_ctrl_recv));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)&Radar_Rxdata,sizeof(RadarPack_Typedef));
	HAL_UART_Receive_DMA(&huart6, STP_Data, sizeof(STP_Data));
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  
	upboard_semaphore=xSemaphoreCreateBinary();
	radar_watchdog=AddWatchDog(RadarDog_Cb,100,NULL,WATCHDOG_MODE_REPEAT);
	chassis_ctrl.cmd=CMD_MODE_LOCK_CHASSIS;
	while(1)
	{
		if(xSemaphoreTake(upboard_semaphore,pdMS_TO_TICKS(500))!=pdTRUE)
		{
			//velocity=0.0f;	//上位机掉线，底盘速度归0
			//omega=0.0f;
		}
        else{
            velocity=sqrt(chassis_ctrl.v_x*chassis_ctrl.v_x+chassis_ctrl.v_y*chassis_ctrl.v_y);		//运动学部分
            if(velocity>0.00001f)
                direction=RAD2ANGLE(atan2f(-chassis_ctrl.v_y,-chassis_ctrl.v_x));
            omega=chassis_ctrl.omega;
        }
		
		//命令执行
		if(chassis_ctrl.cmd==CMD_MODE_MOVE&&state==1)
		{
			xSemaphoreGive(move_action_semphr);
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==CMD_MODE_JUMP&&state==2)
		{
			xSemaphoreGive(jump_action_semphr);
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==CMD_MODE_LAUNCH&&forced_set_dir==0)
		{
			xSemaphoreGive(launch_action_semphr);
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==CMD_MODE_LOCK_CHASSIS)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,SetSteeringEngineRAD270(lock_angle1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,SetSteeringEngineRAD270(lock_angle2*3.14159265f/180.0f));
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==CMD_MODE_UNLOCK_CHASSIS)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,SetSteeringEngineRAD270(unlock_angle1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,SetSteeringEngineRAD270(unlock_angle2*3.14159265f/180.0f));
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==10)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,SetSteeringEngineRAD270(debug1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SetSteeringEngineRAD270(debug2*3.14159265f/180.0f));
			wheel_reset=1;
		}
		else if(chassis_ctrl.cmd==CMD_MODE_WHEEL_RESET&&wheel_reset)
		{
			wheel_reset=0;
			steering1.ready_edge_flag=0;
			steering2.ready_edge_flag=0;
			steering3.ready_edge_flag=0;
			steering4.ready_edge_flag=0;
		}
		else
			wheel_reset=1;
		//TODO:填写并发送底盘反馈
		
		last_chassis_ctrl=chassis_ctrl;
	}
}

void RadarDog_Cb(void* param)
{
	chassis_state.state=0;		//雷达掉线
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart ->Instance == UART5){
        if (Radar_Rxdata.head == 0x2B){
			chassis_state.state=1;	//雷达上线
			FeedDog(radar_watchdog);
			
            chassis_state.x = Radar_Rxdata.x;
            chassis_state.y = Radar_Rxdata.y;
            if (Radar_Rxdata.angle / 180.f + Radar_data_offest.angle - chassis_state.angle > 180)
                    Radar_data_offest.angle -= 360;
            else if (Radar_Rxdata.angle / 180.f + Radar_data_offest.angle - chassis_state.angle < -180)
                    Radar_data_offest.angle += 360;
            chassis_state.angle = Radar_Rxdata.angle / 180.f + Radar_data_offest.angle;
			chassis_state.x_ball = Radar_Rxdata.x_ball;
			chassis_state.y_ball = Radar_Rxdata.y_ball;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)&Radar_Rxdata,sizeof(Radar_Rxdata));
    }
	if(huart->Instance==UART4)	//来自上板的控制信息接收
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chassis_ctrl_recv, sizeof(ChassisCtrl_t));
		if(chassis_ctrl_recv[0]!=0x5A)		//没有通过包头校验
			return;
		if(DataPackCheck(chassis_ctrl_recv,sizeof(chassis_ctrl_recv)-1)!=chassis_ctrl_recv[sizeof(chassis_ctrl_recv)-1])
			return;
		memcpy(&chassis_ctrl,chassis_ctrl_recv,sizeof(ChassisCtrl_t));
        chassis_ctrl_recv[0] = 0;
		BaseType_t temp;
		xSemaphoreGiveFromISR(upboard_semaphore,&temp);
		portYIELD_FROM_ISR(temp);
	}
}

uint8_t DataPackCheck(void* pack,uint32_t datasize)
{
	uint8_t sum=0;
	for(uint32_t i=0;i<datasize;i++)
		sum+=*(((uint8_t*)pack)+i);
	return sum;
}

void UART6_IT()
{
	if(!(__HAL_UART_GET_IT_SOURCE(&huart6,UART_IT_IDLE)&&__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)))
		return ;
    HAL_UART_DMAStop(&huart6);
    STP_23L_DataProcess(STP_Data,&STP_t);
	
     cur_height = STP_t.distance * 0.001f;
//    chassis_state.ground_dis = cur_height;
    HAL_UART_Receive_DMA(&huart6, STP_Data, sizeof(STP_Data));
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)	//收到错误帧处理
{
    if(huart->Instance==UART5)
	{
//        __HAL_UNLOCK(huart);	
		HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)&Radar_Rxdata,sizeof(RadarPack_Typedef));
	}
	else if(huart->Instance==UART4)
	{
		HAL_UART_DMAStop(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4,chassis_ctrl_recv,sizeof(chassis_ctrl_recv));
	}
}



