#include "Task_Init.h"
#include "JY61.h"
#include "encoder.h"
#include "AS5047.h"
#include "spi.h"

SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;

TaskHandle_t Wheel_Handles[4];
TaskHandle_t Move_Task_Handle;

uint8_t usart4_dma_buff[30];
UART_DataPack RemoteData;  //将串口接收的数据存到这里
Remote_Handle_t Remote_Control; //取出遥控器数据

extern SemaphoreHandle_t remote_semaphore;

/*
    编码器结构体
*/
Encoder_HandleTypeDef encoderA;
Encoder_HandleTypeDef encoderB;
Encoder_HandleTypeDef encoderC;
Encoder_HandleTypeDef encoderD;

void Task_Init(void)
{
	//遥控器
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));

	  Encoder_Init(&encoderA, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 8000, 1.0f, 0.6f);
    Encoder_Init(&encoderB, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3, 8000, 1.0f, 0.6f);
    Encoder_Init(&encoderC, GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, 8000, 1.0f, 0.6f);
    Encoder_Init(&encoderD, GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, 8000, 1.0f, 0.6f);
	
    steeringWheelArray[0].Key_GPIO_Port = GPIOA;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_7;
    steeringWheelArray[1].Key_GPIO_Port = GPIOA;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_5;
    steeringWheelArray[2].Key_GPIO_Port = GPIOA;
    steeringWheelArray[2].Key_GPIO_Pin = GPIO_PIN_6;
    steeringWheelArray[3].Key_GPIO_Port = GPIOA;
    steeringWheelArray[3].Key_GPIO_Pin = GPIO_PIN_4;
    
    steeringWheelArray[0].encoder = encoderA;
    steeringWheelArray[1].encoder = encoderB;
    steeringWheelArray[2].encoder = encoderC;
    steeringWheelArray[3].encoder = encoderD;

    wheelArray[0].pos.x = -0.32f;
    wheelArray[0].pos.y =  0.32f;
    wheelArray[1].pos.x =  0.32f;
    wheelArray[1].pos.y =  0.32f;
    wheelArray[2].pos.x =  0.32f;
    wheelArray[2].pos.y = -0.32f;
    wheelArray[3].pos.x = -0.32f;
    wheelArray[3].pos.y = -0.32f;
    for(int i = 0; i < 4; i++)
    {
        wheelArray[i].pos.z = 0.0f;
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].set_target_cb = SetWheelTarget_Callback;
        wheelArray[i].reset_cb = WheelReset_Callback;
        wheelArray[i].state_cb = WheelState_Callback;
        wheelArray[i].get_vel_cb = GetWheelVelocity_Callback;
        chassis.wheel[i] = &wheelArray[i];
        steeringWheelArray[i].DriveMotor.motor_id = i;
        xTaskCreate(Wheel_Task, "wheel_task", 128, &wheelArray[i], 4, &Wheel_Handles[i]);
    }

    chassis.mass = 10.0f;
    chassis.I = 1.25f;
    chassis.barycenter.x = 0.0f;
    chassis.barycenter.y = 0.0f;
    chassis.dead_zone = 0.001f;
    chassis.update_dt_ms = 5;
    chassis.wheel_err_cb = WheelError_Callback;
    ChassisInit(&chassis, wheelArray, chassis.barycenter, chassis.mass, chassis.I, chassis.dead_zone, chassis.update_dt_ms, 512, 4);

    xTaskCreate(Move_Task, "Move_Task", 256, NULL, 5, &Move_Task_Handle);
}

float FinalCurrent = 0.0f;

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    Wheel_t *wheel=(Wheel_t *)pvParameters;
    SteeringWheel *swheel = (SteeringWheel *)wheel->user_data;

    swheel->Steering_Vel_PID.Kp = 10.0f;
    swheel->Steering_Vel_PID.Ki = 0.0f;
    swheel->Steering_Vel_PID.Kd = 0.0f;
    swheel->Steering_Vel_PID.limit = 10000.0f;
    swheel->Steering_Vel_PID.output_limit = 10000.0f;

    swheel->Steering_Dir_PID.Kp = 200.0f;
    swheel->Steering_Dir_PID.Ki = 0.0f;
    swheel->Steering_Dir_PID.Kd = 3.3f;
    swheel->Steering_Dir_PID.limit = 10.0f;
    swheel->Steering_Dir_PID.output_limit = 10000.0f;

    swheel->Driver_Vel_PID.Kp = 5.0f;
    swheel->Driver_Vel_PID.Ki = 0.0f;
    swheel->Driver_Vel_PID.Kd = 1.0f;
    swheel->Driver_Vel_PID.limit = 10000.0f;
    swheel->Driver_Vel_PID.output_limit = 50.0f;

    swheel->offset = 0.0f;
    swheel->maxRotateAngle = 350.0f;
    swheel->floatRotateAngle = 340.0f;
    swheel->ready_edge_flag = 0;
    swheel->addoffsetangle = 0.0f;
		
    for(;;)
    {
        PID_Control2(swheel->currentDirection, swheel->putoutDirection, &swheel->Steering_Dir_PID);//角度环
        PID_Control2(swheel->SteeringMotor.Speed, swheel->Steering_Dir_PID.pid_out, &swheel->Steering_Vel_PID);//速度环

        PID_Control2(swheel->DriveMotor.rpm, swheel->expextVelocity * 60.0f / (2.0f * PI * n * wheel_radius), &swheel->Driver_Vel_PID);//驱动电机速度环
        FinalCurrent = swheel->Driver_Vel_PID.pid_out + swheel->expextForce;
        VESC_SetCurrent(&swheel->DriveMotor, FinalCurrent);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

ChassisMode chassis_mode = REMOTE;

Vector3D cur_pos;
Vector3D exp_pos;
Vector3D exp_vel;
PID2 Pos_PID_x;
PID2 Pos_PID_y;
PID2 Pos_PID_z;
extern JY61_Typedef JY61;
uint8_t Gyro_Rx_Buffer[22];

AS5047P_HandleTypeDef sensors[4];
float sensor_angles[4];

void Move_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    AS5047P_Init(&sensors[0], &hspi2, GPIOB, GPIO_PIN_12);
    AS5047P_Init(&sensors[1], &hspi2, GPIOB, GPIO_PIN_12);
    AS5047P_Init(&sensors[2], &hspi2, GPIOB, GPIO_PIN_12);
    AS5047P_Init(&sensors[3], &hspi2, GPIOB, GPIO_PIN_12);

    for(;;)
    {
        AS5047P_ReadAllSensors(sensors, sensor_angles, 4);

        if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            memcpy(&RemoteData, usart4_dma_buff, sizeof(RemoteData));
            UpdateKey(&Remote_Control);
            Remote_Control.Ex = RemoteData.rocker[0];
            Remote_Control.Ey = RemoteData.rocker[1];
            Remote_Control.Eomega = RemoteData.rocker[2];
            Remote_Control.mode = RemoteData.rocker[3];
            Remote_Control.Key_Control = &RemoteData.Key;
        }else{
            Remote_Control.Ex = 0;
            Remote_Control.Ey = 0;
            Remote_Control.Eomega = 0;
            Remote_Control.mode = 0;
            //按键状态清零
            memset(&RemoteData.Key, 0, sizeof(hw_key_t));
            Remote_Control.Key_Control = &RemoteData.Key;
        }

        if(chassis_mode == AUTO)
        {
            JY61_Receive(&JY61, Gyro_Rx_Buffer, sizeof(Gyro_Rx_Buffer));
            PID_Control2(cur_pos.x, exp_pos.x, &Pos_PID_x);
            PID_Control2(cur_pos.y, exp_pos.y, &Pos_PID_y);
            PID_Control2(cur_pos.z, exp_pos.z, &Pos_PID_z);

            Vector3D exp_vel_world;
            exp_vel_world.x = Pos_PID_x.pid_out + exp_vel.x;
            exp_vel_world.y = Pos_PID_y.pid_out + exp_vel.y;
            exp_vel_world.z = Pos_PID_z.pid_out + exp_vel.z;

            float c = arm_cos_f32(cur_pos.z);
            float s = arm_sin_f32(cur_pos.z);
            chassis.exp_vel.x = exp_vel_world.x * c + exp_vel_world.y * s;
            chassis.exp_vel.y = exp_vel_world.y * c - exp_vel_world.x * s;
            chassis.exp_vel.z = exp_vel_world.z;
        }else if(chassis_mode == REMOTE)
        {
            chassis.exp_vel.x = Remote_Control.Ex / 2047.0f * MAX_ROBOT_VEL; //遥控器控制，最大1.5m/s
            chassis.exp_vel.y = Remote_Control.Ey / 2047.0f * MAX_ROBOT_VEL;
            chassis.exp_vel.z = Remote_Control.Eomega / 2047.0f * MAX_ROBOT_OMEGA; //最大180度/s
        }
       
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

void UpdateKey(Remote_Handle_t * xx) { //遥控器数据更新
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_Event(&encoderA, GPIO_Pin);
    Encoder_Event(&encoderB, GPIO_Pin);
    Encoder_Event(&encoderC, GPIO_Pin);
    Encoder_Event(&encoderD, GPIO_Pin);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
   if(hspi->Instance == SPI2)
   {
        for(int i = 0; i < 4; i++)
        {
            AS5047P_ResetAngle(&sensors[i]);
        }
   }
}
