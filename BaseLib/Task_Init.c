#include "Task_Init.h"

SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;

TaskHandle_t Wheel_Handles[4];

void Task_Init(void)
{
    steeringWheelArray[0].Key_GPIO_Port = GPIOA;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_7;
    steeringWheelArray[1].Key_GPIO_Port = GPIOC;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_5;
    steeringWheelArray[2].Key_GPIO_Port = GPIOA;
    steeringWheelArray[2].Key_GPIO_Pin = GPIO_PIN_6;
    steeringWheelArray[3].Key_GPIO_Port = GPIOC;
    steeringWheelArray[3].Key_GPIO_Pin = GPIO_PIN_4;

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

Vector3D cur_pos;
Vector3D exp_pos;
PID2 Pos_PID;
PID2 Vel_PID;

void AutoPilot_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for(;;)
    {
        PID_Control2(cur_pos.x, exp_pos.x, &Pos_PID);
        PID_Control2(cur_pos.y, exp_pos.y, &Pos_PID);
        

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}
