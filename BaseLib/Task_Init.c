#include "Task_Init.h"

SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;

TaskHandle_t Wheel_Handles[4];

void Task_Init(void)
{

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

        xTaskCreate(Wheel_Task, "wheel_task", 128, &steeringWheelArray[i], 4, &Wheel_Handles[i]);

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

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    SteeringWheel *wheel=(SteeringWheel *)pvParameters;

    for(;;)
    {
        MinorArcDeal(wheel);
        

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}
