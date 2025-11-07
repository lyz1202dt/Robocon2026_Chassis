#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAX_ROBOT_VEL 1.5f // m/s
#define MAX_ROBOT_OMEGA ANGLE2RAD(30.0f) // rad/s

void Task_Init(void);
void Wheel_Task(void *pvParameters);

#pragma pack(1)
typedef struct{
   uint8_t Left_Key_Up : 1;         
   uint8_t Left_Key_Down : 1;       
   uint8_t Left_Key_Left : 1;       
   uint8_t Left_Key_Right : 1;       
   uint8_t Left_Switch_Up_or_Left : 1;       
   uint8_t Left_Switch_Down_or_Right: 1;       
   uint8_t UNUSED1 : 1;
   uint8_t UNUSED2 : 1;

   uint8_t Right_Key_Up : 1;        
   uint8_t Right_Key_Down : 1;      
   uint8_t Right_Key_Left : 1;      
   uint8_t Right_Key_Right : 1;     
   uint8_t Right_Switch_Up_or_Right : 1;      
   uint8_t Right_Switch_Down_or_Left : 1;      
   uint8_t UNUSED3 : 1; 
   uint8_t UNUSED4 : 1;
} hw_key_t;
  
typedef struct {
	uint8_t head;
	int16_t rocker[4];
	hw_key_t Key;
	uint8_t end;
} UART_DataPack;

#pragma pack()

typedef struct {
    int16_t Ex;
    int16_t Ey;
    int16_t Eomega;
	  int16_t mode;
    hw_key_t *Key_Control;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STP,
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;

void UpdateKey(Remote_Handle_t * xx);
void Move_Task(void *pvParameters);
void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);

#endif
