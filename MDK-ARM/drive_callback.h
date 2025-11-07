#ifndef DRIVE_CALLBACK_H
#define DRIVE_CALLBACK_H

#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "pid_old.h"
#include "motor.h"
#include "Odrive.h"
#include "vesc.h"
#include "encoder.h"

#define PI 3.14159265358979f
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define RAD2ANGLE(x) (x) * 180.0f / PI
#define n 3.0f //减速比
#define wheel_radius 0.05f //轮子半径，单位m
#define KV 170.0f //电机KV值
#define VEL_TRANSFORM (10.0f/(3.0f*3.14159265f*0.096f)) //将m/s速度转换为电机r/s

typedef struct
{
  ODrive DriveMotor;           // 5055电机
  // VESC_t DriveMotor;           //8311电机
  M2006_TypeDef SteeringMotor; // M2006电机
  PID2 Steering_Dir_PID; // 转向电机PID角度环控制器
  PID2 Steering_Vel_PID; // 转向电机PID速度环控制器
  PID2 Driver_Vel_PID;   // 驱动电机PID速度环控制器
  float currentDirection;//当前方向角度（°）
	float D_angle;         //在同一周期内求旋转角
  float expectDirection; //期望方向角度（°）
  float expextVelocity;
  float expextForce;
  float putoutDirection; //弧度
  float putoutVelocity; // m/s
  float last_expDirection;

  float offset;           // 2006电机起始误差
  float maxRotateAngle;   // 最大正反转度数
  float floatRotateAngle; // 柔性区间度数（防止因目标值在机械角度限制附近振动导致输出震荡，应当略小于maxRotateAngle大约10~20度）

  GPIO_TypeDef *Key_GPIO_Port; // 光电门引脚端口
  uint16_t Key_GPIO_Pin;       // 光电门引脚号
  GPIO_TypeDef *error_GPIO_Port; // 故障指示引脚端口
  uint16_t error_GPIO_Pin;       // 故障指示引脚号
  uint8_t ready_edge_flag;     // 舵轮处于奇点附近，舵轮已经复位（0b G000 HIJK）

  Encoder_HandleTypeDef encoder; // 编码器读取2006角度

} SteeringWheel;

void SetWheelTarget_Callback(Wheel_t *_this, float rad, float velocity, float force);
void WheelError_Callback(Chassis_t *_this, Wheel_t *wheel);
Vector2D GetWheelVelocity_Callback(Wheel_t *_this);
WheelState WheelState_Callback(Wheel_t *_this);
void WheelReset_Callback(Wheel_t *_this);

void LimitAngle(float* angle);
float AngleDiffer(float angle1,float angle2);
void MinorArcDeal(SteeringWheel *motor);
uint8_t SteeringWheelReady(SteeringWheel *StrWhe);
void Reset_Function(SteeringWheel *pSteWhe);
void Angle_Update(SteeringWheel *motor);
#endif
