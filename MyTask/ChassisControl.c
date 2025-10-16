#include "ChassisControl.h"
#include "matrix.h"


SteeringWheel steering1, steering2, steering3, steering4;
int16_t motorCurrentBuf[4] = {0};
int16_t motorCurrentBuf2[4] = {0};
float velocity = 0.0f, direction = 0.0f, omega = 0.0f;

float dir1,dir2,dir3,dir4,vel;
uint8_t forced_set_dir=0;

M2006_TypeDef lift_motor1;
M2006_TypeDef lift_motor2;
PID2 lift_motor1_vel_pid;
PID2 lift_motor2_vel_pid;
float lift_motor1_target_vel=0.0f;
float lift_motor2_target_vel=0.0f;
float last_vel=0.0f;

ChassisState_t chassis_state = {.head=0x1B};
float Integral_Transmit_T;


float A[8][3] = {
        {1.0f, 0.0f, -CHASSIS_L * 0.5f},
        {0.0f, 1.0f,  CHASSIS_L * 0.5f},
        {1.0f, 0.0f, -CHASSIS_L * 0.5f},
        {0.0f, 1.0f, -CHASSIS_L * 0.5f},
        {1.0f, 0.0f,  CHASSIS_L * 0.5f},
        {0.0f, 1.0f, -CHASSIS_L * 0.5f},
        {1.0f, 0.0f, CHASSIS_L * 0.5f},
        {0.0f, 1.0f, CHASSIS_L * 0.5f}
    };

float filter_gate=0.21f;
float b[8][1];
float last_b[8][1];
float x[3][1];

	
	
#define WINDOW_SIZE 200

// === 滑动窗口结构体 ===
typedef struct {
    float buffer[WINDOW_SIZE]; // 环形缓冲区
    int index;                 // 当前写入索引
    int count;                 // 当前有效样本数量
} SlidingWindow;

// === 初始化滑动窗口 ===
void init_window(SlidingWindow* window) {
    window->index = 0;
    window->count = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        window->buffer[i] = 0.0f;
    }
}

// === 添加样本 ===
void add_sample(SlidingWindow* window, float value) {
    window->buffer[window->index] = value;
    window->index = (window->index + 1) % WINDOW_SIZE;
    if (window->count < WINDOW_SIZE) {
        window->count++;
    }
}

// === 计算方差 ===
float compute_variance(SlidingWindow* window) {
    if (window->count < WINDOW_SIZE) {
        return -1.0f; // 数据不足
    }

    float sum = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window->buffer[i];
    }
    float mean = sum / WINDOW_SIZE;

    float variance = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float diff = window->buffer[i] - mean;
        variance += diff * diff;
    }
    return variance / WINDOW_SIZE;
}
	
	
SlidingWindow sli_omega;
SlidingWindow sli_vel;
float var_omega;
float var_vel;
	
void ChassisControl(void *param)
{
	
	lift_motor1_vel_pid.Kp = 8.0f;
	lift_motor1_vel_pid.Ki = 0.3f;
	lift_motor1_vel_pid.Kd = 0.0f;
	lift_motor1_vel_pid.limit = 10000.0f;
	lift_motor1_vel_pid.output_limit = 10000.0f;
	lift_motor2_vel_pid=lift_motor1_vel_pid;
	
  // 舵轮通用参数
  steering1.Steering_Vel_PID.Kp = 10.0f; // 设置速度环PID
  steering1.Steering_Vel_PID.Ki = 0.3f;
  steering1.Steering_Vel_PID.Kd = 0.0f;
  steering1.Steering_Vel_PID.output_limit=10000.0f;
  steering1.Steering_Vel_PID.limit = 10000.0f;

  steering1.Steering_Dir_PID.Kp = 200.0f; // 设置角度环PID
  steering1.Steering_Dir_PID.Ki = 0.0f;
  steering1.Steering_Dir_PID.Kd = 3.3f;
  steering1.Steering_Dir_PID.output_limit = 10000.0f;
  steering1.Steering_Dir_PID.limit = 10.0f;

  steering1.offset = 0.0f; // 稍后由复位程序重新设置
  steering1.postureRadius=0.385*1.414f*0.5f; //轮子到几何中心的距离
  steering1.maxRotateAngle = 350.0f;    //机械限位角度
  steering1.floatRotateAngle = 340.0f; // 柔性区间大小为10度
  steering1.ready_edge_flag = 0;
  steering1.DriveMotor.hcan = &hcan1;   //使用CAN1总线

  steering4 = steering3 = steering2 = steering1;  //参数拷贝
	
  // 舵轮个性化参数
  steering1.addoffsetangle = 0.0f;
  steering1.postureAngle = 135.0f;
  steering1.Key_GPIO_Port = GPIOA;
  steering1.Key_GPIO_Pin = GPIO_PIN_7;
  steering1.DriveMotor.motorID = 0x00;

  steering2.addoffsetangle = 0.0f;
  steering2.postureAngle = 45.0f;
  steering2.Key_GPIO_Port = GPIOC;
  steering2.Key_GPIO_Pin = GPIO_PIN_5;
  steering2.DriveMotor.motorID = 0x10;

  steering3.addoffsetangle = 0.0f;
  steering3.postureAngle = -135.0f;
  steering3.Key_GPIO_Port = GPIOA;
  steering3.Key_GPIO_Pin = GPIO_PIN_6;
  steering3.DriveMotor.motorID = 0x20;

  steering4.addoffsetangle = 0.0f;
  steering4.postureAngle = -45.0f;
  steering4.Key_GPIO_Port = GPIOC;
  steering4.Key_GPIO_Pin = GPIO_PIN_4;
  steering4.DriveMotor.motorID = 0x30;


	vTaskDelay(pdMS_TO_TICKS(5000));
  TickType_t last_wake_time = xTaskGetTickCount();
    
    uint32_t last_Transmit_time = HAL_GetTick();
	
	init_window(&sli_omega);
	init_window(&sli_vel);

  while (1)
  {
      //if(ABS(velocity-last_vel)>0.5f)
		//  velocity=last_vel;
	  
		if(!forced_set_dir)
		{
			dir1=dir2=dir3=dir4=direction;
			vel=velocity;
		}
		last_vel=velocity;
		
    SteeringWheel_Deal(&steering1, vel, dir1, omega);
    SteeringWheel_Deal(&steering2, vel, dir2, omega);
    SteeringWheel_Deal(&steering3, vel, dir3, omega);
    SteeringWheel_Deal(&steering4, vel, dir4, omega);
	
    motorCurrentBuf[0] = steering1.Steering_Vel_PID.pid_out;
    motorCurrentBuf[1] = steering2.Steering_Vel_PID.pid_out;
    motorCurrentBuf[2] = steering3.Steering_Vel_PID.pid_out;
    motorCurrentBuf[3] = steering4.Steering_Vel_PID.pid_out;
    MotorSend(&hcan2, 0x200, motorCurrentBuf); // C610电调控制-进行转向控制
		
		PID_Control2(lift_motor1.Speed,lift_motor1_target_vel,&lift_motor1_vel_pid);
		PID_Control2(lift_motor2.Speed,lift_motor2_target_vel,&lift_motor2_vel_pid);
		motorCurrentBuf2[0]=lift_motor1_vel_pid.pid_out;
		motorCurrentBuf2[1]=lift_motor2_vel_pid.pid_out;
		MotorSend(&hcan2, 0x1FF, motorCurrentBuf2);	//C610电调控制-桅杆抬升电机
		
    ODriveSetVelocity(&steering1.DriveMotor, steering1.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveSetVelocity(&steering2.DriveMotor, steering2.putoutVelocity*VEL_TRANSFORM, 0.0f);
    vTaskDelay(1);
    ODriveSetVelocity(&steering3.DriveMotor, steering3.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveSetVelocity(&steering4.DriveMotor, -steering4.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveGetEncoderEstimate(&steering1.DriveMotor);
    vTaskDelay(1);
    ODriveGetEncoderEstimate(&steering2.DriveMotor);
    ODriveGetEncoderEstimate(&steering3.DriveMotor);
    ODriveGetEncoderEstimate(&steering4.DriveMotor);
	
	  b[0][0] = -steering4.DriveMotor.posVelEstimateGet.velocity * arm_cos_f32(ANGLE2RAD(steering4.currentDirection)) / VEL_TRANSFORM;
    b[1][0] = -steering4.DriveMotor.posVelEstimateGet.velocity * arm_sin_f32(ANGLE2RAD(steering4.currentDirection)) / VEL_TRANSFORM;
    b[2][0] = steering2.DriveMotor.posVelEstimateGet.velocity * arm_cos_f32(ANGLE2RAD(steering2.currentDirection)) / VEL_TRANSFORM;
    b[3][0] = steering2.DriveMotor.posVelEstimateGet.velocity * arm_sin_f32(ANGLE2RAD(steering2.currentDirection)) / VEL_TRANSFORM;
    b[4][0] = steering1.DriveMotor.posVelEstimateGet.velocity * arm_cos_f32(ANGLE2RAD(steering1.currentDirection)) / VEL_TRANSFORM;
    b[5][0] = steering1.DriveMotor.posVelEstimateGet.velocity * arm_sin_f32(ANGLE2RAD(steering1.currentDirection)) / VEL_TRANSFORM;
    b[6][0] = steering3.DriveMotor.posVelEstimateGet.velocity * arm_cos_f32(ANGLE2RAD(steering3.currentDirection)) / VEL_TRANSFORM;
    b[7][0] = steering3.DriveMotor.posVelEstimateGet.velocity * arm_sin_f32(ANGLE2RAD(steering3.currentDirection)) / VEL_TRANSFORM;

    for(int i=0;i<8;i++)  //对轮子速度进行滤波
    {
      b[i][0]=filter_gate*b[i][0]+(1.0f-filter_gate)*last_b[i][0];
      last_b[i][0] = b[i][0];
    }

	int ret = solve_linear_system_8x3(A, b, x);	//求解速度
	
	if(ret==0)
	{
		chassis_state.v_x=x[0][0];
		chassis_state.v_y=x[1][0];
		chassis_state.omega=x[2][0];     //拿出车辆的运动状态*/
	}

	
	add_sample(&sli_omega,chassis_state.omega);
	var_omega = sqrt(compute_variance(&sli_omega))*50;
	
	add_sample(&sli_vel,chassis_state.v_x);
	var_vel = sqrt(compute_variance(&sli_vel))*50;
	
	
        Integral_Transmit_T += (HAL_GetTick() - last_Transmit_time);
        last_Transmit_time = HAL_GetTick();
        if(Integral_Transmit_T > 10){
            Integral_Transmit_T = 0;
            chassis_state.check=DataPackCheck((uint8_t *)&chassis_state,sizeof(chassis_state)-1);
            HAL_UART_Transmit_DMA(&huart4,(uint8_t *)&chassis_state,sizeof(chassis_state));
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收oDrive的反馈
{
  if (hcan->Instance == CAN1)
  {
    uint8_t Recv[8] = {0};
    uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	ODriveRecvServe(&steering1.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering2.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering3.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering4.DriveMotor, ID, Recv);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收2006的反馈
{
  if (hcan->Instance == CAN2)
  {
    uint8_t Recv[8] = {0};
    uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
    if (ID == 0x201) // 左上，象限2
    {
      M2006_Receive(&steering1.SteeringMotor, Recv);
    }
    else if (ID == 0x202) // 右上(象限1)
    {
      M2006_Receive(&steering2.SteeringMotor, Recv);
    }
    else if (ID == 0x203) // 左下(象限3)
    {
      M2006_Receive(&steering3.SteeringMotor, Recv);
    }
    else if (ID == 0x204) // 右下(象限4)
    {
      M2006_Receive(&steering4.SteeringMotor, Recv);
    }
		else if(ID==0x205)		//桅杆抬升电机
		{
			M2006_Receive(&lift_motor1, Recv);
		}
		else if (ID == 0x206)
    {
      M2006_Receive(&lift_motor2, Recv);
    } 
  }
}
