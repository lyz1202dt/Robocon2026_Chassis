#ifndef __AS5047_H__
#define __AS5047_H__

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>

#define AS5047P_ANGLECOM 0x3FFF //具有动态角度误差补偿的测量角度

typedef struct 
{
    SPI_HandleTypeDef *hspi; // 指向 HAL SPI 句柄（DataSize=16bit）
    GPIO_TypeDef* cs_port;   // 片选引脚端口
    uint16_t cs_pin;         // 片选引脚编号
    int16_t rotation_count;   // 旋转圈数
    float total_angle;    // 连续化后的角度值（弧度制）
    bool first_read;          // 首次读取标志
    int32_t last_raw;          /* 上次读取的 14-bit 原始数据 */
    bool initialized;           // 初始化标志
} AS5047P_HandleTypeDef;

void AS5047P_Init(AS5047P_HandleTypeDef *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
float AS5047P_ReadContinuousAngle(AS5047P_HandleTypeDef *sensor);
void AS5047P_ReadALLSensors(AS5047P_HandleTypeDef sensors[], float angles[], uint8_t sensor_count);

#endif
