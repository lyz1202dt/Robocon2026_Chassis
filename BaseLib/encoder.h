#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef *GPIO_encord_A; // A相引脚端口
    uint16_t GPIO_Pin_A;         // A相引脚号
    GPIO_TypeDef *GPIO_encord_B; // B相引脚端口
    uint16_t GPIO_Pin_B;         // B相引脚号

    volatile int32_t  Count;              // 编码器计数值（累计）
    int32_t  Last_Count;                  // 上次用于速度计算的计数值

    uint32_t pulses_per_rev;              // 每转脉冲数（PPR）
    float    gear_ratio;                  // 机械减速比

    float angle_deg;                      // 0..360 实时角度
    float angular_velocity_dps;           // 角速度 °/s

    float   lp_alpha;            // 速度低通滤波系数
} Encoder_HandleTypeDef;

void Encoder_Init(Encoder_HandleTypeDef * henc, GPIO_TypeDef *A_port, uint16_t A_pin,
                  GPIO_TypeDef *B_port, uint16_t B_pin,
                  uint32_t ppr, float gear_ratio, float lp_alpha);
void Encoder_Event(Encoder_HandleTypeDef *henc, uint16_t GPIO_Pin);
void Encoder_Update(Encoder_HandleTypeDef *henc, float dt);

#endif
