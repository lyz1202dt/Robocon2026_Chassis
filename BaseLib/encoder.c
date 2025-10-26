#include "encoder.h"
#include <math.h>

void Encoder_Init(Encoder_HandleTypeDef * henc, GPIO_TypeDef *A_port, uint16_t A_pin,
                  GPIO_TypeDef *B_port, uint16_t B_pin,
                  uint32_t ppr, float gear_ratio, float lp_alpha)
{
    if(henc == NULL)
        return;
    henc->GPIO_encord_A = A_port;
    henc->GPIO_Pin_A = A_pin;
    henc->GPIO_encord_B = B_port;
    henc->GPIO_Pin_B = B_pin;
    henc->pulses_per_rev = ppr;
    henc->gear_ratio = gear_ratio;
    henc->lp_alpha = lp_alpha;
    henc->Count = 0;
    henc->Last_Count = 0;
    henc->angle_deg = 0.0f;
    henc->angular_velocity_dps = 0.0f;
}

void Encoder_Event(Encoder_HandleTypeDef *henc, uint16_t GPIO_Pin)
{
    uint8_t a_state = (HAL_GPIO_ReadPin(henc->GPIO_encord_A, henc->GPIO_Pin_A)) == GPIO_PIN_SET ? 1 : 0;
    uint8_t b_state = (HAL_GPIO_ReadPin(henc->GPIO_encord_B, henc->GPIO_Pin_B)) == GPIO_PIN_SET ? 1 : 0;

    if (GPIO_Pin == henc->GPIO_Pin_A) {
        if (a_state) {
            if (b_state) henc->Count++;
            else        henc->Count--;
        } else {
            if (b_state) henc->Count--;
            else        henc->Count++;
        }
    } else if (GPIO_Pin == henc->GPIO_Pin_B) {
        if (a_state) {
            if (b_state) henc->Count--;
            else        henc->Count++;
        } else {
            if (b_state) henc->Count++;
            else        henc->Count--;
        }
    }
}

void Encoder_Update(Encoder_HandleTypeDef *henc, float dt)
{
    if (henc == NULL || dt <= 0.0f)
        return;
    if (henc->pulses_per_rev == 0 || henc->gear_ratio == 0.0f)
        return;

    /* 计算增量计数 */
    int32_t delta_count = (int32_t)(henc->Count - henc->Last_Count);
    /* 更新 Last_Count */
    henc->Last_Count = henc->Count;

    float denom = (float)(henc->pulses_per_rev * henc->gear_ratio);
    float revs = (float)delta_count / denom;

    /* 增量角度（本次采样的角增量，单位度） */
    float ddeg = revs * 360.0f;
    float inst_dps = ddeg / dt;

    /* 一阶低通滤波 */
    float alpha = henc->lp_alpha;
    if (alpha >= 1.0f) {
        henc->angular_velocity_dps = inst_dps;
    } else if (alpha <= 0.0f) {
        /* 不更新速度 */
    } else {
        henc->angular_velocity_dps = henc->angular_velocity_dps * (1.0f - alpha) + inst_dps * alpha;
    }

    /* 连续累加角度 */
    henc->angle_deg += ddeg;
}

