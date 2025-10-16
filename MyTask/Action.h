#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

void SwitchMoveMode(void* param);
void SwitchJumpMode(void* param);
void SwitchLaunchMode(void* param);


uint16_t SetSteeringEngineRAD270(float rad);
