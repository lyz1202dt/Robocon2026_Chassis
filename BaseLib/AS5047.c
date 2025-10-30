#include "AS5047.h"
#include "spi.h"

static uint16_t SPI_TransmitReceive16Bit(AS5047P_HandleTypeDef *sensor, uint16_t data)
{
    uint16_t received_data = 0;
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET); // 片选拉低
    __NOP(); // 确保片选信号稳定
    HAL_SPI_TransmitReceive(sensor->hspi, (uint8_t*)&data, (uint8_t*)&received_data, 1, 100);
    __NOP(); // 确保片选信号稳定
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET); // 片选拉高
    return received_data;
}

static uint8_t CalculateEvenParity(uint16_t value)
{
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        parity ^= (value >> i) & 0x01;
    }
    return parity;
}

static uint16_t ReadAngleRegister(AS5047P_HandleTypeDef *sensor)
{
    uint16_t command = AS5047P_ANGLECOM; // 读取角度寄存器命令
    command |= (1 << 14); // 设置读操作位

    // 计算并设置奇偶校验位
    uint8_t parity = CalculateEvenParity(command);
    command |= (parity << 15);

    uint16_t response = SPI_TransmitReceive16Bit(sensor, command);

    if(response & (1 << 14)) // 检查错误位
    {
        return 0;
    }

    return response & 0x3FFF; // 返回14位角度数据
}

void AS5047P_Init(AS5047P_HandleTypeDef *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    sensor->hspi = hspi;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;
    sensor->rotation_count = 0;
    sensor->total_angle = 0.0f;
    sensor->first_read = true;
    sensor->last_raw = 0;
    sensor->initialized = true;

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET); // 片选拉高
    __NOP();
}

float AS5047P_ReadContinuousAngle(AS5047P_HandleTypeDef *sensor)
{
    if(!sensor->initialized)
    {
        return 0.0f; // 未初始化，返回0
    }

    uint16_t raw_angle = ReadAngleRegister(sensor);
    if(raw_angle == 0)
    {
        return sensor->total_angle; // 读取错误，返回上次角度
    }

    if(sensor->first_read)
    {
        sensor->last_raw = raw_angle;
        sensor->first_read = false;
        sensor->rotation_count = 0;
        sensor->total_angle = raw_angle* 2.0f * 3.14159265358f / 16384.0f; // 转换为弧度
    }

    int32_t delta = (int32_t)raw_angle - (int32_t)sensor->last_raw;

    if(delta > 8192) // 正向跨越零点
    {
        sensor->rotation_count--;
    }
    else if(delta < -8192) // 反向跨越零点
    {
        sensor->rotation_count++;
    }

    sensor->last_raw = raw_angle;
    sensor->total_angle = sensor->rotation_count * 2.0f * 3.14159265358f + (raw_angle * 2.0f * 3.14159265358f / 16384.0f); // 转换为弧度

    return sensor->total_angle;
}

void AS5047P_ResetAngle(AS5047P_HandleTypeDef *sensor)
{
    sensor->rotation_count = 0;
    sensor->total_angle = 0.0f;
    sensor->first_read = false;
    sensor->last_raw = 0;
}

void AS5047P_ReadALLSensors(AS5047P_HandleTypeDef sensors[], float angles[], uint8_t sensor_count)
{
    for(uint8_t i = 0; i < sensor_count; i++)
    {
        if(sensors[i].initialized)
        {
            angles[i] = AS5047P_ReadContinuousAngle(&sensors[i]);
        }
        else
        {
            angles[i] = 0.0f;
        }
    }
}


