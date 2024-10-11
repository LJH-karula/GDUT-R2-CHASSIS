#pragma once
#include <stddef.h>
#include <limits.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "string.h"
#include "FreeRTOS.h"
#include "data_pool.h"
#include "service_communication.h"

#ifdef __cplusplus
class IMU : Tools
{
public:
    UART_TxMsg IMU_TxMsg;
    float aX, aY, aZ;                      // 加速度
    float pitch, roll, yaw;                // 姿态
    float pitch_rate, roll_rate, yaw_rate; // 角速度

    int8_t IMU_GetAcceleration(int8_t *buffer);
    int8_t IMU_GetAngle(int8_t *buffer);
    int8_t IMU_GetAngleRate(int8_t *buffer);
    void IMU_GetData(int8_t *buffer);

    void IMU_SetBaudRate(int BaudRate, UART_HandleTypeDef *huart);
    void IMU_Clear_Z_Angle(void);
    void IMU_Calibration(void);

private:
    // 内部命令
    uint8_t Baud[3] = {0xFF, 0xAA, 0x63};   // 波特率,默认115200
    uint8_t ClearZ[3] = {0xFF, 0xAA, 0x52}; // 清除Z轴姿态
    uint8_t Calib[3] = {0xFF, 0xAA, 0x67};  // 校准

    int8_t HeadA[2] = {0x55, 0x51}; // 加速度包头
    int8_t HeadV[2] = {0x55, 0x52}; // 角速度包头
    int8_t HeadG[2] = {0x55, 0x53}; // 角度包头
};

extern IMU imu;
#endif
