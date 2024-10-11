/**
 * @file IMU.cpp
 * @author Py
 * @brief 陀螺仪的数据获取，基于串口通信
 * @attention 基于Wit的WT61C-TTL的三维运动姿态测量
 * @version 0.1
 * @date 2024-09-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "IMU.h"
#include "usart.h"

IMU imu;

/**
 * @brief 获取加速度数据
 * @param buffer 接收数据缓冲区
 * @retval 1 成功获取数据
 *         0 数据校验失败
 */
int8_t IMU::IMU_GetAcceleration(int8_t *buffer)
{
    if (*buffer == HeadA[0] && *(buffer + 1) == HeadA[1])
    {
        aX = ((int16_t) * (buffer + 3) << 8 | (uint8_t) * (buffer + 2)) / 208.979;
        aY = ((int16_t) * (buffer + 5) << 8 | (uint8_t) * (buffer + 4)) / 208.979;
        aZ = ((int16_t) * (buffer + 7) << 8 | (uint8_t) * (buffer + 6)) / 208.979;
        if (*(buffer + 10) != sum_crc8(buffer, 11))
        {
            aX = 0;
            aY = 0;
            aZ = 0;
            return 0;
        }
        else
            return 1;
    }
    else
    {
        aX = 0;
        aY = 0;
        aZ = 0;
        return 0;
    }
}

/**
 * @brief 获取角速度数据
 * @param buffer 接收数据缓冲区
 * @retval 1 成功获取数据
 *         0 数据校验失败
 */
int8_t IMU::IMU_GetAngleRate(int8_t *buffer)
{
    if (*buffer == HeadV[0] && *(buffer + 1) == HeadV[1])
    {
        roll_rate = ((int16_t) * (buffer + 3) << 8 | (uint8_t) * (buffer + 2)) / 16.384;
        pitch_rate = ((int16_t) * (buffer + 5) << 8 | (uint8_t) * (buffer + 4)) / 16.384;
        yaw_rate = ((int16_t) * (buffer + 7) << 8 | (uint8_t) * (buffer + 6)) / 16.384;
        if (*(buffer + 10) != sum_crc8(buffer, 11))
        {
            roll_rate = 0;
            pitch_rate = 0;
            yaw_rate = 0;
            return 0;
        }
        else
            return 1;
    }
    else
    {
        roll_rate = 0;
        pitch_rate = 0;
        yaw_rate = 0;
        return 0;
    }
}

/**
 * @brief 获取角度数据
 * @param buffer 接收数据缓冲区
 * @retval 1 成功获取数据
 *         0 数据校验失败
 */
int8_t IMU::IMU_GetAngle(int8_t *buffer)
{
    if (*buffer == HeadG[0] && *(buffer + 1) == HeadG[1])
    {
        roll = ((int16_t) * (buffer + 3) << 8 | (uint8_t) * (buffer + 2)) / 182.044;
        pitch = ((int16_t) * (buffer + 5) << 8 | (uint8_t) * (buffer + 4)) / 182.044;
        yaw = ((int16_t) * (buffer + 7) << 8 | (uint8_t) * (buffer + 6)) / 182.044;
        if (*(buffer + 10) != sum_crc8(buffer, 11))
        {
            roll = 0;
            pitch = 0;
            yaw = 0;
            return 0;
        }
        else
            return 1;
    }
    else
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
        return 0;
    }
}

/**
 * @brief 获取加速度、角速度、角度数据
 * @param buffer 接收数据缓冲区
 */
void IMU::IMU_GetData(int8_t *buffer)
{
    IMU_GetAcceleration(buffer);
    IMU_GetAngleRate(buffer + 11);
    IMU_GetAngle(buffer + 22);
}

/**
 * @brief 设置陀螺仪波特率
 * @param BaudRate 波特率 9600/115200
 * @param huart 串口句柄
 * @retval None
 */
void IMU::IMU_SetBaudRate(int BaudRate, UART_HandleTypeDef *huart)
{
    if (BaudRate == 9600)
    {
        Baud[2] = 0x64;
    }
    else if (BaudRate == 115200)
    {
        Baud[2] = 0x63;
    }

    IMU_TxMsg.data_addr = Baud;
    IMU_TxMsg.len = 3;
    IMU_TxMsg.huart = huart;
    xQueueSend(UART_TxPort, &IMU_TxMsg, 0);
}

/**
 * @brief 清除Z轴角度
 */
void IMU::IMU_Clear_Z_Angle(void)
{
    IMU_TxMsg.data_addr = ClearZ;
    IMU_TxMsg.len = 3;
    xQueueSend(UART_TxPort, &IMU_TxMsg, 0);
}

/**
 * @brief 陀螺仪校准
 */
void IMU::IMU_Calibration(void)
{
    IMU_TxMsg.data_addr = Calib;
    IMU_TxMsg.len = 3;
    IMU_TxMsg.huart = &huart3;
    xQueueSend(UART_TxPort, &IMU_TxMsg, 0);
}
