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
typedef struct ZigbeeSendData
{  
    uint8_t SendPort;
    uint8_t SenfMode;
    uint16_t TargetAddr;
    uint8_t TargetPort;
    uint8_t Num;
    uint8_t Direction;
    uint16_t ClusterID;
    uint16_t FirmID;
    uint8_t AnswerMode;
    uint8_t OrderID;
}ZigbeeSendData;



class ZigBee:Tools
{
public:
    uint8_t ZigBee_Init(ZigbeeSendData* ZigbeeSetting,
                        uint8_t SendPort,uint8_t SenfMode,
                        uint16_t TargetAddr,uint8_t TargetPort,
                        uint8_t Num,uint8_t Direction,
                        uint16_t ClusterID,uint16_t FirmID,
                        uint8_t AnswerMode,uint8_t OrderID);

    uint8_t ZigBee_SendData(UART_HandleTypeDef *huart,ZigbeeSendData* ZigbeeSendData,uint8_t *data, uint16_t len);
    uint8_t ZigBee_ReceiveData(uint8_t *data, uint16_t *len);
};

#endif