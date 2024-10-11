
#include "ZigBee.h"
 ZigbeeSendData Remote;//发送给遥控的数据
 ZigbeeSendData R1;//发送给R1的数据

 ZigBee Zigbee;

/*
函数名称：ZigBee_Init
函数功能：初始化ZigBee
函数参数：ZigbeeSetting:Zigbee发送参数设置
          SendPort:发送端口
          SenfMode:发送模式
          TargetAddr:目标地址
          TargetPort:目标端口
          Num:帧序号
          Direction:方向
          ClusterID:簇ID
          FirmID:厂商ID
          AnswerMode:应答模式
          OrderID:指令ID
函数返回：无
*/
 uint8_t ZigBee::ZigBee_Init(ZigbeeSendData* ZigbeeSetting,
                        uint8_t SendPort,uint8_t SenfMode,
                        uint16_t TargetAddr,uint8_t TargetPort,
                        uint8_t Num,uint8_t Direction,
                        uint16_t ClusterID,uint16_t FirmID,
                        uint8_t AnswerMode,uint8_t OrderID)
 {
     ZigbeeSetting->SendPort = SendPort;
     ZigbeeSetting->SenfMode = SenfMode;
     ZigbeeSetting->TargetAddr = TargetAddr;
     ZigbeeSetting->TargetPort = TargetPort;
     ZigbeeSetting->Num = Num;
     ZigbeeSetting->Direction = Direction;
     ZigbeeSetting->ClusterID = ClusterID;
     ZigbeeSetting->FirmID = FirmID;
     ZigbeeSetting->AnswerMode = AnswerMode;
     ZigbeeSetting->OrderID = OrderID;
     return 1;
 }


uint8_t ZigBee::ZigBee_SendData(UART_HandleTypeDef *huart,ZigbeeSendData* ZigbeeSendData,uint8_t *data, uint16_t len)
{
    
}


