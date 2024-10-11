#pragma once
#include "stdint.h"
#include "drive_uart.h"
#include "data_pool.h"
#include "tool.h"

typedef struct readFromRos
{
    float x;
    float y;
    float relative_Vx;
    float relative_Vy;
    unsigned char ctrl_mode;
}readFromRos;

typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus

class ROS : Tools
{
public:
    ROS()
    {
			//��ͷ��β
      header[0] = 0x55;
      header[1] = 0xAA;
      tail[0] = 0x0D;
      tail[1] = 0x0A;
    }
    void Send_To_ROS(Robot_Twist_t speed);
    int8_t Recieve_From_ROS(uint8_t *buffer);
    readFromRos readFromRosData;
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    static SystemTick_Fun get_systemTick;
    
private:
    UART_TxMsg TxMsg;
    uint8_t header[2];
    uint8_t tail[2];
    uint8_t lenth=0;
};
extern ROS ros;
#endif
