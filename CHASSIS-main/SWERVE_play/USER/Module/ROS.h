#pragma once
#include "stdint.h"
#include "drive_uart.h"
#include "data_pool.h"
#include "tool.h"
#include "pid.h"
#include "math.h"
#include "robot_def.h"

typedef struct readFromRos
{
    float x;
    float y;
    float z;
    float relative_Vx;
    float relative_Vy;
    uint8_t ctrl_mode;
    uint8_t ctrl_flag;
    uint8_t chassis_init;
    Robot_Status_t status;
}readFromRos;

typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus

class ROS : Tools
{
public:
    ROS()
    {
			//°üÍ·°üÎ²
      header[0] = 0x55;
      header[1] = 0xAA;
      tail[0] = 0x0D;
      tail[1] = 0x0A;
    }
    void Send_To_ROS(Robot_Twist_t speed);
    int8_t Recieve_From_ROS(uint8_t *buffer);
    int8_t Recieve_From_ROS_Pos(uint8_t *buffer);
    readFromRos readFromRosData;
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    static SystemTick_Fun get_systemTick;
		void Point_tracking_PID_Mode_Init(float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out);
		void Point_tracking_PID_Param_Init(float Kp, float Ki, float Kd, float Integral_Max, float Out_Max, float DeadZone);
    
private:
    UART_TxMsg TxMsg;
    uint8_t header[2];
    uint8_t tail[2];
    uint8_t lenth=0;

    PID PID_position;
};


//extern ROS ros;
#endif
