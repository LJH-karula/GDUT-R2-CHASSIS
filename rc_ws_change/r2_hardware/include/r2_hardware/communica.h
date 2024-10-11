#ifndef __COMMUNICA_H
#define __COMMUNICA_H

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "stdint.h"
#include "r2_msgs/stm32.h"
#include "r2_msgs/controller_cmd.h"


typedef enum
{
    BALL_OUTSIDE=0,     //未拿到球(放球成功以及第一次进入三区时的状态) 0
    BALL_HAVE_TAKEN,    //已拿到球(机器人拿到球，但夹爪还没碰到微动开关) 1
    BALL_IN_CLAW,        //拿到球且夹爪到位(球在夹爪中，已经触碰到微动开关) 2
}BALL_STATUS;

typedef enum
{
    TAKE_BALL = 2,   //取球 2
    SHOOT_BALL,     //出球 3
    CONTROLLER_OFF, //关闭 4
    CONTROLLER_ERROR    //错误状态 5
}CONTROLLER_STATE;

typedef enum
{
    NO_BALL = 0,    //无球
    BLUE_BALL,      //蓝球
    PURPLE_BALL,    //紫球
    RED_BALL        //红球
}BALL_COLOR;

typedef enum
{
    INVERSE_BALL = 0,   //正常吸球
    LEFT_TAKE_BALL,         //左吸球
    RIGHT_TAKE_BALL         //右吸球
}TAKE_BALL_STATE;

namespace control_base
{
class CONTROL_BASE
{
public:
    boost::asio::serial_port* boost_serial_point;       //上层串口
    boost::asio::serial_port* chassis_boost_serial_point;     //底盘串口
    //上层接收
    bool ROS_READ_FROM_UPPER(unsigned char &now_controller_state,unsigned char &ball_state,unsigned char &arrive_flag,unsigned char &lock_flag,unsigned char &touch_flag,
     unsigned char &start_flag,unsigned char &red_blue_flag,unsigned char &restart_flag,unsigned char &if_auto_flag,unsigned char &test_cnt);
    //上层发送
   void UPPER_TO_STM32(unsigned char next_ctrl_state,unsigned char take_ball_flag,float Yaw,unsigned char clean_ball_flag);
    //底盘发送
    void CHASSIS_TO_STM32(float chassis_x, float chassis_y, float chassis_w, unsigned char chassis_control_flag,int8_t control_flag,int8_t chassis_init);                            
    CONTROL_BASE();
    void SerialInit(const char* serial);    //上层串口初始化
    void ChassisSerialInit(const char* serial); //底盘串口初始化
    unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);


private:
    boost::asio::io_service boost_io_service;   //上层io_service
    boost::asio::io_service chassis_boost_io_service;    //底盘io_service 
    // std::string stm32_serial_port_;
    ros::Publisher pub;
    unsigned char Recieve_buf[16] = {0}; 
    unsigned char Upper_Buf[13] = {0};
    unsigned char Chassis_Buf[21] = {0}; //底盘发送缓存区   
    // union Recieve_Union0
    // {
    //     short data;
    //     unsigned char tmp_array[2];
    // }Stm32ToRos_RPM1,Stm32ToRos_RPM2,Stm32ToRos_RPM3,Stm32ToRos_RPM4;

    // union Recieve_Union1
    // {
    //     float data;
    //     unsigned char tmp_array[4];
    // }ROBOT_POS_X,ROBOT_POS_Y,ROBOT_POS_YAW;
    union Send_Union0
    {
        float data;
        unsigned char tmp_array[4];
    }ROBOT_YAW;
    union Send_Union1
    {
        float data;
        unsigned char tmp_array[4];
    }Swerve_Chassis_X,Swerve_Chassis_Y,Swerve_Chassis_W;


    const unsigned char serial_header[2] = {0x55,0xaa};
    const unsigned char serial_ender[2] = {0x0d,0x0a};
};
} // namespace control_base



#endif
