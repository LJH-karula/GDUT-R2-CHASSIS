/**
 * @file ROS.cpp
 * @brief 上下位机的通信文件，包括数据的打包和解包，数据的发送和接收。使用串口DMA
 * @version 0.1
 * @date 2024-04-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "ROS.h"

SystemTick_Fun ROS::get_systemTick = NULL;
/*ROS ros;
readFromRos ROS_Data;
union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,vx,vy;*/

static union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,z,vx,vy,pos_x,pos_y;

uint8_t ROS::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        ROS::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}


/**
 * @brief stm32 send data to ROS
 * @note 该函数用于将数据打包发送给ROS, 通过轮子速度解算后的速度转化为了mm/s和mRad/s
 */
void ROS::Send_To_ROS(Robot_Twist_t speed)
{
    uint8_t buffer[6+6];
    int index = 0;
    buffer[index++] = header[0];
    buffer[index++] = header[1];
    buffer[index++] = 1;
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.x*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.y*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.angular.z*1000), &index);
    buffer[index++] = serial_get_crc8_value(buffer, 4);
    buffer[index++] = tail[0];
    buffer[index++] = tail[1];
}


/**
 * @brief upack the data from ROS
 * @param buffer pack that recieved from ROS
 * @return int8_t unpack success return 0, else return 1
 */
int8_t ROS::Recieve_From_ROS(uint8_t *buffer)
{
    int index = 0;
    for(int i = 0; i < 2; i++)
    {
        if(buffer[index++] != header[i])
            return 2;
    }

    lenth = buffer[index++];

    for(int i=0; i<2; i++)
    {
        if(buffer[4+lenth+i] != tail[i])
            return 1;
    }
    
    for(int i=0; i<4; i++)
    {
        x.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        y.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        z.c[i] = buffer[index++];
    }

    readFromRosData.x = x.f;
    readFromRosData.y = y.f;
    readFromRosData.z = z.f;
    readFromRosData.ctrl_mode = buffer[index++];
    readFromRosData.ctrl_flag = buffer[index++];
    readFromRosData.chassis_init = buffer[index++];
    readFromRosData.status.robot_init = (PLAYLIST)buffer[index++];
    readFromRosData.status.path_mode = (PLAYLIST)buffer[index++];
    readFromRosData.status.sensor = (PLAYLIST)buffer[index++];
    readFromRosData.status.control_mode = (PLAYLIST)buffer[index++];

    if(buffer[index++]!=serial_get_crc8_value(buffer, lenth+3))
    {
        readFromRosData.x = 0;
        readFromRosData.y = 0;
        readFromRosData.z = 0;
        readFromRosData.ctrl_mode = NORMAL;
        readFromRosData.ctrl_flag = 0;
        readFromRosData.chassis_init = false;
    }

    return 0;
}

/**
 *@brief 打包来自 ROS 的坐标数据
 *@param 从ROS收到的缓冲包
 *@return int8_t 解包成功返回0
 */
int8_t ROS::Recieve_From_ROS_Pos(uint8_t *buffer)
{
    int index = 0;
    for(int i = 0; i < 2; i++)
    {
        if(buffer[index++] != header[i])
        return 2;
    }

    lenth = buffer[index++];

    for(int i=0; i<2; i++)
    {
        if(buffer[4+lenth+i] != tail[i])
        return 1;
    }
    for(int i=0; i<4; i++)
    {
        pos_x.c[i] = buffer[index++];       
    }
    for(int i=0; i<4; i++)
    {
        pos_y.c[i] = buffer[index++];
    }
    for(int i=0; i<4; i++)
    {
        vx.c[i] = buffer[index++];
    }
    for(int i=0; i<4; i++)
    {
        vy.c[i] = buffer[index++];
    }

    //float distance = 1.5;   // 定义跟随距离为1.5米    	（先暂时这么设置跟随距离，后面再优化）
    float length = sqrt(pos_x.f * pos_x.f + pos_y.f * pos_y.f);     // 计算当前位置到原点的距离
    PID_position.target = length - DISTANCE_TO_POINT; 						// 设置PID控制器的目标值为当前位置距离目标距离的差值，target.current已经初始化为0
    float PID_position_out = PID_position.Adjust(); 				// 通过PID控制器计算输出值
		
    // 以下代码被注释掉，原本用于分别计算x和y方向的目标值
    /*
    PID_position_x.target = pos_x.f - distance * pos_x.f / length;
    PID_position_y.target = pos_y.f - distance * pos_y.f / length;
    readFromRosData.x = PID_position_x.Adjust();
    readFromRosData.y = PID_position_y.Adjust();
    */
		
    // 根据PID输出值调整目标位置的x和y坐标
    readFromRosData.x = PID_position_out * pos_x.f / length; 
    readFromRosData.y = PID_position_out * pos_y.f / length; 

    readFromRosData.relative_Vx = vx.f;
    readFromRosData.relative_Vy = vy.f;
	//控制状态
    readFromRosData.ctrl_mode = buffer[index++];
    readFromRosData.ctrl_flag = 1;     // 设定控制标志为1，表示开启控制
    if(buffer[index++]!=serial_get_crc8_value(buffer, lenth+3))
    {
        readFromRosData.x = 0;
        readFromRosData.y = 0;
        readFromRosData.relative_Vx = 0;
        readFromRosData.relative_Vy = 0;
        readFromRosData.ctrl_mode = NORMAL;
        readFromRosData.ctrl_flag = 0;
        readFromRosData.chassis_init = false;
    }


    return 0;
}

void ROS::Point_tracking_PID_Mode_Init(float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
		PID_position.PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
}

void ROS::Point_tracking_PID_Param_Init(float Kp, float Ki, float Kd, float Integral_Max, float Out_Max, float DeadZone)
{
		PID_position.PID_Param_Init(Kp,Ki,Kd,Integral_Max,Out_Max,DeadZone);
}	
