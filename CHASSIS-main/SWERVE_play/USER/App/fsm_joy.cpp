/**
 * @file fsm_joy.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括上位机控制接口的调用以及stm32手柄的调试，开关是通过宏定义来控制的(USE_ROS_CONTROL)。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "IMU.h"
#include "robot_def.h"
#include "drive_iwdg.h"

class ROS ros;


//int c;
//int8_t msg[11];
void Air_Joy_Task(void *pvParameters)
{
	//int8_t Msg[11];

    SG_TypeDef *SysGuard;
    SG_TypeDef fsm_joy_Task_IWDG;
    fsm_joy_Task_IWDG.Enable = 1;
    fsm_joy_Task_IWDG.Counter = 100;
    fsm_joy_Task_IWDG.reload_count = 100;  //100ms
    fsm_joy_Task_IWDG.errcallback = NULL;

    SysGuard =SysGuard_Reg(&fsm_joy_Task_IWDG);
    for(;;)
    {
        SysGuard_Reload(SysGuard);   //“喂狗”
#if USE_ROS_CONTROL
            ROS_Cmd_Process();
#else 
			Robot_Twist_t twist;
						if(abs(air_joy.LEFT_X-1500)<100 && abs(air_joy.LEFT_Y-1500)<100)
						{
							if(air_joy.LEFT_X>1465&&air_joy.LEFT_X<1535)
									air_joy.LEFT_X = 1500;
							if(air_joy.LEFT_Y>1465&&air_joy.LEFT_Y<1535)
									air_joy.LEFT_Y = 1500;
							if(air_joy.RIGHT_X>1465&&air_joy.RIGHT_X<1535)
									air_joy.RIGHT_X = 1500;
							if(air_joy.RIGHT_Y>1465&&air_joy.RIGHT_Y<1535)  
									air_joy.RIGHT_Y = 1500;
					    }
						
            if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
            {
                if(air_joy.SWA>1950&&air_joy.SWA<2050)      //总开关——拨杆1<SWA>   (1档位关闭 2档位开启)
                {
                        if(air_joy.SWD>950&air_joy.SWD<1050)        //手动模式——拨杆4<SWD>  (1档位)
                        {
                                if(air_joy.SWC>950&&air_joy.SWC<1050)           //普通模式——拨杆3<SWC>  (1档位)
                                {
                                        twist.chassis_mode = NORMAL;
                                }
                                else if(air_joy.SWC>1450&&air_joy.SWC<1550)     //x轴平移模式——拨杆3<SWC>  (2档位)
                                {
                                        twist.chassis_mode = X_MOVE;
                                }
                                else if(air_joy.SWC>1950&&air_joy.SWC<2050)     //y轴平移模式——拨杆3<SWC>  (3档位)
                                {
                                        twist.chassis_mode = Y_MOVE;
                                }
                                else
                                {
                                        twist.chassis_mode = NORMAL;
                                }
                                
                                /*if(air_joy.SWB>950&&air_joy.SWB<1050)
                                {
                                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);//翻转收起
                                }
                                else if(air_joy.SWB>1450&&air_joy.SWB<1550)
                                {
                                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);//翻转翻出
                                }
                                else
                                {
                                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);//翻转收起
                                }

                                if(air_joy.SWD>950&&air_joy.SWD<1050)
                                {
                                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);//爪子张开
                                }
                                else if(air_joy.SWD>1950&&air_joy.SWD<2050)
                                {
                                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);//爪子闭合
                                }*/                                                                      
                                //  <不用了先注释掉>
                                
                                twist.linear.x = (air_joy.LEFT_Y - 1500)/500.0 * 3;
                                twist.linear.y = (air_joy.LEFT_X - 1500)/500.0 * 3;
                                twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 4;
                                xQueueSend(Chassia_Port, &twist, 0);
                        }
                        else if(air_joy.SWD>1950&air_joy.SWD<2050)      //ros控制模式——拨杆4<SWD>  (2档位)
                        {
                                ROS_Cmd_Process();
                        }
                }
            }
            else
            {
							memset(&twist,0,sizeof(Robot_Twist_t));
            }

#endif
        osDelay(1);
    }
}


void Broadcast_Task(void *pvParameters)
{
    static Broadcast broadcast;
    for(;;)
    {
        Robot_Status_t status = {STOP};
        if(xQueueReceive(Broadcast_Port, &status, 0) == pdPASS)
        {
            broadcast.Send_To_Broadcast(status);
        }
        // status.robot_init = AUTO_MODE;
        broadcast.Send_To_Broadcast(status);
        osDelay(1);
    }
}


// ROS ros;
void ROS_Cmd_Process(void)
{	
    static uint32_t dt=0,now=0,last=0;;
    dt = now - last;    //ms
    UART_TxMsg Msg;
    static Robot_Twist_t twist,twist_ros;
    static Robot_Status_t status;

    static uint8_t ctrl_flag=0;
    twist.chassis_mode = NORMAL;
    if(xQueueReceive(Recieve_ROS_Port, &Msg, 0) == pdPASS)
    {
        last = now;
        ros.Recieve_From_ROS_Pos((uint8_t *)Msg.data_addr);
        twist.linear.x = ros.readFromRosData.x;
        twist.linear.y = ros.readFromRosData.y;
        twist.chassis_mode = (CHASSIS_MODE)ros.readFromRosData.ctrl_mode;
        ctrl_flag = ros.readFromRosData.ctrl_flag;

        //以后如果有需求可以设置一个标志位，来选择ros的控制类型
        /*ros.Recieve_From_ROS((uint8_t *)(Msg.data_addr));
        twist.linear.x = ros.readFromRosData.x;
        twist.linear.y = ros.readFromRosData.y;
        twist.angular.z = ros.readFromRosData.z;
        twist.chassis_mode = (CHASSIS_MODE)ros.readFromRosData.ctrl_mode;
        ctrl_flag = ros.readFromRosData.ctrl_flag;
        status.robot_init = ros.readFromRosData.status.robot_init;
        status.path_mode = ros.readFromRosData.status.path_mode;
        status.sensor = ros.readFromRosData.status.sensor;
        status.control_mode = ros.readFromRosData.status.control_mode;*/
        twist_ros = twist;
    }

    if(dt>100)
    {
        memset(&twist,0,sizeof(Robot_Twist_t));
        ctrl_flag = 0;
    }

    if(ctrl_flag == 1)
    xQueueSend(Chassia_Port, &twist_ros, 0);
    now = ros.get_systemTick()/1000;    //ms

    xQueueSend(Broadcast_Port, &status, 0);
}



void Point_tracking_PID_Init(void)
{
    ros.Point_tracking_PID_Mode_Init(POINT_TRACK_LOWPASS_ERROR, POINT_TRACK_LOWPASS_D_ERR, POINT_TRACK_D_OF_CURRENT, POINT_TRACK_IMCREATEMENT_OF_OUT);
	ros.Point_tracking_PID_Param_Init(POINT_TRACK_KP,POINT_TRACK_KI,POINT_TRACK_KD,POINT_TRACK_I_TERM_MAX,POINT_TRACK_OUT_MAX,POINT_TRACK_DEADZONE);
}
