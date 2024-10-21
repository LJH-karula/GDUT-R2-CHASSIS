/**
 * @file robot_def.h
 * @author LJH
 * @brief 参数配置文件，便于后续调试工作
 * @version 0.1
 * @date 2024-10-21
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

//使用ROS控制舵轮底盘
#define USE_ROS_CONTROL 0

//舵向偏移量 （+为顺时针，-为逆时针）
#define WHEEL1_OFFSET 75     //右前
#define WHEEL2_OFFSET 75     //左前
#define WHEEL3_OFFSET 120    //左后
#define WHEEL4_OFFSET 88     //右后

//舵轮底盘最大速度
#define SWERVE_MAX_SPEED_X 3    //x方向最大速度
#define SWERVE_MAX_SPEED_Y 3    //y方向最大速度    
#define SWERVE_MAX_SPEED_Z 4    //最大角速度
#define SWERVE_MAX_ACCEL_VEL 1.5    //加速度

//点追踪模式参数
#define DISTANCE_TO_POINT 1.5f   //距离目标点的距离阈值

#define POINT_TRACK_LOWPASS_ERROR 0.8   //误差低通滤波器截止频率
#define POINT_TRACK_LOWPASS_D_ERR 1     //微分项低通滤波器截止频率,不完全微分，1为不启用
#define POINT_TRACK_D_OF_CURRENT true   //是否启用微分先行
#define POINT_TRACK_IMCREATEMENT_OF_OUT false   //true:输出增量模式，false:输出位置模式。
#define POINT_TRACK_KP 5.0              //KP
#define POINT_TRACK_KI 0.0              //KI
#define POINT_TRACK_KD 10.0             //KD
#define POINT_TRACK_I_TERM_MAX 400      //I项限幅
#define POINT_TRACK_OUT_MAX 3.0         //输出限幅
#define POINT_TRACK_DEADZONE 0.1        //死区   



#endif // ROBOT_DEF_H(header guard)
