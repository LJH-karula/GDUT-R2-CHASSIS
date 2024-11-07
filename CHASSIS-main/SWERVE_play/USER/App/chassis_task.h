#pragma once

#include "data_pool.h"
#include "chassis.h"
#include "robot_def.h"
#include "drive_iwdg.h"


#ifdef __cplusplus
void Chassis_Pid_Init(void);
extern "C" {
#endif
void Chassis_Task(void *pvParameters);

extern Swerve_Chassis chassis;
extern Robot_Twist_t twist;
extern int a;
extern Robot_Twist_t twist1;

#ifdef __cplusplus
}
#endif
