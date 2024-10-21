#pragma once
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ROS.h"
#include "air_joy.h"
#include "Broadcast.h"

#ifdef __cplusplus

extern "C" {
#endif
void ROS_Cmd_Process(void); 
void Air_Joy_Task(void *pvParameters);
void Broadcast_Task(void *pvParameters);
extern int c;
extern int8_t msg[11];
	
void Point_tracking_PID_Init(void);

#ifdef __cplusplus
}

#endif
