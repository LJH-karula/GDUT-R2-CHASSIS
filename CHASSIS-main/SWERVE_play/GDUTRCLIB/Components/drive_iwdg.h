#ifndef __IWDG_DRIVE_H
#define __IWDG_DRIVE_H

#include "main.h"
#include "iwdg.h"
#include "cmsis_os.h"

#define MX_CUT 64

#ifdef __cplusplus
extern "C" {
#endif

/* SG_TypeDef结构体定义 */
typedef struct
{
	//const char Name;
	uint8_t Enable;  			//使能位
	int Counter;				//计数器
	uint32_t reload_count;		//重装载值
	
	void (*errcallback)(void);	//异常处理函数
} SG_TypeDef;


/**
 * @brief 注册一个监视任务
 *
 * @param SG_REG 初始化配置
 * @return SG_TypeDef * 返回实例指针
 */
SG_TypeDef *SysGuard_Reg(SG_TypeDef *SG_REG);

/**
 * @brief 监视所有已正确注册并开启的监视任务
 */
void SysGuard_Scan(void);

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载Counter值,相当于"喂狗"
 *
 * @param daemon SG_TypeDef实例指针
 */
void SysGuard_Reload(SG_TypeDef *daemon);

/**
 * @brief 检测到任务异常，可调用该函数取消“喂狗”，进行reset
 */
void SysGuard_Reset(void);

void IWDG_detecting_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_DRIVE_H */
