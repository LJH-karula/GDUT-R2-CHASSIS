#include "drive_iwdg.h"

//保存所有的监视任务
static SG_TypeDef *SysGuard_instance[MX_CUT] = {NULL};
static uint8_t idx = 0;
static uint8_t HAL_IWDG_STATE_READY = 1;   //是否“喂狗”

SG_TypeDef *SysGuard_Reg(SG_TypeDef *SG_REG)
{
	//if(SG_REG->Name == NULL)
		//SG_REG->Name = "SG_NAME_UNDEF";
	if(SG_REG->errcallback == NULL)
		SG_REG->errcallback = SysGuard_Reset;

	SysGuard_instance[idx++] = SG_REG;
	return SG_REG;
}

void SysGuard_Scan(void)
{
	for(int i = 0; i < idx; i++)
	{
		if(SysGuard_instance[i]->Enable == 1 && SysGuard_instance[i]->reload_count != 0)
		{
			SysGuard_instance[i]->Counter--;
			if(SysGuard_instance[i]->Counter <= 0)
			{
				SysGuard_instance[i]->errcallback();
			}
		}
	}
}

void SysGuard_Reload(SG_TypeDef *daemon)
{
	daemon->Counter = daemon->reload_count;    //重置至重装载值
}

void SysGuard_Reset(void)
{
	HAL_IWDG_STATE_READY = 0;       
}



void IWDG_detecting_Task(void *argument)
{
	while(1)
	{
		if(HAL_IWDG_STATE_READY == 1)
			HAL_IWDG_Refresh(&hiwdg);
		SysGuard_Scan();

		osDelay(1);
	}
}
