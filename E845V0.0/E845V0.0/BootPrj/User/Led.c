/***********************************************************************
关于LED系统指示灯操作
浙江恒强科技
软件平台部:F11500
2022/11/15
************************************************************************/

#include "Led.h"
#include "iwdg.h"

unsigned int 		ledTimeCount = 0;

/*
看门狗刷新
*/
void IwdgRefresh(void)
{
	 __HAL_IWDG_RELOAD_COUNTER(&hiwdg);	
}

/*
系统指示灯翻转
	带看门狗刷新
*/
void LedTask(unsigned short time)
{
		if(ledTimeCount >= time)
		{			
			LED1_TOGGLE();
			IwdgRefresh();
			ledTimeCount=0;
		}	
}


