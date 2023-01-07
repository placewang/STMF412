/***********************************************************************
传感器输入封装
浙江恒强科技
软件平台部:F11500
2022/11/17
************************************************************************/

#include "SenSor.h"
#include "gpio.h"

/*
系统ID获取
00：开环二系统 
01：闭环二系统
10：开环单系统 
11：闭环单系统
*/
unsigned int arch_GetBoardID(void)
{
	unsigned int ret = 0;
//	ret= SYSTEMID0_STATE();
//	ret |=(SYSTEMID1_STATE()<<1);
	return (ret+1);
}











