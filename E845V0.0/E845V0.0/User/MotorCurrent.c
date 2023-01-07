/***********************************************************************
关于步进电机的电流控制接口定义
浙江恒强科技
软件平台部:F11500
2022/11/18
**********************************************************************/

#include "MotorCurrent.h"
#include "tim.h"

/*
启动PWM通道（1-4）
num:1-4:要打开通道
		 0:打开所有
*/
signed char StarTime2PwmCH(unsigned char num)
{
	switch (num)
	{
		case 1:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			break;
		case 2:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			break;
		case 3:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			break;
		case 4:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			break;	
		case 0:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			break;
		default:
				break;		
	}
	return 0;
}

/*
修改PWM通道占空比
PWMCH：1-4：通道编号
			　０：设置全部通道
dutyratio:0-100占空比 
*/
signed char  SetTime2DutyRatio(unsigned char PWMCH,float dutyratio)
{
	unsigned short PulseVal=0;
	PulseVal=(unsigned short)(dutyratio*TIM2COUNTVAL);
	switch (PWMCH)
	{
		case 1:
			TIM2->CCR1=PulseVal;
			break;
		case 2:
			TIM2->CCR2=PulseVal;
			break;
		case 3:
			TIM2->CCR3=PulseVal;
			break;	
		case 4:
			TIM2->CCR4=PulseVal;
			break;
		case 0:
			TIM2->CCR1=PulseVal;
			TIM2->CCR2=PulseVal;
			TIM2->CCR3=PulseVal;
			TIM2->CCR4=PulseVal;	
		default:
				break;		
	}
	return 0;
}
/*
电流设置
		Mnum:1-4电机编号
					0所有电机
		Cval:电流值单位mA	占空比3%误差
参考DRV8818 I=Vrefx/(8*R)
*/
signed char MotorCurrentSet(unsigned char Munm,unsigned short Cval)
{
	double Vrefx=0.0,Dys=0.0;

	if(Cval>2000)
	{
		return -1;
	}
	Dys=(float)Cval/1000;
	Vrefx=(0.86*Dys*1.9)/3.27;

	switch (Munm)
	{
		case 1:	
				SetTime2DutyRatio(Munm,Vrefx);
		case 2:	
				SetTime2DutyRatio(Munm,Vrefx);
		case 3:	
				SetTime2DutyRatio(Munm,Vrefx);
		case 4:	
				SetTime2DutyRatio(Munm,Vrefx);
		case 0:	
				SetTime2DutyRatio(Munm,Vrefx);
		default:
				break;		
	}
	return 0;
}
	

