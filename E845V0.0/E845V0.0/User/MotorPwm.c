/***********************************************************************
关于比较输出PWM操作
			脉冲/脉冲数量
			加减速
浙江恒强科技
软件平台部:F11500
2022/11/21
************************************************************************/

#include "MotorPwm.h"
#include "tim.h"

AccelerationAndDeceleration MA={0};

/*
PWM比较输出启动（比较中断）
Snum：启动通道
*/
signed char PwmTim1OutStar(unsigned Snum)
{
	unsigned int  CH1val=0,CH2val=0,CH3val=0,CH4val=0;
	switch (Snum)
	{
		case 1:
			CH1val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0xffff&(CH1val+1));
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
			break;
		case 2:
			CH2val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0xffff&(CH2val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
			break;
		case 3:
			CH3val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0xffff&(CH3val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);
			break;
		case 4:
			CH4val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0xffff&(CH4val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
			break;
		case 0:
			CH1val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0xffff&(CH1val+1));
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
			CH2val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0xffff&(CH2val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
			CH3val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0xffff&(CH3val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);
			CH4val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0xffff&(CH4val+1));			
			HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);		
			break;		
		default:
				break;		
	}
	return 0;
}

/*
PWM比较输出停止（比较中断）
num：启动通道
*/
signed char PwmTim1OutStop(unsigned Tnum)
{
	switch (Tnum)
	{
		case 1:
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
			break;
		case 2:
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			break;
		case 3:
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_3);
			break;
		case 4:
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_4);
			break;
		case 0:
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_3);
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_4);		
			break;		
		default:
				break;		
	}
	return 0;
}

/*
PWM频率设置
		计数器最大65535
MHZ：频率
MtNumber：1-4通道1-4
					0:所有通道
*/

signed PwmTim1SetFrequency(unsigned int MZH,unsigned char MtNumber)
{
	unsigned int CountVAL=0;
	unsigned int  CH1val=0,CH2val=0,CH3val=0,CH4val=0;
	float tval=0.0,tval2=0.0;
	tval=1/(float)MZH;
	tval2=tval/(float)TIM1COUNTCYCLE;			
	CountVAL=((unsigned int)tval2)/2;
	if(MZH<10)
	{
		return -1;
	}
	switch (MtNumber)
	{
		case 1:
			CH1val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0xffff&(CH1val+CountVAL));
			break;
		case 2:
			CH2val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0xffff&(CH2val+CountVAL));
			break;
		case 3:
			CH3val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0xffff&(CH3val+CountVAL));
			break;
		case 4:
			CH4val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0xffff&(CH4val+CountVAL));
			break;
		case 0:
			CH1val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0xffff&(CH1val+CountVAL));
			CH2val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0xffff&(CH2val+CountVAL));
			CH3val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0xffff&(CH3val+CountVAL));
			CH4val=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0xffff&(CH4val+CountVAL));	
			break;		
		default:
				break;		
	}

	return 0;
}
/*
加速度处理(一个比较周期速度（HZ）+1)
	Msu:加减速属性句柄
*/
signed char MotorSpeedUpCheck(AccelerationAndDeceleration*Msu)
{
	
	if(Msu->PulseTargetCount[DMMOTOR1]>0)//有新的运动目标
	{
		PwmTim1OutStar(1);                 //打开输出通道
		if(Msu->TargetSpeed==Msu->CurrentSpeed)
		{
			
		}
		if(Msu->PulseCount[DMMOTOR1]%2==0) //一个比较周期速度+1
		{
			Msu->CurrentSpeed[DMMOTOR1]++;
		}
	}

	return 0;
}	


/*7
比较中断处理函数
*/
	
signed char TIM1ComparisonInterrupt(void)
{
	if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_CC1))
	{
		MA.PulseCount[DMMOTOR1]++;
//		PwmTim1SetFrequency(MA.CurrentSpeed[DMMOTOR1],DMMOTOR1+1);
		__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_CC1);
	}
	if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_CC2))
	{
		MA.PulseCount[DMMOTOR2]++;
//		PwmTim1SetFrequency(MA.CurrentSpeed[DMMOTOR2],DMMOTOR2+1);
		PwmTim1SetFrequency(1000*10,2);
		__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_CC2);
	}
	if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_CC3))
	{
		MA.PulseCount[DMMOTOR3]++;
//		PwmTim1SetFrequency(MA.CurrentSpeed[DMMOTOR3],DMMOTOR3+1);
		PwmTim1SetFrequency(1000*10,3);
		__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_CC3);
	}
	if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_CC4))
	{
		MA.PulseCount[DMMOTOR4]++;
		PwmTim1SetFrequency(MA.CurrentSpeed[DMMOTOR4],DMMOTOR4+1);
		__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_CC4);
	}	
	return 0;
}

