/***********************************************************************
关于电磁阀的IO操作
浙江恒强科技
软件平台部:F11500
2022/11/17
************************************************************************/

#include "SolenoidValve.h"
#include "ADCVoltage.h"
#include "gpio.h"
#include "queue.h"
#include "SenSor.h"

SZPowerOnTime SZ_PowerTime={0};                         //电磁阀动作计时集合   
SZ_MeasuringResistance  CalculationOfResistance={0};    //电阻计属性算集合


void SZ_Timing_Interruption(SZPowerOnTime *szp,SZ_MeasuringResistance *cr)
{
    szp->OvercurrentTiming++;
	for(int p=0;p<SOLENIDVALVENUMBER;p++)
	{
		if(szp->InMark[p]==1||szp->OutMark[p]==1)
		{
			szp->SV[p]++;
		}
	  if(cr->OperationFlag[p]==1)
	  {
			cr->Rtime[p]++;
	  }
	}
}
/*
电磁阀开关操作
num:操作编号
flag:0 关
		 1 开
*/
signed char SolenidValveSetSwitch(SolenoidValve num,unsigned char flag)
{
	unsigned short PinNum[SOLENOIDVAVELONUMBER]={SV1_Pin,SV2_Pin,SV3_Pin,SV4_Pin,   \
                                                 SV5_Pin,SV6_Pin,SV7_Pin,SV8_Pin,   \
                                                 SV9_Pin,SV10_Pin,SV11_Pin,SV12_Pin,\
                                                 SHAZUI_POWERSWITCH_Pin	
                                                };
	
	GPIO_TypeDef* Port[SOLENOIDVAVELONUMBER]={SV1_Port,SV2_Port,SV3_Port,SV4_Port,  \
                                                SV5_Port,SV6_Port,SV7_Port,SV8_Port,  \
                                                SV9_Port,SV10_Port,SV11_Port,SV12_Port,\
                                                SHAZUI_POWERSWITCH_Port
                                            };
	if(flag==0)
	{
		HAL_GPIO_WritePin(Port[num],PinNum[num],GPIO_PIN_RESET);
	}
	else if(flag==1)
	{
		HAL_GPIO_WritePin(Port[num],PinNum[num],GPIO_PIN_SET);
	}
	else
	{
		return -1;
	}
	return 0;
}	

/*
电磁阀全出(用于开机默认状态)
*/
void SolenidValveSetAllOut(void)
{
	SolenoidValve InOutName[]={SHAZUI1_OUT,SHAZUI2_OUT,SHAZUI3_OUT,
															SHAZUI4_OUT,SHAZUI5_OUT,SHAZUI6_OUT			
														};
	for(int i=0;i<6;i++)
	{
		SolenidValveSetSwitch(InOutName[i],1);
		HAL_Delay(15);
		SolenidValveSetSwitch(InOutName[i],0);
		HAL_Delay(50);
	}
}
/*
电磁阀全进(用于开机默认状态或测试)
*/
void SolenidValveSetAllIn(void)
{
	SolenoidValve InOutName[]={SHAZUI1_IN,SHAZUI2_IN,SHAZUI3_IN,\
															SHAZUI4_IN,SHAZUI5_IN,SHAZUI6_IN			
														};
	for(int i=0;i<6;i++)
	{
		SolenidValveSetSwitch(InOutName[i],1);
		HAL_Delay(15);
		SolenidValveSetSwitch(InOutName[i],0);
		HAL_Delay(50);
	}
}

/*
单个电磁阀出
	SZU:计时集合
	Num:电磁阀编号
*/
signed SolenidValveSetOut(SZPowerOnTime *SZU,unsigned char Num)
{
		SolenoidValve InOutName[]={SHAZUI1_OUT,SHAZUI2_OUT,SHAZUI3_OUT,
															SHAZUI4_OUT,SHAZUI5_OUT,SHAZUI6_OUT			
														};
	if(SZU->ActionSwitchFlag_OUT[Num]==1)	
	{
			if(!SZU->OutMark[Num]&&!SZU->InMark[Num])
			{
				SolenidValveSetSwitch(InOutName[Num],1);
				SZU->SV[Num]=0;
				SZU->OutMark[Num]=1;
                SZU->SwitchState|=(0x01<<Num);
			}	
			else if(SZU->OutMark[Num]==1&&SZU->SV[Num]>=SZU->OUTKeepTime)
			{
				SolenidValveSetSwitch(InOutName[Num],0);
				SZU->OutMark[Num]=0;
				SZU->SV[Num]=0;
				SZU->ActionSwitchFlag_OUT[Num]=0;
			}
	}
	return 0;
}
/*
单个电磁阀进
	SZU:计时集合
	pum:电磁阀编号
*/
signed SolenidValveSetIN(SZPowerOnTime *SIN,unsigned char pum)
{
	SolenoidValve InOutName[]={SHAZUI1_IN,SHAZUI2_IN,SHAZUI3_IN,\
															SHAZUI4_IN,SHAZUI5_IN,SHAZUI6_IN			
														};
	if(SIN->ActionSwitchFlag_IN[pum]==1)	
	{
			if(!SIN->OutMark[pum]&&!SIN->InMark[pum])
			{
				SolenidValveSetSwitch(InOutName[pum],1);
				SIN->SV[pum]=0;
				SIN->InMark[pum]=1;
                SIN->SwitchState&=~(0x01<<pum);
			}	
			else if(SIN->InMark[pum]==1&&SIN->SV[pum]>=SIN->INKeepTime)
			{
				SolenidValveSetSwitch(InOutName[pum],0);
				SIN->InMark[pum]=0;
				SIN->SV[pum]=0;
				SIN->ActionSwitchFlag_IN[pum]=0;
			}
	}
	return 0;
}

/*
电磁阀线圈阻值测算
cr:电磁阀线圈电阻测试参数
cunm:电磁阀编号
*/
signed char  CalculateCoilResistanceOfSolenoid(SZ_MeasuringResistance  *cr,unsigned char cunm)
{
	SolenoidValve InOutName[]={SHAZUI1_IN,SHAZUI2_IN,SHAZUI3_IN,\
															SHAZUI4_IN,SHAZUI5_IN,SHAZUI6_IN			
														};	
	unsigned short	CoilResistanceValue=0;
	unsigned char MsData[8]={0x05,0x05,0x00};													
	if(cr->MeasureMarkPosition[cunm]==1)
	{
		if(!cr->OperationFlag[cunm])
		{
			SolenidValveSetSwitch(InOutName[cunm],1);
			cr->OperationFlag[cunm]=1;
			cr->Rtime[cunm]=0;
		}
		else if(cr->OperationFlag[cunm]==1&&cr->Rtime[cunm]>=(SZ_PowerTime.INKeepTime+SZ_SAMPINGTIME))
		{
				CoilResistanceValue=24/(CurrentSamplingClass.ValueCurrentEnd/1000);
				cr->LastCoilResistanceValue[cunm] =CoilResistanceValue;            
				cr->CurrentCoilResistanceValue[cunm]=CoilResistanceValue;
				cr->OperationFlag[cunm]=2;
		}
		else if(cr->OperationFlag[cunm]==2)
		{
			SolenidValveSetSwitch(InOutName[cunm],0);
			cr->OperationFlag[cunm]=0;
			cr->MeasureMarkPosition[cunm]=0;
			cr->Rtime[cunm]=0;
            
            MsData[3]=cunm;
            MsData[4]=(unsigned char)(cr->CurrentCoilResistanceValue[cunm]&0xff);    
            MsData[5]|=(unsigned char)cr->CurrentCoilResistanceValue[cunm]>>8; 
            CAN1_Send_Msg(MsData,8,arch_GetBoardID());
		}
	}
	return 0;
}















