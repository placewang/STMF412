/***********************************************************************
关于电磁铁应用任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#include "CanApp.h"
#include "SolenoidValve.h"

/*
电磁铁电源开关操作
	PS:协议命令缓存体
*/
signed char PowerSwitchTask(CmdFlag *PS)
{
	if(PS->PowerDC24VSwitchMask==1)
	{
		SolenidValveSetSwitch(POWERSWITCH,1);		
		PS->PowerDC24VSwitchMask=0;
	}	
	else if(PS->PowerDC24VSwitchMask==2)
	{
		SolenidValveSetSwitch(POWERSWITCH,0);		
		PS->PowerDC24VSwitchMask=0;		
	}
	return 0;
}

/*
电磁铁进出操作
	Emof:协议命令缓存体
	SZpt:  纱嘴电磁铁操作体
*/
signed char ElectricMagnetONOrOFFTask(CmdFlag *Emof,SZPowerOnTime *SZpt)
{
	if(Emof->ElectricMagnetMask==1)
	{
		SZpt->ActionSwitchFlag_OUT[Emof->ElectricMagnetNum]=1;
		Emof->ElectricMagnetMask=0;
	}
	else if(Emof->ElectricMagnetMask==2)
	{
		SZpt->ActionSwitchFlag_IN[Emof->ElectricMagnetNum]=1;
		Emof->ElectricMagnetMask=0;
	}
	return 0;
}

/*
测试电磁铁电阻值
	Ertt:协议命令缓存体
	Szcr:纱嘴电磁铁测电阻操作体
*/
signed char ElectricalResistanceTestTask(CmdFlag *Ertt,SZ_MeasuringResistance *Szcr)
{
	if(Ertt->TestResistanceMask==1)
	{
		Szcr->MeasureMarkPosition[Ertt->TestResistanceNum]=1;			
		Ertt->TestResistanceMask=0;
	}
	else if(Ertt->ResistanceValRequestMask==1)
	{
		
		Ertt->ResistanceValRequestMask=0;
	}
		
	return 0;
}











