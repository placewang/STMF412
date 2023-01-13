/***********************************************************************
关于电磁铁应用任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#include "CanApp.h"
#include "SolenoidValve.h"
#include "ElectricMagnetApp.h"
#include "queue.h"
#include "SenSor.h"
/*

电磁铁相关参数初始化

*/

void ElectricMagnetInit(void)
{
    SZ_PowerTime.OUTKeepTime=SOLENIDVALVETIMINGOPERATION;
    SZ_PowerTime.INKeepTime= SOLENIDVALVETIMINGOPERATION; 
    SZ_PowerTime.SwitchState=0xff;    
}
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
    unsigned char MsData[8]={0x05,0x06,0x00};	
	if(Ertt->TestResistanceMask==1)
	{
		Szcr->MeasureMarkPosition[Ertt->TestResistanceNum]=1;			
		Ertt->TestResistanceMask=0;
	}
	else if(Ertt->ResistanceValRequestMask==1)
	{
        MsData[3]=Ertt->ResistanceValRequesttNum;
        MsData[4]=(unsigned char)(Szcr->LastCoilResistanceValue[Ertt->ResistanceValRequesttNum]&0xff);    
        MsData[5]|=(unsigned char)Szcr->LastCoilResistanceValue[Ertt->ResistanceValRequesttNum]>>8; 
        CAN1_Send_Msg(MsData,8,arch_GetBoardID());		
		Ertt->ResistanceValRequestMask=0;
	}
		
	return 0;
}

/*
电磁阀进出任务
lp_sv:计时状态集合
*/
signed char SolenidValveInOrOutTask_loop(CmdFlag *cmd,SZPowerOnTime *lp_sv)
{
        ElectricMagnetONOrOFFTask(cmd,lp_sv);
        PowerSwitchTask(cmd);
		SolenidValveSetOut(lp_sv,0);
		SolenidValveSetOut(lp_sv,1);
		SolenidValveSetOut(lp_sv,2);	
		SolenidValveSetOut(lp_sv,3);
		SolenidValveSetOut(lp_sv,4);
		SolenidValveSetOut(lp_sv,5);
			
		SolenidValveSetIN(lp_sv,0);
		SolenidValveSetIN(lp_sv,1);
		SolenidValveSetIN(lp_sv,2);
		SolenidValveSetIN(lp_sv,3);
		SolenidValveSetIN(lp_sv,4);
		SolenidValveSetIN(lp_sv,5);	
		return 0;
}


/*
电磁阀测阻抗任务

*/

signed char CalculateCoilResistanceOfSolenoidTask_loop(CmdFlag *cmd,SZ_MeasuringResistance *cr)
{
    ElectricalResistanceTestTask(cmd,cr);
    CalculateCoilResistanceOfSolenoid(cr,0);
    CalculateCoilResistanceOfSolenoid(cr,1);    
    CalculateCoilResistanceOfSolenoid(cr,2);   
    CalculateCoilResistanceOfSolenoid(cr,3);
    CalculateCoilResistanceOfSolenoid(cr,4);    
    CalculateCoilResistanceOfSolenoid(cr,5);
   
    return  0;    
}







