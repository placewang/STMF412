/***********************************************************************
关于电磁铁应用任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/
#ifndef __ELECTRICMAGNETAPP__H
#define __ELECTRICMAGNETAPP__H
#include "SolenoidValve.h"
#include "CanApp.h"



signed char SolenidValveInOrOutTask_loop(CmdFlag *cmd,SZPowerOnTime *lp_sv);
signed char CalculateCoilResistanceOfSolenoidTask_loop(CmdFlag *cmd,SZ_MeasuringResistance *cr);
signed char PowerSwitchTask(CmdFlag *PS);




#endif



