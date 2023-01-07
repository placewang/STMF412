/***********************************************************************
关于CAN任务声明
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#ifndef __CANAPP__H
#define __CANAPP__H


//can命令类型
enum CANCMD
{
   CanCmdPeripheral=0x01,     //外设类型
   CanCmdSystem=0x08,         //系统
   CanCmdElectromagnet=0x05,  //电磁铁
   CanCmd_DM_Motor=0x03,      //度目电机

};

typedef struct
{
	unsigned char ResistanceValRequestMask: 1;   //电磁铁电阻值请求状态
	unsigned char TestResistanceMask:       1;	 //测试电磁铁阻抗状态	
	unsigned char SersorStateRequestMask:   2;   //传感器状态请求标志 1:单个 2：所有
	unsigned char ElectricMagnetMask:       2;   //电磁铁指令状态     1：出  2：收	
	unsigned char PowerDC24VSwitchMask :    2;   //DC24V电源开关标志  1：开  2：关	
	
	unsigned char ElectricMagnetNum;             //电磁铁操作编号
	unsigned char ResistanceValRequesttNum;      //电磁铁电阻值请求编号		
	unsigned char SersorStateRequestNum;         //要请求传感器的编号
	unsigned char TestResistanceNum;             //测试电磁铁阻抗编号
	
}CmdFlag;
	



extern CmdFlag  ProtocolCmd;     //协议命令缓存体


signed CanCmdAnalysisTask(void);


#endif


