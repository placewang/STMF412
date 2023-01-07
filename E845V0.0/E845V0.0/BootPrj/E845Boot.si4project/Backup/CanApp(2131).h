/***********************************************************************
关于CAN任务声明
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#ifndef __CANAPP__H
#define __CANAPP__H


typedef struct
{
	
	unsigned char CheckBoardVersiontMask: 1;         //取机头板类型请求状态
	unsigned char RestartMask:1;                     //机头版重启标志
	unsigned char MasterControlSendUpgradeData:1;    //主控下发升级数据标志
	unsigned char MasterControlSendLastPacketData:1; //主控下发最后一包数据
		
}BootCmdFlag;
	



extern BootCmdFlag  ProtocolCmd;     //协议命令缓存体





#endif


