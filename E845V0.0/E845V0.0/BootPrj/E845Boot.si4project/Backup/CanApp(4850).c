/***********************************************************************
关于CAN数据解析分包任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#include "CanApp.h"
#include "queue.h"


CmdFlag  ProtocolCmd = {0};     //协议命令缓存体

/*
CAN数据出队--电磁铁相关命令解析
		DCT:协议数据
Parsing task
*/

signed char CanDataOutTeamElectricMagnet(QUEUE_DATA_T *DCT)
{
	  //电磁铁关
		if(DCT->RxData[1]==0x00&&DCT->RxData[2]<=0x07 \
			&&DCT->RxData[3]==0x00&&DCT->RxData[4]==0x00)
		{
				ProtocolCmd.ElectricMagnetMask=2;
				ProtocolCmd.ElectricMagnetNum=DCT->RxData[2];
		}
		//电磁铁开
		else if(DCT->RxData[1]==0x00&&DCT->RxData[2]<=0x07\
					 &&(DCT->RxData[3]!=0x00||DCT->RxData[4]!=0x00))
	  {
				ProtocolCmd.ElectricMagnetMask=1;
				ProtocolCmd.ElectricMagnetNum=DCT->RxData[2];						
		}
		//测试某个电磁铁的阻抗值（动作）
		else if(DCT->RxData[1]==0x05&&DCT->RxData[2]<=0x07\
					 &&(DCT->RxData[3]!=0x00||DCT->RxData[4]!=0x00))
	  {
			ProtocolCmd.TestResistanceMask=1;
			ProtocolCmd.TestResistanceNum=DCT->RxData[2];
		}
		//获取某个电磁铁的阻抗值（上一次动作值）
		else if(DCT->RxData[1]==0x06&&DCT->RxData[2]<=0x07\
					 &&(DCT->RxData[3]!=0x00||DCT->RxData[4]!=0x00))
	  {
			ProtocolCmd.ResistanceValRequestMask=1;
			ProtocolCmd.ResistanceValRequesttNum=DCT->RxData[2];
		}
		//DC24V 电源开
		else if(DCT->RxData[1]==0x01&&(DCT->RxData[2]!=0x00\
					 ||DCT->RxData[3]!=0x00))
		{
			ProtocolCmd.PowerDC24VSwitchMask=1;
		}
		//DC24V 电源关	
		else if(DCT->RxData[1]==0x01&&DCT->RxData[2]==0x00\
					 &&DCT->RxData[3]==0x00)
		{
			ProtocolCmd.PowerDC24VSwitchMask=2;
		}		
	return 0;
}

/*
CAN数据出队--电机相关命令解析
		Mr:协议数据
CAN协议 主控-->机头板 16进制 0x01+0x05+0x02  打开主动上报  
														 0x01+0x05+0x03  关闭主动上报
*/
signed char CanDataOutTeamMotor(QUEUE_DATA_T *Mr)
{
		unsigned short  sensornumber=0; 
  	//取单个度目的零位传感器状态
		if(Mr->RxData[1]==0x02&&Mr->RxData[2]==0x00\
			&&Mr->RxData[3]==0x01)
		{
			sensornumber =Mr->RxData[4];
			sensornumber|=Mr->RxData[5]<<8;
			if(sensornumber<=0x0f)
			{
				ProtocolCmd.SersorStateRequestMask=1;
				ProtocolCmd.SersorStateRequestNum=sensornumber;
			}
		}
		//取所有度目的零位传感器状态
		else if(Mr->RxData[1]==0x02&&Mr->RxData[2]==0x00\
					 &&Mr->RxData[3]==0x00)
	  {
			ProtocolCmd.SersorStateRequestMask=2;
		}

	return 0;
}













