/***********************************************************************
关于CAN数据解析分包任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#include "CanApp.h"
#include "queue.h"
#include "SenSor.h"
#include "string.h"

BootCmdFlag  ProtocolCmd = {0};     //协议命令缓存体

/*
CAN数据出队--升级相关命令解析
		DCT:协议数据
Parsing task
*/

signed char CanDataOutTeamElectricMagnet(QUEUE_DATA_T *DCT)
{
        unsigned char data[8]={0};
	  //获取板子类型
		if(DCT->RxData[0]==0x08&&DCT->RxData[1]==0x02)
		{
				ProtocolCmd.CheckBoardVersiontMask =1;
		}
	   else if(DCT->RxData[0]==0x08&&DCT->RxData[1]==0x0a)
	   {
            memset((unsigned char *)data, 0x00, 8); 
			data[0]=0x08;
            data[1]=0x0a;
            data[4]=0x0a;
            CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));
	   } 
       //取机头程序版本号
 	   else if(DCT->RxData[0]==0x08&&DCT->RxData[1]==0x01)
	   {
           memset((unsigned char *)data, 0x00, 8);  
			data[0]=0x08;
            data[1]=0x01;
            data[2]=0x99;
 			data[3]=0x1d;
            data[4]=0x63;
            data[6]=0x01;          
            CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));
	   } 
 	   else if(DCT->RxData[0]==0x08&&DCT->RxData[1]==0x14&&DCT->RxData[2]==0x00)
	   {
            memset((unsigned char *)data, 0x00, 8);  
			data[0]=0x08;
            data[1]=0x14;
            data[4]=0x01;
            CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));
       }           
		//机头重启
		else if(DCT->RxData[0]==0x08&&DCT->RxData[1]<=0x06)
	  	{
				ProtocolCmd.RestartMask=1;						
		}
		//主控下发升级数据
		else if(DCT->RxData[0]==0x3b)
	  	{
			ProtocolCmd.MasterControlSendUpgradeData=1;
		}
        //主控下发数据应答
        else if(DCT->RxData[0]==0x3d)
        {
           ProtocolCmd.MasterSendACk=1; 
        }
		//主控下发最后一包数据
	   else if(DCT->RxData[0]==0x3c)
	   {
			ProtocolCmd.MasterControlSendLastPacketData =1;
	   }
	
      
       //
	return 0;
}














