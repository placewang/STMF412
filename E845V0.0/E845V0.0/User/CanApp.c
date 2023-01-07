/***********************************************************************
关于CAN数据解析分包任务
浙江恒强科技
软件平台部:F11500
2022/12/13
************************************************************************/

#include "CanApp.h"
#include "queue.h"
#include "SenSor.h"
#include "stmflash.h"
#include "string.h"
CmdFlag  ProtocolCmd = {0};     //协议命令缓存体
QUEUE_DATA_T CanCmdRevCache;    //CAN命令接收缓存

/*
CAN数据出队--电磁铁相关命令解析
		DCT:协议数据
Parsing task
*/
signed char CanDataOutTeamElectricMagnet(QUEUE_DATA_T *DCT)
{
	  //电磁铁关
		if(DCT->RxData[1]==0x01&&DCT->RxData[2]==0x00\
			&&DCT->RxData[3]<=0x07&&DCT->RxData[4]==0x00&&DCT->RxData[5]==0x00)
		{
				ProtocolCmd.ElectricMagnetMask=2;
				ProtocolCmd.ElectricMagnetNum=DCT->RxData[3];
		}
		//电磁铁开
		else if(DCT->RxData[1]==0x01&&DCT->RxData[2]==0x00\
				&&DCT->RxData[3]<=0x07&&(DCT->RxData[4]!=0x00||DCT->RxData[5]!=0x00))
	    {
				ProtocolCmd.ElectricMagnetMask=1;
				ProtocolCmd.ElectricMagnetNum=DCT->RxData[3];						
		}
		//测试某个电磁铁的阻抗值（动作）
		else if(DCT->RxData[1]==0x05&&DCT->RxData[2]==0x00\
				&&DCT->RxData[3]<=0x07)
        {
			ProtocolCmd.TestResistanceMask=1;
			ProtocolCmd.TestResistanceNum=DCT->RxData[3];
		}
		//获取某个电磁铁的阻抗值（上一次动作值）
		else if(DCT->RxData[1]==0x06&&DCT->RxData[2]==0x00\
					 &&DCT->RxData[3]<=0x07)
	  {
			ProtocolCmd.ResistanceValRequestMask=1;
			ProtocolCmd.ResistanceValRequesttNum=DCT->RxData[3];
		}
		
	return 0;
}
/*
外设命令解析
*/
signed CanDataPeripheral(QUEUE_DATA_T *pm)
{
		//DC24V 电源开
		if(pm->RxData[1]==0x01&&(pm->RxData[2]!=0x00\
					 ||pm->RxData[3]!=0x00))
		{
			ProtocolCmd.PowerDC24VSwitchMask=1;
		}
		//DC24V 电源关	
		else if(pm->RxData[1]==0x01&&pm->RxData[2]==0x00\
					 &&pm->RxData[3]==0x00)
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

/*
系统命令解析
*/
signed CanDataSystme(QUEUE_DATA_T *sy)
{
    unsigned int cmd=0;
    unsigned char data[8]={0};    
    if(sy->RxData[1]==0x02)
    {
        memset((unsigned char *)data, 0x00, 8);  
        data[0]=0x08;
        data[1]=0x02;
        data[2]=0x09;
        data[3]=0x80;
        data[4]=0x63;
        data[6]=0x01;       
        CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));
    }
    else if(sy->RxData[1]==0x0a)
    {
        memset((unsigned char *)data, 0x00, 8); 
        data[0]=0x08;
        data[1]=0x0a;
        data[4]=0x0a;
        CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));
        
    }   
     else if(sy->RxData[1]==0x01)
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
     else if(sy->RxData[1]==0x14&&sy->RxData[2]==0x00)
    {  
        memset((unsigned char *)data, 0x00, 8);  
        data[0]=0x08;
        data[1]=0x14;
        data[4]=0x01;
        CAN1_Send_Msg(data,8,(0x732+arch_GetBoardID()));        
    }
    else if(sy->RxData[1]==0x06)
    {
          cmd=CMD_UPGRADE_START;
          STMFLASH_Write(BOOT_FLASH_UPGRADE_ADDRESS ,&cmd,1);
          __set_PRIMASK(1);   //关闭所有中端
          NVIC_SystemReset(); //复位     
    }
    
    
    return 0;
}
/*
   CAN命令解析任务
*/
signed CanCmdAnalysisTask(void)
{

    if(DeQueue(&Can1_revQueuebuff,&CanCmdRevCache))
    {
        if(CanCmdRevCache.RxData[0]==CanCmdSystem)
        {    
             CanDataSystme(&CanCmdRevCache);
        }
        else if(CanCmdRevCache.RxData[0]==CanCmdElectromagnet)
        {
             CanDataOutTeamElectricMagnet(&CanCmdRevCache);
        }
        else if(CanCmdRevCache.RxData[0]==CanCmdPeripheral)
        {
             CanDataPeripheral(&CanCmdRevCache);
        }
        else if(CanCmdRevCache.RxData[0]== CanCmd_DM_Motor)
        {
             CanDataOutTeamMotor(&CanCmdRevCache);
        }
    }
    return 0;
}









