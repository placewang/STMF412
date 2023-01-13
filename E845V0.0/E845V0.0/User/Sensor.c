/***********************************************************************
传感器输入封装
浙江恒强科技
软件平台部:F11500
2022/11/17
************************************************************************/

#include "SenSor.h"
#include "gpio.h"

#include "string.h"
#include "queue.h"
MotorSersor   MotorSensorState={0};                          //电机传感器状态

/*
系统ID获取
00：开环二系统 
01：闭环二系统
10：开环单系统 
11：闭环单系统
*/
unsigned int arch_GetBoardID(void)
{
	unsigned int ret = 0;
	ret= SYSTEMID0_STATE();
	ret |=(SYSTEMID1_STATE()<<1);
    switch(ret)
    {
        case 0x03:
           ret=0x733; 
           break;
        default:
            break;
    }
	return ret;
}

/*
传感器状态轮询装载
    Ms:电机传感器状态集合
*/

void MotorSensorStateLoad(MotorSersor  *Ms)
{
    
   Ms->DM_Sensorstate  = MOTOR_B_QSJ_ZEROSTATE();
   Ms->DM_Sensorstate |= MOTOR_B_QDM_ZEROSTATE()<<1;  
   Ms->DM_Sensorstate |= MOTOR_B_HSJ_ZEROSTATE()<<2;
   Ms->DM_Sensorstate |= MOTOR_B_HDM_ZEROSTATE()<<3;        
}

/*
电机状态请求返回
    Mrro:电机传感器状态集合
    Mcmd:命令标记符
*/
signed char MotorSensorRequestReturnOperation(MotorSersor  *Mrro,CmdFlag *Mcmd)
{

    unsigned char MsData[8]={0};
    //状态翻转自动上报
    if(Mcmd->SersorStateActiveReport==1&&(Mrro->DM_LastSensorstate&0xff)!=(Mrro->DM_Sensorstate&0xff))
    {
        memset(MsData,0x00,8);
        MsData[0]=0x03;
        MsData[1]=0x02;
        MsData[3]=0xff;
        MsData[2]=0xf0;    
        MsData[2]|=Mrro->DM_Sensorstate&0xff;
        CAN1_Send_Msg(MsData,8,arch_GetBoardID()); 
        Mrro->DM_LastSensorstate=Mrro->DM_Sensorstate;
    }
    //度目电机所有零位传感器状态返回
    if(Mcmd->SersorStateRequestMask==0x02)
    {
        memset(MsData,0x00,8);
        MsData[0]=0x03;
        MsData[1]=0x02;
        MsData[3]=0xff;
        MsData[2]=0xf0; 
        MsData[2]|=Mrro->DM_Sensorstate&0xff;
        CAN1_Send_Msg(MsData,8,arch_GetBoardID());  
        Mcmd->SersorStateRequestMask=0;
    }
    //取单个度目的零位传感器状态返回
    else if(Mcmd->SersorStateRequestMask==0x01)
    {
        memset(MsData,0x00,8);
        MsData[0]=0x03;
        MsData[1]=0x02;
        MsData[2]=((Mrro->DM_Sensorstate&0xff)&1<<(Mcmd->SersorStateRequestNum))>0?1:0;
        CAN1_Send_Msg(MsData,8,arch_GetBoardID());        
        Mcmd->SersorStateRequestMask=0;
    }
    //为了兼容程序0x01类型电机的零位信号
    else if(Mcmd->SersorStateMotorZeroPos==1)
    {
        memset(MsData,0x00,8);
        MsData[0]=0x01;
        MsData[1]=0x05;
        MsData[3]=0xff;
        MsData[2]=0xf0; 
        MsData[2]|=Mrro->DM_Sensorstate&0xff;
        CAN1_Send_Msg(MsData,8,arch_GetBoardID());        
        Mcmd->SersorStateMotorZeroPos=0;
    }
    return 0;
}    


