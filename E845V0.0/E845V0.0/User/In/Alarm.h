/***********************************************************************
关于报警处理声明
浙江恒强科技
软件平台部:F11500
2023/1/9
************************************************************************/

#ifndef __ALARM__H
#define __ALARM__H

/*
错误屏蔽位（bit0-15）			
屏蔽位	含义		对应报警号
bit0	机头板+24V保险丝失效		0xBC
bit1	机头板-24V保险丝失效		0xBD
bit2	机头板+12V保险丝失效		0xBE
bit3	探针报警（SW1）左		    0xC1
bit4	探针报警（SW2）右		    0xC2
bit5	电机1故障（电机芯片）		0xB0
bit6	电机2故障（电机芯片）		0xB1
bit7	电机3故障（电机芯片）		0xB2
bit8	电机4故障（电机芯片）		0xB3
bit9	电机5故障（电机芯片）		0xB4
bit10	电机6故障（电机芯片）		0xB5
bit11	撞针报警            		0xBB
*/

// 报警屏蔽
enum
{
    ProbeMaskBitR=0x10,
    ProbeMaskBitL=0x08,
    SZMaskBit    =0x04,
    
};


typedef struct
{
  unsigned char  TZ_StateLock:1;         //探针报警状态锁
  unsigned char  TZ_AlarmStateBit:1;     //探针状态位  0xC1	探针报警（SW2）右		参数=1--右 
  unsigned char  SZ_StateLock:2;         //纱嘴过流报警
  unsigned char  SZ_AlarmStateBit:1;     //纱嘴过流报警状态位
  unsigned char  DC12V_StateBit:1;       //12V电源短路报警
  unsigned char  DC12V_StateLock:1;      //12V电源报警锁
  unsigned char  SZ_OverloadTimeCount;   //纱嘴过流计数  
  unsigned short Mask;                   //报警处理掩码  
}Alarm;


extern Alarm AlarmSet;                


void AlarmTaskLoop(Alarm *atp);

#endif





