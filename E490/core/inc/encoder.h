/**
  *****************************************************************************
  * @file		encoder.h
  * @author	ZhuQW
  * @version	V1.0.0
  * @date    	2016-05-25
  * @brief   
  *          
  *****************************************************************************
  * @note
  * Copyright (C) 2016 Hangzhou JiaGu CNC Limited. All rights reserved. 
  *****************************************************************************
**/

#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Public typedef ------------------------------------------------------------*/
/* Public define -------------------------------------------------------------*/

#define ENCODER_SAVE_BOARD		0		//零位不保存到内部flash
#define ENCODER_INIT_POS		1		// 上电自动校正位置
#define ENCODER_REAL_TIME		1		// 编码值实时读取


#define ENCODER_DIR_CW			0		//AS5045 芯片正上方往下 逆时针
#define ENCODER_DIR_CCW			1

#define ENCODER_NUM_MAX			8		// 注意: 目前电机编码和编码器编号一一对应

#define ENCODER_PPR						(4096u)		//编码器一圈值
#define ENCODER_STEPS					(800u)		//电机一圈步数

#define PWM_ECODE_CHECK_DELAY_TIME_2MS	8


/* Public macro --------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/

extern int Encoder_setCheck(int eid, int chk);

extern int Encoder_setDir(int eid, int dir,int isset);

/* 
设置电机类型
type: 0: 总步数不超出一圈 1总步数超出一圈
*/
extern int Encoder_setType(int eid, int type);

/*
设置编码器一圈对应步数
*/
extern int Encoder_setSteps(int eid, unsigned short steps);

/*
设置位置检查报警阀值
threshold: 2 <= x <= (ENCODER_PPR >> 2)
*/
extern int Encoder_setThreshold(int eid, unsigned short threshold);


/*
设置当前编码器值为零位
*/
extern int Encoder_maskZero(int eid);

extern unsigned char Encoder_in_Get_St(short icflag,unsigned short st);

/*
设编码器零位值
zero: 0-(ENCODER_PPR-1)
注: 设置值后会保存到flash上
*/
extern int Encoder_setZero(int eid, int zero);

/*
自动找零开启
*/
extern void Encoder_autoZero(int eid, unsigned short maxstep,unsigned char is_s_mode);

/*
编码零位值
*/
extern int Encoder_getZero(int eid);

/*
发错误码到主机
*/
extern void Encoder_sendError(void);

/*

*/
extern int Encoder_step2coder(int eid, int pos);

/*
编码器回读 -- 立即
*/
extern int Encoder_getCoder(int eid, int *coder, int *state);

/*
将当前编码值换算成电机位置值，并设为当前电机位置
onoff: 0关闭；1打开
*/
extern int Encoder_rPos(int eid, int onoff);

/*
当前编码器值转换为步进pos
dir: [0-(steps-1)] 
return: <0读取失败
*/
extern int Encoder_getPos(int eid, int dir, int *pos);

/*
位置自动修正
注意: 修正范围半圈之内
*/
extern int Encoder_adjPos(int eid);

/*
调整所有报警过的电机位置
*/
extern void Encoder_adjPosAll(void);

/*
电机运动之前位置修正 
-- 此处修正为减少运行积累误差
*/
extern int Encoder_RunPos(int eid, short *pos,char whichexe);

/*
电机运行中编码器检测
*/
extern int Encoder_RunCheck(int eid, int speed, short pos);


/*
电机复位对应编码器复位
return: <0时编码器启用，但是读取错误, 
		0 编码器未启用，
		>0编码器读取位置正常 pos为当前电机步数
*/
extern int Encoder_Reset(int eid, int *pos);

extern void Encoder_deInit(int eid);

extern void Encoder_Enable(int onoff);

/*
当前编码是否正常工作
*/
extern int Encoder_Work(int eid);

/*
编码器回读探测
eid: 0xFF 读取所有
*/
extern int Encoder_Probe(int eid);

extern void Encoder_Poll(void);

extern int Encoder_Init(int nums);

extern int Encoder_Get_state(unsigned int retdata,unsigned char arg);

extern int Encoder_Set_Goback_Steps(unsigned short eid,short Steps);

extern int Encoder_Set_minsteps_rang(int eid,int Steps);
extern int Encoder_Get_Max_Steps(int eid);
extern void Encoder_Enable_mask(int onoffmask);

extern int Encode_Error_Time_Check(int eid,unsigned int alarm_ms);
extern int Encode_Stop_Time_Check(int eid,unsigned int alarm_ms);
extern int Encoder_Set_maxsteps_cw(int eid,int Steps);
extern int Encoder_rechk_over(int eid);

//extern void myprintf(const char *format, ...);

#ifdef DEBUG_MT6813_PWM

void Ecode_read_error_send(unsigned char eid,unsigned short errno);
#endif


#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */

