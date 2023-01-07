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

#define ENCODER_SAVE_BOARD		0		//��λ�����浽�ڲ�flash
#define ENCODER_INIT_POS		1		// �ϵ��Զ�У��λ��
#define ENCODER_REAL_TIME		1		// ����ֵʵʱ��ȡ


#define ENCODER_DIR_CW			0		//AS5045 оƬ���Ϸ����� ��ʱ��
#define ENCODER_DIR_CCW			1

#define ENCODER_NUM_MAX			8		// ע��: Ŀǰ�������ͱ��������һһ��Ӧ

#define ENCODER_PPR						(4096u)		//������һȦֵ
#define ENCODER_STEPS					(800u)		//���һȦ����

#define PWM_ECODE_CHECK_DELAY_TIME_2MS	8


/* Public macro --------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/

extern int Encoder_setCheck(int eid, int chk);

extern int Encoder_setDir(int eid, int dir,int isset);

/* 
���õ������
type: 0: �ܲ���������һȦ 1�ܲ�������һȦ
*/
extern int Encoder_setType(int eid, int type);

/*
���ñ�����һȦ��Ӧ����
*/
extern int Encoder_setSteps(int eid, unsigned short steps);

/*
����λ�ü�鱨����ֵ
threshold: 2 <= x <= (ENCODER_PPR >> 2)
*/
extern int Encoder_setThreshold(int eid, unsigned short threshold);


/*
���õ�ǰ������ֵΪ��λ
*/
extern int Encoder_maskZero(int eid);

extern unsigned char Encoder_in_Get_St(short icflag,unsigned short st);

/*
���������λֵ
zero: 0-(ENCODER_PPR-1)
ע: ����ֵ��ᱣ�浽flash��
*/
extern int Encoder_setZero(int eid, int zero);

/*
�Զ����㿪��
*/
extern void Encoder_autoZero(int eid, unsigned short maxstep,unsigned char is_s_mode);

/*
������λֵ
*/
extern int Encoder_getZero(int eid);

/*
�������뵽����
*/
extern void Encoder_sendError(void);

/*

*/
extern int Encoder_step2coder(int eid, int pos);

/*
�������ض� -- ����
*/
extern int Encoder_getCoder(int eid, int *coder, int *state);

/*
����ǰ����ֵ����ɵ��λ��ֵ������Ϊ��ǰ���λ��
onoff: 0�رգ�1��
*/
extern int Encoder_rPos(int eid, int onoff);

/*
��ǰ������ֵת��Ϊ����pos
dir: [0-(steps-1)] 
return: <0��ȡʧ��
*/
extern int Encoder_getPos(int eid, int dir, int *pos);

/*
λ���Զ�����
ע��: ������Χ��Ȧ֮��
*/
extern int Encoder_adjPos(int eid);

/*
�������б������ĵ��λ��
*/
extern void Encoder_adjPosAll(void);

/*
����˶�֮ǰλ������ 
-- �˴�����Ϊ�������л������
*/
extern int Encoder_RunPos(int eid, short *pos,char whichexe);

/*
��������б��������
*/
extern int Encoder_RunCheck(int eid, int speed, short pos);


/*
�����λ��Ӧ��������λ
return: <0ʱ���������ã����Ƕ�ȡ����, 
		0 ������δ���ã�
		>0��������ȡλ������ posΪ��ǰ�������
*/
extern int Encoder_Reset(int eid, int *pos);

extern void Encoder_deInit(int eid);

extern void Encoder_Enable(int onoff);

/*
��ǰ�����Ƿ���������
*/
extern int Encoder_Work(int eid);

/*
�������ض�̽��
eid: 0xFF ��ȡ����
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

