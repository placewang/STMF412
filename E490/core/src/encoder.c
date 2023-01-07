/**
  *****************************************************************************
  * @file		encoder.c
  * @author	ZhuQW
  * @version	V1.0.0
  * @date    2016-05-25
  * @brief   
  *          
  *****************************************************************************
  * @note
  * Copyright (C) 2016 Hangzhou JiaGu CNC Limited. All rights reserved. 
  *****************************************************************************
**/
  
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "arch.h"
#include "command.h"
#include "alert.h"
#include "config.h"
#include "step.h"
#include "platform_config.h"

#include "encoder.h"
#include "massage.h"
#include "type.h"
#include "eeprom.h"
//#include "cortexm3_macro.h"


/* Private typedef -----------------------------------------------------------*/


enum {
	EAUTO_ZERO_FINISH = 0,
	EAUTO_ZERO_START,
	EAUTO_ZERO_GOTO_CW_MAX,
	EAUTO_ZERO_GOTO_CCW_MAX,
	EAUTO_ZERO_CALC_CHECK,
	EAUTO_ZERO_EXEC_RESET,
};


/* ���浽flash�Ľṹ�� */
typedef struct {
	unsigned short encoder_init;
	unsigned short encoder_zero[ENCODER_NUM_MAX];
	unsigned short rsvd[6];
	unsigned short Ecode_CRC;
} ENCODER_DEF;

struct ENC_RUN_BITS {
   uint16_t rzero		:1;		// 0: ��ǵ�ǰ���λ��Ϊ��λ
   uint16_t rchk		:1;		// 1: λ�ü��
   uint16_t rpos		:1;		// 2: ������ֵת��Ϊpos
   uint16_t rrst		:1;		// 3: ��λ
   uint16_t radj		:1;		// λ��������
   uint16_t raut		:4;		// �Զ�����״̬
   uint16_t rlast		:1;		/*��һ���ڵ�״̬0-ֹͣ��1--����*/
   uint16_t rgetecord	:1;		/*�Ƿ��ȡ���µı�����ֵ��0--�ǵģ�1--û��*/	
   uint16_t rdo_2_adj	:4;		/*��һ�η���ʧ���ˣ���ô����������*/
   uint16_t isredo		:1;		// reserved
};

union ENC_RUN_REG {
   uint16_t	all;
   struct ENC_RUN_BITS bit;
};

struct ENC_STA_BITS {
	uint16_t son				:1; 	// ����ʹ��
	uint16_t init				:1;		// ��һ���ϵ�� 1-��λֵΪ��Ч
	uint16_t sdir				:1;		// ��������������
	uint16_t stat				:1;		// ����ֵ��ȡ״̬�������
	uint16_t alert				:1;		// ����״̬
	uint16_t stype				:1;		// ����Ƿ��и����г� --0ֻ�������� 1����������
	uint16_t runchk			:1;		// ������м��λ��
	uint16_t dir_is_set			:1;			/*dir is set*/
	uint16_t autozero_mode	:1;			/**/
	uint16_t delayok			:1;		/*��ʱOK*/
	uint16_t rsvd				:6;
};

union ENC_STA_REG {
   uint16_t	all;
   struct ENC_STA_BITS bit;
};

typedef struct {
	union ENC_RUN_REG run;
	union ENC_STA_REG sta;

	short max_pos_N;				/*������������г�*/

	//#ifdef ENCODER_IC_IS_5045OR8800
	short coder_IC_flag;/*0--δʶ��>0��ʾ5045;   <0 ��ʾ8800*/
	//short rreett;
	//#endif
	
	uint16_t steps_ppr;				//һȦ����ܲ���
	uint16_t coder_zero;			// ��λ����ֵ -- ע����Ҫ��ȷ���÷���
	unsigned short coder_max_alert;	//������Χ -- ƫ��������ֵ
	unsigned short step_max_alert;	//������Χ -- ƫ������� -- Ϊ�˼�������
	
	uint16_t coder_max_ccw;			// �Զ����㷴�����������ֵ
	uint16_t coder_max_cw;			// �Զ������������������ֵ
	uint16_t steps_max_auto;		// �Զ�����ʱ�����
	uint16_t steps_min_auto;		//�Զ�����ʱ����С����
	uint16_t last_coder;			//���һ�ζ�ȡ����������ֵ
	unsigned short last_status;		//���һ�ζ�ȡ������״̬�Ĵ�
	unsigned short last_st_ex;	//���һ�ζ�ȡ�������ɹ���־
	
	unsigned short check_delay;
	unsigned short alert_count;		// ƫ��޼���
	
	short zero_goback_code;		// �Զ�����֮��������ٲ� ���������λ
	unsigned short steps_max_auto_get;		// �Զ������ʱ���Զ����Ե����г�ֵ
		
	unsigned short step_idx;			/* ������*/
	unsigned int 	first_error_time;		/*��һ�λ�ȡ������״̬��ʱ��(250usΪ��λ)*/
	unsigned int 	first_stop_time;		/*��һ�λ�ȡ�����ͣ�µ�ʱ��(250usΪ��λ)*/

	unsigned short ECODE_first_code;	/*���ͣ��֮���һ�ζ�ȡ����ֵ*/
	unsigned short ECODE_read_cnt;	/*���ͣ��֮���ȡ�Ĵ���*/
	unsigned short ECODE_read_err_cnt;	/*���ͣ��֮���ȡ���쳣����*/
	unsigned short ECODE_read_continue_err_cnt;	/*���ͣ��֮���ȡ�������쳣����*/
	unsigned short ECODE_read_continue_max_err_cnt; /*���ͣ��֮���ȡ�������쳣������*/
	unsigned short ECODE_max_code;	/*���ͣ��֮���ȡ�������ֵ*/
	unsigned short ECODE_min_code;	/*���ͣ��֮���ȡ������Сֵ*/
	unsigned short ECODE_last_code;	/*���ͣ��֮�����һ�ζ�ȡ����ֵ*/
		
	uint16_t	steps_ppr_cw;			/*�������ֵ�����ڸ�ֵ��Ϊ����,Ĭ��ֵΪ800��һ��*/
	//unsigned int 	check_timeout;   	/*�����ͣ��֮����ô��ʱ�䣬�����������*/
	unsigned short timeout_alert;		
}ENCODER_t;

/* Private define ------------------------------------------------------------*/
//#define ENCODER_CHECK_DELAY_RUN5		5

#define ENCODER_CHECK_DELAY_RUN5		25

#define ENCODER_CHECK_DELAY_RUN		25
#define ENCODER_CHECK_DELAY_RST		50
#define ENCODER_CHECK_INTERVAL		10
#define ENCODER_HW_ERROR_DELAY		100

#define ENCODER_AUTO_STEP_RANGE		(60)	//�Զ�����ʱ��Ӧ���г̵Ĳ�ֵ����


#define ENCODER_STATE_ERRPR_ALARM_DELAY (30000)

#define ALM_ENC_SSI 	(0x0)	// ������Ӳ����ȡֵ���� -- ����3�ζ�ȡ����
#define ALM_ENC_RST		(0x1)	// ��λʧ��
#define ALM_ENC_POS		(0x2)
#define ALM_ENC_POS_IS		(0x3)
#define ALM_ENC_POS_NOT		(0x4)
#define ALM_ENC_REDO			(0x05)
#define ALM_ENC_REDOSET			(0x06)

#ifdef DEBUG_MT6813_PWM
#define ALM_ENC_READERR		(0x08)

#endif




/* Private macro -------------------------------------------------------------*/


//#define ENCODER_DEBUG
#ifdef ENCODER_DEBUG
#define DEBUG_ENCODER(...) 			myprintf(__VA_ARGS__) 
#else
#define DEBUG_ENCODER(args...)
#endif

#define MSG_ENCODER(...)			myprintf(__VA_ARGS__) 

/* Private variables ---------------------------------------------------------*/

extern STEP_TYPE STEPMOTOR[STEP_NUM/*10*/];

ENCODER_t g_encoder[ENCODER_NUM_MAX];

static unsigned int encoder_time_stamp[ENCODER_NUM_MAX];

#ifdef QL_DEBUG_STEP_MOTOR_ECORD
unsigned char ecord_index=0;

unsigned int ecord_read_temp[200];
#endif

//int coder_read_fifo_cnt=0;

static unsigned int coder_err;

static unsigned char encoder_nums;		//������ʹ���ܸ���

/* Private function prototypes -----------------------------------------------*/

extern int SSI_Read(int idx,short *IC_flag, int *coder, int *status);
extern int myprintf(const char *format, ...);

/* Private functions ---------------------------------------------------------*/

static void EMsg_autoZero(int eid, int ret)
{
	//MSG_ENCODER("\r\n[%d]E-autoZero ret[%d]\r\n", eid, ret);
	Message_Send_4halfword(((CMD_ENCODER_ZERO_MASK << 8) | CMDTYPE_PERIPHERAL), ret, eid, 0);
}

static void EReport_Error(int eid, uint8_t type, unsigned short error)
{
	int arg=0;
	coder_err = ALERT_CODE_ENCODE;

	#ifdef NEW_ALARM_STYLE
	alert_push(ENCODE_ERR_CODE_ARG((eid<<8)|(type)));
	#else

	arg = type;
	if (type==ALM_ENC_SSI)
	{
		arg |=(error<<8); 
	}
	if (type==ALM_ENC_POS)
	{
		arg = (arg<<12) | (error & 0xFFF);
		//arg |=(error<<8); 
	}

	#ifdef DEBUG_MT6813_PWM
	if (type==ALM_ENC_READERR)
	{
		arg = (arg<<12) | (error & 0xFFF);
		//arg |=(error<<8); 
	}
	#endif
	alert_push((coder_err | (eid << 8)), arg /*| (error << 8)*/);

	#endif
	
	//DEBUG_ENCODER("[%d]E-Error[0x%x %d]\r\n", eid, type, error);
}

/*
������ֵ��ȡͳһ�ӿ�
*/
static int ERead_Coder(int eid, ENCODER_t *penc)
{
	int ret=-1;
	int err_alert;
	unsigned short lastcode;
	extern int SSI_Error(int idx);
	//unsigned short lastst;
	
	
#if ENCODER_DMA_SUPPORT
#ifdef  ENCODER_IC_IS_5045OR8800
	ret = SSI_Read(eid,(short *)&penc->coder_IC_flag, (int *)&lastcode, (int *)&penc->last_status);
#else
	ret = SSI_Read(eid,NULL, (int *)&lastcode, (int *)&penc->last_status);
#endif
	penc->last_st_ex = ret?1:0;
	if (ret == 0)
	{
		penc->last_coder = lastcode;
		penc->sta.bit.stat = 1;
		encoder_time_stamp[eid] = arch_get_ticktime();

		if (penc->sta.bit.sdir == ENCODER_DIR_CCW) //������� -- ���ֱ�������������͵��������ͬ
		{
			penc->last_coder = (ENCODER_PPR - 1 - penc->last_coder);
		}
	}
	else
	{
		penc->sta.bit.stat = 0;
		if (SSI_Error(eid) > 100) // �������߿��ܱ��γ�
		{
			if (penc->sta.bit.alert == 0)
			{
				penc->sta.bit.alert = 1;
				EReport_Error(eid, ALM_ENC_SSI, Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status));
				//EReport_Error(eid, ALM_ENC_SSI, (uint8_t)ret);
			}
		}
	}
#else
	do {
		if (penc->sta.bit.son!=1) 
			break;
		//ͣ�µ�����¼�¼��ȡ����	
		if (penc->run.bit.rlast==0)
			penc->ECODE_read_cnt++;


		#ifdef ENCODER_IC_IS_5045OR8800
		ret = SSI_Read(eid,(short *)&penc->coder_IC_flag, (int *)&lastcode, (int *)&penc->last_status);
		#else
		ret = SSI_Read(eid,NULL, (int *)&lastcode, (int *)&penc->last_status);
		#endif
		//penc->last_coder = lastcode;
		//penc->rreett = ret;
		penc->last_st_ex = ret?1:0;
		if (ret == 0)
		{
			penc->last_coder = lastcode;
			//penc->last_status =lastst;
			if (penc->first_error_time)
				penc->first_error_time =0;
			penc->sta.bit.stat = 1;
			encoder_time_stamp[eid] = arch_get_ticktime();

			if (penc->sta.bit.sdir == ENCODER_DIR_CCW) //������� -- ���ֱ�������������͵��������ͬ
			{
				penc->last_coder = (ENCODER_PPR - 1 - penc->last_coder);
			}
			#ifdef QL_DEBUG_STEP_MOTOR_ECORD
			if (penc->step_idx==0)
			{
				if (ecord_index<200)
					//ecord_index =0;
				ecord_read_temp[ecord_index++] = penc->last_coder;
			}
			#endif


			if (penc->run.bit.rlast==0)
			{
				if (penc->ECODE_first_code==0xFFFF)
					penc->ECODE_first_code = penc->last_coder;	

				if (penc->ECODE_max_code<penc->last_coder)
				{
					penc->ECODE_max_code = penc->last_coder;
				}
				if (penc->ECODE_min_code>penc->last_coder)
				{
					penc->ECODE_min_code=penc->last_coder;
				}

				if (penc->ECODE_read_continue_err_cnt)
				{
					if (penc->ECODE_read_continue_max_err_cnt<penc->ECODE_read_continue_err_cnt)
					{
						penc->ECODE_read_continue_max_err_cnt = penc->ECODE_read_continue_err_cnt;
					}
					penc->ECODE_read_continue_err_cnt =0;
	
				}
				
			}

			Set_step_alert_st_ecoder(penc->step_idx,0);
			
			break;     /*��ȷ�Ļ��ͷ�����*/
		}
		else
		{
			if (penc->sta.bit.stat)  /*��һ�γ����쳣,��¼��ǰʱ��*/
			{
				penc->first_error_time =arch_get_ticktime();
				err_alert = 0;
			}
			else			
				err_alert = Encode_Error_Time_Check(eid,1000);

			if (penc->run.bit.rlast==0)
			{	
				penc->ECODE_read_err_cnt++;
				penc->ECODE_read_continue_err_cnt++;
			}

			
		}
		
		penc->sta.bit.stat = 0;

		Set_step_alert_st_ecoder(penc->step_idx,1);		/*���쳣�ˡ���Ҫ�����쳣*/
		if (Encode_Stop_Time_Check(eid,5000)&&(penc->run.bit.rlast==0))
		{
			/*˵��ͣ������5�����ˣ��ǾͲ�������*/
			break;
		}
		
		if (((SSI_Error(eid) > ENCODER_STATE_ERRPR_ALARM_DELAY)&&(0==1))||(err_alert))/*20161227������*/
		{
			penc->check_delay = ENCODER_HW_ERROR_DELAY;
			penc->first_error_time =arch_get_ticktime();
			
			if (penc->sta.bit.alert == 0)
			{
				penc->sta.bit.alert = 1;				
				EReport_Error(eid, ALM_ENC_SSI, Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status));
			}
			break;
		}
		//break;
	}while (0);
#endif /* ENCODER_DMA_SUPPORT */
	return ret;
}

/*
����newcoder  �� oldcoder �Ĳ�ֵ
return: < 0 newcoder��oldcoder��CCW��; > 0 ΪCW��
*/
static int ECalc_Diff( int oldcoder, int newcoder)
{
	int diff;
	
	diff = newcoder - oldcoder;
	if (diff == 0)
		return 0;
	
	if (abs(diff) > (ENCODER_PPR >> 1))
	{
		if (diff < 0)
			diff += ENCODER_PPR;
		else
			diff -= ENCODER_PPR;
	}
	
	return diff;
}

/*

*/
static int ECalc_coder2step(int eid, int dir, int coder, int base)
{
	long enc_diff;
	long pos, steps;
	
	steps = g_encoder[eid].steps_ppr;
	
	enc_diff = ENCODER_PPR;
	if (dir == ENCODER_DIR_CCW)
	{
		enc_diff += (coder - base);
	}
	else
	{
		enc_diff += (base - coder);
	}
	
	if (enc_diff >= ENCODER_PPR)
		enc_diff -= ENCODER_PPR;

	pos = enc_diff * steps / (long)ENCODER_PPR;
	
	return (int)pos;
}

/*
��ǰ������λ�ü�����λ��ֵ
*/

#ifdef YF_

#define ACT_STEP_AREA_CUSTOM

#endif

static int Step_after_do_report(ENCODER_t *penc)
{
	StepMotor_afterrun_toreport(penc->step_idx,0);
	return 0;
}

#ifdef PWM_ECODE_ERROR_AUTO_ADJ

static int Step_motor_redo_2_adj(ENCODER_t *penc)
{
	if (penc->run.bit.isredo)
		penc->run.bit.rdo_2_adj++;

	//Message_send_log_ecode(penc->step_idx|(0x00<<8),penc->run.bit.rdo_2_adj,penc->run.bit.rrst);
	StepMotor_setpos_2main_postion(penc->step_idx);
	StepMotor_exec(penc->step_idx,StepMotor_Get_Position_2(penc->step_idx),1,0,0);
	//StepMotor_Set_waittime(penc->step_idx,8);
	return 0;
}


static int Step_motor_redo_reset_2_adj(ENCODER_t *penc)
{
if (penc->run.bit.isredo)
		penc->run.bit.rdo_2_adj++;
	//Message_send_log_ecode(penc->step_idx|(0x00<<8),penc->run.bit.rdo_2_adj,penc->run.bit.rrst);	
	StepMotor_Reset(penc->step_idx,0);
}

#endif
static int ECalc_rPos(int eid, ENCODER_t *penc)
{
	int pos;
	//short maxppos=70;
	pos = ECalc_coder2step(eid, ENCODER_DIR_CCW, penc->last_coder, penc->coder_zero);
	if (penc->sta.bit.stype == 0)
	{
		//maxppos = penc->sta.bit.autozero_mode?200:70;		/*hlc 20191026*/
		if (pos > (penc->steps_ppr_cw)) //��Ŀ������λ�õ�720 
		{
			pos -= penc->steps_ppr;
		}
	}
	else
	{
		#ifdef ACT_STEP_AREA_CUSTOM
		if (pos > (penc->steps_ppr_cw))
		#else
		if (pos > (penc->steps_ppr_cw))	//ʵ��λ�ò�����ڰ�Ȧ��λ��/*  �����Χ��-480---310 */
		#endif
		{
			pos -= penc->steps_ppr;
		}
	}

	return pos;
}

/*
����ֵ��Ӧλ�ü��
������󱨾�������
*/
static int ECheck_Pos(int eid, int pos, int time, ENCODER_t *penc)
{
	long iencoder;
	long steps;
	int errno;
	//short epos;

	if (penc->run.bit.rchk==0)
	{
		return 0;
	}
	//Message_Send_4halfword(0x8899,eid,penc->last_coder,pos);	
	
	steps = penc->steps_ppr;

	if (Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status))
		return 0;
	

	iencoder = (long)pos * (long)ENCODER_PPR / steps;
	iencoder += penc->coder_zero;
	iencoder %= ENCODER_PPR;
	if (iencoder < 0)
	{
		iencoder += ENCODER_PPR;
	}
	errno = ECalc_Diff(iencoder, penc->last_coder);
	errno = abs(errno);

	if (penc->coder_max_alert 
		&& errno > penc->coder_max_alert)
	{
#ifdef ENCODER_DEBUG
		//dbgu_motor_clr(eid);
#endif /* ENCODER_DEBUG */
		
		//DEBUG_ENCODER("[%d]E-Chk[mpos %4d][coder %4d][diff %4d]\r\n", eid, pos, penc->last_coder, errno);
		penc->alert_count++;
		if (penc->alert_count < time)
		{
			return 1;
		}
		
		penc->sta.bit.alert = 1;		
		{
		#ifdef PWM_ECODE_ERROR_AUTO_ADJ
		#ifdef ECODE_USE_MT6813_PWM
		//	if ((!Check_is_MT6813PWM_Mode())&&(penc->run.bit.rdo_2_adj<2))			//�Զ���������
		//#else
			if (penc->run.bit.rdo_2_adj<2)			//�Զ���������
		#endif
			{
				//penc->run.bit.rdo_2_adj =1;
				penc->run.bit.isredo=1;
				penc->sta.bit.alert = 0;
				//Message_send_log_ecode(penc->step_idx|(0xFF<<8),errno,pos);
				Message_Send_4halfword(0xFDDD,eid,pos,penc->last_coder);
				if(Check_is_MT6813PWM_Mode() )/*�ٶ�һ��*/
				{
					if(penc->last_coder==0)
					{
					//EReport_Error(eid, ALM_ENC_POS, (unsigned short)errno);
						penc->run.bit.rdo_2_adj++;
						penc->timeout_alert =PWM_ECODE_CHECK_DELAY_TIME_2MS;
						penc->sta.bit.delayok=0;
					}
					else
					{
						goto do_readj_aut;	
					}
				}
				else
				{
					do_readj_aut:
				
					if (penc->run.bit.rrst)
					{
						Step_motor_redo_reset_2_adj(penc);
					}
					else
						Step_motor_redo_2_adj(penc);
				//penc->run.bit.rdo_2_adj =2;
				}
				
			}else
			{
				alertecodeerr:
				penc->run.bit.rdo_2_adj =0;
				penc->run.bit.isredo =0;
				//EReport_Error(eid, penc->run.bit.rrst?ALM_ENC_REDOSET:ALM_ENC_REDO, (unsigned short)errno);
				EReport_Error(eid, penc->run.bit.rrst?ALM_ENC_RST:ALM_ENC_POS, (unsigned short)errno);
				//penc->sta.bit.alert = 0;
				
				Message_Send_4halfword(0xFFDD,eid,pos,penc->last_coder);
				penc->run.bit.rchk = 0;

			}
		#else
			if (penc->run.bit.rrst)
			{
				EReport_Error(eid, ALM_ENC_RST, 0);
			}
			else
			EReport_Error(eid, ALM_ENC_POS, (unsigned short)errno);
			penc->run.bit.rchk = 0;
		#endif
		}
	}
	else
	{
		if (penc->run.bit.isredo)
		{
			penc->run.bit.rdo_2_adj =0;
			penc->run.bit.isredo =0;
		}
		Step_after_do_report(penc);
		penc->run.bit.rchk = 0;
	}
	#if 0 //QL_ECODE_ERROR_AUTO_ADJ
	else
	{	
		if (penc->run.bit.rdo_2_adj)
			penc->run.bit.rdo_2_adj =0;
	}
	#endif
	#if 0
	else if (errno > 6/* 2�� */ && penc->run.bit.rchk)
	{
		/* ���λ�ü��δ����ֵ��������ֹͣ���Զ��������λ�� -- ����������� */
		//penc->run.bit.rpos = 1;
		epos = ECalc_rPos(eid, penc);
		errno = abs(pos - epos);
		if (errno < (penc->steps_ppr >> 3))	//����λ�����Խ��Ͳ�����λ�� -- ���ܴ˴�û�б�Ҫ
		{
			StepMotor_Modfiy_Position(eid, epos, 1);
		}
	}
	#endif
	
	penc->alert_count = 0;
	if (penc->run.bit.isredo==0)
		penc->run.bit.rrst = 0;
	
	penc->run.bit.radj = 0;
	return 0;
}

#if ENCODER_SAVE_BOARD
static void ELoad_Cfg(void)
{
	int ret;
	short i;
	ENCODER_DEF ENC_def;
	unsigned short *p = (unsigned short *)&ENC_def;
	unsigned short Ecode_CRC = 0;
	short len = sizeof(ENC_def) / sizeof(short);
	
	
	ret = EE_Read(10, (unsigned short*)&ENC_def, len);
	if (ret != 0)

	{
		MSG_ENCODER("E-Load NULL:%d\r\n", ret);
		return;
	}
	
	for (i = 0; i < len; i ++)
	{
		Ecode_CRC += p[i];
	}

	if (Ecode_CRC)
	{
		MSG_ENCODER("E-Load CRC Error\r\n");
		return;
	}

	for (i = 0; i < ENCODER_NUM_MAX; i ++)
	{
		g_encoder[i].sta.bit.init = (ENC_def.encoder_init & (1 << i)) ? 1 : 0;
		g_encoder[i].coder_zero = ENC_def.encoder_zero[i];

		MSG_ENCODER("[%d]E-zero[%d]\r\n", i, g_encoder[i].coder_zero);
	}
}

static void ESave_Cfg(void)
{
	short i;
	ENCODER_DEF ENC_def;
	unsigned short *p = (unsigned short *)&ENC_def;
	unsigned short Ecode_CRC;
	//extern volatile unsigned short writeflash_index;
	short len = sizeof(ENC_def) / sizeof(short);

	for (i = 0; i < len; i ++)
	{
		p[i] = 0;
	}

	ENC_def.encoder_init = 0;
	for (i = 0; i < ENCODER_NUM_MAX; i ++)
	{
		if (g_encoder[i].sta.bit.init)
			ENC_def.encoder_init |= (1 << i);
		
		ENC_def.encoder_zero[i] = g_encoder[i].coder_zero;
	}
	
	Ecode_CRC = 0;
	for (i = 0; i < len; i ++)
	{
		Ecode_CRC += p[i];
	}
	ENC_def.Ecode_CRC = -Ecode_CRC;
	//writeflash_index=WRTEFLASH_BASE+5;
	EE_Write(10, (unsigned short*)&ENC_def, len);
	
	DEBUG_ENCODER("E-Save\r\n");
}
#endif /* ENCODER_SAVE_BOARD */

/*
�Զ����㴦��
-- ��ѭ������
Note: ������г̳����ܲ����˴�δ���쳣����
*/
static void EAuto_Zero(int eid, ENCODER_t *penc)
{
	int ret;
	int pos, enc_diff;
	short pos2; 
	
	penc->run.bit.rchk = 0;
	switch (penc->run.bit.raut)
	{
		case EAUTO_ZERO_START:

			StepMotor_Add_curr(penc->step_idx);
			
			penc->coder_max_cw = penc->last_coder;			//�Ƚ���ǰλ�ü�¼
			
			StepMotor_Modfiy_Position(penc->step_idx, 0, 1);
			pos2 = penc->sta.bit.autozero_mode?(penc->steps_max_auto+200) : ((penc->steps_max_auto * 120)/100);
			StepMotor_exec(penc->step_idx, pos2, 0,0,0);
			StepMotor_setMaxSpeed(penc->step_idx,1000);
			//STEPMOTOR[penc->step_idx].max_speed =1000;
			//Message_Send_4halfword(0xF2F1,eid,penc->step_idx,penc->run.bit.raut);
			
			penc->run.bit.raut = EAUTO_ZERO_GOTO_CW_MAX;
			break;
			
		case EAUTO_ZERO_GOTO_CW_MAX:
			ret = ECalc_Diff(penc->coder_max_cw, penc->last_coder);
			if (ret > 0)
			{
				penc->coder_max_cw = penc->last_coder;		//�����������ֵ
			}
			else
			{
				if (StepMotor_Get_Running(penc->step_idx))
					break;
				
				penc->coder_max_ccw = penc->coder_max_cw;
				pos2 = penc->sta.bit.autozero_mode?(penc->steps_max_auto) : ((penc->steps_max_auto * 120)/100);
				StepMotor_Modfiy_Position(penc->step_idx, pos2, 1);
				//Message_Send_4halfword(0xF7F1,eid,pos2,StepMotor_Get_Position(penc->step_idx));
			
				StepMotor_exec(penc->step_idx, 0, 0,0,0);
				StepMotor_setMaxSpeed(penc->step_idx,1000);
				//Message_Send_4halfword(0xF3F1,eid,penc->step_idx,penc->run.bit.raut);
			
				penc->run.bit.raut = EAUTO_ZERO_GOTO_CCW_MAX;
			}
			break;

		case EAUTO_ZERO_GOTO_CCW_MAX:
			ret = ECalc_Diff(penc->coder_max_ccw, penc->last_coder);
			if (ret < 0)
			{
				penc->coder_max_ccw = penc->last_coder;		//���淴�����ֵ
			}
			else
			{
				if (StepMotor_Get_Running(penc->step_idx))
					break;
				
				StepMotor_Modfiy_Position(penc->step_idx, 0, 1);
				if(!penc->sta.bit.autozero_mode)
					StepMotor_exec(penc->step_idx, STEPS_ERROR_RANGE, 0,0,0);	//����������һЩ
				//Message_Send_4halfword(0xF4F1,eid,penc->step_idx,penc->run.bit.raut);
			
				penc->run.bit.raut = EAUTO_ZERO_CALC_CHECK;
			}
			break;

		case EAUTO_ZERO_CALC_CHECK:
			if (StepMotor_Get_Running(penc->step_idx))
				break;
			penc->run.bit.raut = 0;
			StepMotor_reback_curr(penc->step_idx);	//
			
			enc_diff = penc->coder_max_cw - penc->coder_max_ccw;
			if (enc_diff < 0)
				enc_diff += ENCODER_PPR;

			penc->steps_max_auto_get = (long)enc_diff *(long)penc->steps_ppr  / (long)ENCODER_PPR;
			
			MSG_ENCODER("[%d] cw[%d] - ccw[%d] = [%d]\r\n",
						eid, penc->coder_max_cw, penc->coder_max_ccw, enc_diff);
			
			pos = ECalc_coder2step(eid, ENCODER_DIR_CCW, penc->coder_max_cw, penc->coder_max_ccw);
			
			if ((penc->steps_min_auto)&&(pos < (penc->steps_min_auto!=ENCODER_STEPS?penc->steps_min_auto:(penc->steps_max_auto *85/100))))	//�г�С�����ֵ�� 85%�����쳣//byhlc ��ֵ��������
			{
				if (enc_diff)
					enc_diff = 0 - enc_diff;
				else
					enc_diff = -1;

				EMsg_autoZero(eid, enc_diff);
			}
			else /* ��������г̲��� ��ȷ����λ����λ�� */
			{
				if (penc->sta.bit.stype == 0)
				{
					if(penc->sta.bit.autozero_mode)
					{
						enc_diff = penc->coder_max_ccw;
					}
					else
					{
						enc_diff = penc->coder_max_ccw + (ENCODER_PPR * 15/ penc->steps_ppr);	// ��λֵ��������15��
					}
				}
				else
				{
					enc_diff >>= 1;							//ȡ����ֵ���м�λ��
					enc_diff += penc->coder_max_ccw;
					if(penc->zero_goback_code>0)
					{
						enc_diff +=(ENCODER_PPR * penc->zero_goback_code/ penc->steps_ppr);
					}	
					else
					{
						enc_diff -=(ENCODER_PPR * abs(penc->zero_goback_code)/ penc->steps_ppr);
					}
				}
				enc_diff %= ENCODER_PPR;
				penc->coder_zero = enc_diff;

				//Message_Send_4halfword(0xF5F1,eid,penc->step_idx,penc->run.bit.raut);
			
				
				penc->run.bit.rzero = 0;
				penc->sta.bit.init = 1;
				
			#if ENCODER_SAVE_BOARD
				ESave_Cfg();
			#endif /* ENCODER_SAVE_BOARD */
			
				EMsg_autoZero(eid, enc_diff);

				penc->run.bit.raut = EAUTO_ZERO_EXEC_RESET;
				penc->check_delay = ENCODER_CHECK_DELAY_RST << 3;
			}
			break;

		case EAUTO_ZERO_EXEC_RESET:
			StepMotor_Reset(penc->step_idx,0);
			//break;
			
		default:
			penc->run.bit.raut = 0;
			break;
	}
}

/* Public functions ----------------------------------------------------------*/

#ifdef ENCODER_DEBUG
#define DPOS_SAMPLES		100
#define DPOS_DCNT			50
typedef struct {
	short mpos;
	short epos;
	uint16_t coder;
	uint16_t spd;
	uint16_t diff;
	uint16_t stamp;
	uint32_t tick;
}EPOS_t;

typedef struct {
	uint8_t head;
	uint8_t tail;
	uint8_t maxtime;
	uint8_t maxdiff;
	uint8_t cnterr;		// > 50 �Ĵ���
	uint8_t rsvd;
	uint8_t diffcnt[DPOS_DCNT];	// λ��ƫ��ֲ�����
	EPOS_t sample[DPOS_SAMPLES];
}DPOS_t;

DPOS_t dpos[ENCODER_NUM_MAX];

void Encoder_clr(u8 eid)
{
	if (eid < ENCODER_NUM_MAX)
	{
		memset((void *)&dpos[eid], 0, sizeof(DPOS_t));
	}
}

void Encoder_Veiw(int eid, int type)
{
	int i, tail, head;
	EPOS_t *psmp;

	if (eid >= ENCODER_NUM_MAX)
		return;
	
	if (type & 0x1)
	{
		tail = dpos[eid].tail;
		head = dpos[eid].head;
		
		while (tail != head)
		{
			psmp = &dpos[eid].sample[tail];
			DEBUG_ENCODER("[m %4d][e %4d][d %2d][tm %2d][code %4d][spd %4d][tick %8d]\r\n", 
				psmp->mpos, psmp->epos, psmp->diff, psmp->stamp, psmp->coder, 
				psmp->spd,psmp->tick);
			tail++;
			if (tail >= DPOS_SAMPLES)
				tail = 0;
		}
	}
	
	if (type & 0x2)
	{
		for (i = 0; i < DPOS_DCNT; i++)
		{
			if (dpos[eid].diffcnt[i])
			{
				DEBUG_ENCODER("[diff %2d][cnderr %d]\r\n", i, dpos[eid].diffcnt[i]);
			}
		}
	}
	
	DEBUG_ENCODER("[%d][tm %d][diff %d][cnderr %d]\r\n", 
			eid, dpos[eid].maxtime, dpos[eid].maxdiff, dpos[eid].cnterr);
}
#endif /* DEBUG_ENCODER */
/*----------------------------------------------------------------------------*/

int Encoder_setCheck(int eid, int chk)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	if (chk)
	{
		g_encoder[eid].run.bit.rchk = 1;
		g_encoder[eid].check_delay =g_encoder[eid].run.bit.raut?ENCODER_CHECK_DELAY_RUN:ENCODER_CHECK_DELAY_RUN5;
	}
	else
	{
		g_encoder[eid].run.bit.rchk = 0;
		g_encoder[eid].check_delay = 0;
	}
	
	g_encoder[eid].alert_count = 0;
	
	return 0;
}

int Encoder_setDir(int eid, int dir,int isset)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	if(!g_encoder[eid].sta.bit.dir_is_set)
	{
		g_encoder[eid].sta.bit.sdir = (dir) ? ENCODER_DIR_CCW : ENCODER_DIR_CW;
		if(isset)
		{
			g_encoder[eid].sta.bit.dir_is_set =1;
		}
	}
	return 0;
}



/* 
���õ������
type: 0: �ܲ���������һȦ 1�ܲ�������һȦ
*/
int Encoder_setType(int eid, int type)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	g_encoder[eid].sta.bit.stype = type ? 1 : 0;
	return 0;
}

/*
���ñ�����һȦ��Ӧ����
*/
int Encoder_setSteps(int eid, unsigned short steps)
{
	if (eid >= ENCODER_NUM_MAX
		|| steps == 0)
		return -1;

	g_encoder[eid].steps_ppr = steps;
	return 0;
}

/*
����λ�ü�鱨����ֵ
threshold: 2 <= x <= (ENCODER_PPR >> 2)
*/
int Encoder_setThreshold(int eid, unsigned short threshold)
{
	long temp;

 	#ifdef CODER_ALERT_DEF_0
		return 0;
	#endif
	
	if (eid >= ENCODER_NUM_MAX)
		return -1;

	if (threshold < 2)
		threshold = 2;

	if (threshold > (ENCODER_PPR >> 2))
		threshold = (ENCODER_PPR >> 2);
	
	g_encoder[eid].coder_max_alert = threshold;

	temp = (long)threshold * (long)g_encoder[eid].steps_ppr / (long)ENCODER_PPR;
	g_encoder[eid].step_max_alert = temp;
	
	DEBUG_ENCODER("[%d]E-setTh[%d]\r\n", eid, threshold);
	return 0;
}


/*
���õ�ǰ������ֵΪ��λ
*/
int Encoder_maskZero(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	g_encoder[eid].run.bit.rzero = 1;
	g_encoder[eid].sta.bit.init = 0;
	return 0;
}

/*
���������λֵ
zero: 0-(ENCODER_PPR-1)
ע: ����ֵ��ᱣ�浽flash��
*/
int Encoder_setZero(int eid, int zero)
{
	ENCODER_t *penc;
	
	if (eid >= ENCODER_NUM_MAX
		|| zero >= ENCODER_PPR)
		return -1;
	
	penc = &g_encoder[eid];
	penc->run.all = 0;
	penc->sta.bit.alert = 0;
	penc->sta.bit.init = 1;
	#ifdef ECODE_USE_MT6813_PWM
	penc->run.bit.rlast = 1;/*��ʾ֮ǰ���ڶ��ģ����¿�������λ����*/
	#endif
	if (penc->coder_zero != zero)
	{
		penc->coder_zero = zero;
		penc->run.bit.rpos = 1;	//���¼��㵱ǰλ��
		penc->check_delay = ENCODER_CHECK_DELAY_RST;
	#if ENCODER_SAVE_BOARD
		ESave_Cfg();
	#endif /* ENCODER_SAVE_BOARD */
	}
	
	DEBUG_ENCODER("[%d]E-Zero[%d]\r\n", eid, zero);
	return 0;
}

/*
�Զ����㿪��
maxstep: �Զ������Ӧ�ĵ�����г̲���(���Ǿ�ȷֵ)
*/
void Encoder_autoZero(int eid, unsigned short maxstep,unsigned char is_s_mode)
{
	//int ret;
	ENCODER_t *penc;
	
	if (eid >= ENCODER_NUM_MAX)
		return;
	
	penc = &g_encoder[eid];
	
	if (maxstep <= ENCODER_AUTO_STEP_RANGE)
	{
		maxstep = penc->steps_ppr;
	}

	if (maxstep >= (penc->steps_ppr - ENCODER_AUTO_STEP_RANGE))
	{
		maxstep = penc->steps_ppr - ENCODER_AUTO_STEP_RANGE;
	}
	
	penc->steps_max_auto = maxstep;
	penc->sta.bit.init = 0;
	penc->sta.bit.alert = 0;
	penc->sta.bit.autozero_mode = is_s_mode?1:0;
	
	
	penc->run.all = 0;
	penc->run.bit.raut = EAUTO_ZERO_START;
	penc->check_delay = ENCODER_CHECK_DELAY_RUN;
	penc->alert_count = 0;
	
	MSG_ENCODER("[%d]E-autoZero maxstep[%d]\r\n", eid, maxstep);
}

unsigned char Encoder_in_Get_St(short icflag,unsigned short st)
{
	//if (icflag>=2)
	{
	if(((st & 0x3F) == 0x3F) ||
	   ((st & 0x3F) == 0x0)) {
		return 1;
	}

	// Cordic & Line
	if(st & 0x18) {
		return 2;
	}
	
	// MagINCn & MagDECn
	if((st & 0x6) ) {
		return 3;
	}
	
	// Parity
	if((st & 0x1) ) {
		return 4;
	}
	
	// Compensation
	if((st & 0x20) == 0) {
		return 5;
	}
	}
	#if 0
	else
	if(icflag<=-2)
	{
		if((st & 0x6)) 
		{
			return 3;
		}
		if((st & 0x08)==0)
		{
			return 1;
		}		
	}
	else
	{
		return 1;
	}
	#endif
	
	return 0;
}


/*
��ȡ������״̬
*/
int Encoder_Get_state(unsigned int retdata,unsigned char arg)
{

	int i;
	unsigned char lasts;
	unsigned int allst=0;
	ENCODER_t *penc;
	
	if (arg ==0xFF)
	{
		for (i=0;i<ENCODER_NUM_MAX;i++)
		{
			penc = &g_encoder[i];
			lasts = Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status);
			allst |=(lasts<<(i*3));			
		}		
	}
	else
		if (arg <ENCODER_NUM_MAX)
		{
			penc = &g_encoder[arg];

			#if 0
			if(Check_is_MT6813PWM_Mode())
				allst = Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status);	
			else	
				allst = penc->last_status ;   //Encoder_in_Get_St(penc->last_status);	
			//#else
			#endif
			allst = penc->last_status ;   //Encoder_in_Get_St(penc->last_status);	
			
		}
		else
			return 1;
	Message_Send_4halfword(retdata,arg,allst&0xFFFF,(allst>>16)&0xFFFF);
	return 0;

}



/*
������λֵ
*/
int Encoder_getZero(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	return (int)g_encoder[eid].coder_zero;
}

/*
�������뵽����
*/
void Encoder_sendError(void)
{
	Message_Send(coder_err);
	coder_err = 0xEF;
}

/*

*/
int Encoder_step2coder(int eid, int pos)
{
	long encoder;
	long steps;
	
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	steps = g_encoder[eid].steps_ppr;
	if (pos >= steps)
		pos %= steps;
	
	encoder = (long)pos * (long)ENCODER_PPR / steps;
	encoder -= g_encoder[eid].coder_zero;
	
	encoder %= ENCODER_PPR;
	
	return encoder;
}


/*
�������ض� -- ����
*/


int Encoder_getCoder(int eid, int *coder, int *state)
{
	int ret;
	ENCODER_t *penc;

	if (eid >= ENCODER_NUM_MAX)
		return -1;

	penc = &g_encoder[eid];

	//���ж�
	//CLI();
	//__SETPRIMASK();
	//ret = ERead_Coder(eid, penc);
	//���ж�
	//SEI();
	if (coder)
	{
		*coder = penc->last_coder;
	}
	
	if (state)
	{
		*state = penc->last_status;
	}
	#ifdef ENCODER_IC_IS_5045OR8800
	ret = penc->coder_IC_flag;
	#else
	ret =0;
	#endif
	
	return ret;
}

/*
����ǰ����ֵ����ɵ��λ��ֵ������Ϊ��ǰ���λ��
*/
int Encoder_rPos(int eid, int onoff)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;

	g_encoder[eid].run.bit.rpos = onoff ? 1 : 0;

	return 0;
}

/*
��ǰ������ֵת��Ϊ����pos
dir: [0-(steps-1)]
return: <0��ȡʧ��
*/
int Encoder_getPos(int eid, int dir, int *pos)
{
	int ret, postmp;
	ENCODER_t *penc;
	
	if (eid >= ENCODER_NUM_MAX
		|| pos == NULL)
		return -1;
	
	penc = &g_encoder[eid];
	ret = ERead_Coder(eid, penc);
	if (ret == 0)
	{
		postmp = ECalc_coder2step(eid, dir, penc->last_coder, penc->coder_zero);
		if (dir == ENCODER_DIR_CCW)	// ��ǰλ������λ������ pos >= 0
			*pos = postmp;
		else
			*pos = 0 - postmp;
	}
	
	return ret;
}

/*
λ���Զ�����
ע��: ������Χ��Ȧ֮��
*/
int Encoder_adjPos(int eid)
{
	short mpos;
	ENCODER_t *penc;
	
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	penc = &g_encoder[eid];
	if (penc->sta.bit.init == 0)	//��λδ����
		return -1;

	/* �����������״̬ */
	penc->sta.bit.alert = 0;
	if (penc->run.bit.radj)
		return -1;
	
	penc->run.all = 0;
	penc->run.bit.radj = 1;
	penc->first_stop_time = arch_get_ticktime();/*������Ҫע�⣬���ܵ�������������ֵ��Ҫ����һ��*/

	StepMotor_setpos_2main_postion(penc->step_idx);
	mpos = StepMotor_Get_Position_2(penc->step_idx);
	StepMotor_exec(penc->step_idx, mpos, 1,0,0);

	DEBUG_ENCODER("[%d]E-adj[mpos %d]\r\n", eid, mpos);

	Message_Send_4halfword(0x1901,eid,mpos,0);
	
	return 0;
}

/*
�������б������ĵ��λ��
*/
void Encoder_adjPosAll(void)
{
	int eid;
	
	for (eid = 0; eid < encoder_nums; eid++)
	{

		if(g_encoder[eid].sta.bit.son)
		{
			if(g_encoder[eid].last_st_ex)
			{
				EReport_Error(eid, ALM_ENC_SSI, Encoder_in_Get_St(g_encoder[eid].coder_IC_flag,g_encoder[eid].last_status));//;
				
			}
			else
			if (g_encoder[eid].sta.bit.alert)
			{
				Encoder_adjPos(eid);
			}
		}
		
	
	}
}

/*
�������λ�ü�鿪��
*/
void Encoder_setRunCheck(int eid, int onoff)
{
	if (eid >= encoder_nums)
		return;
	
	g_encoder[eid].sta.bit.runchk = onoff ? 1 : 0;
	DEBUG_ENCODER("[%d]RunCheck[%d]\r\n", eid, onoff);
}



int Encode_Stop_Time_Check(int eid,unsigned int alarm_ms)
{
	ENCODER_t *penc;
	unsigned int time;
	if (eid >= encoder_nums)
		return 0;
	penc = &g_encoder[eid];
	
	time = arch_get_ticktime();
	time -= penc->first_stop_time;
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	if ((time>>2) > alarm_ms)   	// 250us ת����1ms 
	{
		return 1;
	}
	else return 0;	

}


void Encode_Set_First_stop_time(int eid)
{
	ENCODER_t *penc;
	//unsigned int time;
	if (eid >= encoder_nums)
		return ;
	penc = &g_encoder[eid];
	penc->first_stop_time = arch_get_ticktime();
	
}


int Encode_Error_Time_Check(int eid,unsigned int alarm_ms)
{
	//ENCODER_t *penc;
	unsigned int time;
	if (eid >= encoder_nums)
		return 0;
	//penc = &g_encoder[eid];
	
	time = arch_get_ticktime();
	time -= encoder_time_stamp[eid];
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	if ((time>>2) > alarm_ms)   	// 250us ת����1ms 
	{
		return 1;
	}
	else return 0;	

}

/*
����˶�֮ǰλ������ 
-- �˴�����Ϊ�������л������
*/
int Encoder_RunPos(int eid, short *pos,char whichexe)
{
	int ret;
	short epos;
	ENCODER_t *penc;
	#ifdef QL_DEBUG_STEP_MOTOR_ECORD
		short mpos,diff_pos;
	#endif
	
	ret = 0;

	do {
		if (eid >= encoder_nums)
			break;
		
		penc = &g_encoder[eid];

		if (penc->run.bit.raut)
			break;
#if 0
		if (penc->run.all)
		{
			#ifdef QL_ECODE_ERROR_AUTO_ADJ
			if ((penc->run.bit.radj == 0) &&(penc->run.bit.isredo== 0))
				break;
			#else
			if (penc->run.bit.radj == 0) 
				break;
			#endif
		}
		#endif
		if (penc->sta.bit.alert)
			penc->sta.bit.alert =0;

		if (penc->sta.bit.stat==0)
			break;
		
		
		epos = ECalc_rPos(eid, penc);
		#ifdef QL_DEBUG_STEP_MOTOR_ECORD
		mpos = *pos;
		diff_pos = abs(mpos - epos);
		#endif
#if 0		
		//DEBUG_ENCODER("[%d][mpos:%d epos:%d]\r\n", eid, mpos, epos);
		//steps_ppr >> 4

	if ((penc->run.bit.radj == 1) 
			#ifdef QL_ECODE_ERROR_AUTO_ADJ
			|| (penc->run.bit.isredo)
			#endif
			|| (diff_pos < (penc->coder_max_alert)))	//����λ�����Խ��Ͳ�����λ�� -- ���ܴ˴�û�б�Ҫ
#endif			
		{
			//if (!isnocheck)
			{
				*pos = epos;
				ret = 1;				
			}

			#if 0
			if(penc->run.bit.isredo==0)
			{
				penc->run.bit.rdo_2_adj ==0;
			}
			#endif
			
			#ifdef QL_DEBUG_STEP_MOTOR_ECORD
				//	if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_LIFT)
			if (diff_pos>2)
				Message_Send_4halfword((whichexe?0x99:0x98)|(penc->step_idx<<8),mpos,epos,diff_pos);


			#endif
		}

#if 0    // 2016 11 20 honglicheng
		if (penc->run.bit.radj==0)
		{
			ECheck_Pos(eid,mpos,1,penc);
		}
#endif
		
	} while (0);
	
	return ret;
}

unsigned short Ecode_Get_last_ecode(int eid)
{
	unsigned short ret=0xffff;
	ENCODER_t *penc;
	
	if (eid >= encoder_nums)
		return ret;

	penc = &g_encoder[eid];
	if (!Encoder_in_Get_St(penc->coder_IC_flag,penc->last_status))
		ret = penc->last_coder;
	return ret;
	
}

/*
��������б��������
*/
int Encoder_RunCheck(int eid, int speed, short pos)
{
	int ret;
	unsigned int time;
	short epos, pos_diff;
	ENCODER_t *penc;
	
	if (eid >= encoder_nums)
		return 0;

	penc = &g_encoder[eid];

	/* ��ѭ������Ҫ�����һЩҵ���������м�� */
	if (penc->sta.bit.runchk == 0
		|| penc->run.all != 0
		|| penc->step_max_alert < 2)
		return 0;
	
#if ENCODER_DMA_SUPPORT
	ret = SSI_Update(eid);	
	if (ret)
	{
		ret = ERead_Coder(eid, penc);
		if (ret < 0)
		{
			return ret;
		}
	}
	else //δ��ȡ��ֵ������һ�ε�ֵ
	{
		;
	}
#endif /* ENCODER_DMA_SUPPORT */
	/*
	if (penc->sta.bit.alert)
	{
		return -1;
	}
	*/

	/* �˴�ʹ�õı���ֵ����ѭ���ж�ȡ */
	epos = ECalc_rPos(eid, penc);
	pos_diff = abs(pos - epos);
	
	time = arch_get_ticktime();
	time -= encoder_time_stamp[eid];
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	
	if (pos_diff > penc->step_max_alert)
	{
		penc->alert_count++;

		if (time > 20)	//���ʱ�䳬��1ms����Ҫ���Ǽ�����ͺ���
		{
			ret = ((speed * time) >> 2) >> 10/* 1000 */;
			if (pos < (ret + penc->step_max_alert))
			{
				penc->alert_count--;	//�����ô�
			}
		}
		
		if (penc->alert_count > 3)
		{
			penc->alert_count = 0;
			penc->sta.bit.alert = 1;
			EReport_Error(eid, ALM_ENC_POS_IS, pos_diff);
		}
	}
	else
	{
		penc->alert_count = 0;
	}
	
#ifdef ENCODER_DEBUG
{
	EPOS_t *psmp;
	int head = dpos[eid].head;

	if (head >= DPOS_SAMPLES)
		head = 0;

	if (pos_diff >= DPOS_DCNT)
	{
		dpos[eid].cnterr++;
	}
	else
	{
		dpos[eid].diffcnt[pos_diff]++;
	}
	
	psmp = &(dpos[eid].sample[head]);
	psmp->coder = penc->last_coder;
	psmp->mpos = pos;
	psmp->epos = epos;
	psmp->diff = pos_diff;
	psmp->spd = speed;
	psmp->stamp = time;
	psmp->tick = arch_get_ticktime();
	
	if (time > dpos[eid].maxtime)
		dpos[eid].maxtime = time;
	if (pos_diff > dpos[eid].maxdiff)
		dpos[eid].maxdiff = pos_diff;
	
	head++;
	if (head >= DPOS_SAMPLES)
		head = 0;
	
	if (head == dpos[eid].tail)
	{
		dpos[eid].tail++;
		if (dpos[eid].tail >= DPOS_SAMPLES)
			dpos[eid].tail = 0;
	}
	dpos[eid].head = head;
}
#endif /* ENCODER_DEBUG */

	return 0;
}

/*
�����λ��Ӧ��������λ
return: <0ʱ���������ã����Ƕ�ȡ����, 
		0 ������δ���ã�
		>0��������ȡλ������ posΪ��ǰ�������
*/
int Encoder_Reset(int eid, int *pos)
{
	
	int epos;
	ENCODER_t *penc;
	#if ENCODER_DMA_SUPPORT
	int ret;
	int overtime = 0xFFFFF;
	#endif
	
	if (eid >= encoder_nums
		|| pos == NULL)
		return -1;
	
	penc = &g_encoder[eid];
	if (penc->sta.bit.son == 0)
		return 0;
	
	/* ֻҪ��λ����ԭ���ı�����־ */
	penc->sta.bit.alert = 0;
	
#if ENCODER_DMA_SUPPORT
	do {
		ret = SSI_Update(eid);
		overtime--;
		if (overtime == 0)
		{
			DEBUG_ENCODER("[%d]E-Rst overtime\r\n", eid);
			break;
		}
	} while(ret == 0);
#endif /* ENCODER_DMA_SUPPORT */

	if (penc->sta.bit.stat==0)
		return 0;
#if 0
	ret = ERead_Coder(eid, penc);
	if (ret < 0)
		return ret;
#endif
	epos = ECalc_rPos(eid, penc);
	*pos = epos;
	penc->run.bit.rrst = 1;

	
	
	//DEBUG_ENCODER("[%d]E-Rst[epos %d]\r\n", eid, epos);
	return 1;
}

/*

*/
void Encoder_deInit(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return;

	g_encoder[eid].sta.bit.init = 0;
}

void Encoder_Enable(int onoff)
{
	int i;

	onoff = onoff ? 1 : 0;
	for (i = 0; i < ENCODER_NUM_MAX; i++)
		g_encoder[i].sta.bit.son = onoff;
}

void Encoder_Enable_mask(int onoffmask)
{
	int i;

	
	for (i = 0; i < ENCODER_NUM_MAX; i++)
		g_encoder[i].sta.bit.son = ((onoffmask>>i)&0x01)?1:0;
}


/*
��ǰ�����Ƿ���������
*/
int Encoder_Work(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return 0;
	
	return g_encoder[eid].sta.bit.son;
}

int Encoder_rechk_over(int eid)
{
	int ret=0;
	if (eid >= ENCODER_NUM_MAX)
	return ret;	
	if (g_encoder[eid].run.bit.rchk==0)
	{
		ret=1;
	}	
	return ret;	
}

int Encoder_Get_Max_Steps(int eid)
{
	int ret=0;
	ENCODER_t *penc;

	if (eid < ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		ret = penc->steps_max_auto_get;
	}
	return ret;
}



int Encoder_Set_Goback_Steps(unsigned short eid,short Steps)
{
	//int ret=0;
	ENCODER_t *penc;

	if ((Steps>200)||(Steps<-200))
	return 0;	
		
	
	if (eid < ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		penc->zero_goback_code = Steps;
		//ret = penc->steps_max_auto_get;
	}
	return 1;
}


int Encoder_Set_maxsteps_cw(int eid,int Steps)
{
	//int ret=0;
	ENCODER_t *penc;

	if (Steps>800)
	return 0;	
		
	
	if (eid < ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		penc->steps_ppr_cw = Steps;
		//ret = penc->steps_max_auto_get;
	}
	return 1;

}


int Encoder_Set_minsteps_rang(int eid,int Steps)
{
	//int ret=0;
	ENCODER_t *penc;

	if ((Steps>800) ||(Steps<0))
	return 0;	
		
	
	if (eid < ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		penc->steps_min_auto = Steps;
		//ret = penc->steps_max_auto_get;
	}
	return 1;
}


/*
�������ض�̽��
eid: 0xFF ��ȡ����
*/
int Encoder_Probe(int eid)
{
	int ret, encoder_status = 0;
	ENCODER_t *penc;

	if (eid < ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		ret = ERead_Coder(eid, penc);
		
		DEBUG_ENCODER("SSI_Read= %d ...%d [coder:%d stat:0x%x]\r\n", ret, eid, penc->last_coder, penc->last_status);
		if (ret != -100)
		{
			penc->sta.bit.stat = 1;
			encoder_status = (penc->last_status >> 1);
		}
		else
		{
			penc->sta.bit.stat = 0;
			encoder_status = 0xFFFF;
		}
	}
	else if (eid == 0xFF)
	{
		for (eid = 0; eid < encoder_nums; eid++)
		{
			penc = &g_encoder[eid];
#if ENCODER_DMA_SUPPORT
			do {
				ret = SSI_Update(eid);
			}while(ret == 0);
#endif /* ENCODER_DMA_SUPPORT */
			ret = ERead_Coder(eid, penc);
			MSG_ENCODER("SSI_Read= %d ...%d [coder:%d stat:0x%x]\r\n", ret, eid, penc->last_coder, penc->last_status);
			if (ret == 0)
			{
				encoder_status |= (1 << eid);
				penc->sta.bit.stat = 1;
			}
			else
			{
				penc->sta.bit.stat = 0;
			}
		}
		
		DEBUG_ENCODER("E-status 0x%x\r\n", encoder_status);
	}
	return encoder_status;
}


unsigned short ECoder_data_send=0;
unsigned short Ecoder_sendidx=0;
unsigned char Ecoder_logid=0xff;


void Setlogid(unsigned char id)
{
	Ecoder_logid = id;
}

unsigned int Check_Ecoder_is_stable(unsigned char eid)
{
	ENCODER_t *penc;
	unsigned int ret=0;
	if(eid<ENCODER_NUM_MAX)
	{
		penc = &g_encoder[eid];
		ret = penc->sta.bit.delayok;
	}
	return ret;
}

void Encoder_Poll(void)
{
	static uint8_t eid = 0;
	int ret,runing;
	short epos, mpos;
	ENCODER_t *penc;
	char isfirstreadecode=0;

	//coder_read_fifo_cnt++;
	do {
		eid++;
		eid %= ENCODER_NUM_MAX;

#if 0
		if(Ecoder_logid<ENCODER_NUM_MAX)
		{
			eid = Ecoder_logid;
		}

		#endif
		
		penc = &g_encoder[eid];
		
	#if (!ENCODER_REAL_TIME)
		if (penc->run.all == 0)
			break;
	#endif /* ENCODER_REAL_TIME */
	
		if (penc->sta.bit.son == 0)
			break;

		/* �������ʱ����ʱ��ȡ */
		runing = StepMotor_Get_Running(penc->step_idx);
		if (runing == 0)
		{
			if (penc->check_delay)
			{
				penc->check_delay--;
				break;
			}
			penc->check_delay = ENCODER_CHECK_INTERVAL;
			
			if (penc->run.bit.rlast)  /*˵����һ�λ��ڶ�����β�����,�����ڿ�ʼ��ʱ*/
			{
				penc->first_stop_time = arch_get_ticktime();				
				penc->run.bit.rlast = 0;
				isfirstreadecode = 1;
				#ifdef QL_DEBUG_STEP_MOTOR_ECORD
				if (penc->step_idx==0)
				{
					ecord_index =0;
				}
				#endif

				{//20170710 ��¼ֵ���
					penc->ECODE_first_code =0xFFFF;
					penc->ECODE_last_code = 0;
					penc->ECODE_max_code = 0;
					penc->ECODE_min_code =0xFFFF;
					penc->ECODE_read_cnt = 0;
					penc->ECODE_read_err_cnt=0;
					penc->ECODE_read_continue_err_cnt=0;		
					penc->ECODE_read_continue_max_err_cnt = 0;
				}	

				#ifdef ECODE_USE_MT6813_PWM
				if(Check_is_MT6813PWM_Mode())
				{
					penc->timeout_alert =PWM_ECODE_CHECK_DELAY_TIME_2MS;
					penc->sta.bit.delayok=0;
				}
				#endif
				
				
			}
			
		}
		else
		{
			if (penc->run.bit.rlast==0)			// ��һ�λ���STOP״̬
			{
				unsigned short emax_min;
				if (penc->sta.bit.stat)
				penc->ECODE_last_code =penc->last_coder;
				else
				penc->ECODE_last_code = 0xFFFF;
				emax_min=abs(penc->ECODE_max_code-penc->ECODE_min_code);
				if ((penc->ECODE_read_err_cnt)||((emax_min>20)&&(emax_min<4076)))
				{
					Message_send_log_ecode(penc->step_idx|(0x00<<8),penc->ECODE_first_code,penc->ECODE_last_code);
					Message_send_log_ecode(penc->step_idx|(0x01<<8),penc->ECODE_max_code,penc->ECODE_min_code);
					Message_send_log_ecode(penc->step_idx|(0x02<<8),penc->ECODE_read_cnt,penc->ECODE_read_err_cnt);
					Message_send_log_ecode(penc->step_idx|(0x03<<8),penc->ECODE_read_continue_err_cnt,penc->ECODE_read_continue_max_err_cnt);
				}
				#ifdef ECODE_USE_MT6813_PWM
				if(Check_is_MT6813PWM_Mode())
				{
					penc->timeout_alert =0;
					penc->sta.bit.delayok=0;
				}
				#endif
			}
			penc->run.bit.rlast = 1;
			if (penc->first_stop_time)
				penc->first_stop_time = 0 ;

			
		}
		
#if ENCODER_DMA_SUPPORT
		ret = SSI_Update(eid);
		if (ret == 0)
			break;
#endif /* ENCODER_DMA_SUPPORT */
		
		ret = ERead_Coder(eid, penc);
		if (ret < 0)
		{
			if (isfirstreadecode)				/*�����һ�ξͶ���ʧ���ˣ���Ҫ�����ٶ�һ��*/
				penc->check_delay =0;
			break;
		}
		/* ͨ�����������תȡ������ֵ������λ */
#if 0
		if((eid==Ecoder_logid)&&(runing==0))//&&Check_is_MT6813PWM_Mode())
		{
			if(abs(ECoder_data_send-penc->last_coder)>=3)
			{
				unsigned int bas_t = arch_get_ticktime();
				ECoder_data_send = penc->last_coder;
				Message_Send_4halfword(0xFEFE,ECoder_data_send,bas_t & 0xffff,bas_t>>16);
			}
		}
#endif
		
		if (penc->run.bit.raut)
		{
			EAuto_Zero(eid, penc);
			break;
		}

		/* �����λ */
		if (penc->run.bit.rzero)
		{
			penc->run.bit.rzero = 0;
			penc->sta.bit.init = 1;
			penc->coder_zero = penc->last_coder;
			StepMotor_Modfiy_Position(penc->step_idx, 0, 0);//����ǰλ����Ϊ0
		#if ENCODER_SAVE_BOARD
			ESave_Cfg();
		#endif /* ENCODER_SAVE_BOARD */
		}

		if (runing || (penc->sta.bit.init == 0))
		{
			if (runing && (penc->sta.bit.init == 0))
			{
				/* �����Ӧ���ж��� -- ��ʱ���·��ͱ��� */
				EReport_Error(eid, ALM_ENC_POS_NOT, 0);
			}
			break;
		}

		/* �ñ�����������λ�� */
		if (penc->run.bit.rpos)
		{
			#ifdef ECODE_USE_MT6813_PWM
			if((Check_is_MT6813PWM_Mode() && (penc->sta.bit.delayok))
				||(!Check_is_MT6813PWM_Mode()))
			#endif
			{
			epos = ECalc_rPos(eid, penc);
			StepMotor_Modfiy_Position(penc->step_idx, epos, 1);
			DEBUG_ENCODER("[%d]E-ePos[%d][%d]\r\n", eid, epos, penc->last_coder);
			penc->run.bit.rpos = 0;
			}
		}
		else if (penc->sta.bit.alert == 0)/*  ��ǰλ�ü��  */
		{
			//�ô����ֻ�ڵ��ֹͣ������¼��
			// 1. ������н����жϵ��ͣ��λ���Ƿ�����
			// 2. ������û�����У����Ǳ���ֵ�仯��Χ������
			#ifdef ECODE_USE_MT6813_PWM
			if((Check_is_MT6813PWM_Mode() && (penc->sta.bit.delayok))
				||(!Check_is_MT6813PWM_Mode()))
			#endif
			{
				
				mpos = StepMotor_Get_Position_2(penc->step_idx);
				ECheck_Pos(eid, mpos, 1, penc);
			}
		}
	}while (0);
}

#ifdef ECODE_USE_MT6813_PWM
void Encoder_timeout_check_2ms()
{
	ENCODER_t *penc;
	int i;
	for (i = 0; i < ENCODER_NUM_MAX; i++)
	{		
		penc = &g_encoder[i];
		if(penc->timeout_alert)
		{
			penc->timeout_alert--;
			if(!penc->timeout_alert)
				penc->sta.bit.delayok=1;
		}		
	}
}

#endif	

int Encoder_set_stepno()
{
	int i;
	ENCODER_t *penc;
	STEP_TYPE *Step;
	unsigned char stepidx;

	for (i = 0; i < ENCODER_NUM_MAX; i++)
	{
		stepidx =i;
		penc = &g_encoder[i];
		penc->step_idx = stepidx;
	}
	
}

int Encoder_Init(int nums)
{
	int i;
	ENCODER_t *penc;
	STEP_TYPE *Step;
	unsigned char stepidx;

	encoder_nums = nums;
	if (nums > ENCODER_NUM_MAX)
	{
		encoder_nums = ENCODER_NUM_MAX;
		MSG_ENCODER("E-Num too large[%d]\r\n", nums);
	}

	for (i = 0; i < ENCODER_NUM_MAX; i++)
	{
		#ifdef E490_V10_BOARD
		if (i>=6)
		{
			stepidx = 12+(i-6);
		}
		else
			stepidx =i;
		#else
			stepidx =i;
		#endif
	
		Step = &STEPMOTOR[stepidx];

		if (Step->moto_remap_config.moto_attr.bit.moto_ecode_index!=i)
		{
			MSG_ENCODER("Stepno_enidx[%d]!=Encode_idx[%d]\r\n",Step->moto_remap_config.moto_attr.bit.moto_ecode_index, i);
		}
		
		penc = &g_encoder[i];
		penc->step_idx = stepidx;
		penc->run.all = 0;
		penc->sta.all = 0;
		penc->coder_IC_flag =0;

		Encoder_setDir(i, Step->step_st.bit.dir_High,0);	//Step->step_st.bit.dir_High == 0 => ENCODER_DIR_CW

		if (stepidx > 3)
			penc->sta.bit.stype = 1;

		penc->sta.bit.runchk = 1;
		penc->sta.bit.stat = 1;
		penc->sta.bit.son =0;
		penc->coder_zero = 2;
		penc->zero_goback_code=ENCODER_PPR * STEPS_ZERO_GOBACK / ENCODER_STEPS;
		//penc->steps_max_auto_get = 
		penc->steps_ppr = ENCODER_STEPS;
		penc->alert_count = 0;
		penc->check_delay = ENCODER_CHECK_DELAY_RST;
		#ifdef CODER_ALERT_DEF_0
		//#define STEPS_ERROR_RANGE_0	(0)	//��λƫ��������
		penc->coder_max_alert =0; //ENCODER_PPR * STEPS_ERROR_RANGE / ENCODER_STEPS;
		penc->step_max_alert = 0;//STEPS_ERROR_RANGE;
		#else
		penc->coder_max_alert = ENCODER_PPR * STEPS_ERROR_RANGE / ENCODER_STEPS;
		penc->step_max_alert = STEPS_ERROR_RANGE;
		#endif
		penc->steps_max_auto = ENCODER_STEPS - ENCODER_AUTO_STEP_RANGE;
		penc->steps_min_auto = ENCODER_STEPS;
		penc->steps_max_auto_get = penc->steps_max_auto;
		penc->first_error_time = 0;/*Ĭ��0 ����û���������Ҳ�п��ܳ����㣬��ôҲ��һ��һ���ڿ�ʼ��¼*/
		penc->run.bit.rlast=0;
		penc->first_stop_time =0;

		if(penc->sta.bit.stype)
		{
			penc->steps_ppr_cw = ENCODER_STEPS>>1;
		}
		else
		{
			penc->steps_ppr_cw = penc->steps_ppr -70;  /*Ĭ��730*/
		}
	}
	
#if ENCODER_SAVE_BOARD
	ELoad_Cfg();
#endif /* ENCODER_SAVE_BOARD */
	
	/* ��һ�ζ�ȡ���粻��ȡ���ܵ�һ�ζ�ȡ���� */
	SSI_Read(0,0, 0, 0);
	
	Encoder_Probe(0xFF);	//@ ��ʱ��ȡ�ı���ֵ��Ĭ�Ϸ����ֵ -- ������������и���ֵ���

	//Encoder_Enable(0xFF);
	return 0;
}


/*----------------------------------------------------------------------------*/
/*
����1: ģ��IO��ʽ��ȡ
	ÿ����������ȡʱ�� ������ٶ��� 1MHz�Ļ���Ҫ 20uS����
	8�������ȡ������Ҫ170uS => 200uS => 5KHz
	��������²����ǵ���ж�����ʱ�����ٶȱ������ 5KHz

����2: SPI��ȡ������ 16bits
	����ֵͨ�����������С��Ƭ��ʵʱ��ȡ
	16λ����ͨ��Э�鶨��
	����ע������λ����ܱ��浽С��Ƭ���ϵĿ�����

����3: CPLDֱ�Ӷ�ȡ
	FSMC��ȡ
*/
/*----------------------------------------------------------------------------*/


void Encoder_clear_restbit( unsigned int eid)
{
if (eid >= ENCODER_NUM_MAX)
		return ;
	
	g_encoder[eid].run.bit.rrst= 0;
	
}


void Ecode_CS_set(unsigned char whichid,unsigned char isenable)
{
	SSI_Select_with_id(whichid,isenable?1:0);

	if(isenable)
	{
		#ifdef ECODE_USE_MT6813_PWM
		Set_Duty_Fre_reset();
		#endif
	}	
}

void Ecode_cs_select_all()
{
	SSI_Select_all_ecode();
}


unsigned char Curr_eid=0xff;

unsigned int Ecode_data_[ENCODER_NUM_MAX]={0,0,0,0,0,0,0,0};
unsigned char Ecode_st_[ENCODER_NUM_MAX]={0,0,0,0,0,0,0,0};

void Ecode_CS_for_MT6813()
{
	static unsigned char Ecodeid=0;
	unsigned char addcnt=0;

	do_again:
	if(Ecodeid>=ENCODER_NUM_MAX)
		Ecodeid =0;
	if(g_encoder[Ecodeid].sta.bit.son)
	{
		Curr_eid = Ecodeid;
		Ecode_CS_set(Ecodeid,1);
		Ecodeid++;
		return ;
	}
	else
	{
		Ecodeid++;
		if(addcnt++>=ENCODER_NUM_MAX)
		{
			return;
		}
		else
		goto do_again;
		
	}
}

void Set_Ecode_data(unsigned int x,unsigned int  y)
{
 	if(Curr_eid>=ENCODER_NUM_MAX) return;

	#ifdef DEBUG_MT6813_PWM
	{
		unsigned short abs_deffecode=0;
		//#ifdef DEBUG_MT6813_PWM
		extern unsigned int Debug_led_cnt;
		extern unsigned char Debug_led_alarmit;

		if(x==0)
		{		
				arch_LED_On();
				Debug_led_alarmit =1;
				Debug_led_cnt =2;
				Ecode_read_error_send(0,0);
				
		}
		else
			{

		abs_deffecode = abs(Ecode_data_[Curr_eid] -(( x <<12 ) /4000));
		if((abs_deffecode<=3000)&&(abs_deffecode>=500))
		{
			arch_LED_On();
			Debug_led_alarmit =1;
			Debug_led_cnt =3;
			Ecode_read_error_send(0,abs_deffecode);
		}}
	}
	#endif
	
	
	Ecode_data_[Curr_eid] =( x <<12 ) /4000;
	Ecode_st_[Curr_eid] =  0;
}

void Set_Ecode_st(unsigned int st)
{
 	if(Curr_eid>=ENCODER_NUM_MAX) return;
	Ecode_st_[Curr_eid] =  st;
}

void Get_code_data_from_mem(unsigned char eid,unsigned long *edata)
{
	if(eid>=ENCODER_NUM_MAX) return;
	if(edata)
	{
		*edata = Ecode_data_[eid]<<6;
		*edata |= (Ecode_st_[eid]?0x3F:0x20);	
	}	
}

unsigned char Get_Curr_eid()
{
	return Curr_eid;
}


unsigned long ecode_senddata[ENCODER_NUM_MAX]={0,0,0,0,0,0,0,0};
void Send_last_ecode_data()
{
	static unsigned char eid_last=0;
	unsigned char eid_c;
	unsigned long  ed;
	
		eid_c = Get_Curr_eid()-1;
		if(eid_c>=ENCODER_NUM_MAX)
			eid_c=(ENCODER_NUM_MAX-1);
		if(eid_last !=eid_c)
		{
			eid_last = eid_c;
			Get_code_data_from_mem(eid_last,&ed);
			if(ed!=ecode_senddata[eid_last])
			{
				ecode_senddata[eid_last] = ed;
				Message_Send_4halfword(0x2f01,ed>>6,ed & 0x3F,eid_last);
			}
		}
}


#ifdef DEBUG_MT6813_PWM

void Ecode_read_error_send(unsigned char eid,unsigned short errno)
{
	EReport_Error(Curr_eid, ALM_ENC_READERR, (unsigned short)errno);
	
}

#endif


