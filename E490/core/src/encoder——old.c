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


/* Private typedef -----------------------------------------------------------*/


enum {
	EAUTO_ZERO_FINISH = 0,
	EAUTO_ZERO_START,
	EAUTO_ZERO_GOTO_CW_MAX,
	EAUTO_ZERO_GOTO_CCW_MAX,
	EAUTO_ZERO_CALC_CHECK,
	EAUTO_ZERO_EXEC_RESET,
};


/* 保存到flash的结构体 */
typedef struct {
	unsigned short encoder_init;
	unsigned short encoder_zero[ENCODER_NUM_MAX];
	unsigned short rsvd[6];
	unsigned short Ecode_CRC;
} ENCODER_DEF;

struct ENC_RUN_BITS {
   uint16_t rzero		:1;		// 0: 标记当前电机位置为零位
   uint16_t rchk		:1;		// 1: 位置检查
   uint16_t rpos		:1;		// 2: 编码器值转换为pos
   uint16_t rrst		:1;		// 3: 复位
   uint16_t radj		:1;		// 位置修正中
   uint16_t raut		:4;		// 自动找零状态
   uint16_t rsvd		:7;		// reserved
};

union ENC_RUN_REG {
   uint16_t	all;
   struct ENC_RUN_BITS bit;
};

struct ENC_STA_BITS {
	uint16_t son		:1; 	// 编码使能
	uint16_t init		:1;		// 第一次上电后 1-零位值为有效
	uint16_t sdir		:1;		// 编码器计数方向
	uint16_t stat		:1;		// 编码值读取状态正常与否
	uint16_t alert		:1;		// 报警状态
	uint16_t stype		:1;		// 电机是否有负向行程 --0只有正方向 1是有正负向
	uint16_t runchk		:1;		// 电机运行检查位置
	uint16_t rsvd		:9;
};

union ENC_STA_REG {
   uint16_t	all;
   struct ENC_STA_BITS bit;
};

typedef struct {
	union ENC_RUN_REG run;
	union ENC_STA_REG sta;
	uint16_t steps_ppr;				//一圈电机总步数
	uint16_t coder_zero;			// 零位编码值 -- 注意需要先确定好方向
	unsigned short coder_max_alert;	//报警范围 -- 偏差最大编码值
	unsigned short step_max_alert;	//报警范围 -- 偏差最大步数 -- 为了减少运算
	
	uint16_t coder_max_ccw;			// 自动找零反向采样最大编码值
	uint16_t coder_max_cw;			// 自动找零正向采样最大编码值
	uint16_t steps_max_auto;		// 自动找零时最大步数
	uint16_t last_coder;			//最后一次读取编码器返回值
	unsigned short last_status;		//最后一次读取编码器状态寄存
	
	unsigned short check_delay;
	unsigned short alert_count;		// 偏差超限计数
	unsigned short step_idx;			/* 电机序号*/
}ENCODER_t;

/* Private define ------------------------------------------------------------*/

#define ENCODER_CHECK_DELAY_RUN		25
#define ENCODER_CHECK_DELAY_RST		50
#define ENCODER_CHECK_INTERVAL		10
#define ENCODER_HW_ERROR_DELAY		100

#define ENCODER_AUTO_STEP_RANGE		(60)	//自动找零时对应总行程的差值调整


#define ALM_ENC_SSI 	(0x0)	// 编码器硬件读取值报警 -- 连续3次读取错误
#define ALM_ENC_RST		(0x1)	// 复位失败
#define ALM_ENC_POS		(0x2)

/* Private macro -------------------------------------------------------------*/


#define ENCODER_DEBUG
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


static unsigned int coder_err;

static unsigned char encoder_nums;		//编码器使用总个数

/* Private function prototypes -----------------------------------------------*/

extern int SSI_Read(int idx, int *coder, int *status);

/* Private functions ---------------------------------------------------------*/

static void EMsg_autoZero(int eid, int ret)
{
	MSG_ENCODER("\r\n[%d]E-autoZero ret[%d]\r\n", eid, ret);
	Massage_Send_4halfword(((CMD_ENCODER_ZERO_MASK << 8) | CMDTYPE_PERIPHERAL), ret, eid, 0);
}

static void EReport_Error(int eid, uint8_t type, uint8_t error)
{
	coder_err = ALERT_CODE_ENCODE;
	alert_push((coder_err | (eid << 8)), type /*| (error << 8)*/);
	
	//DEBUG_ENCODER("[%d]E-Error[0x%x %d]\r\n", eid, type, error);
}

/*
编码器值读取统一接口
*/
static int ERead_Coder(int eid, ENCODER_t *penc)
{
	int ret;
	
#if ENCODER_DMA_SUPPORT
	ret = SSI_Read(eid, (int *)&penc->last_coder, (int *)&penc->last_status);
	if (ret == 0)
	{
		penc->sta.bit.stat = 1;
		encoder_time_stamp[eid] = arch_get_ticktime();

		if (penc->sta.bit.sdir == ENCODER_DIR_CCW) //反向计数 -- 保持编码器正向计数和电机正向相同
		{
			penc->last_coder = (ENCODER_PPR - 1 - penc->last_coder);
		}
	}
	else
	{
		penc->sta.bit.stat = 0;
		if (SSI_Error(eid) > 100) // 编码器线可能被拔出
		{
			if (penc->sta.bit.alert == 0)
			{
				penc->sta.bit.alert = 1;
				EReport_Error(eid, ALM_ENC_SSI, (uint8_t)ret);
			}
		}
	}
#else
	do {
		if (penc->sta.bit.son!=1) 
			break;
		ret = SSI_Read(eid, (int *)&penc->last_coder, (int *)&penc->last_status);
		if (ret == 0)
		{
			penc->sta.bit.stat = 1;
			encoder_time_stamp[eid] = arch_get_ticktime();

			if (penc->sta.bit.sdir == ENCODER_DIR_CCW) //反向计数 -- 保持编码器正向计数和电机正向相同
			{
				penc->last_coder = (ENCODER_PPR - 1 - penc->last_coder);
			}
			break;
		}
		
		penc->sta.bit.stat = 0;
		if (SSI_Error(eid) > 3)
		{
			penc->check_delay = ENCODER_HW_ERROR_DELAY;
			
			if (penc->sta.bit.alert == 0)
			{
				penc->sta.bit.alert = 1;
				EReport_Error(eid, ALM_ENC_SSI, (uint8_t)ret);
			}
			break;
		}
	}while (1);
#endif /* ENCODER_DMA_SUPPORT */
	return ret;
}

/*
计算newcoder  于 oldcoder 的差值
return: < 0 newcoder在oldcoder的CCW向; > 0 为CW向
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
当前编码器位置计算电机位置值
*/
static int ECalc_rPos(int eid, ENCODER_t *penc)
{
	int pos;
	
	pos = ECalc_coder2step(eid, ENCODER_DIR_CCW, penc->last_coder, penc->coder_zero);
	if (penc->sta.bit.stype == 0)
	{
		if (pos > (penc->steps_ppr - (70))) //度目电机最大位置到720 
		{
			pos -= penc->steps_ppr;
		}
	}
	else
	{
		if (pos > (penc->steps_ppr >> 1))	//实际位置不会大于半圈的位置
		{
			pos -= penc->steps_ppr;
		}
	}

	return pos;
}

/*
编码值对应位置检查
检查错误后报警到主机
*/
static int ECheck_Pos(int eid, int pos, int time, ENCODER_t *penc)
{
	long iencoder;
	long steps;
	int errno;
	short epos;
	
	steps = penc->steps_ppr;

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
		dbgu_motor_clr(eid);
#endif /* ENCODER_DEBUG */
		
		//DEBUG_ENCODER("[%d]E-Chk[mpos %4d][coder %4d][diff %4d]\r\n", eid, pos, penc->last_coder, errno);
		penc->alert_count++;
		if (penc->alert_count < time)
		{
			return 1;
		}
		
		penc->sta.bit.alert = 1;
		
		if (penc->run.bit.rrst)
			EReport_Error(eid, ALM_ENC_RST, 0);
		else
			EReport_Error(eid, ALM_ENC_POS, (uint8_t)errno);
	}
	#if 0
	else if (errno > 6/* 2步 */ && penc->run.bit.rchk)
	{
		/* 如果位置检查未超限值则在运行停止后自动调整电机位置 -- 消除积累误差 */
		//penc->run.bit.rpos = 1;
		epos = ECalc_rPos(eid, penc);
		errno = abs(pos - epos);
		if (errno < (penc->steps_ppr >> 3))	//计算位置如果越界就不修正位置 -- 可能此处没有必要
		{
			StepMotor_Modfiy_Position(eid, epos, 1);
		}
	}
	#endif
	
	penc->alert_count = 0;
	penc->run.bit.rrst = 0;
	penc->run.bit.rchk = 0;
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
	
	EE_Write(10, (unsigned short*)&ENC_def, len);
	
	DEBUG_ENCODER("E-Save\r\n");
}
#endif /* ENCODER_SAVE_BOARD */

/*
自动找零处理
-- 主循环调用
Note: 如果总行程超过总步数此处未做异常处理
*/
static void EAuto_Zero(int eid, ENCODER_t *penc)
{
	int ret;
	int pos, enc_diff;
	
	penc->run.bit.rchk = 0;
	switch (penc->run.bit.raut)
	{
		case EAUTO_ZERO_START:

			StepMotor_Add_curr(penc->step_idx);
			
			penc->coder_max_cw = penc->last_coder;			//先将当前位置记录
			
			StepMotor_Modfiy_Position(penc->step_idx, 0, 1);
			StepMotor_exec(penc->step_idx, ((penc->steps_max_auto * 120)/100), 0);
			StepMotor_setMaxSpeed(penc->step_idx,1000);
			//STEPMOTOR[penc->step_idx].max_speed =1000;
			
			penc->run.bit.raut = EAUTO_ZERO_GOTO_CW_MAX;
			break;
			
		case EAUTO_ZERO_GOTO_CW_MAX:
			ret = ECalc_Diff(penc->coder_max_cw, penc->last_coder);
			if (ret > 0)
			{
				penc->coder_max_cw = penc->last_coder;		//保存正向最大值
			}
			else
			{
				if (StepMotor_Get_Running(penc->step_idx))
					break;
				
				penc->coder_max_ccw = penc->coder_max_cw;
				
				StepMotor_Modfiy_Position(penc->step_idx, ((penc->steps_max_auto * 120)/100), 1);
				StepMotor_exec(penc->step_idx, 0, 0);
				StepMotor_setMaxSpeed(penc->step_idx,1000);
				
				penc->run.bit.raut = EAUTO_ZERO_GOTO_CCW_MAX;
			}
			break;

		case EAUTO_ZERO_GOTO_CCW_MAX:
			ret = ECalc_Diff(penc->coder_max_ccw, penc->last_coder);
			if (ret < 0)
			{
				penc->coder_max_ccw = penc->last_coder;		//保存反向最大值
			}
			else
			{
				if (StepMotor_Get_Running(penc->step_idx))
					break;
				
				StepMotor_Modfiy_Position(penc->step_idx, 0, 1);
				StepMotor_exec(penc->step_idx, STEPS_ERROR_RANGE, 0);	//再往正向走一些
				
				penc->run.bit.raut = EAUTO_ZERO_CALC_CHECK;
			}
			break;

		case EAUTO_ZERO_CALC_CHECK:
			if (StepMotor_Get_Running(penc->step_idx))
				break;
			penc->run.bit.raut = 0;
			
			enc_diff = penc->coder_max_cw - penc->coder_max_ccw;
			if (enc_diff < 0)
				enc_diff += ENCODER_PPR;
			
			MSG_ENCODER("[%d] cw[%d] - ccw[%d] = [%d]\r\n",
						eid, penc->coder_max_cw, penc->coder_max_ccw, enc_diff);
			
			pos = ECalc_coder2step(eid, ENCODER_DIR_CCW, penc->coder_max_cw, penc->coder_max_ccw);
			
			if (pos < ((penc->steps_max_auto * 85)/100))	//行程小于最大值的 85%属于异常
			{
				if (enc_diff)
					enc_diff = 0 - enc_diff;
				else
					enc_diff = -1;

				EMsg_autoZero(eid, enc_diff);
			}
			else /* 计算最大行程步数 并确认零位编码位置 */
			{
				if (penc->sta.bit.stype == 0)
				{
					enc_diff = penc->coder_max_ccw + (ENCODER_PPR * 15/ penc->steps_ppr);	// 零位值往正向移15步
				}
				else
				{
					enc_diff >>= 1;							//取两端值的中间位置
					enc_diff += penc->coder_max_ccw;
				}
				enc_diff %= ENCODER_PPR;
				penc->coder_zero = enc_diff;
				
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
			StepMotor_Reset(penc->step_idx);
			//break;
			
		default:
			penc->run.bit.raut = 0;
			StepMotor_reback_curr(penc->step_idx);
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
	uint8_t cnterr;		// > 50 的次数
	uint8_t rsvd;
	uint8_t diffcnt[DPOS_DCNT];	// 位置偏差分布计数
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
		g_encoder[eid].check_delay = ENCODER_CHECK_DELAY_RUN;
	}
	else
	{
		g_encoder[eid].run.bit.rchk = 0;
		g_encoder[eid].check_delay = 0;
	}
	
	g_encoder[eid].alert_count = 0;
	
	return 0;
}

int Encoder_setDir(int eid, int dir)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	g_encoder[eid].sta.bit.sdir = dir ? ENCODER_DIR_CCW : ENCODER_DIR_CW;
	return 0;
}

/* 
设置电机类型
type: 0: 总步数不超出一圈 1总步数超出一圈
*/
int Encoder_setType(int eid, int type)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	g_encoder[eid].sta.bit.stype = type ? 1 : 0;
	return 0;
}

/*
设置编码器一圈对应步数
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
设置位置检查报警阀值
threshold: 2 <= x <= (ENCODER_PPR >> 2)
*/
int Encoder_setThreshold(int eid, unsigned short threshold)
{
	long temp;
	
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
设置当前编码器值为零位
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
设编码器零位值
zero: 0-(ENCODER_PPR-1)
注: 设置值后会保存到flash上
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
	if (penc->coder_zero != zero)
	{
		penc->coder_zero = zero;
		penc->run.bit.rpos = 1;	//重新计算当前位置
		penc->check_delay = ENCODER_CHECK_DELAY_RST;
	#if ENCODER_SAVE_BOARD
		ESave_Cfg();
	#endif /* ENCODER_SAVE_BOARD */
	}
	
	DEBUG_ENCODER("[%d]E-Zero[%d]\r\n", eid, zero);
	return 0;
}

/*
自动找零开启
maxstep: 自动找零对应的电机总行程步数(不是精确值)
*/
void Encoder_autoZero(int eid, unsigned short maxstep)
{
	int ret;
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
	
	penc->run.all = 0;
	penc->run.bit.raut = EAUTO_ZERO_START;
	penc->check_delay = ENCODER_CHECK_DELAY_RUN;
	penc->alert_count = 0;
	
	MSG_ENCODER("[%d]E-autoZero maxstep[%d]\r\n", eid, maxstep);
}

/*
编码零位值
*/
int Encoder_getZero(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	return (int)g_encoder[eid].coder_zero;
}

/*
发错误码到主机
*/
void Encoder_sendError(void)
{
	Massage_Send(coder_err);
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
编码器回读 -- 立即
*/
int Encoder_getCoder(int eid, int *coder, int *state)
{
	int ret;
	ENCODER_t *penc;

	if (eid >= ENCODER_NUM_MAX)
		return -1;

	penc = &g_encoder[eid];
	
	ret = ERead_Coder(eid, penc);
	
	if (coder)
	{
		*coder = penc->last_coder;
	}
	
	if (state)
	{
		*state = penc->last_status;
	}
	
	return ret;
}

/*
将当前编码值换算成电机位置值，并设为当前电机位置
*/
int Encoder_rPos(int eid, int onoff)
{
	if (eid >= ENCODER_NUM_MAX)
		return -1;

	g_encoder[eid].run.bit.rpos = onoff ? 1 : 0;
	return 0;
}

/*
当前编码器值转换为步进pos
dir: [0-(steps-1)]
return: <0读取失败
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
		if (dir == ENCODER_DIR_CCW)	// 当前位置在零位的正向 pos >= 0
			*pos = postmp;
		else
			*pos = 0 - postmp;
	}
	
	return ret;
}

/*
位置自动修正
注意: 修正范围半圈之内
*/
int Encoder_adjPos(int eid)
{
	short mpos;
	ENCODER_t *penc;
	
	if (eid >= ENCODER_NUM_MAX)
		return -1;
	
	penc = &g_encoder[eid];
	if (penc->sta.bit.init == 0)	//零位未设置
		return -1;

	/* 必须清楚报警状态 */
	penc->sta.bit.alert = 0;
	if (penc->run.bit.radj)
		return -1;
	
	penc->run.all = 0;
	penc->run.bit.radj = 1;
	
	mpos = StepMotor_Get_Position(penc->step_idx);
	StepMotor_exec(penc->step_idx, mpos, 1);

	DEBUG_ENCODER("[%d]E-adj[mpos %d]\r\n", eid, mpos);
	return 0;
}

/*
调整所有报警过的电机位置
*/
void Encoder_adjPosAll(void)
{
	int eid;
	
	for (eid = 0; eid < encoder_nums; eid++)
	{
		if (g_encoder[eid].sta.bit.alert)
		{
			Encoder_adjPos(eid);
		}
	}
}

/*
电机运行位置检查开关
*/
void Encoder_setRunCheck(int eid, int onoff)
{
	if (eid >= encoder_nums)
		return;
	
	g_encoder[eid].sta.bit.runchk = onoff ? 1 : 0;
	DEBUG_ENCODER("[%d]RunCheck[%d]\r\n", eid, onoff);
}

/*
电机运动之前位置修正 
-- 此处修正为减少运行积累误差
*/
int Encoder_RunPos(int eid, short *pos)
{
	int ret;
	short epos, mpos, diff_pos;
	ENCODER_t *penc;
	
	ret = 0;

	do {
		if (eid >= encoder_nums)
			break;
		
		penc = &g_encoder[eid];
		if (penc->run.all)
		{
			if (penc->run.bit.radj == 0)
				break;
		}
		
		mpos = *pos;
		epos = ECalc_rPos(eid, penc);
		diff_pos = abs(mpos - epos);
		
		//DEBUG_ENCODER("[%d][mpos:%d epos:%d]\r\n", eid, mpos, epos);
		
		if (penc->run.bit.radj == 1
			|| diff_pos < (penc->steps_ppr >> 4))	//计算位置如果越界就不修正位置 -- 可能此处没有必要
		{
			*pos = epos;
			ret = 1;
		}
	} while (0);
	
	return ret;
}

/*
电机运行中编码器检测
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

	/* 主循环中需要处理的一些业务跳过运行检查 */
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
	else //未读取到值就用上一次的值
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

	/* 此处使用的编码值在主循环中读取 */
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

		if (time > 20)	//如果时间超过1ms则需要考虑计算的滞后步数
		{
			ret = ((speed * time) >> 2) >> 10/* 1000 */;
			if (pos < (ret + penc->step_max_alert))
			{
				penc->alert_count--;	//跳过该次
			}
		}
		
		if (penc->alert_count > 3)
		{
			penc->alert_count = 0;
			penc->sta.bit.alert = 1;
			EReport_Error(eid, ALM_ENC_POS, pos_diff);
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
电机复位对应编码器复位
return: <0时编码器启用，但是读取错误, 
		0 编码器未启用，
		>0编码器读取位置正常 pos为当前电机步数
*/
int Encoder_Reset(int eid, int *pos)
{
	int ret;
	int epos;
	ENCODER_t *penc;
	int overtime = 0xFFFFF;
	
	if (eid >= encoder_nums
		|| pos == NULL)
		return -1;
	
	penc = &g_encoder[eid];
	if (penc->sta.bit.son == 0)
		return 0;
	
	/* 只要复位就清原来的报警标志 */
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

	ret = ERead_Coder(eid, penc);
	if (ret < 0)
		return ret;

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
当前编码是否正常工作
*/
int Encoder_Work(int eid)
{
	if (eid >= ENCODER_NUM_MAX)
		return 0;
	
	return g_encoder[eid].sta.bit.son;
}

/*
编码器回读探测
eid: 0xFF 读取所有
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


void Encoder_Poll(void)
{
	static uint8_t eid = 0;
	int ret;
	short epos, mpos;
	ENCODER_t *penc;

	do {
		eid++;
		eid %= ENCODER_NUM_MAX;
		penc = &g_encoder[eid];
		
	#if (!ENCODER_REAL_TIME)
		if (penc->run.all == 0)
			break;
	#endif /* ENCODER_REAL_TIME */
	
		if (penc->sta.bit.son == 0)
			break;

		/* 电机运行时不延时读取 */
		if (StepMotor_Get_Running(penc->step_idx) == 0)
		{
			if (penc->check_delay)
			{
				penc->check_delay--;
				break;
			}
			penc->check_delay = ENCODER_CHECK_INTERVAL;
		}
		
#if ENCODER_DMA_SUPPORT
		ret = SSI_Update(eid);
		if (ret == 0)
			break;
#endif /* ENCODER_DMA_SUPPORT */
		
		ret = ERead_Coder(eid, penc);
		if (ret < 0)
			break;

		/* 通过两个方向堵转取最大编码值计算零位 */
		if (penc->run.bit.raut)
		{
			EAuto_Zero(eid, penc);
		}

		/* 标记零位 */
		if (penc->run.bit.rzero)
		{
			penc->run.bit.rzero = 0;
			penc->sta.bit.init = 1;
			penc->coder_zero = penc->last_coder;
			StepMotor_Modfiy_Position(penc->step_idx, 0, 0);//将当前位置设为0
		#if ENCODER_SAVE_BOARD
			ESave_Cfg();
		#endif /* ENCODER_SAVE_BOARD */
		}

		/* 零位未设置不往下执行 */
		if (penc->sta.bit.init == 0)
			break;

		/* 电机运行不执行以下处理 */
		if (StepMotor_Get_Running(penc->step_idx))
			break;

		/* 用编码器换算电机位置 */
		if (penc->run.bit.rpos)
		{
			epos = ECalc_rPos(eid, penc);
			StepMotor_Modfiy_Position(penc->step_idx, epos, 1);
			DEBUG_ENCODER("[%d]E-ePos[%d][%d]\r\n", eid, epos, penc->last_coder);
			penc->run.bit.rpos = 0;
		}
		else if (penc->sta.bit.alert == 0)/*  当前位置检查  */
		{
			//该处检查只在电机停止的情况下检查
			// 1. 电机运行结束判断电机停的位置是否正常
			// 2. 电机如果没有运行，但是编码值变化范围过大检查
			mpos = StepMotor_Get_Position(penc->step_idx);
			ECheck_Pos(eid, mpos, 3, penc);
		}
	}while (0);
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

		if (Step->moto_remap_config.moto_ecode_index!=i)
		{
			MSG_ENCODER("Stepno_enidx[%d]!=Encode_idx[%d]\r\n",Step->moto_remap_config.moto_ecode_index, i);
		}
		
		penc = &g_encoder[i];
		penc->step_idx = stepidx;
		penc->run.all = 0;
		penc->sta.all = 0;

		Encoder_setDir(i, Step->dir_High);	//Step->dir_High == 0 => ENCODER_DIR_CW
		
		if (stepidx > 3)
			penc->sta.bit.stype = 1;
		
		penc->sta.bit.runchk = 1;
		penc->sta.bit.stat = 1;
		penc->sta.bit.son =0;
		penc->coder_zero = 2;
		penc->steps_ppr = ENCODER_STEPS;
		penc->alert_count = 0;
		penc->check_delay = ENCODER_CHECK_DELAY_RST;
		penc->coder_max_alert = ENCODER_PPR * STEPS_ERROR_RANGE / ENCODER_STEPS;
		penc->step_max_alert = STEPS_ERROR_RANGE;
		penc->steps_max_auto = ENCODER_STEPS - ENCODER_AUTO_STEP_RANGE;
	}
	
#if ENCODER_SAVE_BOARD
	ELoad_Cfg();
#endif /* ENCODER_SAVE_BOARD */
	
	/* 第一次读取，如不读取可能第一次读取出错 */
	SSI_Read(0, 0, 0);
	
	Encoder_Probe(0xFF);	//@ 此时读取的编码值是默认方向的值 -- 如果主机方向有改数值会变

	//Encoder_Enable(0xFF);
	return 0;
}


/*----------------------------------------------------------------------------*/
/*
方案1: 模拟IO方式读取
	每个编码器读取时间 按最高速度算 1MHz的话需要 20uS左右
	8个电机读取至少需要170uS => 200uS => 5KHz
	理想情况下不考虑电机中断运算时间电机速度必须低于 5KHz

方案2: SPI读取编码器 16bits
	编码值通过编码器板的小单片机实时读取
	16位数据通过协议定义
	另外注意电机零位如果能保存到小单片机上的可能性

方案3: CPLD直接读取
	FSMC读取
*/
/*----------------------------------------------------------------------------*/

