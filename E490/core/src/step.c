//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arch.h"
#include "command.h"
#include "alert.h"
#include "config.h"
#include "step.h"
#include "platform_config.h"
#include "encoder.h"


#define DEBUG_STEP_PRINTF(...)		//myprintf(__VA_ARGS__)


#define FULL	0
#define HALF	1
#define QUARTER	2
#define EIGHTH	3

#define RESOLUTION	2

#define NEW_ZERO_DETECT_MODE


#define MAX_STEPS	2000L

/*定义生克最大步数800步*/
/*2018 12 27  生克电机最大行程由原来的1200改成2000*/
#define MAX_STEPS_SK	2000L		


STEP_BIND_cmd DM_motor_bind_cmd[2];/*最大2组*/

STEP_TYPE STEPMOTOR[STEP_NUM/*10*/];

extern volatile unsigned char Yarn_Step_Updown;

volatile unsigned int stepmotor_current[MOTOR_TYPE_COUNT];/*记录当前电机的电流值*/
volatile unsigned int stepmotor_speed[MOTOR_TYPE_COUNT][2];	//速度值 -Hz
volatile unsigned int stepmotor_speed_isset[MOTOR_TYPE_COUNT][2];	//是否已经设置

volatile unsigned int stepmotor_steps[MOTOR_TYPE_COUNT][2]; //步数，部分电机用到

volatile unsigned int stepmotor_AccSteps[MOTOR_TYPE_COUNT]; //加减速步数，

volatile unsigned char StepMotor_idself_sys[MOTOR_TYPE_COUNT];

short Step_postion_set[STEP_NUM][8];	/*目前最大支持8个工位*/
volatile unsigned int Step_check_is_check=0;/**/

volatile unsigned short Motor_acc_steps = 313;

volatile unsigned short Density_zero_check=0;/*by hlc 2019 02 26 默认负数方向超出当前值不检查零位状态，0表示默认全检查*/

extern volatile unsigned int basetime_tick;

#ifdef NEW_ALARM_STYLE

volatile unsigned short stepmotor_alarm_id[MOTOR_TYPE_COUNT][4];

volatile unsigned short which_head_code=0;

volatile unsigned short which_sys_code=0; 
	
#endif

//unsigned short step_postion_ecode[802];
/*定义步数少于这个值之后，电机减速*/
#define SLOWSPEED_STEPS	20

#ifdef DMSTEP_MOVERTOZERO_DEC_ADV
volatile unsigned short step_movetozero_adv[MOTOR_TYPE_COUNT];				//走零过程提前减速
#endif

volatile char step_log_remap[MOTOR_TYPE_COUNT+1]={0xFF,0,4,6,10,12,8};
volatile unsigned char motor_type_remap[MOTOR_TYPE_COUNT+1]={MOTOR_TYPE_SKINER,MOTOR_TYPE_ACTION,MOTOR_TYPE_FEET,MOTOR_TYPE_YARN,MOTOR_TYPE_LIFT,MOTOR_TYPE_DENSITY,MOTOR_TYPE_OTHER,0};

volatile unsigned int step_in_isr=0;

volatile unsigned char step_just_run=0;

volatile unsigned int step_run_mode;
//volatile unsigned int step_reset_speed;
//volatile unsigned int step_max_speed;
#ifdef STEP_MOTOR_DDDM_SUPPORT
volatile unsigned int step_base_speed[MOTOR_TYPE_COUNT];
#else
volatile unsigned int step_base_speed;

#endif
// by xhl 2010/05/19
//volatile unsigned int sinker_max_speed;
volatile unsigned int sinker_add_speed; //hlc 20140725

//volatile unsigned int sinker_max_speed_isset;   //hlc 20140905

volatile short DM_check_area_max=800;
volatile short DM_check_area_min=-1;

volatile unsigned char LX_ACT_check_lost_cnt[2];//

volatile unsigned char StepMotor_Count_MAX=0; //系统支持的电机数

volatile unsigned char StepMotor_LX_ACTStep_autoadj=1; //连兴动作电机自动修复功能 （默认使能）

volatile unsigned char F_ACT_Motor_MOVE_zero_type =0;//F型山板过-150的时候减速通过

#ifdef LX_ACT_SPECIAL

volatile unsigned char StepMotor_LX_ACT_CrossZero = 1;/*20190611 0--普通，1--连兴动作电机负数到0全部走出传感器再回去，复位过程也是一样*/

#else
volatile unsigned char StepMotor_LX_ACT_CrossZero = 0;/*20190611 0--普通，1--连兴动作电机负数到0全部走出传感器再回去，复位过程也是一样*/

#endif
// by xhl 2010/09/15
//volatile unsigned int sinker_base_speed;
//volatile unsigned int Step_Zero_Check_Delay[10];
//volatile unsigned int step_active;
//volatile unsigned int step_poweron;
// by xhl 2010/06/03
volatile unsigned char Head_Mode_=0;
// by xhl 2010/06/05
volatile unsigned int step_resolution[MOTOR_TYPE_COUNT];
// by xhl 2011/01/01

// by xhl 2012/03/05
volatile unsigned int step_interval;
// 复位时，电机离开零位后等待delay_time * 2ms后再归零
//volatile unsigned int step_reset_delay_time[STEP_NUM];

volatile unsigned int Sys_Step_need_check_ddm=0;

volatile int feet_enable;
#ifdef LOGOUT_ENABLE
extern volatile unsigned short logout_enable;
#endif


extern volatile unsigned char SYS_is_PowerOn;
extern volatile unsigned char Yarn_use_Step;
extern volatile unsigned char TZ_use_Step;

volatile unsigned int step_work_sign_alarmenable;

#ifdef E480_BOARD_V10
 volatile unsigned int POS_Yarn_DO=200;
#endif

extern unsigned short cpld_name;

extern volatile unsigned int act_check_enable;

#ifdef LOG_DEBUG_FOR_LX_AT_CH
extern volatile unsigned char enable_log_tosend_LX;
extern volatile unsigned char enable_log_zero_change;
				
#endif

/*压脚电机最大数量*/
#define MAX_MOTOR_COUNT_FEET	4
// by xhl 2012/03/08
volatile int Motor_run_max_step[MOTOR_TYPE_COUNT];
volatile int Motor_run_max_step_Feet[MAX_MOTOR_COUNT_FEET];/*20181102 压脚行程各电机自己参数*/

volatile int feet_alarm_enable;
volatile int feet_sts;
volatile int feet_used_num;
volatile int feet_first_act;

void check_feet_sts(STEP_TYPE *Step);
//volatile unsigned int testalarmcount = 0;

//extern void Message_Send_4halfword_debug(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);

extern void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);

extern void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);

int alert_push(int alert_code, int alert_arg);

void StepMotor_Feet_exec_no(int stno,int stidself, int arg,unsigned short otherarg);

#ifdef DEBUG_STEP
volatile unsigned int Debug_Count[2][4];
#endif


#define STEP_ZERO_ADJ_DEF 2

volatile int step_zero_adj_sk_lxex=130;/*连兴特殊生克电机需要加这么一个值*/

volatile int step_N_act_interval_lx=STEPS_LEAVEZERO_FOR_LX;/*连兴舢板负方向缺口的值，默认200，可改变*/

volatile int step_zero_adj[MOTOR_TYPE_COUNT];				//到了0位之后，还要走这么些步数
volatile int step_zero_detect[MOTOR_TYPE_COUNT];			//20170318 
volatile int step_alert_detect_setup[MOTOR_TYPE_COUNT];	//电机失步报警精度(按照电机类型来)

volatile int LX_ACT_zero_adj=STEP_ZERO_GOON_STEPS;

volatile int step_alert_detect_default=20;
// by xhl 2012/05/19
// 当生克位置小于设定值时，零位有效不报警
volatile int sinker_zero_area;
#ifdef ZERO2_SUPPORT
//volatile unsigned int step_zero2_enable;
volatile int step_setup_zero2_detect_steps;
volatile int step_setup_zero2_detect_steps_ex;

void StepMotor_Detect_Zero2(unsigned int stepno);
#endif


volatile unsigned int max_triangle_num;

#ifdef TRY_ZERO_MIDD
volatile unsigned short	 try_steps_to_zero=50;

#endif
volatile unsigned int Triangle_sts[STEP_NUM];					//


volatile unsigned char Steps_ID_NO[MOTOR_TYPE_COUNT+1][16];		//0-idall,1-idself_d,2-idself_s,3-idself_a,4-idself_f



//volatile unsigned int max_sinker_num;
int is_sinker(int stepno);
int is_sinker_or_other(int stepno);




#ifdef NEW_ALARM_STYLE

#define STEP_ERR_CODE(_idx,_type)	(((_type<MOTOR_TYPE_MAX) ? (stepmotor_alarm_id[_type-1][_idx]) :\
										(DEVICE_CONFIG_NUM_ERROR)) |0x03<<12)


#else


#if 0
#define STEP_ERR_CODE(_idx,_type)	((_type==MOTOR_TYPE_DENSITY) ? (STEP_ERR + _idx) : \
									((_type==MOTOR_TYPE_SKINER) ? (SINKER1_ERR + _idx) :\
									((_type==MOTOR_TYPE_ACTION) ? (TRIANGLE1_ERR+_idx):\
									((_type==MOTOR_TYPE_FEET) ? (FEET1_ERR + _idx):\
									((_type==MOTOR_TYPE_YARN)?(YARN_STEP1_ERR+_idx):\
									((_type==MOTOR_TYPE_LIFT)?(LIFT_STEP1_ERR+_idx):\
									((_type==MOTOR_TYPE_OTHER)?(JDM_STEP1_ERR+_idx):\
									(DEVICE_CONFIG_NUM_ERROR))))))))
#endif
#define STEP_ERR_CODE(_idx,_type)		Step_Err_Code_Fun(_idx,_type)

#endif



#define COUNTS(_X)	(sizeof(_X) / sizeof(_X[0]))


#ifdef STEP_DEBUG_HZ
//#define DEBUG_HZ_MAX_ACCSTEPS 	16

volatile unsigned int Step_pulse_HZ[] = {

2500,
3000,
3500,
4000,
4500,
5000,
5500,
6000,
6500,
7000,
7500,
8000,
8500,
9000,
9500,
10000


};

#define DEBUG_HZ_MAX_ACCSTEPS	COUNTS(Step_pulse_HZ)

#endif

#ifndef STEP_MOTOR_DDDM_SUPPORT
volatile unsigned int Step_pulse[] = {
	170, 165, 158, 150, 140,
	130, 115, 102,  87,  73,
	 65,  57,  49,  42,  37,
     31,  26,  21,  17,  13,
	 11,  8,    5,   3,   1,
};

volatile unsigned int Step_pulse_sinker[] = {
	170, 165, 158, 150, 140,
	128, 113, 98,  83,  70,
	 57,  47,  37,  31,  26,
	 21,  17,  13,  11,  8,
	 5,  3,    1,
};

volatile unsigned int Step_pulse_triangle[] = {
	170, 165, 158, 150, 140,
	128, 113, 98,  83,  70,
	 57,  47,  37,  31,  26,
	 21,  17,  13,  11,  8,
	 5,  3,    1,
};

volatile unsigned int Step_pulse_feet[] = {
	170, 165, 158, 150, 140,
	128, 113, 98,  83,  70,
	 57,  47,  37,  31,  26,
	 21,  17,  13,  11,  8,
	 5,  3,    1,
};

volatile unsigned int Step_pulse_Yarn[] = {
	170, 165, 158, 150, 140,
	128, 113, 98,  83,  70,
	 57,  47,  37,  31,  26,
	 21,  17,  13,  11,  8,
	 5,  3,    1,
};

volatile unsigned int Step_pulse_lift[] = {
	170, 165, 158, 150, 140,
	128, 113, 98,  83,  70,
	 57,  47,  37,  31,  26,
	 21,  17,  13,  11,  8,
	 5,  3,    1,
};

#endif

#ifdef STEP_MOTOR_DDDM_SUPPORT

#if 0

#define ACC_STEPS_ACTION	(30)
#define ACC_STEPS_FEET  	(30)
#define ACC_STEPS_YARN  	(30)
#define ACC_STEPS_DM		(30)
#define ACC_STEPS_SINKER	(30)
#define ACC_STEPS_LIFT  		(30)


#else

#if 1
#define ACC_STEPS_ACTION	(36)//(17)
#define ACC_STEPS_FEET  		(17)
#define ACC_STEPS_YARN  		(17)
#define ACC_STEPS_DM		STEP_DDM_STEPS+1
#define ACC_STEPS_SINKER	(17)
#define ACC_STEPS_LIFT  		(17)

#else

#define ACC_STEPS_ACTION	(30)
#define ACC_STEPS_FEET  		(30)
#define ACC_STEPS_YARN  		(30)
#define ACC_STEPS_DM		(30)
#define ACC_STEPS_SINKER	(30)
#define ACC_STEPS_LIFT  		(30)


#endif

#endif
#else
#define ACC_STEPS_ACTION	COUNTS(Step_pulse_triangle)
#define ACC_STEPS_FEET  	COUNTS(Step_pulse_feet)
#define ACC_STEPS_YARN  	COUNTS(Step_pulse_Yarn)
#define ACC_STEPS_DM		COUNTS(Step_pulse)
#define ACC_STEPS_SINKER	COUNTS(Step_pulse_sinker)
#define ACC_STEPS_LIFT		COUNTS(Step_pulse_lift)
#endif



#ifdef E480_BOARD_V10

extern volatile unsigned int emf_status[3]; 
extern volatile unsigned int arch_board_id;
#endif



#define ACC_STEP	Step->acc_steps

//#define ZERO_WIDTH	1
#define ZERO_DETECT		Step->input_errorstep

//#ifdef SINKER_MAXSPEED_ISFAST

#ifdef NEW_STEP_12A

#define SK_ADD_SPEED_ALL 	(50)
#else
#define SK_ADD_SPEED_ALL 	(100)

#endif

#define SK_ADD_SPEED_FAST	(50)    //申克快速模式
//#else
//#define SK_ADD_SPEED_ALL 	(0)
//#endif


#define STEP_DIR_CHANGE_DELAY_MS	7

#define STEP_GO_SLOW_SPEED_STEPS    12

#define DEFSTEPMOTOR_MAX_SPEED_HZ 4000

#define DEF_STEPMOTOR_MAX_ACC_SPEED_HZ	100
#define DEF_STEPMOTOR_MAX_DEC_SPEED_HZ	-60


static unsigned short stepmotor_speed_max_DD=DEFSTEPMOTOR_MAX_SPEED_HZ;


unsigned int StepMotor_get_no_with_IDall(unsigned int);

static int stepspeed_Q2R(int Q,unsigned int mototype);

unsigned short  Step_Err_Code_Fun(unsigned char idself,unsigned char steptype)
{
	switch(steptype)
	{
		case MOTOR_TYPE_DENSITY:
		{
			return STEP_ERR+idself;
		}
		break;
		case MOTOR_TYPE_SKINER:
		{
			if(idself<2)
			{
				return SINKER1_ERR+idself;
			}
			else
			{
				return SINKER3_ERR+(idself-2);
			}
		}
		break;
		case MOTOR_TYPE_ACTION:
		{
			return TRIANGLE1_ERR+idself;
		}
		break;
		case MOTOR_TYPE_FEET:
		{
			return FEET1_ERR+idself;
		}
		break;
		case MOTOR_TYPE_YARN:
		{
			return YARN_STEP1_ERR+idself;
		}
		break;
		case MOTOR_TYPE_LIFT:
		{
			return LIFT_STEP1_ERR+idself;
		}
		break;
		case MOTOR_TYPE_OTHER:
		{
			return JDM_STEP1_ERR+idself;
		}
		break;
		default:
		{
			return DEVICE_CONFIG_NUM_ERROR;
		}
		break;
	}	

}

int Check_step_stop_timeout(unsigned int lastt)
{
	unsigned int time;
	time = basetime_tick;
	time -= lastt;
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	if ((time>>2) >= STEP_DIR_CHANGE_DELAY_MS)   	// 250us 转换成1ms 
	{
		return 0;
	}
	else return (STEP_DIR_CHANGE_DELAY_MS-(time>>2));		
}

int Get_zero_work_area(STEP_TYPE *Step)
{
	int ret;
	ret = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

	if (step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
	{
		ret +=step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//?step_zero_detect[Step->moto_remap_config.moto_type_config-1]:()
	}
	else
		if (step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
		{
			ret +=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else
		{
			ret +=step_alert_detect_default;
		}

	if(Step->moto_remap_config.moto_attr.bit.moto_type_exe==DENSITY_MOTOR_TYPE_LX)//取中间值
	{
		ret = (Step->moto_zero_width_self-Step->input_errorstep)>>1;
		ret+=Step->input_errorstep;
	}
	
	return  stepspeed_Q2R(ret,Step->moto_remap_config.moto_attr.bit.moto_type_config);
}


#ifdef TRY_ZERO_MIDD

void Set_try_steps_to_zero(unsigned short zs)
{
	if (zs)
	try_steps_to_zero = zs;
}

#endif



#ifdef NEW_ALARM_STYLE

void Set_Test_new_alsrm_style()
{
	int i;
	for (i=0;i<24;i++)
	{
		Message_Send_4halfword(STEP_ERR_CODE((i&0x03),(i>>2)+1),0,0,i);		
	}
}

void Set_stepmotor_alarm_id(unsigned int sysadd,unsigned int *systype)
{
	//volatile unsigned short stepmotor_alarm_id[MOTOR_TYPE_COUNT][4];
	int i;
	unsigned int sys1;
	unsigned int sys2;

	sys1 = (unsigned int)systype[0];
	sys2 = (unsigned int)systype[1];

	which_head_code =0;  //不分左右
	which_sys_code =0; //不分系统

	for (i=0;i<4;i++)
	{
		stepmotor_alarm_id[MOTOR_TYPE_DENSITY-1][i] = ((i<2)?(sysadd*2+i):(sysadd*2+i+(sys1-1)*2))+0x00;
		stepmotor_alarm_id[MOTOR_TYPE_SKINER-1][i] = ((i<2)?(i):0)+0x40 ;
		stepmotor_alarm_id[MOTOR_TYPE_ACTION-1][i] =( (i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0x60;
		stepmotor_alarm_id[MOTOR_TYPE_FEET-1][i] = ((i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0x80;
		stepmotor_alarm_id[MOTOR_TYPE_LIFT-1][i] = ((i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0xA0;
		stepmotor_alarm_id[MOTOR_TYPE_YARN-1][i] = i+0xC0;
		
	}
	which_sys_code = sysadd +1;

	if (sys2)
	{
		if (sys2 < 5)  // 4+4 模式(包括3+3,2+2,等)
		{
			if (sysadd>3)  // 4,5,6,7   换成 右边的0,1,2,3,
			{
				which_head_code =2;  //右边
				for (i=0;i<4;i++)
				{
					stepmotor_alarm_id[MOTOR_TYPE_DENSITY-1][i] = ((i<2)?((sysadd-4)*2+i):((sysadd-4)*2+i+(sys2-1)*2))+0x00;
					stepmotor_alarm_id[MOTOR_TYPE_DENSITY-1][i] |=(which_head_code<<8);   //表示右机头
					stepmotor_alarm_id[MOTOR_TYPE_SKINER-1][i] = ((i<2)?(i):0)+0x40 ;
					stepmotor_alarm_id[MOTOR_TYPE_SKINER-1][i] |=(which_head_code<<8);   //表示右机头
					stepmotor_alarm_id[MOTOR_TYPE_ACTION-1][i] = ((i<2)?( (i<1)?((sysadd-4)*1+i):((sysadd-4)*1+i+(sys2-1)*1)):(0))+0x60;
					stepmotor_alarm_id[MOTOR_TYPE_ACTION-1][i] |=(which_head_code<<8);   //表示右机头
					stepmotor_alarm_id[MOTOR_TYPE_FEET-1][i] = ((i<2)?( (i<1)?((sysadd-4)*1+i):((sysadd-4)*1+i+(sys2-1)*1)):(0))+0x80;
					stepmotor_alarm_id[MOTOR_TYPE_FEET-1][i] |=(which_head_code<<8);   //表示右机头
					stepmotor_alarm_id[MOTOR_TYPE_LIFT-1][i] =( (i<2)?( (i<1)?((sysadd-4)*1+i):((sysadd-4)*1+i+(sys2-1)*1)):(0))+0xA0;
					stepmotor_alarm_id[MOTOR_TYPE_LIFT-1][i]  |=(which_head_code<<8);   //表示右机头 
					stepmotor_alarm_id[MOTOR_TYPE_YARN-1][i] = i+0xC0;
					stepmotor_alarm_id[MOTOR_TYPE_YARN-1][i]  |=(which_head_code<<8);   //表示右机头
				}
				which_sys_code -=4;//右边机头箱，那就减去4个系统
			}
			else
			{
				which_head_code =1;  //左边
				for (i=0;i<4;i++)
				{
					stepmotor_alarm_id[MOTOR_TYPE_DENSITY-1][i] = ((i<2)?((sysadd)*2+i):((sysadd)*2+i+(sys1-1)*2))+0x00;
					stepmotor_alarm_id[MOTOR_TYPE_DENSITY-1][i] |=(which_head_code<<8); //表示左机头
					stepmotor_alarm_id[MOTOR_TYPE_SKINER-1][i] = ((i<2)?(i):0)+0x40 ;
					stepmotor_alarm_id[MOTOR_TYPE_SKINER-1][i] |=(which_head_code<<8);   //表示左机头
					stepmotor_alarm_id[MOTOR_TYPE_ACTION-1][i] = ((i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0x60;
					stepmotor_alarm_id[MOTOR_TYPE_ACTION-1][i] |=(which_head_code<<8);   //表示左机头
					stepmotor_alarm_id[MOTOR_TYPE_FEET-1][i] = ((i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0x80;
					stepmotor_alarm_id[MOTOR_TYPE_FEET-1][i] |=(which_head_code<<8);   //表示左机头
					stepmotor_alarm_id[MOTOR_TYPE_LIFT-1][i] =((i<2)?( (i<1)?(sysadd*1+i):(sysadd*1+i+(sys1-1)*1)):(0))+0xA0;
					stepmotor_alarm_id[MOTOR_TYPE_LIFT-1][i]  |=(which_head_code<<8);   //表示左机头
					stepmotor_alarm_id[MOTOR_TYPE_YARN-1][i] = i+0xC0;
					stepmotor_alarm_id[MOTOR_TYPE_YARN-1][i]  |=(which_head_code<<8);   //表示左机头
				}
			}
		}		
	}	
}	
#endif


#if 0

int Get_zero_work_area(STEP_TYPE *Step)
{
	int ret;
	ret = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

	if (step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
	{
		ret +=step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//?step_zero_detect[Step->moto_remap_config.moto_type_config-1]:()
	}
	else
		if (step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
		{
			ret +=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else
		{
			ret +=step_alert_detect_default;
		}
	
	return  stepspeed_Q2R(ret,Step->moto_remap_config.moto_attr.bit.moto_type_config);
}

#endif

unsigned long Get_STEPMOTO_add(unsigned int stepno)
{
	return (unsigned long)&STEPMOTOR[stepno];
}


void StepMotor_setVeryEnable(unsigned int verydata)
{
	unsigned int i;
	unsigned char stepid;

	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		stepid =Step->moto_remap_config.moto_remap_id_all;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable = (verydata>>stepid) & 0x01;		
	}
}


void StepMotor_setVeryEnable_log(unsigned int verydata)
{
#if 0
	unsigned int i;
	//unsigned char stepid;
	unsigned char whichbitno=0;
	
	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_UNDEF)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config<=MOTOR_TYPE_COUNT)
		{
			whichbitno = step_log_remap[Step->moto_remap_config.moto_attr.bit.moto_type_config];
			whichbitno += Step->moto_remap_config.moto_remap_id_self ;			
		}
		else
			continue;
		
		//stepid =Step->moto_remap_config.moto_remap_id_all;		
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable = (verydata>>whichbitno) & 0x01;	
		
	}

	/*
	dir bit位解释:
	bit0-1-2-3	度目电机	0-后左-1-后右，2-前左，3-前右		
	bit4-5	生克电机	4-后床，5-前床		
	bit6-7	动作电机	6-后床，7-前床		
	bit8-9	推针电机	8-后床，9-前床		
	bit10-11	压脚电机	10-后床，11-前床		
	bit12-13-14-15	纱嘴电机			

	*/
#endif	
}



void StepMotor_setActiveEnable_log(unsigned int activedata)
{
	unsigned int i;
	//unsigned char stepid;
	unsigned char whichbitno=0;
	
	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_UNDEF)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config<=MOTOR_TYPE_COUNT)
		{
			whichbitno = step_log_remap[Step->moto_remap_config.moto_attr.bit.moto_type_config];
			whichbitno += Step->moto_remap_config.moto_remap_id_self ;
			
		}
		else
			continue;
		
		//stepid =Step->moto_remap_config.moto_remap_id_all;		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable = (activedata>>whichbitno) & 0x01;
		
	}

	/*
	dir bit位解释:
	bit0-1-2-3	度目电机	0-后左-1-后右，2-前左，3-前右		
	bit4-5	生克电机	4-后床，5-前床		
	bit6-7	动作电机	6-后床，7-前床		
	bit8-9	推针电机	8-后床，9-前床		
	bit10-11	压脚电机	10-后床，11-前床		
	bit12-13-14-15	纱嘴电机			

	*/
	

	

}


void StepMotor_setActiveEnable(unsigned int activedata)
{
	unsigned int i;
	unsigned char stepid;

	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		stepid =Step->moto_remap_config.moto_remap_id_all;
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable = (activedata>>stepid) & 0x01;
		
	}

}

void StepMotor_setStep_Fastmode_log(unsigned int Fastdata)
{
	unsigned int i;
	//unsigned char stepid;
	unsigned char whichbitno=0;
	
	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_UNDEF)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config<=MOTOR_TYPE_COUNT)
		{
			whichbitno = step_log_remap[Step->moto_remap_config.moto_attr.bit.moto_type_config];
			whichbitno += Step->moto_remap_config.moto_remap_id_self ;
			
		}
		else
			continue;
		
		//stepid =Step->moto_remap_config.moto_remap_id_all;		
		Step->moto_remap_config.moto_attr.bit.is_fast_mode = (Fastdata>>whichbitno) & 0x01;
		
	}

	/*
	dir bit位解释:
	bit0-1-2-3	度目电机	0-后左-1-后右，2-前左，3-前右		
	bit4-5	生克电机	4-后床，5-前床		
	bit6-7	动作电机	6-后床，7-前床		
	bit8-9	推针电机	8-后床，9-前床		
	bit10-11	压脚电机	10-后床，11-前床		
	bit12-13-14-15	纱嘴电机			

	*/

}

void StepMotor_setStep_Fastmode(unsigned int Fastdata)
{
	unsigned int i;
	unsigned char stepid;

	for(i = 0; i < StepMotor_Count_MAX; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		stepid =Step->moto_remap_config.moto_remap_id_all;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode = (Fastdata>>stepid) & 0x01;
		
	}

}
#if 0
unsigned char Get_StepMotor_isveryenable_NO(unsigned char stepno)
{
	if (stepno>=StepMotor_Count_MAX)
		return 1;
	{
		STEP_TYPE *Step = &STEPMOTOR[stepno];
		return Step->moto_remap_config.moto_attr.bit.is_verystep_enable;
	}
}

#endif

unsigned char Get_StepMotor_isveryenable_IDall(unsigned int step_idall)
{
	unsigned int i;
	//unsigned char stepid;
	i=Steps_ID_NO[0][step_idall &0x0f];

	if (i<StepMotor_Count_MAX)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_remap_id_all == step_idall)
		{
				return Step->moto_remap_config.moto_attr.bit.is_verystep_enable ;
		}
		else
			return 0x01;
	}
	return 0x01;
}

unsigned char Get_StepMotor_isveryenable_IDself(unsigned int step_IDself,unsigned int step_type)
{
	unsigned int i;
	//unsigned char stepid;

	if (step_type<=MOTOR_TYPE_COUNT)
	{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
	}
	else
	{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self == step_IDself)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == step_type))
		{
				return Step->moto_remap_config.moto_attr.bit.is_verystep_enable ;
		}
		else
			return 0x01;
	}
	return 0x01;
}



void init_step_data_fill_zero()
{
	memset((void *)Steps_ID_NO,0xff,(MOTOR_TYPE_COUNT+1)<<4);
	memset((void *)STEPMOTOR,0x00,sizeof(STEPMOTOR));
}

void Stepmotor_alert_delay_init(STEP_TYPE *Step)
{
		Step->alert_delay_cnt = 0;
		Step->alert_delay_max = 0;
		Step->alert_delay_zerolost_cnt= 0;		
}



void Step_msg_init(STEP_TYPE *Step,unsigned char stepno,unsigned int idself,unsigned int type)
{
		int i = stepno;

		Step->moto_remap_config.moto_remap_id_all = stepno;
		Step->moto_remap_config.moto_attr.bit.moto_type_config = type;
		Step->moto_remap_config.moto_remap_id_self= idself;
		
		Step->Had_next_cmd=0;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test=0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		switch (Step->moto_remap_config.moto_attr.bit.moto_type_config)
		{
			case MOTOR_TYPE_UNDEF:			/*未定义*/
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0;
				}
				break;
			case MOTOR_TYPE_DENSITY:		/*度目电机*/
				{
					Step->moto_remap_config.moto_attr.bit.moto_ecode_index=i;
				}
				break;
			case MOTOR_TYPE_SKINER:			/*生克*/
				break;
			case MOTOR_TYPE_ACTION:		/*三角*/
				{
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
					Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				}
				break;
			case MOTOR_TYPE_FEET:			/*压脚*/
				{
					Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
					Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
					//Step->moto_remap_config.moto_attr.bit.moto_type_exe =FEET_MOTOR_TYPE_CX2;
					#ifdef CX_FEET_INPUT_NC
					Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg =1;
					Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg =1;
					#endif
				}
				break;
			case MOTOR_TYPE_YARN:			/*纱嘴*/
				{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
				}
				break;
			case MOTOR_TYPE_LIFT:			/*推针*/
				{
					if (Head_Mode_ == HEAD_MODE_SKMODE2)  /*特殊的，可以把推针电机接到非推针位置上*/
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = 1;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
						
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =((i-6)>=0)?(i-6):i;
						//Step->input_errorstep = 6;
					}
					Step->input_errorstep = 6;
				}
				break;
			case MOTOR_TYPE_OTHER:			/*自定义*/
				break;
				
		}
	
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
	
}


void Stepmotor_remape_SKmode2(char is2sk)   //普通2生克
{
	unsigned int i,j;
	unsigned char is_no_tzboard=0;

	//if(is2sk)
	{
		init_step_data_fill_zero();
	}
	#if 0
	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}
	#endif

	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		if(is2sk)	
		{
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;	
				Step->moto_remap_config.moto_remap_id_self = i-4;
				//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i+8;		
			}
			else
				if (i<8)
				{
					if (TZ_use_Step)
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;//MOTOR_TYPE_ACTION;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0;
						Step->moto_remap_config.moto_remap_id_self = i-6;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-6;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = 1;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
						Step->input_errorstep = 6;
						#ifdef HF_
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
						#else
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
						#endif
						is_no_tzboard =1;
					}
				}
				else
				{
					if (i>=12)
					{
						if (TZ_use_Step)
						{
							//Step->step_max = 500;	//零位信号装中间位置
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
							Step->moto_remap_config.moto_remap_id_self = i-12;
							//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
							Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
							Step->input_errorstep = 6;
							#ifdef HF_
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
							#else
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
							#endif
						}
						else
						{
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;//MOTOR_TYPE_ACTION;
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0;
							Step->moto_remap_config.moto_remap_id_self = i-12;
						}
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;

					}
					
				}
		#else
			{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					if (arch_Get_ID())
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
					}
			}
		#endif	
		}
		else
		{
			#ifdef E480_BOARD_V10		
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
					Step->moto_remap_config.moto_remap_id_self = i-8;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
					Step->moto_zero_width = 20;
					Step->moto_zero_width_self= 66;
					
				}
			#else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					if (arch_Get_ID())
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
					}
				}
			#endif	
		}
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg)) 
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
		
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 1;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);

		
	}
	#ifdef ENCODER_SUPPORT
	if(is_no_tzboard)
	{
		Encoder_set_stepno();	
	}
	#endif


}

#if 0
void Stepmotor_remape_SKmode4()   //普通2生克
{
	unsigned int i,j;

	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10		
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
					Step->moto_remap_config.moto_remap_id_self = i-8;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
					Step->moto_zero_width = 20;
					Step->moto_zero_width_self= 66;
					
				}
		#else
			{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					if (arch_Get_ID())
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
					}
			}
		#endif	

		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)) {
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 1;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);

		
	}
}

#endif


#ifdef E490_V10_BOARD

void Step_set_LX_act_motor_par()
{
	step_zero_adj[MOTOR_TYPE_ACTION-1] = 20;//;
	step_alert_detect_setup[MOTOR_TYPE_ACTION-1]=0; //20180713由30改成0，默认关闭  //30;//100;//30;//100;
	step_base_speed[MOTOR_TYPE_ACTION-1] 	= STEP_START_SPEED_HZ_ACT_LX;
	stepmotor_speed[MOTOR_TYPE_ACTION-1][1]= (ACT_MAX_SPEED_LX << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
}

void Step_zero_adj_set()
{
			step_zero_adj[MOTOR_TYPE_DENSITY-1] = 6;////10;//;
			step_zero_adj[MOTOR_TYPE_SKINER-1] = 20;//;
			step_zero_adj[MOTOR_TYPE_FEET-1] = 20;//;			
			step_zero_adj[MOTOR_TYPE_YARN-1] = 20;//;
			//step_zero_adj[MOTOR_TYPE_LIFT-1] = 20;//;
			Step_set_LX_act_motor_par();


}



void Stepmotor_remape_Lift_LX(char islx)	// add by hlc 20161129
{
	unsigned int i,j;

	init_step_data_fill_zero();
#if 0
	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}
	#endif

#ifdef LX_ACT_DONOT_ADJ
if (islx)
{
	LX_ACT_check_lost_cnt[0]=0;
	LX_ACT_check_lost_cnt[1]=0;
}	
#endif

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		//Step->Triangle_sts = 0;
		Step->step_debug_test_st.bit.justrun_test =0;
		Step->step_reset_delay_time =10;
		Step->Had_next_cmd=0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				Step->moto_remap_config.moto_attr.bit.moto_type_exe = islx?ACT_MOTOR_TYPE_LX:ACT_MOTOR_TYPE_FH;
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
			}
			else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-6;/*0,1号生克电机*/
					//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
					if(islx)
						Step->alert_delay_max= ALERT_DELAY_CNT_DEFAULT;

#if 0
					if (((arch_Get_ID()==0) &&(i==7))
						||((arch_Get_ID()==2) &&(i==6)))
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_exe =SINKER_MOTOR_TYPE_LX_EX;
					}

					#endif
					
				}
				else
				{
					#ifdef E692_STEPMOTOR_EX
					
					if(i<12)
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
					else
					if (i<14)
					{
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
					}
					else
					{					
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
						Step->moto_remap_config.moto_remap_id_self = i-14+2;/*2,3号生克电机*/					
						if(islx)
							Step->alert_delay_max= ALERT_DELAY_CNT_DEFAULT;
						
					}
						
					#else
				
					if (i>=12)
					{
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
					#endif
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_exe ==ACT_MOTOR_TYPE_FH)
				Step->step_st.bit.zero2_mode = 1;
			else
				Step->step_st.bit.zero2_mode = 0;
			//Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	Step_zero_adj_set();

}

#if 0
void Stepmotor_remape_Lift_FH()	// add by hlc 20170703
{
	unsigned int i,j;

	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		//Step->Triangle_sts = 0;
		Step->step_debug_test_st.bit.justrun_test =0;
		Step->step_reset_delay_time =10;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				Step->moto_remap_config.moto_attr.bit.moto_type_exe = ACT_MOTOR_TYPE_FH;
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
			}
			else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
				}
				else
				{
					if (i>=12)
					{
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)) {
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		

		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_exe ==ACT_MOTOR_TYPE_FH)
				Step->step_st.bit.zero2_mode = 1;
			else
				Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	Step_zero_adj_set();

}

#endif


//#define DOTACT_STEP /*by hlc 20910926 动作电机零位用一个点表示*/


//void Stepmotor_remape_Lift_HP()	// add by hlc 20160630
void Stepmotor_remape_Lift_HP(char isHP)	// add by hlc 20160630
{
	unsigned int i,j;
	
	init_step_data_fill_zero();
#if 0
	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}
	#endif

	for (i = 0; i < StepMotor_Count_MAX/*10*/; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Step->Had_next_cmd=0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//?a1μ??°￡??íê??é±?μ??ú		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //?a1 ±íê?ê1?ü
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		if(isHP)
		Step->input_errorstep = 6;
		//Step->zero_width= 66;
		
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->step_max = MAX_STEPS/*1000*/;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
	
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->pos_2_main = 2000;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//oó???ùéè????è?μ?μ??úààDí
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				if(isHP)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_exe = ACT_MOTOR_TYPE_HP;
					Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =1;
				}
				#ifdef DOTACT_STEP
				else
					Step->moto_remap_config.moto_attr.bit.moto_type_exe = ACT_MOTOR_TYPE_DOT;
				#endif
			}
			else
				if (i<8)
				{
					if(isHP)
					{	
						if (TZ_use_Step) 
						{
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
							Step->moto_remap_config.moto_remap_id_self = i-6;
						}//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
						else
						{
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
							Step->moto_remap_config.moto_remap_id_self = i-6;
							//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable = 1;
							Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
						}
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
						Step->moto_remap_config.moto_remap_id_self = i-6;
						//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;

					}
				}
				else
				{
					if (i>=12)
					{
						if(isHP)
						{
							if (TZ_use_Step)
							{
							//Step->step_max = 500;	//á???D?o?×°?D??????
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
							Step->moto_remap_config.moto_remap_id_self = i-12;
							//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
							Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
							}
							else
								Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;
						}
						else
						{
							//Step->step_max = 500;	//á???D?o?×°?D??????
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
							Step->moto_remap_config.moto_remap_id_self = i-12;
							//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
							Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
							Step->input_errorstep = 6;
							#ifdef HF_
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
							#else
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
							#endif		
						}
						
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //?a0 ±íê????ü
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
			
		Step->step_st.bit.is_poweron =0;		//0±íê??1???ˉ1y
		Step->error_count_check = 0;	//timer ?D?ì2aê?·?μ???μ???êy?÷
		Step->input_errorstep = 0;	//??è??T?ó2?;//?1??óD??3??ó2?
	
		
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if(isHP)
			{
				#ifdef QL_
				Step->step_st.bit.dir_High = (Step->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_HP)?1:0;
				#else
				if (!( i & 0x01 ))
				{
					Step->step_st.bit.dir_High = 0;
				}
				else
				{
					
					Step->step_st.bit.dir_High = (Step->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_HP)?0:1;
				}
				#endif
			}
			else
			{
				if (!( i & 0x01 ))
				{
					Step->step_st.bit.dir_High = 0;
				}
				else
				{
					Step->step_st.bit.dir_High = 1;
				}
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = isHP?1:0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

}



//#define TEST_LIFT_STEP_ZERO_DIR	

#if 0
void Stepmotor_remape_Lift_ex()	// add by hlc 20160630
{
	unsigned int i,j;
	
	for (i = 0; i <= MOTOR_TYPE_COUNT; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for (i = 0; i < StepMotor_Count_MAX/*10*/; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->step_max = MAX_STEPS/*1000*/;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->pos_2_main = 2000;
		Step->step_alert_st.all=0;
		
		if (i<4)
		{
			#ifdef TEST_LIFT_STEP_ZERO_DIR
//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;						
						Step->input_errorstep = 6;
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;		//后面再设置默认的电机类型
						Step->moto_remap_config.moto_remap_id_self = i;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = 1;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;	
						#ifdef HF_
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
						#else
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
						#endif
			#else
			
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				#if 0
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
				
				#endif
			}
			else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
				}
				else
				{
					if (i>=12)
					{
			#ifdef TEST_LIFT_STEP_ZERO_DIR			
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
						Step->moto_remap_config.moto_remap_id_self = i-12;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
						#ifdef LX_DM_SPECIAL_
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
						#else
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
						#endif
			#else
						
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
						Step->input_errorstep = 6;
						#ifdef HF_
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
						#else
						Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
						#endif
			#endif			

						
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)) {
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

}

#endif

void Stepmotor_remap_allstep_cx()
{
	unsigned int i,j;
	
	for (i = 0; i <= MOTOR_TYPE_COUNT; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for (i = 0; i < StepMotor_Count_MAX/*10*/; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->step_max = MAX_STEPS/*1000*/;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				Step->moto_remap_config.moto_attr.bit.moto_type_exe = ACT_MOTOR_TYPE_LX;
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
			}
			else
				if (i<8)
				{						
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_FEET;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
					//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
					#ifdef CX_FEET_INPUT_NC
					Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg =1;
					Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg =1;
					#endif
					#ifdef IS_CX_FEET_3_INPUT_LIKE_SAMPLE
						Step->moto_remap_config.moto_attr.bit.moto_type_exe =FEET_MOTOR_TYPE_CX3;
						Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=1;
						Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST= 1;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = 1;
					#endif
				}
				else
				{					
					if(i<12)
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
					else
					if( i<14)
					{
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
						Step->moto_remap_config.moto_remap_id_self = i-14;
					}
				}				
		
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	//Step_set_LX_act_motor_par();
	Step_zero_adj_set();


}

void Stepmotor_remape_Feet(unsigned char iscxfeet)	// add by hlc 20160702
{
	unsigned int i,j;
	
	for (i = 0; i <= MOTOR_TYPE_COUNT; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for (i = 0; i < StepMotor_Count_MAX/*10*/; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->step_max = MAX_STEPS/*1000*/;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_FEET;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
				#ifdef CX_FEET_INPUT_NC
					Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg =1;
					Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg =1;
				#endif
			}
			else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
				}
				else
				{
					if (i>=12)
					{
						#if 0
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						//Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
						#else
						if (iscxfeet)
						{
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
							Step->moto_remap_config.moto_remap_id_self = i-12;
							Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
							//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
							//Step->moto_remap_config.moto_attr.bit.moto_type_exe= ACT_MOTOR_TYPE_LX;
							//Step->moto_zero_width_self= 180;
							//Step->input_errorstep= 6;
							Step->moto_remap_config.moto_attr.bit.is_activestep_enable = TZ_use_Step?1:0;
							Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i-6;
							Step->input_errorstep = 6;
							#ifdef HF_
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_HF;
							#else
							Step->moto_remap_config.moto_attr.bit.moto_type_exe=LIFT_MOTOR_TYPE_PT;
							#endif

						}
						else	
						{
							Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
							Step->moto_remap_config.moto_remap_id_self = i-12;
							Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-12+4;	
							Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
							//Step->moto_remap_config.moto_attr.bit.moto_ecode_index =i;
							Step->moto_remap_config.moto_attr.bit.moto_type_exe= ACT_MOTOR_TYPE_LX;
							Step->moto_zero_width_self= 180;
							Step->input_errorstep= 6;
						}
						#endif
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	Step_set_LX_act_motor_par();

}




#endif

void Stepmotor_remape_Lift()	// add by zhuqiwei 160601
{

#if 0
	unsigned int i,j;
	
	for (i = 0; i <= MOTOR_TYPE_COUNT; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for (i = 0; i < StepMotor_Count_MAX/*10*/; i ++)
	{
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe=ACT_MOTOR_TYPE_PT;
		
		Step->step_max = MAX_STEPS/*1000*/;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				#ifdef E490_V10_BOARD
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				#else
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i+8;	
				#endif
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 0;
				#if 0
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
				
				#endif

				
			}
			else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_LIFT;
					Step->moto_remap_config.moto_remap_id_self = i-6;
				}
				else
				{
					if (i>=12)
					{
						//Step->step_max = 500;	//零位信号装中间位置
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;		//
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = arch_is_EMF_2_SK_board()?1:0;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
				}
		#else
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
				Step->moto_remap_config.moto_remap_id_self = i-4;
				if (arch_Get_ID())
				{
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
				}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)) {
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
		{
			Step->step_st.bit.dir_High = i & 0x1;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
		}
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 0;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}
#endif
}


void Stepmotor_remape_Simple(char isSim)   //简易单系统
{
	unsigned int i,j;

	#if 0
	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}
	#else
	init_step_data_fill_zero();
	#endif

	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		Step->step_reset_delay_time =10;
		Step->step_debug_test_st.bit.justrun_test =0;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_remap_config.moto_attr.bit.moto_ecode_index =0xff;
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->pos_2_main = 2000;
		Step->step_alert_st.all=0;
		Step->last_stop_systick =0;
		Step->step_st.bit.last_dir=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
		#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;//MOTOR_TYPE_SKINER;	
				Step->moto_remap_config.moto_remap_id_self = i-4;
				#ifdef E490_V10_BOARD
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				#else
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i+8;	
				#endif
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 1;
				#if 0
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
				
				#endif
			}
			else
			if(isSim)
			{
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0;
					Step->moto_remap_config.moto_remap_id_self = i-6;
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
					Step->moto_remap_config.moto_remap_id_self = i-8;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
					Step->moto_zero_width = 20;
					Step->moto_zero_width_self= 66;
					
				}
			}
			else
			{
				#ifdef E499_BOARD_SUPPORT_
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					if (i>=12)
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;//MOTOR_TYPE_ACTION;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = arch_is_EMF_2_SK_board()?1:0;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
					
				}

				#else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
					Step->moto_remap_config.moto_remap_id_self = i-8;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
					Step->moto_zero_width = 20;
					Step->moto_zero_width_self= 66;
					
				}
				#endif
			}
		#else
			{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					if (arch_Get_ID())
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
					}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
			Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
		{
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
		Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 1;
			Step->act_is_justcheck_zero =0;   /*20191031 简易机零位的时候不查另外一个信号*/
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
	}
	if (!isSim)
	{
		step_zero_adj[MOTOR_TYPE_ACTION-1]=4;// ql TESHU
		stepmotor_speed[MOTOR_TYPE_ACTION-1][1]= (ACT_MAX_SPEED_SMP << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
		stepmotor_speed[MOTOR_TYPE_ACTION-1][0]= (ACT_BASE_SPEED_SMP << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
	}

}




#if 0
void Stepmotor_remape_default()
{
	unsigned int i,j;

	

	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Steps_ID_NO[i][j] = 0xff;
		}
	}

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		STEP_TYPE *Step = &STEPMOTOR[i];		
		Step->moto_remap_config.moto_remap_id_all =i;
		Steps_ID_NO[0][i]=i;
		//Step->Triangle_sts = 0;
		Step->step_debug_test_st.bit.justrun_test =0;
		Step->step_reset_delay_time =10;
		Stepmotor_alert_delay_init(Step);
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;
		Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		//Step->input_errorstep = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe =MOTOR_TYPE_EXE_PT;
		Step->check_work_pos = DEF_CHECK_WORK_POS;
		Step->chk.all=0;
		Step->st_no_cl.all=0;
		Step->steps_check_zero = 0;
		Step->pos_2_main = 2000;
		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =0;
		Step->step_alert_st.all=0;
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
			#ifdef LX_DM_SPECIAL_
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_LX;
			#else
			Step->moto_remap_config.moto_attr.bit.moto_type_exe=DENSITY_MOTOR_TYPE_PT;
			#endif
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;//MOTOR_TYPE_SKINER;	
				Step->moto_remap_config.moto_remap_id_self = i-4;
				#ifdef E490_V10_BOARD
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i-4+16;	
				#else
				Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i+8;	
				#endif
				Step->moto_remap_config.moto_attr.bit.is_fast_mode = 1;
				Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST =0;// 1;  //强隆默认1
				Step->moto_remap_config.moto_attr.bit.workPos_Work_ST=0;// 1;//强隆默认1
				#if 0
				
				Step->moto_zero_width_self= 180;
				Step->input_errorstep= 6;
				
				#endif
			}
			else
				#ifdef E499_BOARD_SUPPORT_
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					if (i>=12)
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;//MOTOR_TYPE_ACTION;
						Step->moto_remap_config.moto_remap_id_self = i-12;
						Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = i-12+4;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = arch_is_EMF_2_SK_board()?1:0;
					}
					else
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
						Step->moto_remap_config.moto_remap_id_self = i-8;
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
						Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
						Step->moto_zero_width = 20;
						Step->moto_zero_width_self= 66;
					}
					
				}

				#else
				if (i<8)
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
					Step->moto_remap_config.moto_remap_id_self = i-6;
					//Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0;
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_YARN;
					Step->moto_remap_config.moto_remap_id_self = i-8;
					Step->moto_remap_config.moto_attr.bit.is_activestep_enable = Yarn_use_Step?1:0;
					Step->moto_remap_config.moto_attr.bit.is_fast_mode = Yarn_use_Step?1:0;
					Step->moto_zero_width = 20;
					Step->moto_zero_width_self= 66;
					
				}
				#endif
		#else
			{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;
					Step->moto_remap_config.moto_remap_id_self = i-4;
					if (arch_Get_ID())
					{
						Step->moto_remap_config.moto_attr.bit.is_activestep_enable =0; //为0 表示禁能
					}
			}
		#endif	

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=i;
			
		Step->step_st.bit.is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)) {
			Step->position = 0;
		}
		else {
			Step->position = 100;
		}
		Step->pos_2 = Step->position;
		Step->step_st.bit.running = 0;

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->step_st.bit.dir_High =0x1;
		}
		else
			Step->step_st.bit.dir_High = i & 0x1;

	
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)
		{	
			Step->chk.bit.check_sts_enable =0;
			Step->change_dir_count =0;
			//Step->done_steps =0;
			//Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
Step->step_max = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_SKINER?MAX_STEPS_SK: MAX_STEPS/*1000*/;
		
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec = 313;
		Step->low_speed = 1000;
		Step->max_speed = 3000;
		#endif
		Step->alarm_step = 0;
		Step->step_st.bit.check_delay_count = 0;
		Step->step_check_interval = 0;

	#ifdef ZERO2_SUPPORT
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION )
		{
			Step->step_st.bit.zero2_mode = 1;
		}
		else
			Step->step_st.bit.zero2_mode = 0;
	#endif
		Step->steps = 0;
		Step->step_wait_time = 0;

		Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;

		//myprintf("\r\n==     Step-YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}
	step_zero_adj[MOTOR_TYPE_ACTION-1]=4;// ql TESHU
	stepmotor_speed[MOTOR_TYPE_ACTION-1][1]= (ACT_MAX_SPEED_SMP << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_ACTION-1][0]= (ACT_BASE_SPEED_SMP << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;

}

#endif

#ifdef DMSTEP_MOVERTOZERO_DEC_ADV
void StepMotor_Set_movetozero_adv_steps(unsigned short steps,unsigned short motortype )
{
	if ((motortype>MOTOR_TYPE_UNDEF)&&(motortype<=MOTOR_TYPE_COUNT))

	step_movetozero_adv[motortype-1] = steps;

}
#endif

void StepMotor_Init()
{
	switch (cpld_name)
	{
		default:
		case BOARD_NAME_E475:
			Head_Mode_ = HEAD_MODE_SKMODE2;
			Stepmotor_remape_SKmode2(1);
			break;
			
		case BOARD_NAME_E480:
			Head_Mode_ = HEAD_MODE_DEFAULT;	
//			Stepmotor_remape_default();
			Stepmotor_remape_Simple(0);
			break;
			
		case BOARD_NAME_E490:
			#ifdef E490_V10_BOARD
			#ifdef HF_
			Head_Mode_ = HEAD_MODE_SKMODE2;
			Stepmotor_remape_SKmode2(1);
			#else
			
			Head_Mode_ = HEAD_MODE_LIFT_EX;
			Stepmotor_remape_Lift_HP(0);
			#endif
			#else
			Head_Mode_ = HEAD_MODE_LIFT2;
			Stepmotor_remape_Lift();
			#endif
			break;
	}

	{
		int i=0;
		for (i=0;i<MOTOR_TYPE_COUNT;i++ )
		{
			stepmotor_speed_isset[i][0]=1;
			stepmotor_speed_isset[i][1]=1;
			step_resolution[i] = QUARTER;
			step_alert_detect_setup[i]=0;//默认20 改成默认0，关闭（20180713）;
			step_zero_adj[i] = 4;//;
			step_zero_detect[i] = 0;//2017 03 18 
			stepmotor_speed[i][0] =STEP_LOW_SPD;
		}
		step_zero_detect[MOTOR_TYPE_DENSITY-1]=20;
		step_zero_detect[MOTOR_TYPE_SKINER-1]=20;//2019 0618 改成20 //60; /*生克电机默认间隙大一些*/
		step_zero_detect[MOTOR_TYPE_OTHER-1]=20;
		
		step_zero_detect[MOTOR_TYPE_LIFT-1]=40;

		step_zero_adj[MOTOR_TYPE_ACTION-1]=20;// ql TESHU

		

		#ifdef SK_STEP_LOST_ALERT_0
			step_alert_detect_setup[MOTOR_TYPE_SKINER-1]=0;
		#endif

		#ifdef TZ_LOST_ALARM_STEPS_50
			step_alert_detect_setup[MOTOR_TYPE_LIFT-1]=15;
		#endif

		#ifdef LX_DM_MOVETOZERO_FAST_SPEED
			step_alert_detect_setup[MOTOR_TYPE_DENSITY-1]=100;
		#endif
		
	}	

	step_run_mode = 0;	
	stepmotor_AccSteps[MOTOR_TYPE_DENSITY-1] = ACC_STEPS_DM -1;
	stepmotor_AccSteps[MOTOR_TYPE_SKINER-1] = ACC_STEPS_SINKER -1;	
	stepmotor_AccSteps[MOTOR_TYPE_FEET-1] = ACC_STEPS_FEET -1;	
	stepmotor_AccSteps[MOTOR_TYPE_ACTION-1] = ACC_STEPS_ACTION -1;
	stepmotor_AccSteps[MOTOR_TYPE_YARN-1] = ACC_STEPS_YARN -1;
	stepmotor_AccSteps[MOTOR_TYPE_LIFT-1] = ACC_STEPS_LIFT -1;
	stepmotor_AccSteps[MOTOR_TYPE_OTHER-1] = ACC_STEPS_SINKER -1;

	stepmotor_speed[MOTOR_TYPE_ACTION-1][1]= (ACT_MAX_SPEED << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_YARN-1][1]= (YARN_MAX_SPEED << step_resolution[MOTOR_TYPE_YARN-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_DENSITY-1][1]  = (STEP_MAX_SPD << step_resolution[MOTOR_TYPE_DENSITY-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_SKINER-1][1]= (SK_MAX_SPEED << step_resolution[MOTOR_TYPE_SKINER-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_LIFT-1][1]= (LIFT_MAX_SPEED << step_resolution[MOTOR_TYPE_LIFT-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_FEET-1][1]= (FEET_MAX_SPEED << step_resolution[MOTOR_TYPE_FEET-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_OTHER-1][1]= (SK_MAX_SPEED << step_resolution[MOTOR_TYPE_SKINER-1]) >> QUARTER;
#ifdef DMSTEP_MOVERTOZERO_DEC_ADV
	step_movetozero_adv[MOTOR_TYPE_DENSITY-1]=50;
	step_movetozero_adv[MOTOR_TYPE_SKINER-1]=20;
	step_movetozero_adv[MOTOR_TYPE_ACTION-1]=ACC_STEPS_ACTION;
	step_movetozero_adv[MOTOR_TYPE_FEET-1]=20;
	step_movetozero_adv[MOTOR_TYPE_LIFT-1]=20;
	step_movetozero_adv[MOTOR_TYPE_YARN-1]=20;
	step_movetozero_adv[MOTOR_TYPE_OTHER-1]=20;
	

#endif
	
	
	#ifdef STEP_MOTOR_DDDM_SUPPORT
	step_base_speed[MOTOR_TYPE_DENSITY-1] 	= STEP_START_SPEED_HZ_DM;
	step_base_speed[MOTOR_TYPE_SKINER-1] 	= STEP_START_SPEED_HZ_SK;
	step_base_speed[MOTOR_TYPE_ACTION-1] 	= STEP_START_SPEED_HZ_ACT;
	step_base_speed[MOTOR_TYPE_FEET-1] 	= STEP_START_SPEED_HZ_FEET;
	step_base_speed[MOTOR_TYPE_YARN-1] 	= STEP_START_SPEED_HZ_YARN;
	step_base_speed[MOTOR_TYPE_LIFT-1] 	= STEP_START_SPEED_HZ_LIFT;
	step_base_speed[MOTOR_TYPE_OTHER-1] 	= STEP_START_SPEED_HZ_SK;
	#else
	step_base_speed = 0/*STEP_LOW_SPD*/;
	#endif
	
	stepmotor_current[MOTOR_TYPE_DENSITY-1]=DEFAULT_DA_CURRENT_DEN_MA;
	stepmotor_current[MOTOR_TYPE_SKINER-1]=DEFAULT_DA_CURRENT_SK_MA;
	stepmotor_current[MOTOR_TYPE_ACTION-1]=DEFAULT_DA_CURRENT_ACT_MA;
	stepmotor_current[MOTOR_TYPE_FEET-1]=DEFAULT_DA_CURRENT_FEET_MA;
	stepmotor_current[MOTOR_TYPE_YARN-1]=DEFAULT_DA_CURRENT_YARN_MA;
	stepmotor_current[MOTOR_TYPE_LIFT-1]=DEFAULT_DA_CURRENT_LIFT_MA;
	stepmotor_current[MOTOR_TYPE_OTHER-1]=DEFAULT_DA_CURRENT_SK_MA;

	sinker_add_speed= 0;   //hlc 2014-07-25
	// by xhl 2012/05/19
	sinker_zero_area = 10;//20190618 改成10 60;    //10/*0*/;0;

#ifdef ZERO2_SUPPORT
	//step_zero2_enable = 0;
	step_setup_zero2_detect_steps = 20;//30;//20;
	step_setup_zero2_detect_steps_ex =50;
#endif

	//feet_enable = 0;
	//feet_run_max_step = 500;
	Motor_run_max_step[MOTOR_TYPE_FEET-1]=500;

	feet_alarm_enable = 0;
	feet_sts = 0x0;
	feet_used_num = MAX_MOTOR_COUNT_FEET;
	{
		int i=0;
		for (i=0;i<feet_used_num;i++ )
		{
			Motor_run_max_step_Feet[i] = Motor_run_max_step[MOTOR_TYPE_FEET-1];
		}
	}
	feet_first_act = 0x0f;
	step_interval = 0;	
//#ifdef STEP_WORK_SIGN_SUPPORT
	step_work_sign_alarmenable =0;
//#endif
}

void StepMotor_Set_Reset_Delay_Time(int stepid, unsigned int delay)
{
	unsigned int stepno;
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	stepno =StepMotor_get_no_with_IDall((unsigned int)stepid);
	if (stepno>=StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	Step->step_reset_delay_time= delay;
}


#if 0
void StepMotor_Set_Mode(unsigned int mode)
{
	step_run_mode = mode;						//动作
	if(step_run_mode) step_zero_adj[MOTOR_TYPE_ACTION-1] = 10;
}
#endif

void StepMotor_Set_VeryMode(unsigned int verymask,unsigned char islog)
{
	//vary_step_enable = verymask;
	if (islog)
	{
		StepMotor_setVeryEnable_log(verymask);
	}
	else		
		StepMotor_setVeryEnable(verymask);
	
}

void StepMotor_Set_ZeroPos_WorkST(unsigned int steptype,unsigned char wst)
{
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{	
		unsigned int i;
		//unsigned char stepid;

		for(i = 0; i < StepMotor_Count_MAX; i ++)
		{
			STEP_TYPE *Step = &STEPMOTOR[i];
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config != steptype)
			{
				continue;
			}
			//if (steptype == MOTOR_TYPE_ACTION )
			{
				Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST= (wst & 0x01)?1:0;
				Step->moto_remap_config.moto_attr.bit.workPos_Work_ST= (wst & 0x02)?1:0;
			}
			
		}	
	
	}
	
}


void StepMotor_Set_Zero_go_Steps(unsigned int steptype,unsigned int gos)
{
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{	
		
		if ((gos<200)&&(gos>0))
		step_zero_adj[steptype-1] = gos;

		#if 0
		if (steptype ==MOTOR_TYPE_ACTION)
		{
			LX_ACT_zero_adj = gos;
		}
		#endif
	}
	
}

void StepMotor_Set_Zero_go_Steps_ex(unsigned int steptype, short gos)
{
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{	
		
		#if 1
		if (steptype ==MOTOR_TYPE_ACTION)
		{
			LX_ACT_zero_adj = gos;
		}
		#endif
	}
	
}

void StepMotor_Set_Work_disable_with_ZeroPos(unsigned int steptype, short dis)
{
	int i;
	STEP_TYPE *Step;
	//if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	if (steptype ==MOTOR_TYPE_ACTION)
	{	
		for(i = 0; i < StepMotor_Count_MAX; i++) 
		{
			Step = &STEPMOTOR[i];
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
				continue;
			Step->act_is_justcheck_zero = dis?1:0;
		}
		
	}
	
}

void StepMotor_set_motor_signal_edg_return(unsigned char steptype,unsigned char stepno,unsigned char rdata)
{
	int i;
	STEP_TYPE *Step;
	unsigned char stid = 0xff;

	
	if(stepno) 
		stid=stepno-1;
		
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		if((stid!=0xff) &&(stid!=Step->moto_remap_config.moto_remap_id_self))
		{
			continue;
		}
		Step->check_signal_edge = rdata;
	}
	
}



void StepMotor_alert_delay_cnt_set(unsigned char steptype,unsigned char stepno,unsigned char ncno)
{
	int i;
	STEP_TYPE *Step;
	unsigned char stid = 0xff;

	
	if(stepno) 
		stid=stepno-1;
		
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		if((stid!=0xff) &&(stid!=Step->moto_remap_config.moto_remap_id_self))
		{
			continue;
		}
		Step->alert_delay_max = ncno;
	}
	
}



void StepMotor_input_NCorNO_set(unsigned char steptype,unsigned char stepno,unsigned char ncno)
{
	int i;
	STEP_TYPE *Step;
	unsigned char stid = 0xff;

	#ifdef CX_FEET_INPUT_NC
	return;
	#endif

	#ifdef IS_CX_FEET_3_INPUT_LIKE_SAMPLE
	return;	
	#endif
	
	if(stepno) 
		stid=stepno-1;
		
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		if((stid!=0xff) &&(stid!=Step->moto_remap_config.moto_remap_id_self))
		{
			continue;
		}
		Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg = (ncno>>0) & 0x01;
		Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg = (ncno>>1) & 0x01;
		//Step->moto_remap_config.moto_attr.bit.moto_type_exe= (dm_type & 0x03);
	}
	
}


void StepMotor_Set_zero_type_(unsigned int steptype,unsigned char whichstep,unsigned int zt)
{
	
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{
		unsigned int i;
		//unsigned char stepid;
		unsigned char stid = 0xff;
		if(whichstep) 
			stid=whichstep-1;

		for(i = 0; i < StepMotor_Count_MAX; i ++)
		{
			STEP_TYPE *Step = &STEPMOTOR[i];
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config != steptype)
			{
				continue;
			}
			if((stid!=0xff) &&(stid!=Step->moto_remap_config.moto_remap_id_self))
			{
				continue;
			}

#ifndef IS_CX_FEET_3_INPUT_LIKE_SAMPLE
			if(steptype == MOTOR_TYPE_FEET)
				Step->moto_remap_config.moto_attr.bit.moto_type_exe=zt & 0x03; //zt?FEET_MOTOR_TYPE_CX:FEET_MOTOR_TYPE_PT;
#endif				
			
			if (steptype == MOTOR_TYPE_LIFT)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_exe= zt & 0x03; //?LIFT_MOTOR_TYPE_HF:LIFT_MOTOR_TYPE_PT;
				if(Step->moto_remap_config.moto_attr.bit.moto_type_exe == LIFT_MOTOR_TYPE_HF_EX)
				{
					Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg =1;	
				}
				else
				{
					Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg =0;
				}				
			}
	
			
			if (steptype == MOTOR_TYPE_ACTION )
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_exe= zt & 0x07;
				if (Step->moto_remap_config.moto_attr.bit.moto_type_exe ==ACT_MOTOR_TYPE_FH)
				{
					Step->step_st.bit.zero2_mode = 1;
				}
			}
			if (steptype == MOTOR_TYPE_SKINER)
			{
				#if 0
				if (((arch_Get_ID()==0) &&(i==7))
						||((arch_Get_ID()==2) &&(i==6)))
				{
					continue;
				}
				#endif
				Step->moto_remap_config.moto_attr.bit.moto_type_exe = zt?((zt==1)?SINKER_MOTOR_TYPE_LX:SINKER_MOTOR_TYPE_LX_EX):SINKER_MOTOR_TYPE_PT;
				if (Step->moto_remap_config.moto_attr.bit.moto_type_exe !=SINKER_MOTOR_TYPE_PT)
				{
					Step->alert_delay_max = 0;
				}
				else
				{
					if (Head_Mode_ == HEAD_MODE_LX_ACT)
					{
						Step->alert_delay_max = ALERT_DELAY_CNT_DEFAULT;
					}
				}

			}
			if (steptype == MOTOR_TYPE_DENSITY)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_exe= zt?DENSITY_MOTOR_TYPE_LX:DENSITY_MOTOR_TYPE_PT;
			}
			if (steptype == MOTOR_TYPE_YARN)
			{
				Step->moto_remap_config.moto_attr.bit.moto_type_exe= zt?YARN_MOTOR_TYPE_LX:YARN_MOTOR_TYPE_PT;
				if(Step->moto_remap_config.moto_attr.bit.moto_type_exe==YARN_MOTOR_TYPE_LX)
				{
					//Step->moto_zero_width_self=90;
				}
			}
		}
	}
	
}

void StepMotor_Set_Input_error(unsigned int stepno,unsigned int inerr)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if(stepno >= StepMotor_Count_MAX) return;
	
	Step = &STEPMOTOR[stepno];

	if (inerr>30) //(inerr<0)||(
	{			
		//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(0x08<<12)|(inerr&0xfff));	
		return;
	}
		


	
	Step->input_errorstep = inerr;
	
	
}
	
void StepMotor_Set_FastMode(unsigned int fastmask,unsigned char islog)
{

	if (islog)
		StepMotor_setStep_Fastmode_log(fastmask);
	else
		StepMotor_setStep_Fastmode(fastmask);
	
}

void StepMotor_Setup_Active(unsigned int active,unsigned char islog)
{
	//step_active = active;

	if (islog)
		StepMotor_setActiveEnable_log(active);
	else		
		StepMotor_setActiveEnable(active);
}

#ifdef E490_V10_BOARD

static volatile int STEP_Dir_remap_allstep[] = {0,1,2,3,14,15,4,5,8,9,10,11,6,7,12,13};	// 慈星全电机


static volatile int STEP_Dir_remap[] = {0,1,2,3,4,5,12,13,8,9,10,11,6,7,14,15};	// 强隆

static volatile int STEP_Dir_remap_default[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};	// 强隆

#endif

void StepMotor_Setup_Direction(unsigned int dir)
{
	STEP_TYPE *Step ;//= STEPMOTOR;
	unsigned int i;
	int idremap=0;
	
	DEBUG_STEP_PRINTF("M-SetDir[0x%x]\r\n", dir);
	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if((Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION) &&(Step->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_HP))
			continue;
		
		if ((Head_Mode_==HEAD_MODE_DEFAULT)||(Head_Mode_== HEAD_MODE_FH_ACT)||(Head_Mode_== HEAD_MODE_LX_ACT))
		{
				idremap =STEP_Dir_remap_default[Step->moto_remap_config.moto_remap_id_all];
		}
		else
			if ((Head_Mode_==HEAD_MODE_LIFT2)
				||(Head_Mode_==HEAD_MODE_LIFT_EX)							
				||(Head_Mode_==HEAD_MODE_FEET_CX))
			{
				
				idremap =STEP_Dir_remap[Step->moto_remap_config.moto_remap_id_all];
			}
			else
				if(Head_Mode_==HEAD_MODE_ALL_STEP_CX)
				{
					idremap =STEP_Dir_remap_allstep[Step->moto_remap_config.moto_remap_id_all];
				}
			else 
				if ((Head_Mode_==HEAD_MODE_SKMODE2)||(Head_Mode_==HEAD_MODE_LIFT_HP))	
				{
					if (TZ_use_Step)
					{
						idremap =STEP_Dir_remap[Step->moto_remap_config.moto_remap_id_all];
					}
					else
					{
						idremap =STEP_Dir_remap_default[Step->moto_remap_config.moto_remap_id_all];
					}
				}
				else
				{
					idremap =Step->moto_remap_config.moto_remap_id_all;
				}
		if(dir & (0x1 << idremap)) 
		{
			Step->step_st.bit.dir_High = 0;
			
		}
		else 
		{
			Step->step_st.bit.dir_High = 1;
		}
		if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
		{
			Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
		}
		
		
	#ifdef ENCODER_SUPPORT
		Encoder_setDir(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, Step->step_st.bit.dir_High,0);
	#endif /* ENCODER_SUPPORT */
	}
}


void StepMotor_Setup_Direction_log(unsigned int dir)
{
	STEP_TYPE *Step ;//= STEPMOTOR;
	unsigned int i;	
	unsigned char whichbitno=0;
	/*
	dir bit位解释:
	bit0-1-2-3	度目电机	0-后左-1-后右，2-前左，3-前右		
	bit4-5	生克电机	4-后床，5-前床		
	bit6-7	动作电机	6-后床，7-前床		
	bit8-9	推针电机	8-后床，9-前床		
	bit10-11	压脚电机	10-后床，11-前床		
	bit12-13-14-15	纱嘴电机	
	*/	
	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_UNDEF)
			continue;		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config<=MOTOR_TYPE_COUNT)
		{
			whichbitno = step_log_remap[Step->moto_remap_config.moto_attr.bit.moto_type_config];
			whichbitno += Step->moto_remap_config.moto_remap_id_self ;			
		}
		else
			continue;		
		if(dir & (0x1 << whichbitno)) 
		{
			Step->step_st.bit.dir_High = 0;
		}
		else 
		{
			Step->step_st.bit.dir_High = 1;
		}

		if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
		{
			Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
		}
		
	}			
}


void StepMotor_Setup_Resolution(unsigned int mototype,unsigned int rlt)
{
	STEP_TYPE *Step; //= STEPMOTOR;
	unsigned int i;

	for(i = 0; i < 16; i ++) {
		if((0x1 << i) == rlt) {
			break;
		}
	}
	if(i == 16) {
		return ;
	}
	step_resolution[mototype-1] = i;
	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
		Step= &STEPMOTOR[i];
		
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=mototype)
			continue;
		Step->step_max = (Step->step_max << step_resolution[mototype-1]) >> QUARTER;
		
		#ifdef ENCODER_SUPPORT
		Encoder_setSteps(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, (ENCODER_STEPS << step_resolution[mototype-1]) >> QUARTER);
		#endif /* ENCODER_SUPPORT */
	}

	arch_StepMotor_Set_UMS(step_resolution[mototype-1] ,mototype);

}

void Step_Motor_Set_Check_all(unsigned char whichtype,unsigned int step)
{
#ifdef SK_STEP_LOST_ALERT_0
	if (whichtype ==MOTOR_TYPE_SKINER)
		return ;
#endif

#ifdef TZ_LOST_ALARM_STEPS_50
	if (whichtype ==MOTOR_TYPE_LIFT)
		return ;
#endif
	#ifdef LX_DM_MOVETOZERO_FAST_SPEED
	if (whichtype ==MOTOR_TYPE_DENSITY)
		return;
#endif

	if ((whichtype>=MOTOR_TYPE_DENSITY) &&(whichtype<=MOTOR_TYPE_COUNT))
	{
		step_alert_detect_setup[whichtype-1] = step;
	}
}


#if 0
void StepMotor_Set_Check(unsigned int step)
{
	#ifdef LX_DM_MOVETOZERO_FAST_SPEED
	return;
	#endif

	// by xhl 2011/12/23
	if(step == 0) {
		step_alert_detect_setup[MOTOR_TYPE_DENSITY-1] = 0;
		return ;
	}

	step_alert_detect_setup[MOTOR_TYPE_DENSITY-1] = step;
	

}

#endif




void StepMotor_Set_zero_detect(unsigned char whichtype,unsigned int step)
{

	if ((whichtype>=MOTOR_TYPE_DENSITY) &&(whichtype<=MOTOR_TYPE_COUNT))
	{
		step_zero_detect[whichtype-1] = step;
	}
}

void StepMotor_Set_Ecoder_dir_withzerodir()
{
	STEP_TYPE *Step; //= STEPMOTOR;
	unsigned int i;
	
	for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) 
	{
		Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
		{
			#ifdef ENCODER_SUPPORT
			Encoder_setDir(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, !Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir,0);
			#endif /* ENCODER_SUPPORT */
		}
	}
	
}

void StepMotor_Set_zero_dir(unsigned char whichtype,unsigned short stepno,unsigned short dirdata)
{
	STEP_TYPE *Step; //= STEPMOTOR;
	unsigned int i;
	unsigned int idself;

	if ((whichtype>=MOTOR_TYPE_DENSITY) &&(whichtype<=MOTOR_TYPE_COUNT))
	{
		for(i = 0; i < StepMotor_Count_MAX/*10*/; i ++) {
			Step = &STEPMOTOR[i];
			if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
				continue;
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config != whichtype)
				continue;	
			
			if ((stepno & 0xff)==0xFF)
			{
				idself= Step->moto_remap_config.moto_remap_id_self;
				Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =(dirdata & (0x0001<<idself)) ?1:0;
				Step->moto_remap_config.moto_attr.bit.zero_dir_isset =1;
				//Message_Send_4halfword(0xaa,i,idself,Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir);
			}
			else
			{	
				if (Step->moto_remap_config.moto_remap_id_self!=stepno)
					continue;
				Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =dirdata?1:0;
				Step->moto_remap_config.moto_attr.bit.zero_dir_isset =1;
			}	
			#ifdef ENCODER_SUPPORT
			Encoder_setDir(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, !Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir,0);
			#endif /* ENCODER_SUPPORT */
		}	

		
	}
}



static int stepspeed_R2Q(int R,unsigned int mototype)
{
	R <<= QUARTER;
	R >>= step_resolution[mototype-1];

	return R;
}

static int stepspeed_Q2R(int Q,unsigned int mototype)
{
	Q <<= step_resolution[mototype-1];
	Q >>= QUARTER;

	return Q;
}


void StepMotor_Set_Speed_EX(unsigned char motortype,unsigned char whichspeed,unsigned int speed_hz)
{
#ifdef STEP_MOTOR_DDDM_SUPPORT

	if ((!motortype)||(motortype>MOTOR_TYPE_COUNT))
	{
		return;
	}
	if ( !((whichspeed==0)||(whichspeed==1)))
	{
		return;
	}
	if (speed_hz<SPET_MOTOR_MIN_SPEED_HZ)
	{
		speed_hz= SPET_MOTOR_MIN_SPEED_HZ;
	}
	if (speed_hz>SPET_MOTOR_MAX_SPEED_HZ)
	{
		speed_hz= SPET_MOTOR_MAX_SPEED_HZ;
	}

#ifdef LX_SK_SPEED_5K5
	if (motortype == MOTOR_TYPE_SKINER)
	return;
#endif

if ((motortype == MOTOR_TYPE_ACTION) &&(Head_Mode_ ==HEAD_MODE_DEFAULT))
{
	return;
}
	
	stepmotor_speed[motortype-1][whichspeed]=speed_hz;

#else
	return;
#endif

}



#ifdef STEP_MOTOR_DDDM_SUPPORT
unsigned int StepMotor_Speed_MainData2HZ(unsigned int speed)
{
	unsigned int ret_hz;
	switch (speed)
{
	case (173):		/*主控对应设置值1*/
		ret_hz = 1000;//2000;
		break;
	case (158):		/*主控对应设置值2*/
		ret_hz =1500;// 2200;
		break;	
	case (143):		/*主控对应设置值3*/
		ret_hz =2000; //2400;
		break;	
	case (128):		/*主控对应设置值4*/
		ret_hz = 2500;//2600;
		break;	
	case (113):		/*主控对应设置值5*/
		ret_hz = 2800;
		break;	
	case (98):		/*主控对应设置值6*/
		ret_hz = 3000;
		break;	
	case (83):		/*主控对应设置值7*/
		ret_hz = 4000;
		break;	
	case (68):		/*主控对应设置值8*/
		ret_hz = 5000;
		break;	
	case (53):		/*主控对应设置值9*/
		ret_hz = 6000;
		break;	
	case (43):		/*主控对应设置值10*/
		ret_hz = 7000;
		break;		
	default:
		
		ret_hz = 3000;
		break;
}

if (ret_hz < SPET_MOTOR_MIN_SPEED_HZ)
{
	ret_hz = SPET_MOTOR_MIN_SPEED_HZ;
}	
if (ret_hz > SPET_MOTOR_MAX_SPEED_HZ)
{
	ret_hz = SPET_MOTOR_MAX_SPEED_HZ;
}

return ret_hz;
}

#endif

void StepMotor_Set_Reset_Speed(unsigned int speed,unsigned int mototype)
{

	if ((mototype<=MOTOR_TYPE_COUNT) &&(mototype!=MOTOR_TYPE_UNDEF))
	{	unsigned int speed_tmp;
		

#ifdef STEP_MOTOR_DDDM_SUPPORT
		speed_tmp = StepMotor_Speed_MainData2HZ(speed);
		stepmotor_speed[mototype-1][0] =  speed_tmp;
		stepmotor_speed_isset[mototype-1][0]=1;
		if (mototype==MOTOR_TYPE_DENSITY)
 		{
			int i;

			for (i=MOTOR_TYPE_SKINER;i<= MOTOR_TYPE_COUNT ;i++)
			{
	
				if (!stepmotor_speed_isset[i-1][0])  //if not set then set it
				{
					stepmotor_speed[i-1][0] = speed_tmp;
				}
				else
					continue;

			}	 
		}

#else

		unsigned int step_max_speed_def = STEP_MAX_SPD;

		speed_tmp = stepspeed_R2Q(speed,mototype);

		if(speed_tmp <= STEP_MAX_SPD)
			stepmotor_speed[mototype-1][0]  = STEP_MAX_SPD;
		else
			stepmotor_speed[mototype-1][0]  = speed_tmp;


		
		stepmotor_speed_isset[mototype-1][0]=1;
		
	
 		if (mototype==MOTOR_TYPE_DENSITY)
 		{
			int i;

			for (i=MOTOR_TYPE_SKINER;i<= MOTOR_TYPE_COUNT ;i++)
			{
				if (!stepmotor_speed_isset[i-1][0])  //if not set then set it
				{
					speed_tmp = stepspeed_R2Q(speed,i);
					if(speed_tmp < step_max_speed_def)
						stepmotor_speed[i-1][0] = step_max_speed_def;
					else
						stepmotor_speed[i-1][0] = speed_tmp;

					#ifdef NEW_STEP_12A
					stepmotor_speed[i-1][0]  +=132;
					#endif
				}
				else
					continue;

			}	 
		}

#endif	
	}
	
}

unsigned char Stepmotor_get_Head_Mode()
{
	return Head_Mode_;
}

void Get_Step_data()
{
	unsigned int i,j;

	for (i =0; i<=MOTOR_TYPE_COUNT ; i++)
	{
		for (j=0;j<16;j++)
		{
			Message_Send_4halfword(Steps_ID_NO[i][j],(i<<8)|j,0,0);
			//Steps_ID_NO[i][j] = 0xff;
		}
	}
}

void StepMotor_Set_with_Head_Mode(unsigned int mode)
{
	#ifdef HF_
	return ;
	#endif
	
	if (Head_Mode_ == mode)
		return;

	if (mode >= HEAD_MODE_MAX)
		return;

	switch (cpld_name)
	{
		case BOARD_NAME_E475:
			if ((mode!=HEAD_MODE_SKMODE2) ||(mode!=HEAD_MODE_SIMPLE))
				return;
		//return;	
			break;
		case BOARD_NAME_E480:
			if (mode>HEAD_MODE_SKMODE4)
                		return; 
			break;
		case BOARD_NAME_E490:
			break;
			
		default:
			if ((mode!=HEAD_MODE_SKMODE2) ||(mode!=HEAD_MODE_SIMPLE))
				return;
			break;
	}

	Head_Mode_ = mode;
	
	switch (mode)
	{
		case HEAD_MODE_DEFAULT:
			if (Yarn_use_Step)
				StepMotor_Count_MAX =12;
			else
				StepMotor_Count_MAX =8;
			
			//Stepmotor_remape_default();
			Stepmotor_remape_Simple(0);
			break;
		case HEAD_MODE_SIMPLE:
			Stepmotor_remape_Simple(1);
			break;
		case HEAD_MODE_SKMODE2:
			Stepmotor_remape_SKmode2(1);
			break;
		case HEAD_MODE_SKMODE4:
			Stepmotor_remape_SKmode2(0);
			break;
		#ifdef E490_V10_BOARD
		case HEAD_MODE_LIFT_EX:
		case HEAD_MODE_LIFT2:
			Stepmotor_remape_Lift_HP(0);//Lift_ex;
			break;
		case HEAD_MODE_FEET:
			Stepmotor_remape_Feet(0);
			break;
		case HEAD_MODE_FEET_CX:  //0x0A
			Stepmotor_remape_Feet(1);
			break;
		case HEAD_MODE_LIFT_HP:
			Stepmotor_remape_Lift_HP(1);
			break;
		case HEAD_MODE_LX_ACT:
			Stepmotor_remape_Lift_LX(1);
			break;	
		case HEAD_MODE_FH_ACT:
			Stepmotor_remape_Lift_LX(0);	
			break;
		case HEAD_MODE_ALL_STEP_CX:
			Stepmotor_remap_allstep_cx();	
			break;
		#else
		case HEAD_MODE_LIFT2:
			Stepmotor_remape_Lift();
			break;
			
		#endif

	
			
		default:
			break;
	}
	return;
}


/*
	stepno :表示物理号。对应STEPMOTOR数组的坐标
	idself:表示逻辑号，与类型相关
	type: 类型
*/

int Get_id_self_with_StepType(unsigned int type)
{
	int i;
	for (i=0;i<16;i++)
	{
		if (Steps_ID_NO[type][i]==0xff)
		{
			break;
		}
		else
			continue;
	}
	return i;
	
}

void Stepmotor_Set_ID_Type_with_NO(unsigned char stepno,unsigned int idself,unsigned int type)
{
	STEP_TYPE *Step;
	unsigned int idself_tmp;
	
	
	if (stepno>=StepMotor_Count_MAX) return;
	if (idself>=16) return;
	if (type>=MOTOR_TYPE_MAX) return;

	//;

	Step=&STEPMOTOR[stepno];
	Steps_ID_NO[0][Step->moto_remap_config.moto_remap_id_all &0x0F]=0xff;  //先把之前的清空
	if ((Step->moto_remap_config.moto_attr.bit.moto_type_config<=MOTOR_TYPE_COUNT) 
		&&(Step->moto_remap_config.moto_attr.bit.moto_type_config !=MOTOR_TYPE_UNDEF))
		Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self & 0x0F]=0xff;	

	if(type == MOTOR_TYPE_OTHER)
	{
		idself_tmp=Get_id_self_with_StepType(type);
	}
	else
	{
		idself_tmp=idself;	
	}
	Step_msg_init(Step,stepno,idself_tmp,type);
	Steps_ID_NO[0][stepno]=stepno;	
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_UNDEF)
			Steps_ID_NO[Step->moto_remap_config.moto_attr.bit.moto_type_config][Step->moto_remap_config.moto_remap_id_self]=stepno;

	
}


void Stepmotor_Set_inputID_with_NO(unsigned char stepno,unsigned char in_zero,unsigned int id_work,unsigned int zw_mask)
{
	STEP_TYPE *Step;
	
	if (stepno>=StepMotor_Count_MAX) return;

	Step=&STEPMOTOR[stepno];
	if (zw_mask & 0x01)
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index = get_zeromapid(in_zero);
	else
		Step->moto_remap_config.moto_attr.bit.moto_zero_input_index =0xff;
	if (zw_mask & 0x02)
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index = get_zeromapid(id_work);
	else
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index =0xff;
	
	
}

void Stepmotor_Set_zerotype_with_NO(unsigned char stepno,unsigned char type_zero)
{
	STEP_TYPE *Step;
	
	if (stepno>=StepMotor_Count_MAX) return;

	Step=&STEPMOTOR[stepno];
	Step->moto_remap_config.moto_attr.bit.moto_type_exe = type_zero;	
}



void StepMotor_Other_Set_Work_Enable(unsigned char steptype,unsigned int isenable)
{
	if ((steptype >=MOTOR_TYPE_SKINER) &&(steptype <=MOTOR_TYPE_COUNT))
	{
		STEP_TYPE *Step;
		int i;
		for (i=0;i<StepMotor_Count_MAX ;i++)
		{
			Step=&STEPMOTOR[i];	
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config==steptype)
			{
			#ifdef ZERO2_SUPPORT
				if (Head_Mode_ == HEAD_MODE_FH_ACT)
					continue;
				Step->step_st.bit.zero2_mode =isenable?1:0;
			#endif
			}
		}		
	
	}
}


#ifdef E480_BOARD_V10
void StepMotor_Setup_ACTION_Para(int cmd, int para)
{
 #ifdef ZERO2_SUPPORT

	switch(cmd) 
	{	
	case 0:
		if(para >= 4)
			step_setup_zero2_detect_steps = para;
		break;
	case 1:		
		
		StepMotor_Detect_Zero2(StepMotor_get_IDall_with_IDself(para,MOTOR_TYPE_ACTION));
		break;
	case 2:
		//step_zero2_enable = para;
		StepMotor_Other_Set_Work_Enable(MOTOR_TYPE_ACTION,para);
		break;

	}
#endif	
}

#endif



void StepMotor_Other_SetSpeed(unsigned char steptype,unsigned char whichspeed,unsigned int speeddata)
{
	if (whichspeed)
	{
		StepMotor_Set_Speed(speeddata,steptype);
	}
	else
	{
		StepMotor_Set_Reset_Speed(speeddata,steptype);
	}
	
}

void StepMotor_Set_work_steps(unsigned char steptype,unsigned  int steps,unsigned char whichstepno)
{

	Motor_run_max_step[steptype-1] = steps;
	if (steptype == MOTOR_TYPE_FEET)
	{
		if(whichstepno==0)
		{
			int i=0;
			for (i=0;i<MAX_MOTOR_COUNT_FEET;i++)
			{
				Motor_run_max_step_Feet[i] = steps;
				feet_enable |=(0x01<<i);
			}				
		}
		else
		{
			if(whichstepno<=MAX_MOTOR_COUNT_FEET)
			{
				Motor_run_max_step_Feet[whichstepno-1] = steps;
			}
			feet_enable |=(0x01<<(whichstepno-1)) ;
		}
			
	}
	
}


void StepMotor_Other_Set_isfast(unsigned char steptype,unsigned int isfast)
{
	if ((steptype >=MOTOR_TYPE_SKINER) &&(steptype <=MOTOR_TYPE_COUNT))
	{
		if (steptype==MOTOR_TYPE_SKINER)
		{
			sinker_add_speed = isfast ? SK_ADD_SPEED_FAST:SK_ADD_SPEED_ALL;
		}
	
	}
}

#if 0
void StepMotor_Other_Set_Check(unsigned char steptype,unsigned int lostdata)
{
#ifdef SK_STEP_LOST_ALERT_0
	if (steptype ==MOTOR_TYPE_SKINER)
		return ;
#endif

#ifdef TZ_LOST_ALARM_STEPS_50
	if (steptype ==MOTOR_TYPE_LIFT)
		return ;
#endif

	if ((steptype >=MOTOR_TYPE_SKINER) &&(steptype <=MOTOR_TYPE_COUNT))
	{

		//if (steptype)
		if(lostdata == 0) {
			step_alert_detect_setup[steptype+1] = 0;
			return ;
		}

		step_alert_detect_setup[steptype+1] = lostdata;

#if 0
		if (steptype==0)
		{extern volatile unsigned int arch_board_id;

		alert_push(0x99+arch_board_id, lostdata);
		
		}
#endif
	}
}

#endif

void StepMotor_Set_Speed(unsigned int speed,unsigned int steptype)
{
	if ((steptype<=MOTOR_TYPE_COUNT) &&(steptype!=MOTOR_TYPE_UNDEF))
	{
		unsigned int speed_tmp;	
#ifdef STEP_MOTOR_DDDM_SUPPORT
		speed_tmp = StepMotor_Speed_MainData2HZ(speed);

		if (steptype==MOTOR_TYPE_SKINER)
		{
			#ifdef LX_SK_SPEED_5K5
				return;
			#endif
			
			speed_tmp = StepMotor_Set_SinkerSpeed(speed);
			//stepmotor_speed_isset[steptype-1][1]=1;
			//return;
		}

		stepmotor_speed[steptype-1][1] =  speed_tmp;
		stepmotor_speed_isset[steptype-1][1]=1;
		if (steptype==MOTOR_TYPE_DENSITY)
 		{
			int i;

			for (i=MOTOR_TYPE_SKINER;i<= MOTOR_TYPE_COUNT ;i++)
			{
	
				if (!stepmotor_speed_isset[i-1][1])  //if not set then set it
				{
					stepmotor_speed[i-1][1] = speed_tmp;
				}
				else
					continue;
			}	 
		}

#else
		unsigned int step_max_speed_def = STEP_MAX_SPD;
	
		speed_tmp = stepspeed_R2Q(speed,steptype);
		if(speed_tmp < step_max_speed_def)
			stepmotor_speed[steptype-1][1] = step_max_speed_def;
		else
			stepmotor_speed[steptype-1][1] = speed_tmp;

		stepmotor_speed_isset[steptype-1][1]=1;
	
 		if (steptype==MOTOR_TYPE_DENSITY)
 		{
			int i;

			for (i=MOTOR_TYPE_SKINER;i<= MOTOR_TYPE_COUNT ;i++)
			{
	
				if (!stepmotor_speed_isset[i-1][1])  //if not set then set it
				{
					speed_tmp = stepspeed_R2Q(speed,i);
					if(speed_tmp < step_max_speed_def)
						stepmotor_speed[i-1][1] = step_max_speed_def;
					else
						stepmotor_speed[i-1][1] = speed_tmp;
				}
				else
					continue;

			}	 
		}
#endif		
 	}

}

unsigned int  StepMotor_Set_SinkerSpeed(unsigned int speed)
{

#if 0
	unsigned int sk_max_speed_hlc=60;
	
	 #ifdef NEW_STEP_12A
	if ((speed>200) ||(speed<100))
	{
		speed =131;  //默认值 3K的频率
	}
#endif
	//sinker_max_speed_isset = 1;    //set it and record it

	 #ifdef NEW_STEP_12A
		speed +=98;  //默认+198否则太高了，要失步的确保(1-100不会失步)
		
	 #endif
		
	speed = stepspeed_R2Q(speed,MOTOR_TYPE_SKINER);

	if ( SK_MAX_SPEED>1)
		sk_max_speed_hlc = 1;
	
	if(speed < sk_max_speed_hlc)
		stepmotor_speed[MOTOR_TYPE_SKINER-1][1] = sk_max_speed_hlc;
	else
		stepmotor_speed[MOTOR_TYPE_SKINER-1][1] = speed;
#endif

	#ifdef SKSPEED_FAST
	if(speed < 1)
		speed = 1;
	if(speed > 100)
		speed = 100;

	speed = 100 - speed;

	#ifdef LX_TEST_SK_FAST_
		return 3000 + speed * 30;  // 2500 + speed * 30;
	#else
		return 2500 + speed * 30;  // 2500 + speed * 30;
	#endif
	
	#else
	if(speed < 20)
		speed = 20;
	if(speed > 200)
		speed = 200;

	speed = 200 - speed;
	return 2000 + speed * 10;
	#endif

	//sinker_add_speed = SK_ADD_SPEED_ALL;
	
}


#if 0
void StepMotor_Setup_Para(int cmd, int para)
{
	//myprintf("step setup %d, %d\r\n", cmd, para);
	return ;
}

#endif

unsigned int StepMotor_get_no_with_IDall(unsigned int step_IDall)
{
	//STEP_TYPE *Step;
	//unsigned int i;

	return Steps_ID_NO[0][step_IDall &0x0f];
	
}


unsigned int StepMotor_get_no_with_IDself(unsigned int step_IDself,unsigned char step_type)
{
	//STEP_TYPE *Step;
	//int i;

	if (step_type<=MOTOR_TYPE_COUNT)
		return Steps_ID_NO[step_type][step_IDself & 0x0f];
	return 0xff;
}


unsigned int StepMotor_get_IDall_with_IDself(unsigned int step_IDself,unsigned char step_type)
{
	STEP_TYPE *Step;
	int i;
	if (step_type<=MOTOR_TYPE_COUNT)
		{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
		}
	else
		{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self== step_IDself)
			&& (Step->moto_remap_config.moto_attr.bit.moto_type_config== step_type))
		{
			return Step->moto_remap_config.moto_remap_id_all;
		}
		else
		{
			return 0xff;
		}
	}
	return 0xff;

	
}


unsigned int StepMotor_get_zeroID_with_IDall(unsigned int step_IDall)
{
	STEP_TYPE *Step;
	int i;

	i=Steps_ID_NO[0][step_IDall&0x0f];

	
	
	if (i<StepMotor_Count_MAX)
	{	
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_remap_id_all== step_IDall)
		{
			return Step->moto_remap_config.moto_attr.bit.moto_zero_input_index;
		}
		else
		{
			return 0xff;
		}
	}

	return 0xff;
}

unsigned int StepMotor_get_workID_with_IDall(unsigned int step_IDall)
{
	STEP_TYPE *Step;
	int i;
	
	i=Steps_ID_NO[0][step_IDall&0x0f];

	
	
	if (i<StepMotor_Count_MAX)
	{	
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_remap_id_all== step_IDall)
		{
			return Step->moto_remap_config.moto_attr.bit.moto_work_input_index;
		}
		else
		{
			return 0xff;
		}
	}

	return 0xff;
}

unsigned char StepMotor_get_workID_cfg_with_IDself(unsigned int step_IDself,unsigned int step_type)
{
	STEP_TYPE *Step;
	int i;
	if (step_type<=MOTOR_TYPE_COUNT)
		{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
		}
	else
		{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self== step_IDself)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == step_type))
		{
			return Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg;
		}
		else
		{
			return 0;
		}
	}

	return 0;
}


unsigned int StepMotor_get_workID_with_IDself(unsigned int step_IDself,unsigned int step_type)
{
	STEP_TYPE *Step;
	int i;
	if (step_type<=MOTOR_TYPE_COUNT)
		{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
		}
	else
		{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self== step_IDself)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == step_type))
		{
			return Step->moto_remap_config.moto_attr.bit.moto_work_input_index;
		}
		else
		{
			return 0xff;
		}
	}

	return 0xff;
}


unsigned char StepMotor_get_zeroID_cfg_with_IDself(unsigned int step_IDself,unsigned int step_type)
{
	STEP_TYPE *Step;
	int i;
	
	if (step_type<=MOTOR_TYPE_COUNT)
		{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
		}
	else
		{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self== step_IDself)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == step_type))
		{
			return Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg;
		}
		else
		{
			return 0;
		}
	}

	return 0;
}



unsigned int StepMotor_get_zeroID_with_IDself(unsigned int step_IDself,unsigned int step_type)
{
	STEP_TYPE *Step;
	int i;
	
	if (step_type<=MOTOR_TYPE_COUNT)
		{
		i=Steps_ID_NO[step_type][step_IDself&0x0f];
		}
	else
		{
		i=0xff;
	}
	
	
	if (i<StepMotor_Count_MAX)
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_remap_id_self== step_IDself)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == step_type))
		{
			return Step->moto_remap_config.moto_attr.bit.moto_zero_input_index;
		}
		else
		{
			return 0xff;
		}
	}

	return 0xff;
}





unsigned int StepMotor_get_IDall_with_no(unsigned int step_no)
{
	STEP_TYPE *Step;
	
	
	if (step_no>=StepMotor_Count_MAX) return 0xff;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_remap_id_all;
}

unsigned char StepMotor_get_zeroID_cfg_with_NO(unsigned int step_no)
{
	STEP_TYPE *Step;
	//int i;
	
	if (step_no>=StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg;
}

unsigned char StepMotor_get_workID_cfg_with_NO(unsigned int step_no)
{
	STEP_TYPE *Step;
	//int i;
	
	if (step_no>=StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg;
}


unsigned int StepMotor_get_zeroID_with_NO(unsigned int step_no)
{
	STEP_TYPE *Step;
	//int i;
	
	if (step_no>=StepMotor_Count_MAX) return 0xff;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_attr.bit.moto_zero_input_index;
}

unsigned int StepMotor_get_workID_with_NO(unsigned int step_no)
{
	STEP_TYPE *Step;
	//int i;
	
	if (step_no>=StepMotor_Count_MAX) return 0xff;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_attr.bit.moto_work_input_index;
}


int StepMotor_Get_Running(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if (stepno >= StepMotor_Count_MAX)
		return 0;

	Step = &STEPMOTOR[stepno];
	return Step->step_st.bit.running;
}


short StepMotor_Get_Position_2(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	short pos;
	
	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];
	pos = Step->pos_2;//Step->position;

	if(pos < 0) {
		pos *= -1;
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		pos *= -1;
	}
	else {
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}
	return pos;
}

short StepMotor_Get_Zero_ST(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	short zerost;
	
	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];

	zerost = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
		
	return zerost;

}

short StepMotor_Get_Position(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	short pos;
	
	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];
	pos = Step->position;

	if(pos < 0) {
		pos *= -1;
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		pos *= -1;
	}
	else {
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}
	return pos;
}



short StepMotor_Get_input_error(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	short pos;

	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];

	pos = Step->input_errorstep;

	if(pos < 0) {
		pos *= -1;
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		pos *= -1;
	}
	else {
		pos = stepspeed_R2Q(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	return pos;
}

unsigned int StepMotor_Get_Busyidx(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];

	return Step->step_st.bit.running;
}



// 用于测试电机是否到位
void StepMotor_debug(void)
{
	unsigned int i;
	unsigned int stepid;
	int idx = 0x0;
	STEP_TYPE *Step;
	int ret = 0;

	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		stepid=Step->moto_remap_config.moto_remap_id_all;
		if(Step->step_st.bit.running) 
		{
			idx |= (0x1 << stepid);			
			if(Step->steps > ret)
				ret = Step->steps;
		}
		//if(step_run_mode) i ++;
	}
	alert_push(idx, ret);		//这里怎么是走报警通道的啊
	
}

#ifdef STEP_TEST_IN_NDL
unsigned int StepMotor_Get_Needle(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if(stepno >= StepMotor_Count_MAX) return 0;

	Step = &STEPMOTOR[stepno];

	return Step->needle;
}
#endif

void StepMotor_Set_Position(unsigned int stepno, short pos)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if(stepno >= StepMotor_Count_MAX) return;
	
	DEBUG_STEP_PRINTF("[%d]M-Set[pos %d]\r\n", stepno, pos);

	Step = &STEPMOTOR[stepno];
	if(pos < 0) {
		pos *= -1;
		pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		pos *= -1;
	}
	else {
		pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}
	
	Step->st_no_cl.bit.check_is_first =1;
	Step->st_no_cl.bit.YarnStep_can_check =1;
	Step->step_st.bit.is_poweron = 1;
	Step->position = pos;
	Step->pos_2 = pos;
	Step->pos_2_main = pos;
	

#ifdef ENCODER_SUPPORT
	/* 关闭自动换算位置 -- 续织时设位置可能在自动换算位置之前，故需关闭自动换算 */
	Encoder_rPos(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, 0);
#endif /* ENCODER_SUPPORT */
}

void StepMotor_Modfiy_Position(unsigned int stepno, short pos, int power)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];
	if(pos < 0) {
		pos *= -1;
		pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		pos *= -1;
	}
	else {
		pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
	}

	Step->step_st.bit.is_poweron = power;

	Step->position = pos;
	Step->pos_2 = pos;
}


extern void Exec_Set_Motor_Curr_phi(unsigned char steptype,unsigned short cruu);
void StepMotor_Add_curr(unsigned int stepno)
{

	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int steptype;
	unsigned int curr;
	
	
	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];
	
	steptype = Step->moto_remap_config.moto_attr.bit.moto_type_config;
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{
		curr = (unsigned int)(stepmotor_current[steptype-1]*125/100);
	}
	Exec_Set_Motor_Curr_phi(steptype,curr);

}

void StepMotor_reback_curr(unsigned int stepno)
{

	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int steptype;
	unsigned int curr;
	
	
	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];
	
	steptype = Step->moto_remap_config.moto_attr.bit.moto_type_config;
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{
		curr = (unsigned int)(stepmotor_current[steptype-1]);
	}
	Exec_Set_Motor_Curr_phi(steptype,curr);

}

void Motor_Set_maxsteps_cw(unsigned char steptype,unsigned int data)
{
	int i;
	STEP_TYPE *Step;
	
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		Encoder_Set_maxsteps_cw(Step->moto_remap_config.moto_attr.bit.moto_ecode_index,data);
	}

	
}	

void Motor_Set_minsteps_ecode_rang(unsigned char steptype,unsigned int data)
{
	int i;
	STEP_TYPE *Step;
	
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		Encoder_Set_minsteps_rang(Step->moto_remap_config.moto_attr.bit.moto_ecode_index,data);
	}

	
}

void Motor_Set_DM_check_area( short max_a, short min_a)
{
	DM_check_area_max = max_a;
	DM_check_area_min = min_a;
	
}

void Motor_Set_DM_Type(unsigned char steptype,unsigned char dm_type)
{
	
	int i;
	STEP_TYPE *Step;
	
	for(i = 0; i < StepMotor_Count_MAX; i++) 
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config !=steptype)
			continue;
		Step->moto_remap_config.moto_attr.bit.moto_type_exe= (dm_type & 0x03);
	}
	
}


unsigned int Needle_pos_Get(void);
#if 0
int postion_2_input_st(short postion)
{
	int ret ;
	#ifdef QL_
	if (((postion<-50)&&(postion>=-280))||(postion>280))			
	{
		if (postion>280)
		{
			ret =4;
		}
		else
		{
			ret = 1;
		}
	}
	else
		if ((postion>50)||(postion<-280))
		{
			if (postion>50)
			{
				ret =2;
			}
			else
			{
				ret =3;
			}
		}
		else
		{
			ret =0;
		}
	#else
	{
		if (postion<-50){
			ret =1;
		}
		else
			if (postion>50)
			{
				ret =2;
			}
			else
			{
				ret =0;
			}
	}
	#endif
	return ret;
	
}



unsigned short Step_set_input_cnt_with_st(short postion,short pos2)
{
	unsigned char z_c=0;
	unsigned char w_c=0;
	
	switch (postion_2_input_st(postion))
	{
		case 0:
				switch (postion_2_input_st(pos2))
				{
					case 0:
						
					break;	
					case 1:
						z_c = 2;
						w_c = 1;
					break;
					case 2:
						z_c = 1;
						w_c = 2;

					break;	
					case 3:
						z_c = 3;
						w_c = 2;

					break;
					case 4:
						z_c = 2;
						w_c = 3;
					break;
				}					
			break;
		case 1:			
				switch (postion_2_input_st(pos2))
				{
					case 0:
						z_c = 2;
						w_c = 1;
					break;	
					case 1:
						
					break;
					case 2:
						z_c = 3;
						w_c = 3;
					break;	
					case 3:
						z_c = 1;
						w_c = 1;
					break;
					case 4:
						z_c = 4;
						w_c = 4;
					break;
				}

			break;
		case 2:
				switch (postion_2_input_st(pos2))
				{
					case 0:
						z_c = 1;
						w_c = 2;
					break;	
					case 1:
						z_c = 3;
						w_c = 3;
					break;
					case 2:
						
					break;	
					case 3:
						z_c = 4;
						w_c = 4;
					break;
					case 4:
						z_c = 1;
						w_c = 1;
					break;
				}

			break;
		case 3:
				switch (postion_2_input_st(pos2))
				{
					case 0:
						z_c = 3;
						w_c = 2;
					break;	
					case 1:
						z_c = 1;
						w_c = 1;
					break;
					case 2:
						z_c = 4;
						w_c = 4;
					break;	
					case 3:

					break;
					case 4:
						z_c = 5;
						w_c = 5;
					break;
				}

			break;
		case 4:
				switch (postion_2_input_st(pos2))
				{
					case 0:
						z_c = 2;
						w_c = 3;
					break;	
					case 1:
						z_c = 4;
						w_c = 4;
					break;
					case 2:
						z_c = 1;
						w_c = 1;
					break;	
					case 3:
						z_c = 5;
						w_c = 5;
					break;
					case 4:

					break;
				}

			break;
	}
	return ((unsigned short)z_c<<8)|((unsigned short)w_c);

}

#endif


void Step_input_sts_first(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero;
	int work;
	unsigned int zwsts=0;
	
	if(stepno >= StepMotor_Count_MAX) return;
	
	Step = &STEPMOTOR[stepno];
	zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
	work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
	zwsts = ((work&0x01)<<1)|(zero&0x01);
	Step->last_new_input_sts =0;
	//Step->last_new_input_sts &=0xFFFC;
	Step->last_new_input_sts |=(zwsts & 0x0003);
	//Step->last_new_input_sts |=(zwsts & 0x0003);
	

}
unsigned int Step_input_sts_new(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero;
	int work;
	unsigned int zwsts=0;
	
	if(stepno >= StepMotor_Count_MAX) return 0;
	
	Step = &STEPMOTOR[stepno];
	zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
	work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
	zwsts = ((work&0x01)<<1)|(zero&0x01);
	//Step->last_new_input_sts =0;
	Step->last_new_input_sts &=0xFFF3;
	Step->last_new_input_sts |=((zwsts & 0x0003)<<2);
	if ((Step->last_new_input_sts&0x03)!=((Step->last_new_input_sts>>2)&0x03))
	{
		Step->last_new_input_sts |=((0x01)<<4);
		Step->last_new_input_sts &=0xFFFC;
		Step->last_new_input_sts |= (zwsts & 0x0003);
	}

	return (Step->last_new_input_sts>>4)&0x01;


}
#if 0
unsigned int Step_input_sts_check(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	if(stepno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	return (Step->last_new_input_sts>>4)&0x01;
}

#endif


void StepMotor_isr_exec(unsigned int stepid,short pos,int mode,unsigned short otherarg)
{
	unsigned int stepno;
	
	stepno = StepMotor_get_no_with_IDall(stepid);
	#ifdef TEST_STEPMOTOR_AUTOREPORT
	StepMotor_exec(stepno,pos,mode, 0x0001,0);
	#else
	StepMotor_exec(stepno,pos,mode,(otherarg>>4) & 0x0001,0);

	#endif
}

#ifdef STEP_MOTOR_DDDM_SUPPORT

#if 0
void StepMotor_isr_exec_new_dd(unsigned int stepid,short pos,int time,int mode,int maxspeed)
{
	unsigned int stepno;

	stepno = StepMotor_get_no_with_IDall(stepid);
	StepMotor_exec_new_dd(stepno,pos,time,mode,maxspeed);
	
}
#endif

#ifdef STEP_DEBUG_HZ

int StepMotor_calc_speed(int stepno)
{
	STEP_TYPE *Step;
	int acc;
	int ret;

	Step = &STEPMOTOR[stepno];
	ret = Step_pulse_HZ[Step->speed_HZ_idx];
	

		switch(Step->step_st.bit.phase) {
		case 0:	// Accelerate
			
			Step->speed_HZ_idx++;
			if(Step->speed_HZ_idx >= DEBUG_HZ_MAX_ACCSTEPS) {
				Step->speed_HZ_idx = DEBUG_HZ_MAX_ACCSTEPS-1;
				Step->step_st.bit.phase++;
			}
			else if(Step->steps < DEBUG_HZ_MAX_ACCSTEPS) {
				Step->step_st.bit.phase++;
			}
			break;
		case 1: // Isokinetic
			if(Step->steps < DEBUG_HZ_MAX_ACCSTEPS) {
				Step->step_st.bit.phase++;
			}
			break;
		case 2: // Decelerate

			if(Step->speed_HZ_idx > 0)
				Step->speed_HZ_idx--;

			break;
		default:
			break;
		}

	return 	ret;
	

	if(Step->max_speed == Step->low_speed) {
		return Step->max_speed;
	}

	// DEC
	if(Step->speed > Step->steps * Step->speed_acc_dec + Step->low_speed) {
		Step->speed -= Step->speed_acc_dec;
		//if(Step->speed < Step->low_speed) {
		//	Step->speed = Step->low_speed;
		//}
	}
	// ACC
	else if(Step->speed < Step->max_speed) {
		Step->speed += Step->speed_acc_dec;
		if(Step->speed > Step->max_speed) {
			Step->speed = Step->max_speed;
		}
	}
	// DEC
	else if(Step->speed > Step->max_speed) {
		Step->speed -= Step->speed_acc_dec;
		if(Step->speed < Step->low_speed) {
			Step->speed = Step->low_speed;
		}
	}

	return Step->speed;
}

#else




int StepMotor_calc_speed(int stepno)

{
	STEP_TYPE *Step;
	//int acc;

	Step = &STEPMOTOR[stepno];

	if(Step->max_speed == Step->low_speed) {
		return Step->max_speed;
	}

	

	// DEC
	if(Step->speed > Step->steps * Step->speed_acc_dec + Step->low_speed) {
		Step->speed -= Step->speed_acc_dec;
		//if(Step->speed < Step->low_speed) {
		//	Step->speed = Step->low_speed;
		//}
	}
	// ACC
	else if(Step->speed < Step->max_speed) {
		Step->speed += Step->speed_acc_dec;
		if(Step->speed > Step->max_speed) {
			Step->speed = Step->max_speed;
		}
	}
	// DEC
	else if(Step->speed > Step->max_speed) {
		Step->speed -= Step->speed_acc_dec;
		if(Step->speed < Step->low_speed) {
			Step->speed = Step->low_speed;
		}
	}

	return Step->speed;
}
#endif

int StepMotor_time2speed_DD(int stepno, int pos, int timeus,int max_spd,unsigned char acc)
{
	STEP_TYPE *Step;
	long max_speed;
	long speed;
	//long timeus;
	int i;

	Step = &STEPMOTOR[stepno];

	max_speed = 1000L * (1000L+(long)acc)/*to 1us*/ * (long)pos / (long)timeus;
	if(max_speed > max_spd) {
		return max_spd;
	}
	if(pos <= 16 * 2 || max_speed < 2000)
		return max_speed;
	return max_speed;


}



int StepMotor_time2speed(int stepno, int pos, int time01ms,int max_spd)
{
	//STEP_TYPE *Step;
	long max_speed;
	//long speed;
	//long timeus;
	//int i;

	//Step = &STEPMOTOR[stepno];

	max_speed = 1000L * 10L/*to 0.1ms*/ * (long)pos / (long)time01ms;
	if(max_speed > max_spd) {
		return max_spd;
	}
	if(pos <= STEP_DDM_STEPS * 2 || max_speed < 2000)
		return max_speed;
	return max_speed;


}


void Get_Step_debug_msg(unsigned short whichstep)
{
	
#ifdef QL_
	return ;
#else
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int i;
	
	Step = &STEPMOTOR[whichstep];
	for (i=0;i<4;i++)
	{
		
		//arch_SendMessage(4,(unsigned int *)&Step->Stepdebugmsg[i].position,4);
	}	
#endif
	
}

#ifdef DEBUG_STEP_DDM_CNT
typedef struct {

unsigned char stepno;
unsigned char cnt_cmd_last[2];
unsigned char cnt_cmd[2];
//unsigned char cnt_par_w[2];
short  posdata_last[4][64];
short  posdata[4][64];
}STEP_DEBUG_cnt;

STEP_DEBUG_cnt dm_debug_cnt[4];

extern volatile unsigned int jqd_direction;

void debug_cnt_init()
{

	memset(dm_debug_cnt,0x00,sizeof(dm_debug_cnt)); 
	
}

void Get_pos_debug_cnt()
{
	STEP_DEBUG_cnt *ddcp;
	unsigned char xc,alt_;
	int i,j;
	unsigned char whi = jqd_direction?0:1;
	for (i=0;i<4;i++)
	{
		alt_=0;
		ddcp = &dm_debug_cnt[i];
		Message_send_log_ecode(i|(0xFF<<8),(ddcp->cnt_cmd_last[0]<<8)|ddcp->cnt_cmd[0],(ddcp->cnt_cmd_last[1]<<8)|ddcp->cnt_cmd[1]);
		
		for (j=0;j<ddcp->cnt_cmd[0];j++)
		{
			if (j>63) 
			break;			
			Message_send_log_ecode(i|j<<8,ddcp->posdata_last[0][j],ddcp->posdata[0][j]);											
		}
		for (j=0;j<ddcp->cnt_cmd[1];j++)
		{
			if (j>63) 
			break;			
			Message_send_log_ecode(i|j<<8,ddcp->posdata_last[1][j],ddcp->posdata[1][j]);											
		}
		
		
	}
}

void Check_pos_debug_cnt()
{
	STEP_DEBUG_cnt *ddcp;
	unsigned char xc,alt_;
	int i,j;
	unsigned char whi = jqd_direction?0:1;
	for (i=0;i<4;i++)
	{
		alt_=0;
		ddcp = &dm_debug_cnt[i];
		if (ddcp->cnt_cmd_last[whi])  // 有记录就判断
		{
			if (ddcp->cnt_cmd_last[whi]!=ddcp->cnt_cmd[whi])
			{
				alert_push(0x64,(ddcp->cnt_cmd_last[whi]<<8)|(ddcp->cnt_cmd[whi]));
			}
			else
			{
				for (j=0;j<ddcp->cnt_cmd[whi];j++)
				{
					if (j>63) 
						break;
					if (ddcp->posdata_last[whi][j]!=ddcp->posdata[whi][j])
					{
					//	alert_push(0x65,ddcp->posdata_last[whi][j]);
						alt_++;
						Message_send_log_ecode(i|j<<8,ddcp->posdata_last[whi][j],ddcp->posdata[whi][j]);
					}					
				}
				if (alt_)
				{
					alert_push(0x65,alt_);
				}
					
			}
			
			
		}
		//else
		{
			ddcp->cnt_cmd_last[whi] = ddcp->cnt_cmd[whi];
			memcpy(&ddcp->posdata_last[whi][0],&ddcp->posdata[whi][0],128);
			memcpy(&ddcp->posdata_last[whi?3:2][0],&ddcp->posdata[whi?3:2][0],128);	
			ddcp->cnt_cmd[whi] =0;
			memset(&ddcp->posdata[whi][0],0,128);
			memset(&ddcp->posdata[whi?3:2][0],0,128);			
		}
		
	}

	//if (stepno>3) return;	

}

void Set_pos_debug_cnt_(unsigned int stepno,short pos,short postion_l)
{
	STEP_DEBUG_cnt *ddcp;
	unsigned char xc;

	if (stepno>3) return;
	ddcp = &dm_debug_cnt[stepno];
	xc = ddcp->cnt_cmd[jqd_direction?1:0];
	if (xc>=64)
		xc = 63;
	
	ddcp->posdata[jqd_direction?1:0][xc] = pos;
	ddcp->posdata[jqd_direction?3:2][xc] = postion_l;

	//xc++;
	ddcp->cnt_cmd[jqd_direction?1:0]++;
		
	// (ddcp->cnt_cmd_last[jqd_direction?1:0])
	
}

#endif

#if 0

#define STEPMOTOR_ACC_MAX_HZ_P_STEP  313
#define STEPMOTOR_ACC_MIN_HZ_P_STEP  150
#else
#define STEPMOTOR_ACC_MAX_HZ_P_STEP  250
#define STEPMOTOR_ACC_MIN_HZ_P_STEP  100

#endif


#ifdef SSTI_PULS_SMOOTH

static V_FLOAT scsim_max_acc =100 ;//100;	//
static V_FLOAT scsim_min_acc = 40;	//

static V_FLOAT scsim_max_dec = -60;//-60 /*-100*/;
static V_FLOAT scsim_min_dec = -25 /*-25*/;

#endif


void StepMotor_DD_ReConfig(unsigned char stepid)
{
	unsigned int stepno;
	STEP_TYPE *Step;

	stepno = StepMotor_get_no_with_IDself(stepid,MOTOR_TYPE_DENSITY);//StepMotor_get_no_with_IDall(stepid);
	Step = &STEPMOTOR[stepno];
	if(!Step->moto_dd_config.DD_Next_con_enable)  /*之前的动作还没完成*/
	{
		return;		
	}
	else
	{	
		Step->moto_dd_config.DD_IS_setting =1;
		Step->moto_dd_config.DD_type =1;	
		Step->moto_dd_config.DD_cmd_CNT = Step->moto_dd_config.DD_Next_cmd_CNT;
		Step->moto_dd_config.DD_cnt_cur =0;
		Step->moto_dd_config.DD_Target_POS = Step->moto_dd_config.DD_Next_Target_POS;
		Step->moto_dd_config.DD_cur_cnt_target_pos =0;
		Step->moto_dd_config.DD_last_cmd_time_100us = Step->moto_dd_config.DD_Next_last_cmd_time_100us;
		Step->moto_dd_config.DD_last_interval_time_100us = 0;
		Step->moto_dd_config.DD_Step_overflow_flag =0;
		Step->moto_dd_config.DD_IS_stope =1;
		Step->moto_dd_config.DD_IS_High_speed = 0;
		Step->moto_dd_config.DD_last_interval_time_100us_for_dir=0;
		Step->moto_dd_config.DD_IS_dir_first =0;
		Step->moto_dd_config.DD_speed_last_part_B =0;
		Step->moto_dd_config.DD_speed_last_part_D =0;
		Step->moto_dd_config.DD_last_step_idx =0;
		Step->moto_dd_config.DD_IS_setting =0;	
		Step->moto_dd_config.DD_Stop_immed = 0;
		Step->moto_dd_config.DD_Needle_idx = Step->moto_dd_config.DD_Next_Needle_idx;
		Step->moto_dd_config.DD_Next_con_enable =0;	
		Step->moto_dd_config.DD_cmd_err_cnt = 0;
		memset(Step->moto_dd_config.DD_last_speed,0,32);
		Step->moto_dd_config.DD_last_wp=0;
		Step->moto_dd_config.DD_cnt_cur_nextidx=0;
		//Step->moto_dd_config.DD_is_goto_turnaway=0;
		Step->moto_dd_config.DD_is_overlap_area=0;
		
	}
	
}



void StepMotor_DD_config(unsigned char stepid,short pos,unsigned char cmd_cnt,unsigned char NeedleID,unsigned char isPowerOff)
{
	unsigned int stepno;
	STEP_TYPE *Step;
	int i;
	unsigned char isrunning =0;

	stepno = StepMotor_get_no_with_IDself(stepid,MOTOR_TYPE_DENSITY);//StepMotor_get_no_with_IDall(stepid);
	Step = &STEPMOTOR[stepno];
	isrunning =Step->step_st.bit.running;
	if(isrunning)
	{
		arch_StepMotor_Stop(stepno);
	}
	if(Step->moto_dd_config.DD_type)  /*之前的动作还没完成*/
	{
		if(Step->moto_dd_config.DD_Target_POS == pos)
		{
			goto do_return_isrunning;
		}
		else
		{
			Step->moto_dd_config.DD_Next_con_enable =1;
			Step->moto_dd_config.DD_Next_cmd_CNT = cmd_cnt;
			Step->moto_dd_config.DD_Next_last_cmd_time_100us =Get_systime_100us();
			Step->moto_dd_config.DD_Next_Target_POS = pos;
			Step->moto_dd_config.DD_Next_Needle_idx = NeedleID;
			Step->moto_dd_config.DD_New_Target_POS = pos;

			if(((Step->moto_dd_config.DD_Target_POS > Step->position)
						&&(Step->moto_dd_config.DD_New_Target_POS < Step->position))
					||((Step->moto_dd_config.DD_Target_POS < Step->position)
						&&(Step->moto_dd_config.DD_New_Target_POS >Step->position)))
			{
				unsigned short leavesteps=abs(Step->moto_dd_config.DD_Target_POS - Step->position);
				Step->moto_dd_config.DD_is_goto_turnaway =1;
				 Step->steps = (leavesteps>STEP_GO_SLOW_SPEED_STEPS)?STEP_GO_SLOW_SPEED_STEPS:leavesteps;
				Step->step_st.bit.running = RUNNING_OVER;	
			}	
			Step->moto_dd_config.DD_IS_setting =1;
			
			goto DO_new_28_cmd_config;
		}
	}
	else
		{
	Step->moto_dd_config.DD_IS_setting =1;
	Step->moto_dd_config.DD_is_goto_turnaway=0;	
	DO_new_28_cmd_config:
	
	Step->moto_dd_config.DD_type =1;
	Step->moto_dd_config.DD_dec_speed =0;
	Step->moto_dd_config.DD_Next_con_enable =0;
	Step->moto_dd_config.DD_cmd_CNT = cmd_cnt;
	Step->moto_dd_config.DD_cnt_cur =0;
	Step->moto_dd_config.DD_Target_POS = pos;
	Step->moto_dd_config.DD_cur_cnt_target_pos =0;
	Step->moto_dd_config.DD_last_cmd_time_100us = Get_systime_100us();
	Step->moto_dd_config.DD_last_interval_time_100us = 0;
	Step->moto_dd_config.DD_Step_overflow_flag =0;
	Step->moto_dd_config.DD_IS_stope =1;
	Step->moto_dd_config.DD_IS_High_speed = 0;
	Step->moto_dd_config.DD_last_interval_time_100us_for_dir=0;
	Step->moto_dd_config.DD_IS_dir_first =0;
	Step->moto_dd_config.DD_speed_last_part_B =0;
	Step->moto_dd_config.DD_speed_last_part_D =0;
	Step->moto_dd_config.DD_last_step_idx =0;
	Step->moto_dd_config.DD_IS_setting =0;
	Step->moto_dd_config.DD_Stop_immed = 0;
	Step->moto_dd_config.DD_Needle_idx = NeedleID;
	Step->moto_dd_config.DD_cmd_err_cnt = 0;
	Step->moto_dd_config.DD_cnt_cur_nextidx=0;
	//Step->moto_dd_config.DD_is_goto_turnaway=0;
	Step->moto_dd_config.DD_is_overlap_area=0;
	Step->moto_dd_config.DD_Dont_check_idx = isPowerOff;
	Step->moto_dd_config.DD_Target_arr[0]=Step->position;
	Step->moto_dd_config.DD_Target_arr[1]=Step->position;
	Step->moto_dd_config.DD_Target_arr[2]=Step->position;
	
		
	for( i=0;i<4;i++)
	{
		Step->moto_dd_config.DD_last_i_t_buffer[i]=0;
	}
	Step->moto_dd_config.DD_last_Buf_P =0;
	memset(Step->moto_dd_config.DD_last_speed,0,32);
	
	//{
	Step->moto_dd_config.DD_last_wp=0;
	Step->moto_dd_config.DD_dont_act =0;

	#ifdef SSTI_PULS_SMOOTH
	Step->moto_dd_config.DD_Smooth.mot_phase =0;
	Step->moto_dd_config.DD_Smooth.mot_phase_dec_ex =0;
	#endif

	if(Step->moto_dd_config.DD_Target_POS == Step->position)
	{
		Step->moto_dd_config.DD_dont_act =1;/*确定不用做*/
		//Message_Send_4halfword(0x9E|(stepno<<8),Step->position,Step->moto_dd_config.DD_is_goto_turnaway,Step->running);
		//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x4F1F);
				
	}
//	}
//	Step->moto_dd_config.DD_err_cnt

	}

	do_return_isrunning:
	
	if(isrunning)
	{
		arch_StepMotor_EnableIT(stepno);
	}
	else
	{
		if(Step->moto_dd_config.DD_is_goto_turnaway) /*启动减速过程*/
		{
			arch_StepMotor_Start(stepno);	
		}
	}
	Message_Send_4halfword(0x99|(stepno<<8),Step->position,Step->moto_dd_config.DD_is_goto_turnaway,Step->step_st.bit.running);
	
}


void StepMotor_isr_exec_new_dd(unsigned int stepid,short pos,int time,int mode,int maxspeed,char isnewcmd)
{
	unsigned int stepno;

	stepno = StepMotor_get_no_with_IDself(stepid,MOTOR_TYPE_DENSITY);//StepMotor_get_no_with_IDall(stepid);
	StepMotor_exec_new_dd(stepno,pos,time,mode,maxspeed,isnewcmd);
	
}



int StepMotor_calc_speed_DD(int stepno)
{
	STEP_TYPE *Step;
	int acc;
	unsigned short steps_dd;
	unsigned char stepdir;

	Step = &STEPMOTOR[stepno];

	if(Step->moto_dd_config.DD_Target_POS==Step->moto_dd_config.DD_cur_cnt_target_pos)
	{
		/*说明已经是最后一个行程了*/
		unsigned int dt=0;
		/* bdt*2 / (pos_dif*(pos_dif-1));*/
		dt = Step->moto_dd_config.DD_last_step_idx*Step->moto_dd_config.DD_speed_last_part_D;
		dt /= Step->moto_dd_config.DD_last_steps;
		Step->speed = 1000000L/(Step->moto_dd_config.DD_speed_last_part_B+dt);
		Step->moto_dd_config.DD_last_speed[Step->moto_dd_config.DD_last_wp++]=Step->speed;
		if(Step->moto_dd_config.DD_last_wp>=16)
		{
			Step->moto_dd_config.DD_last_wp =0;	
		}
		Step->moto_dd_config.DD_last_step_idx++;
		//goto general_flow;
		goto return_speed;
	}
	else		/*中间过程*/
	{
		#if 0
			if(Step->max_speed == Step->low_speed) 
			{
				Step->speed = Step->max_speed;
				goto return_speed;//return Step->speed;
			}

			#endif
			
			if(((Step->position<Step->moto_dd_config.DD_Target_POS)&&(Step->step_st.bit.dir))
				||((Step->position>Step->moto_dd_config.DD_Target_POS)&&(!Step->step_st.bit.dir)))
			{
				if(Step->speed < Step->max_speed) 
				{
					Step->speed += Step->speed_acc_dec;
					if(Step->speed > Step->max_speed) 
					{
						Step->speed = Step->max_speed;
					}
				}
				else
				if(Step->speed > Step->max_speed) 
				{
					int speed_dec =Step->speed - Step->max_speed;
					if(speed_dec >Step->speed_acc_dec )
					{
						speed_dec =Step->speed_acc_dec;
					}
					Step->speed -=speed_dec;// Step->speed_acc_dec;
					if(Step->speed < Step->low_speed) 
					{
						Step->speed = Step->low_speed;
					}
				}
			}
			else				/*位置超出以后强制减速*/
			{
				Step->speed -= Step->speed_acc_dec;	
				if(Step->speed < Step->low_speed) 
				{
					Step->speed = Step->low_speed;
				}
			}	

			goto return_speed;
	}

	//steps_dd = abs(Step->moto_dd_config.DD_Target_POS-Step->position);

	general_flow:
	// DEC
	if(Step->speed > Step->steps * Step->speed_acc_dec + Step->low_speed) {
		Step->speed -= Step->speed_acc_dec;
		//if(Step->speed < Step->low_speed) {
		//	Step->speed = Step->low_speed;
		//}
	}
	// ACC
	else if(Step->speed < Step->max_speed) {
		Step->speed += Step->speed_acc_dec;
		if(Step->speed > Step->max_speed) {
			Step->speed = Step->max_speed;
		}
	}
	// DEC
	else if(Step->speed > Step->max_speed) {
		Step->speed -= Step->speed_acc_dec;
		if(Step->speed < Step->low_speed) {
			Step->speed = Step->low_speed;
		}
	}

return_speed:
	return Step->speed;
}


#if 1
void StepMotor_exec_new_dd(unsigned int stepno, short pos, int time, int mode, int maxspd,char isnewcmd)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero_work_area;
	int zero;
	int work;	
	int new_speed;
	int max_speed = stepmotor_speed[0][1] ; 
	int temp_step_acc_dec=0;
    	int ZeroPos_workST;
	unsigned char use_last_dir_speed=0;	
	unsigned int tt;
	int rt;
	int pos_dif;
	unsigned char needleidx;
	unsigned short CMDidx;
	unsigned int abspos_t;
	short pos_t_last;
	short pos_t_last1;
	
	unsigned char isNewTargetPos =0;
	unsigned char isspeeddec=0;

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config>MOTOR_TYPE_COUNT) return;

	zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);
	work = arch_StepMotor_work(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
    		


	if (!isnewcmd)
	{
		if(Step->moto_dd_config.DD_type)
		{
			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x6F7F);
			return;
		}

		max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1] ; 
		if(maxspd > 0) 
		{
			max_speed = maxspd * 100;
		}
		Step->moto_dd_config.DD_Target_POS = pos;
		
	}
	else
	{
		if(!Step->moto_dd_config.DD_type)
		{
			//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_type_config), 0x5F7F);
			return;
		}		
		{
			needleidx = maxspd;
			CMDidx= time & 0x0FFF;
			Step->moto_dd_config.DD_cnt_cur = CMDidx+1;

			Step->moto_dd_config.DD_Target_arr[2] = Step->moto_dd_config.DD_Target_arr[1];
			Step->moto_dd_config.DD_Target_arr[1] = Step->moto_dd_config.DD_Target_arr[0];
			Step->moto_dd_config.DD_Target_arr[0] = pos;
			
			
			#if 0
			if(Step->moto_dd_config.DD_Next_con_enable)
			{
				if(CMDidx==0)/*说明是后面的第一针数据*/
				{
					if((Step->moto_dd_config.DD_Next_Needle_idx & 0x07 )==(needleidx & 0x07))
					{
						/*说明同一针发了28和38，那需要报警*/
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_type_config), 0x4F4F);
						return;
					}
					else  /*把之前的配置信息用起来*/
					{
						Step->moto_dd_config.DD_cnt_cur_nextidx = 0;  /*为了判断CMDidx*/
						isNewTargetPos =1;
					}
				}
				else
				{
					if((Step->moto_dd_config.DD_Next_Needle_idx & 0x07 )!=(needleidx & 0x07))
					{
						/*说明下一针的28 发到上一次的38前两针去了，那需要报警*/
						is_error_recordit:
						Step->moto_dd_config.DD_cmd_err_cnt++;
						if(Step->moto_dd_config.DD_cmd_err_cnt>=6)
						{
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_type_config), 0x4F3F);
						}
						return;
						
					}
					
				}
			}

			#endif

			if((Step->moto_dd_config.DD_cnt_cur_nextidx!=CMDidx)
				&&(!Step->moto_dd_config.DD_Dont_check_idx))
			{
				Step->moto_dd_config.DD_cmd_err_cnt++;
				if(Step->moto_dd_config.DD_cmd_err_cnt>=6)
				{
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x4F3F);
				}
				return;
			}

			

			#if 0
			if(isNewTargetPos)
			{
				StepMotor_DD_ReConfig(stepno);
				Step->moto_dd_config.DD_is_overlap_area =1;
				
			}
			#endif
			
			Step->moto_dd_config.DD_cnt_cur_nextidx = CMDidx+1;
			


			
			
			if(time & 0x1000)  /*is turn away*/
			{
				Step->moto_dd_config.DD_last_interval_time_100us_for_dir=Step->moto_dd_config.DD_last_interval_time_100us;
				Step->moto_dd_config.DD_IS_dir_first =1;
			}
			else
				if(Step->moto_dd_config.DD_IS_dir_first)
				{
					use_last_dir_speed =1;
					Step->moto_dd_config.DD_IS_dir_first =0;
				}
		
			Step->moto_dd_config.DD_last_interval_time_100us = Get_interval_time_100us(Step->moto_dd_config.DD_last_cmd_time_100us);
			Step->moto_dd_config.DD_last_cmd_time_100us = Get_systime_100us();

{
		unsigned short T_buf;
			if((!Step->moto_dd_config.DD_IS_stope)&&(use_last_dir_speed)&&(Step->moto_dd_config.DD_last_interval_time_100us_for_dir))
			{
				T_buf = Step->moto_dd_config.DD_last_interval_time_100us_for_dir;
			}
			else
			{
				T_buf = Step->moto_dd_config.DD_last_interval_time_100us;
			}

			Step->moto_dd_config.DD_last_i_t_buffer[Step->moto_dd_config.DD_last_Buf_P]=T_buf;
			Step->moto_dd_config.DD_last_Buf_P++;
			if(Step->moto_dd_config.DD_last_Buf_P>=4)
				Step->moto_dd_config.DD_last_Buf_P =0;
}
			
			if(Step->moto_dd_config.DD_dont_act)
			{
				if(pos == Step->moto_dd_config.DD_Target_POS)
				{
					Step->moto_dd_config.DD_type =0;
				}
				return;
			}

			if(((Step->moto_dd_config.DD_Target_POS>Step->position)
				&&(pos<Step->position))
				||((Step->moto_dd_config.DD_Target_POS<Step->position)
				&&(pos>Step->position)))
			{

			
				return;  /*反走的情况滤掉*/
			}

			if((Step->moto_dd_config.DD_Target_POS==Step->position))
			{
				if(Step->step_st.bit.step_flags) 
				{			
				//	void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
					Message_Send_4halfword((0x08 << 8) | 0x03,StepMotor_get_IDall_with_no(stepno),Step->position,Step->moto_dd_config.DD_speed_hz_max);
					//#endif	
					Step->step_st.bit.step_flags =0;
				}
				//Message_Send_4halfword(0xbbbb,stepno,Step->moto_dd_config.DD_cmd_CNT,Step->moto_dd_config.DD_cnt_cur);
				if((Step->moto_dd_config.DD_cmd_CNT == (Step->moto_dd_config.DD_cnt_cur))
					||((pos==Step->moto_dd_config.DD_Target_POS)))
					Step->moto_dd_config.DD_type =0;
				return;
				
			}
			
			if(Step->moto_dd_config.DD_is_goto_turnaway==1)
			{
				if(Step->step_st.bit.running)
					arch_StepMotor_Start(stepno);	

				
				return;
			}
			else
			if(Step->moto_dd_config.DD_is_goto_turnaway==2)//
			{
				return;
			}		
			else
			if(Step->moto_dd_config.DD_is_goto_turnaway==3)
			{
				if(((Step->position>pos) &&(Step->position>Step->moto_dd_config.DD_Target_POS))
					||((Step->position<pos) &&(Step->position<Step->moto_dd_config.DD_Target_POS)))
				{
					Step->moto_dd_config.DD_is_goto_turnaway = 0;
					Message_Send_4halfword(0xF4|(stepno<<8),Step->moto_dd_config.DD_Target_POS, pos, Step->position);

				}
				else
				{
					return;
				}
				
			}
			
			arch_StepMotor_Stop(stepno);
			Step->moto_dd_config.DD_IS_setting =1;
		
			if(Step->moto_dd_config.DD_cnt_cur>1)
			{
				if(Step->moto_dd_config.DD_last_interval_time_100us>5000)// 300ms 认为是停止了
				{
					Step->moto_dd_config.DD_IS_stope =1;
				}
				else
				{
					Step->moto_dd_config.DD_IS_stope =0;
				}
					
				Step->moto_dd_config.DD_last_POS = Step->moto_dd_config.DD_cur_cnt_target_pos;
			}
			else /*第一条数据*/
			{
				
				//Step->moto_dd_config.DD_last_cmd_time_100us = Get_systime_100us();
				//Step->moto_dd_config.DD_last_interval_time_100us = 0;
				Step->moto_dd_config.DD_last_POS = Step->position; /*取当前位置*/
				Step->moto_dd_config.DD_speed_hz_max =0;
				Step->moto_dd_config.DD_speed_max_pos = Step->position;

				if(Step->moto_dd_config.DD_last_interval_time_100us<100)
				{
					Step->moto_dd_config.DD_IS_High_speed =1;
				}
				
			}

			pos_t_last = Step->moto_dd_config.DD_cur_cnt_target_pos;
			 Step->moto_dd_config.DD_cur_cnt_target_pos = pos;

			Step->moto_dd_config.DD_cur_steps = abs(Step->moto_dd_config.DD_cur_cnt_target_pos-Step->moto_dd_config.DD_last_POS);

			pos_t_last = abs(Step->moto_dd_config.DD_last_POS-Step->moto_dd_config.DD_Target_POS);
			pos_t_last1 = abs(Step->position-Step->moto_dd_config.DD_Target_POS);

			if(pos_t_last1>(pos_t_last+(Step->moto_dd_config.DD_cur_steps>>2)))/*说明落后比较多*/
			{
				pos_t_last =1;
			}
			else
				pos_t_last =0;

			#ifdef SSTI_PULS_SMOOTH
		
			//Step->moto_dd_config.DD_Smooth.mot_steps = Step->moto_dd_config.DD_cmd_CNT;
			//scsim_step_new((void *)&Step->moto_dd_config.DD_Smooth,CMDidx);
			{
				int steps_dec;
				int steps2;

				// rule(): deceleration phase deciding..
				steps_dec = 10;
				steps2 = Step->moto_dd_config.DD_cmd_CNT >>2;// 之前的三分之一改成四分之一 / 3; //???
				if(steps_dec > steps2)
					steps_dec = steps2;

				if(Step->moto_dd_config.DD_cmd_CNT - CMDidx <= steps_dec /*10*/ /*4*/ /*?*/)
					Step->moto_dd_config.DD_Smooth.mot_phase = 2;
			}
			#endif

			if(Step->moto_dd_config.DD_last_interval_time_100us)
			{

				unsigned char acc_temp =0;

				#ifdef SSTI_PULS_SMOOTH
				//if(Step->moto_dd_config.DD_Step_overflow_flag)
				if(Step->moto_dd_config.DD_Smooth.mot_phase ==2/*vmot0->mot_flags & 0x80*/)
				{
					int pos_dif_2;
					pos_dif_2 = abs(Step->position - pos);
					pos_dif = abs(Step->moto_dd_config.DD_last_POS - pos);	
					if(pos_dif_2 > pos_dif)
					{
						pos_dif = pos_dif_2;
					}
				}
				else		
				#endif	
					pos_dif = abs(Step->position - pos);	
				

				abspos_t=(abs(pos_dif)>1)?(abs(pos_dif)>>1):(1);

				
				{
					int p,k,i;	
					tt=0;
					k=0;
					p = Step->moto_dd_config.DD_last_Buf_P;
					for (i=0;i<4;i++)
					{
						if(p==0)
							p=4;
						p--;
						if(Step->moto_dd_config.DD_last_i_t_buffer[p])
						{
							tt += Step->moto_dd_config.DD_last_i_t_buffer[p];
							k++;
						}
						else
						{
							break;
						}
					}
					if(k==0)
					{
						tt=Step->moto_dd_config.DD_last_interval_time_100us<<2;
					}
					else if(k==1)
					{
						tt<<=2;
					}else if (k==2)
					{
						tt<<=1;
					}else if (k==3)
					{
						tt<<=2;
						tt /=3;
					}

					tt *=25;  /*100us 变成us*/
					
				}
				//tt=Step->moto_dd_config.DD_last_interval_time_100us;

				if(Step->moto_dd_config.DD_cur_cnt_target_pos==Step->moto_dd_config.DD_Target_POS )
				{
					rt = arch_StepMotor_remainTime_us(stepno);
					if (pos_dif>1)
						pos_dif -=1;
					
				}
				else
					rt=0;
				//tt *= 100;
				if(tt>rt)
					tt -=rt;
				
				acc_temp = (Step->moto_dd_config.DD_cnt_cur<=1)?1:0;

#if 0
				if((!Step->moto_dd_config.DD_IS_stope)&&(use_last_dir_speed)&&(Step->moto_dd_config.DD_last_interval_time_100us_for_dir))
				{
					tt=Step->moto_dd_config.DD_last_interval_time_100us_for_dir;
					tt *=100;
				}	

				#endif

				#ifdef SSTI_PULS_SMOOTH
				
				if(Step->moto_dd_config.DD_Smooth.mot_phase ==2/*vmot0->mot_flags & 0x80*/)
				{
					unsigned int newdifpos;
					unsigned int newdifpos2;
					//newdifpos = Step->moto_dd_config.DD_last_steps_cal_HZ +abs(pos_dif);
					//newtt = Step->moto_dd_config.DD_last_time_cal_HZ + tt;	

					newdifpos = abs(Step->position - Step->moto_dd_config.DD_Target_arr[0])+abs(Step->moto_dd_config.DD_Target_arr[2] - Step->moto_dd_config.DD_Target_arr[1]);
					newdifpos2 = abs(Step->moto_dd_config.DD_Target_arr[2] - Step->moto_dd_config.DD_Target_arr[0]);	
					if(newdifpos > newdifpos2)
					{
						newdifpos2 = newdifpos;
					}
					
					max_speed = StepMotor_time2speed_DD(stepno, abs(newdifpos2), (tt<<1),stepmotor_speed_max_DD,acc_temp);
				}
				else
				{
					max_speed = StepMotor_time2speed_DD(stepno, abs(pos_dif), tt,stepmotor_speed_max_DD,acc_temp);
				
				}
				Step->moto_dd_config.DD_last_steps_cal_HZ = abs(pos_dif);
				Step->moto_dd_config.DD_last_time_cal_HZ = tt;

				#else
				{
					max_speed = StepMotor_time2speed_DD(stepno, abs(pos_dif), tt,stepmotor_speed_max_DD,acc_temp);
				}
				#endif	
				
				
				if(Step->moto_dd_config.DD_cur_cnt_target_pos==Step->moto_dd_config.DD_Target_POS )
				{
					
					if((Step->speed<=step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
						&&(max_speed<=step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]))
					{
						Step->moto_dd_config.DD_speed_last_part_B = 1000000L/max_speed;
						Step->moto_dd_config.DD_speed_last_part_D = 0;
					}
					else
					{
						unsigned int Bt;/*us*/
						unsigned int Dt;/*us*/
						int bdt;

						Bt = 1000000L/Step->speed;
						bdt = (tt -pos_dif*Bt);
						
						if((pos_dif>1)&&(bdt>0))
						{
							Dt = bdt*2 / (pos_dif*(pos_dif-1));
						}else
						{
							bdt =0;
							Dt = 0;
						}

						Step->moto_dd_config.DD_last_steps = pos_dif*(pos_dif-1);

						Step->moto_dd_config.DD_speed_last_part_B = Bt;
						Step->moto_dd_config.DD_speed_last_part_D = bdt<<1;
						
						
						
					}
				}

				if((Step->moto_dd_config.DD_IS_High_speed)&&(!Step->moto_dd_config.DD_IS_stope))
				{
					Step->speed_acc_dec = STEPMOTOR_ACC_MAX_HZ_P_STEP;
				}
				else
				{
					if(tt<15000) 
					{
						Step->speed_acc_dec = STEPMOTOR_ACC_MAX_HZ_P_STEP;
					}	
					else
					{
						Step->speed_acc_dec = STEPMOTOR_ACC_MIN_HZ_P_STEP;
					}
				}
			//	Message_Send_4halfword(0xff|(stepno<<8),max_speed, Step->moto_dd_config.DD_last_interval_time_100us, pos_dif);

			}
			else
			{
				max_speed = 1000;
				Step->speed_acc_dec = STEPMOTOR_ACC_MAX_HZ_P_STEP;
			}
			
			new_speed = max_speed;

			if(((Step->moto_dd_config.DD_IS_High_speed)&&(!Step->moto_dd_config.DD_IS_stope)))
			{
				if(new_speed < 300)
					new_speed = 300;
			}
			else
			{
				if(new_speed < 300)
					new_speed = 300;
			}
			if((use_last_dir_speed))
			{
				if(new_speed < 800)
					new_speed = 800;
			}


			//isspeeddec =(new_speed<Step->max_speed)?1:0;

			if((Step->moto_dd_config.DD_dec_speed==0)&&(new_speed<Step->max_speed))
			{
				Step->moto_dd_config.DD_dec_speed =1;
			}
			if((Step->moto_dd_config.DD_cmd_CNT-CMDidx) <=4)
				Step->moto_dd_config.DD_dec_speed =0;
			

			isspeeddec =Step->moto_dd_config.DD_dec_speed;
			Step->max_speed = new_speed;

			Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			if(Step->low_speed > new_speed) {
				Step->low_speed = new_speed;
			}

			#ifdef SSTI_PULS_SMOOTH
			{
				V_FLOAT dist;
				V_FLOAT sdist;
			//Step->moto_dd_config.DD_Smooth.mot_steps = Step->moto_dd_config.DD_cmd_CNT;
			//scsim_step_new((void *)&Step->moto_dd_config.DD_Smooth,CMDidx);
#if 0
			{
				int steps_dec;
				int steps2;

				// rule(): deceleration phase deciding..
				steps_dec = 10;
				steps2 = Step->moto_dd_config.DD_cmd_CNT >>2;// 之前的三分之一改成四分之一 / 3; //???
				if(steps_dec > steps2)
					steps_dec = steps2;

				if(Step->moto_dd_config.DD_cmd_CNT - CMDidx <= steps_dec /*10*/ /*4*/ /*?*/)
					Step->moto_dd_config.DD_Smooth.mot_phase = 2;
			}


			#endif
			
			Step->moto_dd_config.DD_Smooth.mot_estspd = (V_FLOAT)Step->max_speed;
			Step->moto_dd_config.DD_Smooth.mot_thyspd  = Step->moto_dd_config.DD_Smooth.mot_estspd;
			Step->moto_dd_config.DD_Smooth.mot_speed =(V_FLOAT)Step->speed;
			/*
		 * rule(0)
		 */
		if(Step->moto_dd_config.DD_Smooth.mot_thyspd <= Step->moto_dd_config.DD_Smooth.mot_speed &&
		   Step->moto_dd_config.DD_cnt_cur > 8 /*?*/ && 
		   Step->moto_dd_config.DD_Smooth.mot_phase == 0x0)
			 Step->moto_dd_config.DD_Smooth.mot_phase = 0x1; /* running smoothly now */
				
		if( (((Step->moto_dd_config.DD_last_POS<=Step->position)&& (Step->step_st.bit.dir))
			||((Step->moto_dd_config.DD_last_POS>=Step->position)&& (!Step->step_st.bit.dir)))
		   && (Step->moto_dd_config.DD_cnt_cur>2)
		   && (Step->moto_dd_config.DD_Smooth.mot_phase == 0x0))
			 Step->moto_dd_config.DD_Smooth.mot_phase = 0x1; /* running smoothly now */
		
		/*
		 * rule(1)
		 * !
		 */
		if( Step->moto_dd_config.DD_Smooth.mot_phase != 2 /* not deceleration phase! */ &&
		    Step->moto_dd_config.DD_Smooth.mot_phase != 0 /* not acceleration phase! */ &&
		   abs((int)(Step->moto_dd_config.DD_Smooth.mot_estspd - Step->moto_dd_config.DD_Smooth.mot_speed)) < 1000 /*800*/ /*200*/)
			Step->moto_dd_config.DD_Smooth.mot_estspd = Step->moto_dd_config.DD_Smooth.mot_speed;

		/* rule(?) fixup.. */
		if((Step->moto_dd_config.DD_Smooth.mot_phase == 0x1 || 
		    Step->moto_dd_config.DD_Smooth.mot_phase == 0x0) &&
		    Step->moto_dd_config.DD_Smooth.mot_thyspd == (V_FLOAT)stepmotor_speed_max_DD)
			Step->moto_dd_config.DD_Smooth.mot_estspd = (V_FLOAT)stepmotor_speed_max_DD;

		// initial speed..
		//vmot0->mot_inispd = vmot0->mot_speed;
		// accelerator
		if(Step->moto_dd_config.DD_Smooth.mot_estspd<Step->moto_dd_config.DD_Smooth.mot_speed)
		{
			int arg1,arg2;
			arg1=0;
			arg2=1;
			Step->moto_dd_config.DD_Smooth.mot_estspd =(( Step->moto_dd_config.DD_Smooth.mot_estspd*arg2) +(Step->moto_dd_config.DD_Smooth.mot_speed*arg1))/(arg1+arg2);	
		}
		
		dist = (V_FLOAT)abspos_t;
		sdist = Step->moto_dd_config.DD_Smooth.mot_estspd - Step->moto_dd_config.DD_Smooth.mot_speed;
		sdist /= dist ;
		if(Step->moto_dd_config.DD_Smooth.mot_phase ==2/*vmot0->mot_flags & 0x80*/)  // not speed-up phase
			sdist /= 2;
		Step->moto_dd_config.DD_Smooth.mot_acc = sdist;
		if(Step->moto_dd_config.DD_Smooth.mot_acc==0)
		{
			goto rule_2;
		}
		
		if(Step->moto_dd_config.DD_Smooth.mot_acc>0)
		{
			if(Step->moto_dd_config.DD_Smooth.mot_acc > scsim_max_acc)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_max_acc;
			if(Step->moto_dd_config.DD_Smooth.mot_acc < scsim_min_acc)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_min_acc;	
			if(pos_t_last)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_max_acc;
			if(Step->moto_dd_config.DD_Smooth.mot_phase ==1)
			{
				if(Step->moto_dd_config.DD_Smooth.mot_acc>DEF_STEPMOTOR_MAX_ACC_SPEED_HZ)
					Step->moto_dd_config.DD_Smooth.mot_acc= DEF_STEPMOTOR_MAX_ACC_SPEED_HZ;
			}
		}
		else
		{
			if(Step->moto_dd_config.DD_Smooth.mot_acc < scsim_max_dec /*-scsim_max_acc*/)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_max_dec /*-scsim_max_acc*/;
			if(Step->moto_dd_config.DD_Smooth.mot_acc > scsim_min_dec /*-scsim_max_acc*/)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_min_dec /*-scsim_max_acc*/;
			if(pos_t_last)
				Step->moto_dd_config.DD_Smooth.mot_acc = scsim_max_dec;
			if(Step->moto_dd_config.DD_Smooth.mot_phase ==1)
			{
				if(Step->moto_dd_config.DD_Smooth.mot_acc<DEF_STEPMOTOR_MAX_DEC_SPEED_HZ)
					Step->moto_dd_config.DD_Smooth.mot_acc= DEF_STEPMOTOR_MAX_DEC_SPEED_HZ;
			}
		}

		

		rule_2:
		/*
		 * rule(2)
		 */
		if(Step->moto_dd_config.DD_Smooth.mot_phase == 2 /* deceleration phase */ &&
		   Step->moto_dd_config.DD_Smooth.mot_acc > 0) 
		{
			if(Step->moto_dd_config.DD_Smooth.mot_phase_dec_ex!=0)
			{
				Step->moto_dd_config.DD_Smooth.mot_acc = 0;
			}
			
		}

		if(Step->moto_dd_config.DD_Smooth.mot_phase == 2)
		{
			Step->moto_dd_config.DD_Smooth.mot_phase_dec_ex =1;
		}
		
		/*
		 * rule(3)
		 */
		if(Step->moto_dd_config.DD_Smooth.mot_phase == 0 /* acceleration phase */ &&
		   Step->moto_dd_config.DD_Smooth.mot_acc < 0)
			Step->moto_dd_config.DD_Smooth.mot_acc = 0;
				}

			#endif
			

			Step->moto_dd_config.DD_Speed_dir =(Step->speed<Step->max_speed)?0:1;/*加速*/
			
			//Step->moto_dd_config.DD_cnt_cur++;
			if((Step->moto_dd_config.DD_cmd_CNT)&&(Step->moto_dd_config.DD_cnt_cur>Step->moto_dd_config.DD_cmd_CNT))
			{
				Step->moto_dd_config.DD_err_cnt++;
			}

			tt /=100;	
			Message_Send_4halfword(0xff|(stepno<<8),(Step->moto_dd_config.DD_Step_overflow_flag<<8) |Step->moto_dd_config.DD_Smooth.mot_phase, tt, Step->position);
			Message_Send_4halfword(0xfE|(stepno<<8),Step->max_speed, Step->moto_dd_config.DD_speed_hz_max, Step->moto_dd_config.DD_speed_max_pos);
			//Message_Send_4halfword(0xCC|(stepno<<8),Step->speed_acc_dec_back,Step->speed_acc_dec,0);

			Step->moto_dd_config.DD_speed_hz_max = Step->moto_dd_config.DD_Speed_dir ? 20000 :0;
			Step->moto_dd_config.DD_Step_overflow_flag=0;

			#if 0
			if(Step->moto_dd_config.DD_is_overlap_area)
			{
				if(Step->running)
				{
					if(Step->moto_dd_config.DD_is_goto_turnaway)
					{
						return;
					}
					if(((Step->moto_dd_config.DD_Target_POS > Step->position)
						&&(Step->moto_dd_config.DD_New_Target_POS < Step->position))
					||((Step->moto_dd_config.DD_Target_POS < Step->position)
						&&(Step->moto_dd_config.DD_New_Target_POS >Step->position)))
					{
						Step->moto_dd_config.DD_is_goto_turnaway =1;
						if(Step->dir)//++
						{
							Step->pos_2 = Step->position+8;
//							Step->running = RUNNING_OVER;	
						}
						else
						{
							Step->pos_2 = Step->position-8;
							if(Step->pos_2<0)
							{
								Step->pos_2=0;
							}
						}
						Step->steps = abs(Step->pos_2-Step->position);
						
						Step->running = RUNNING_OVER;	

						Step->max_speed  =Step->speed;
						Step->low_speed = step_base_speed[Step->moto_remap_config.moto_type_config-1];
						if(Step->low_speed>Step->speed)
							Step->low_speed = Step->speed;
						Step->speed_acc_dec = (Step->max_speed - Step->low_speed)/(8);
						if(Step->speed_acc_dec>STEPMOTOR_ACC_MAX_HZ_P_STEP)
						{
							Step->speed_acc_dec =STEPMOTOR_ACC_MAX_HZ_P_STEP;
						}
						else
						if(Step->speed_acc_dec<STEPMOTOR_ACC_MIN_HZ_P_STEP)
						{
							Step->speed_acc_dec =STEPMOTOR_ACC_MIN_HZ_P_STEP;
						}	

						Message_Send_4halfword(0xF3|(stepno<<8),Step->low_speed, Step->speed, Step->position);

						
					}
					else
					{
						goto do_is_to_running;
					}
				}
				else
				{
					if(Step->moto_dd_config.DD_is_goto_turnaway)
						Step->moto_dd_config.DD_is_goto_turnaway =0;
					
					if(((Step->moto_dd_config.DD_Target_POS > Step->position)
						&&(Step->moto_dd_config.DD_cur_cnt_target_pos >=Step->position)
						&&(Step->moto_dd_config.DD_Target_POS >= Step->moto_dd_config.DD_cur_cnt_target_pos))
					||((Step->moto_dd_config.DD_Target_POS < Step->position)
						&&(Step->moto_dd_config.DD_cur_cnt_target_pos <=Step->position)
						&&(Step->moto_dd_config.DD_Target_POS <= Step->moto_dd_config.DD_cur_cnt_target_pos)))
					{
						Step->moto_dd_config.DD_is_overlap_area =0;
						Message_Send_4halfword(0xF4|(stepno<<8),Step->max_speed, Step->speed, Step->position);

						//Step->speed = Step->low_speed;
						goto do_motor_run_first;
					}
					else /*do nothing*/
					{
						return ;
					}
				}
			}
			else
			#endif
			{
			

			if(Step->step_st.bit.running)
			{

				do_is_to_running:
				if (Step->step_is_cnt==Step->step_is_cnt_old)   //找机会重新启动。
				{
						arch_StepMotor_Start(stepno);	
				}
				Step->step_is_cnt_old = Step->step_is_cnt;			

				if (Step->pos_2!=Step->moto_dd_config.DD_cur_cnt_target_pos)
				{
					Step->pos_2 = Step->moto_dd_config.DD_cur_cnt_target_pos;
					Step->step_st.bit.running = RUNNING_OVER;					
				}

				if(isNewTargetPos) /*新数据的第一帧*/
				{
					Step->moto_dd_config.DD_Stop_immed =1;
				}
				
				if((Step->moto_dd_config.DD_IS_High_speed)&&(!Step->moto_dd_config.DD_IS_stope))
				{
					if(Step->speed<step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])
					{
						Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
					}
				}

				#ifdef SSTI_PULS_SMOOTH
				//Step->speed_acc_dec_back = abs((int)Step->moto_dd_config.DD_Smooth.mot_acc);
				Step->speed_acc_dec = abs((int)Step->moto_dd_config.DD_Smooth.mot_acc);
				
				#else

				Step->speed_acc_dec = (Step->max_speed - Step->speed)/(abspos_t<<(isspeeddec?1:0));
				if(Step->speed_acc_dec>STEPMOTOR_ACC_MAX_HZ_P_STEP)
				{
					Step->speed_acc_dec =STEPMOTOR_ACC_MAX_HZ_P_STEP;
				}
				else
					if(Step->speed_acc_dec<STEPMOTOR_ACC_MIN_HZ_P_STEP)
					{
						Step->speed_acc_dec =STEPMOTOR_ACC_MIN_HZ_P_STEP;
					}

				#endif
				

				Step->moto_dd_config.DD_IS_stope =0;

				if(Step->moto_dd_config.DD_IS_setting)
					Step->moto_dd_config.DD_IS_setting =0;
				arch_StepMotor_EnableIT(stepno);
				
				return ;
			}
			else
			{

			do_motor_run_first:
				Step->speed = Step->low_speed;
			}

			Step->moto_dd_config.DD_Speed_dir =0;/*加速*/
			Step->moto_dd_config.DD_IS_stope =0;
			
			
			goto do_it_first;
			}
		}
	}
	
	zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);
	work = arch_StepMotor_work(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
    	ZeroPos_workST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno) && is_sinker(stepno) && zero &&
	   /*(Step->run_mode & MODE_ZERO2) &&*/
	   1) {
		if(work!=ZeroPos_workST){
			zero = 0;
		}
	}
#endif
	if(Step->moto_dd_config.DD_type==1)
	{
		goto do_it_first;
	}
	
	if((mode & 0x1) && (step_resolution[Step->moto_remap_config.moto_attr.bit.moto_type_config-1] != QUARTER)) {
		if(pos < 0) {
			pos *= -1;
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			pos *= -1;
		}
		else {
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		}
	}

	if(pos == 0) {
		if((!Step->step_st.bit.is_poweron) 
			|| (zero && Step->position != 0) 
			|| (!zero && Step->position == 0)) {
			StepMotor_Reset(stepno,0);
			return ;
		}		
	}

	Step->max_speed = max_speed;
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	if(time) 
	{
		int pos_dif = Step->position - pos;
#ifdef STEP_DEBUG_DDM
		Step->step_poslist[0][Step->step_poslistid]=Step->position;
		Step->step_poslist[1][Step->step_poslistid++]=Step->speed;

		if (Step->step_poslistid>=26)
		{
			Step->step_poslistid =0;
		}
#endif		
		
		if(pos_dif < 0) {
			pos_dif = -pos_dif;
		}
		temp_step_acc_dec = 313;
		Step->speed_acc_dec = temp_step_acc_dec;
		new_speed = StepMotor_time2speed(stepno, pos_dif, time,max_speed);
		if(new_speed < 500)
			new_speed = 500;
		Step->max_speed = new_speed;
		if(Step->low_speed > new_speed) {
			Step->low_speed = new_speed;
		}
		
		if(Step->step_st.bit.running) 
		{
			if (Step->step_is_cnt==Step->step_is_cnt_old)   //找机会重新启动。
			{
					arch_StepMotor_Start(stepno);	
			}
			Step->step_is_cnt_old = Step->step_is_cnt;			
			if (Step->pos_2!=pos)
			{
				Step->pos_2 = pos;
				Step->step_st.bit.running = RUNNING_OVER;   
			}
			return ;
		}
	}

	if(Step->step_st.bit.running) {
		if(Step->pos_2 != pos)
		{
			Step->pos_2 = pos;			 
			Step->step_st.bit.running = RUNNING_OVER;      //新动作覆盖旧动作
			return ;
		}
		else
			return;
	}

	do_it_first:

	Step->pos_2 = pos;			//记录到达的目的地
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	//Step->speed = step_base_speed;
	Step->state_par = 0;				//just run
	// by xhl 2010/05/18
	Step->alarm_step = 0;   
	Step->step_st.bit.IS_Reset_ex =0;
	//Step->steps_go_ =0 ;

	//Step->movetozero_check_input =1;
	
	zero_work_area = Get_zero_work_area(Step);

	Step->chk.bit.last_zero_bit = zero?0xff:0;

	if(Step->position == pos) {

		if(Step->moto_dd_config.DD_type==0)
		{
			if(!is_LX_DENSITY_Step(stepno))/*连兴度目不考虑负数方向的信号*/
				{
			if(!zero && (Step->position < step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])) {

				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001);
				
			}
			if(zero && (Step->position > zero_work_area)) {
				
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002);
				
			}
				}
		}
		if(Step->moto_dd_config.DD_type==1) 
		{
			if(Step->moto_dd_config.DD_IS_setting)
				Step->moto_dd_config.DD_IS_setting =0;
			if(Step->moto_dd_config.DD_cur_cnt_target_pos == Step->moto_dd_config.DD_Target_POS)
				Step->moto_dd_config.DD_type=0;	

		}
		return ;
	}

	if(Step->moto_dd_config.DD_type==1)
		goto speed_set;

	// by xhl 2011/01/01
	if(Step->moto_remap_config.moto_attr.bit.is_fast_mode) {
		if(Step->position > pos) {
			Step->steps = Step->position - pos;
		}
		else {
			Step->steps = pos - Step->position;
		}
		//myprintf("pos[%d]->[%d]\r\n", Step->position, pos);
		//myprintf("steps = %d\r\n", Step->steps);
		if(Step->steps > 400) {
			goto speed_set;
		}		
	}

	/*
	 * by xhl 2010/09/15
	 * 1) 步进电机从正位置向负位置运动时
	 *    先归零再向负方向运动
	 */
	if(/*(step_run_mode) &&*/
	   (Step->position > zero_work_area /*ZERO_DETECT + step_zero_adj*/ || !zero) &&
	   (pos ==/*<*/ 0)) {
		Step->state_par = MOVETO_ZERO;
		Step->pos_2 = pos;
		pos = 0;
	}

	/*
	 * by xhl 2012/07/12
	 * 1) 步进电机从负位置向正位置运动时
	 *    先到正位置再归零
	 */
	if(/*(step_run_mode) && */
	   (Step->position < 0) &&
	   (pos ==/*>=*/ 0)) {
		//Step->position -= zero_work_area;
		Step->state_par= LEAVE_ZERO;
		//Step->alarm_step = 1;
		Step->pos_2 = pos;
	}

	

speed_set:
	Step->speed = Step->low_speed;

	// Half On
	arch_StepMotor_Half(stepno, 0);

	if(Step->position > pos) {		
		Step->steps = Step->position - pos;
		Step->step_st.bit.dir = 0;
	}
	else {
		
		Step->steps = pos - Step->position;
		 Step->step_st.bit.dir = 1;
	}
	
#if 1

	if((Step->moto_remap_config.moto_attr.bit.is_fast_mode) && (Step->steps > 400)) {
		if(Step->steps > 800) {
			Step->steps %= 800;			
		}
		else {
			Step->steps = 800 - Step->steps;
			Step->step_st.bit.dir = !Step->step_st.bit.dir;
		}
		Step->state_par = JUST_RUN;
		if(Step->step_st.bit.dir) {
			Step->position = pos - Step->steps;			
		}
		else {
			Step->position = pos + Step->steps;			
		}
		//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);
	}
#endif

		if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]+1) << 1)) {
			Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else {
			Step->acc_steps = Step->steps >> 1;
		}

	if (temp_step_acc_dec==0)
	{
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed)  /Step->acc_steps):0 ;
		#endif	
	}

if(Step->moto_dd_config.DD_type==1)
{
	#ifdef SSTI_PULS_SMOOTH
	Step->speed_acc_dec = abs((int)Step->moto_dd_config.DD_Smooth.mot_acc);
	#else
	Step->speed_acc_dec = (Step->max_speed - Step->speed)/(abspos_t<<(isspeeddec?1:0));
	if(Step->speed_acc_dec>STEPMOTOR_ACC_MAX_HZ_P_STEP)
	{
		Step->speed_acc_dec =STEPMOTOR_ACC_MAX_HZ_P_STEP;
	}
	else
		if(Step->speed_acc_dec<STEPMOTOR_ACC_MIN_HZ_P_STEP)
		{
			Step->speed_acc_dec =STEPMOTOR_ACC_MIN_HZ_P_STEP;
		}
	#endif	
}	


	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	//Step->zero_count = 0;

	#ifdef 0
	Step->step_poslist[0]=0;
	#endif
	Step->step_is_cnt=1;
	Step->step_is_cnt_old=1;
	if(mode & 0x200) {
		Step->step_st.bit.step_flags = 1;
	}

	Step->step_wait_time = 0;
	if (Step->step_st.bit.last_dir!=Step->step_st.bit.dir)
	{
		int rettime=0;
		rettime = Check_step_stop_timeout(Step->last_stop_systick);
		if (rettime)
		{
			Step->step_wait_time = rettime;
			Step->dir_is_change_ =3;
		}
		
	}
	if(Step->step_wait_time==0)
	{
		Step->dir_is_change_=0;
		arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);
	}
	//Message_Send_4halfword(0x88|(stepno <<8),Step->position,pos,Step->state_par|(Step->step_st.bit.dir<<8));
	
	


	if(mode & 0x400) {
		Step->state_par = FEET_STEP_ISWORK;
		Step->pos_2 = 0;
		//Step->work_position_count = 0;
	}




#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = work;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}
#endif


if((Step->moto_dd_config.DD_type==1) &&(Step->moto_dd_config.DD_IS_setting))
	Step->moto_dd_config.DD_IS_setting =0;

arch_StepMotor_Start(stepno);

}
#else


void StepMotor_exec_new_dd(unsigned int stepno, short pos, int time, int mode, int maxspd)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero_work_area;
	int zero;
	int work;
	int ZeroPos_workST;
	int encoder_en = 0;
	short mpos;
	int new_speed;
	int max_speed = stepmotor_speed[0][1] ; 

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

	mpos = Step->position;
	
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config>MOTOR_TYPE_COUNT) return;

	max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1] ; 
	if(maxspd > 0) 
	{
		max_speed = maxspd * 100;
	}
#ifdef ENCODER_SUPPORT
	if (1)//判断是否为传感器类型
	{
		encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
		if (encoder_en)
		{
			Encoder_RunPos(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, &mpos,0);
			//Message_Send_4halfword(0xf4|(stepno<<8),mpos,Step->position,pos);
			#ifdef ECODE_USE_MT6813_PWM
			if(Check_is_MT6813PWM_Mode())
			{
				//Get_code_data_from_mem(idx,&encoder);
				#if 1
				if(Check_Ecoder_is_stable(Step->moto_remap_config.moto_attr.bit.moto_ecode_index) && (!Step->step_st.bit.running))
				{
					Step->position = mpos;
				}
				#endif
			}
			else
			#endif
			{
				if (!Step->step_st.bit.running)	
				Step->position = mpos;
			}
		}
	}
#endif /* ENCODER_SUPPORT */

	if (encoder_en == 0)
	{	

		zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
		work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
		ZeroPos_workST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
#ifdef ZERO2_SUPPORT
		if(is_zero2_support(stepno) && is_sinker(stepno) && zero  && (Step->act_is_justcheck_zero==0)&&
		   /*(Step->run_mode & MODE_ZERO2) &&*/
		   1) {
			if(work!=ZeroPos_workST) {
				zero = 0;
			}
		}
#endif
	}

	if((mode & 0x1) && (step_resolution[Step->moto_remap_config.moto_attr.bit.moto_type_config-1] != QUARTER)) {
		if(pos < 0) {
			pos *= -1;
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			pos *= -1;
		}
		else {
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		}
	}


	if (pos == 0) 
	{
		if (!Step->step_st.bit.is_poweron)
		{
			StepMotor_Reset(stepno,0);
			return;
		}
		else if (encoder_en == 0)
		{
			if ( (zero && mpos != 0) 
				|| (!zero && mpos == 0))
			{
				StepMotor_Reset(stepno,0);
				return ;
			}
		}
	}

	Step->max_speed = max_speed;
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];

	

	Step->steps_go_ =0 ;

#ifdef DEBUG_STEP_DDM_CNT
	{
		Set_pos_debug_cnt_(stepno,pos,Step->position);
	}

#endif
	
	
	if(time) 
	{
		int pos_dif = Step->position - pos;
#ifdef STEP_DEBUG_DDM
		Step->step_poslist[0][Step->step_poslistid]=Step->position;
		Step->step_poslist[1][Step->step_poslistid++]=Step->speed;

		if (Step->step_poslistid>=26)
		{
			Step->step_poslistid =0;
		}
#endif		
	
		
		if(pos_dif < 0) {
			pos_dif = -pos_dif;
		}
		Step->speed_acc_dec = Motor_acc_steps;// /*313;//*/
		#if 0
		{
		if (Step->pos_2!=pos)  // 说明是第一条那就最高速跑
		{
			new_speed =max_speed;
		}
		else		
			new_speed = StepMotor_time2speed(stepno, pos_dif, time,max_speed);
		}
		#endif
		new_speed = StepMotor_time2speed(stepno, pos_dif, time,max_speed);
		//new_speed =max_speed;
		if(new_speed < 500)
			new_speed = 500;
		Step->max_speed = new_speed;
		if(Step->low_speed > new_speed) {
			Step->low_speed = new_speed;
		}
		#ifdef DEBUG_STEP_RESET_FAST_SPEED
		Message_Send_4halfword(0xf1ff,Step->max_speed,Step->low_speed,pos_dif);
		#endif	
		
		if(Step->step_st.bit.running) 
		{		
			if (Step->step_is_cnt==Step->step_is_cnt_old)   //找机会重新启动。
			{
					arch_StepMotor_Start(stepno);	
			}
			Step->step_is_cnt_old = Step->step_is_cnt;			
			if (Step->pos_2!=pos)
			{

				//int decsteps=10;
				//int rungo=0;				

				if (Sys_Step_need_check_ddm)
				{					
					if (abs(Step->position-Step->pos_2)>Sys_Step_need_check_ddm)
					{
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR |0x0003);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0xE001|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						#endif
						return;
					}						
				}	
				
				Step->pos_2 = pos;
				Step->pos_2_main = pos;					
				Step->step_st.bit.running = RUNNING_OVER;   
			}
			return ;
		}
	}

	if(Step->step_st.bit.running) {
		if(Step->pos_2 != pos)
		{
			Step->pos_2 = pos;	
			Step->pos_2_main = pos;
			Step->step_st.bit.running = RUNNING_OVER;      //新动作覆盖旧动作
			return ;
		}
		else
			return;
	}

	Step->pos_2 = pos;			//记录到达的目的地
	Step->pos_2_main = pos;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	//Step->speed = step_base_speed;
	Step->state_par = 0;				//just run
	// by xhl 2010/05/18
	Step->alarm_step = 0;   
	Step->step_st.bit.IS_Reset_ex =0;
	//Step->steps_go_ =0 ;

	if (encoder_en == 0)
	{		
		if (is_LX_DENSITY_Step(stepno)  
			|| is_HF_LIFT_Step(stepno)
			||is_EX_OTHER_Step(stepno)
			||is_DOT_ACTION_Step(stepno))
		{
			zero_work_area = (Step->moto_zero_width_self+Step->input_errorstep)>>1;//传感器可能亮的范围
		}
		else
		zero_work_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

		//zero_work_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1] + Step->input_errorstep;
		//zero_work_area = Get_zero_work_area(Step);//stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		if(Step->position == pos) 
		{
			return;
			#if 0
			if (is_LX_DENSITY_Step_ex(Step))
			{
				if(!zero && (mpos <=zero_work_area )&&(mpos >=(0-zero_work_area))) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001);
						#endif
					}
				if(zero && ((mpos > zero_work_area)||(mpos <(0- zero_work_area)))) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_VALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002);
						#endif
					}
			}
			else
			{
				
				if(!zero && (Step->position < step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1])) {
					#ifdef NEW_ALARM_STYLE
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
					#else
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001);
					#endif
				}
				if(zero && (Step->position > zero_work_area)) {
					#ifdef NEW_ALARM_STYLE
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_VALID);
					#else
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002);
					#endif
				}
			}
			return ;
			#endif
		}
	}	

	// by xhl 2011/01/01
	if(Step->moto_remap_config.moto_attr.bit.is_fast_mode) {
		if(Step->position > pos) {
			Step->steps = Step->position - pos;
		}
		else {
			Step->steps = pos - Step->position;
		}
		if(Step->steps > 400) {
			goto speed_set;
		}		
	}

	if (encoder_en>0)
	{

		if (Step->position ==pos )	
		return;	

	
		goto speed_set;
	}

	if (is_LX_DENSITY_Step(stepno))
	{
		if (pos==0)
		{
			Step->state_par = MOVETO_ZERO;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =1;
			#endif
			Step->pos_2 = pos;
		}
		goto speed_set;

	}
		
	/*
	 * by xhl 2010/09/15
	 * 1) 步进电机从正位置向负位置运动时
	 *    先归零再向负方向运动
	 */
	if(/*(step_run_mode) &&*/
	   (Step->position > zero_work_area /*ZERO_DETECT + step_zero_adj*/ || !zero) &&
	   (pos ==/*<*/ 0)) {
		Step->state_par = MOVETO_ZERO;
		#ifdef DEBUG_ALERT_65
		Step->which_gotozero =2;
		#endif
		Step->pos_2 = pos;
		pos = 0;
	}

	/*
	 * by xhl 2012/07/12
	 * 1) 步进电机从负位置向正位置运动时
	 *    先到正位置再归零
	 */
	if(/*(step_run_mode) && */
	   (Step->position < 0) &&
	   (pos ==/*>=*/ 0)) {
		//Step->position -= zero_work_area;
		Step->state_par = LEAVE_ZERO;
		//Step->alarm_step = 1;
		Step->pos_2 = pos;
	}

speed_set:
	Step->speed = Step->low_speed;

	// Half On
	arch_StepMotor_Half(stepno, 0);

	if(Step->position > pos) {		
		Step->steps = Step->position - pos;
		Step->step_st.bit.dir = 0;
	}
	else {
		
		Step->steps = pos - Step->position;
		 Step->step_st.bit.dir = 1;
	}
	
#if 1

	if((Step->moto_remap_config.moto_attr.bit.is_fast_mode) && (Step->steps > 400)) {
		if(Step->steps > 800) {
			Step->steps %= 800;			
		}
		else {
			Step->steps = 800 - Step->steps;
			Step->step_st.bit.dir = !Step->step_st.bit.dir;
		}
		Step->state_par = JUST_RUN;
		if(Step->step_st.bit.dir) {
			Step->position = pos - Step->steps;			
		}
		else {
			Step->position = pos + Step->steps;			
		}
		//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);
	}
#endif
	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);


		if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]) << 1)) {
			Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else {
			Step->acc_steps = Step->steps >> 1;
		}

#ifdef STEP_MOTOR_DDDM_SUPPORT
	//Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed) / Step->acc_steps) : 0;
#endif


	arch_StepMotor_Stop(stepno);
#if 0
	if (Step->step_stop_long>2)
	{
		Step->step_wait_time = 0;
	}
	else
		Step->step_wait_time = 3;
	#else
	Step->step_wait_time = 0;
#endif

	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;

	#if 0
	Step->step_poslist[0]=0;
	#endif
	Step->step_is_cnt=1;
	Step->step_is_cnt_old=1;
	if(mode & 0x200)
	{
		Step->step_st.bit.step_flags=1;
	}

	
	


	if(mode & 0x400) {
		Step->state_par = FEET_STEP_ISWORK;
		//Step->pos_2 = 0;
		//Step->pos_2_main = 0;
		Step->steps_go_temp = 0;
	}


	

	arch_StepMotor_Start(stepno);


	if (encoder_en>0)
		return;

#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = work;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}
#endif

	

}

#endif



#endif

void check_step_pos_isok(short pos,int isexe_code)
{
	return;
	#if 0
	if ((pos!=0)&&(pos>-70)&&(pos<70))
	{
		alert_push(isexe_code+0x88,pos);	
		
	}
	#endif
}


void StepMotor_justrun(unsigned int stepno,unsigned char needreturn, short pos)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	if(stepno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	Step->step_debug_test_st.bit.justrun_test =1;
	if (needreturn) 
	{
		Step->step_debug_test_st.bit.justrun_test |=0x02;
	}
	
	if (pos==0) return;
	
	Step->position = 0 ;
	
	Step->pos_2 = pos;
	Step->pos_2_main = pos;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	#ifdef STEP_DEBUG_HZ
	Step->speed_HZ_idx = 0;
	#endif
	Step->state_par = 0;	
	Step->alarm_step = 0;   
	Step->step_st.bit.IS_Reset_ex =0;
	Step->max_speed = 2000;
	Step->low_speed = 1000;//2000;
	Step->speed = Step->low_speed;

	Step->steps_go_=0;

#ifndef STEP_MOTOR_RUN_HALF_MODE

#ifdef STEP_CUR_HALF_RUN
	arch_StepMotor_Half(stepno, 1);
#else
	arch_StepMotor_Half(stepno, 0);
#endif
#endif
	if(Step->position > pos) {
		
		Step->steps = Step->position - pos;
		Step->step_st.bit.dir = 0;
	}
	else {
		//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		Step->steps = pos - Step->position;
		Step->step_st.bit.dir = 1;
	}
	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);

		if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]) << 1)) {
			Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else {
			Step->acc_steps = Step->steps >> 1;
		}
		

	Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed)  /Step->acc_steps):0 ;
	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	Step->step_wait_time = 0;
	if (Step->step_st.bit.last_dir!=Step->step_st.bit.dir)
	{
		int rettime=0;
		rettime = Check_step_stop_timeout(Step->last_stop_systick);
		if (rettime)
		{
			Step->step_wait_time = rettime;
		}
	}
	arch_StepMotor_Start(stepno);
	

}

void StepMotor_runingover(unsigned int stepno, short pos, int mode)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero_work_area;
	int zero;
	//int isleft;
	//int isright;
	int work;
	int ZeroPos_workST;
	int encoder_en = 0;
	short mpos;
	
	if(stepno >= StepMotor_Count_MAX) return;
	
	Step = &STEPMOTOR[stepno];
	mpos = Step->position;
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config>MOTOR_TYPE_COUNT) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_UNDEF) return;
	
	#ifdef ENCODER_SUPPORT
	#ifdef QL_DEBUG_STEP_MOTOR_ECORD
	Message_Send_4halfword(0x97|(stepno<<8),Step->moto_remap_config.moto_attr.all,Step->position,Step->pos_2);
#endif

	
	
	if (1)//判断是否为传感器类型
	{
		encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
		if (encoder_en)
		{
			Encoder_RunPos(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, &mpos,1);
			Message_Send_4halfword(0xf3|(stepno<<8),mpos,Step->position,pos);
			//Step->position = mpos;
			#ifdef ECODE_USE_MT6813_PWM
			if(Check_is_MT6813PWM_Mode())
			{
				//Get_code_data_from_mem(idx,&encoder);
				#if 1
				if(Check_Ecoder_is_stable(Step->moto_remap_config.moto_attr.bit.moto_ecode_index) && (!Step->step_st.bit.running))
				{
					Step->position = mpos;
				}
				#endif
			}
			else
			#endif
			{
				if (!Step->step_st.bit.running)	
				Step->position = mpos;
			}
		}
	}
#endif /* ENCODER_SUPPORT */

	if (encoder_en == 0)
	{
		zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
		work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
		ZeroPos_workST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
		if(is_HP_ACTION_Step(stepno)) 
		{		
			//Step_input_sts_first(stepno);
			//isleft = Triangle_is_left_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
			//isright =Triangle_is_right_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
			zero =Triangle_is_zero_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		//Step->steps_go_temp=0;	
		//Step->state_par = STEP_RESET_HP_ACT;
		}

		if (is_HF_LIFT_Step(stepno))
		{
			check_step_pos_isok(pos,1);			
		}

		if (is_LX_DENSITY_Step(stepno)  
			|| is_HF_LIFT_Step(stepno)
			||is_EX_OTHER_Step(stepno)
			||is_DOT_ACTION_Step(stepno)
			||is_CX3_Feet_Step(stepno)
			||is_LX_yarn_step(stepno))
		{
			zero_work_area = (Step->moto_zero_width_self+Step->input_errorstep)>>1;//传感器可能亮的范围
		}
		else
		zero_work_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

		zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);


		
#ifdef ZERO2_SUPPORT
		if(is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) && zero && (Step->act_is_justcheck_zero==0))
		{
			if(work!=ZeroPos_workST) {
				zero = 0;
			}
		}
#endif
	}
	
	Step->pos_2 = pos;			//记录到达的目的地
	Step->pos_2_main = pos;
	Step->need_2_pos_after_reset = 0;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.IS_Reset_ex =0;
	Step->steps_go_=0;
	Step->change_speed =0;
	
	if (encoder_en == 0)
	{
		if(Step->position == pos) 
		{		
			return ;
		}
	}

	if(is_HP_ACTION_Step(stepno)) 
	{		
		Step_input_sts_first(stepno);
	}

	// by xhl 2011/01/01
	if(Step->moto_remap_config.moto_attr.bit.is_fast_mode) {
		if(Step->position > pos) {
			Step->steps = Step->position - pos;
		}
		else {
			Step->steps = pos - Step->position;
		}
		if(Step->steps > 400) {
			goto speed_set_over;
		}
	}
	
	if (encoder_en > 0)
	{
		goto speed_set_over;
	}

	if(is_HP_ACTION_Step(stepno)) 
	{	
		if(  ((Step->position !=0 )||( !zero)) &&  (pos == 0)) 
		{
	   		Step->step_st.bit.check_delay_count =0;
			Step->state_par = MOVETO_ZERO;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =3;
			#endif
			Step->pos_2 = pos;		
		}
	}
	else
	{

		if (is_LX_ACTION_Step(stepno))
		{
			if (pos==0)
			{
				if ((Step->position>0)&&(!zero))   //
				{
					Step->state_par = MOVETO_ZERO;
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =4;
					#endif
					Step->pos_2 = pos;
				}
				if (((Step->position<0)&&(!zero))||((Step->position<-100)&&(zero)))
				{
					if(F_ACT_Motor_MOVE_zero_type)
					{
						if((Step->position<-100)&&(zero))
						{
							Step->change_speed =1;	
						}
					}
					
					Step->state_par = MOVETO_ZERO_LX_ACT;
					Step->pos_2 = pos;
				}
			}

		}
		else
		{

		if (is_LIFT_or_Other_Step(stepno) ||(is_sinker(stepno)))
		{
				if (pos == 0)
				{
					Step->pos_2 = pos;
					//Step->state_par = mpos > zero_work_area?MOVETO_ZERO:MOVETO_ZERO_LIFT;
					if (Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)
					{
						Step->state_par = (Step->position <(0-zero_work_area))?MOVETO_ZERO:MOVETO_ZERO_LIFT;
						
					}
					else
					{
						Step->state_par = Step->position > zero_work_area?MOVETO_ZERO:MOVETO_ZERO_LIFT;
					}
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =5;
					#endif
				}

				goto speed_set_over;
		}

		

			if (is_LX_DENSITY_Step(stepno))
			{
				if (pos==0)
				{
					Step->state_par = MOVETO_ZERO;
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =6;
					#endif
					Step->pos_2 = pos;
				}
				goto speed_set_over;

			}
		/*
		 * by xhl 2010/09/15
		 * 1) 步进电机从正位置向负位置运动时
		 *    先归零再向负方向运动
		 */
		if(/*(step_run_mode) &&*/
		   (Step->position > zero_work_area /*ZERO_DETECT + step_zero_adj*/ || !zero) &&
		   (pos ==/*<*/ 0)) {
			Step->state_par = MOVETO_ZERO;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =7;
			#endif
			Step->pos_2 = pos;
			pos = 0;

			//Message_Send_4halfword_debug(0x0909,stepno,0xEEEE,0xffff);
			
		}

		/*
		 * by xhl 2012/07/12
		 * 1) 步进电机从负位置向正位置运动时
		 *    先到正位置再归零
		 */
		if(/*(step_run_mode) && */
		   (Step->position < (-5)) &&
		   (pos ==/*>=*/ 0)) {
			//Step->position -= zero_work_area;
			Step->state_par = LEAVE_ZERO;
			//Step->alarm_step = 1;
			Step->pos_2 = pos;
			//Message_Send_4halfword_debug(0x0909,stepno,0xDDDD,0xffff);
		}

		if(Step->alert_delay_max)
		{
		Step->state_par = JUST_RUN;
		}
			}

	}
speed_set_over:
	//Step->max_speed = Step->need_2_pos_maxspeed;

	if(is_LX_ACTION_Step(stepno))
	{
		Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
	}

#ifdef STEP_MOTOR_DDDM_SUPPORT
	
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//2000;
	//Step->speed = Step->low_speed;

#else
	//Step->speed =step_base_speed;
#endif

	// Half On
#ifndef STEP_MOTOR_RUN_HALF_MODE
	#ifdef STEP_CUR_HALF_RUN
	arch_StepMotor_Half(stepno, 1);
#else
	arch_StepMotor_Half(stepno, 0);
#endif
#endif

	if(Step->position > pos) {
		//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		Step->steps = Step->position - pos;
		Step->step_st.bit.dir = 0;
	}
	else {
		//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		Step->steps = pos - Step->position;
		Step->step_st.bit.dir = 1;
	}
	
#if 1
	if((Step->moto_remap_config.moto_attr.bit.is_fast_mode) && (Step->steps > 400)) {
		if(Step->steps > 800) {
			Step->steps %= 800;
		}
		else {
			Step->steps = 800 - Step->steps;
			Step->step_st.bit.dir = !Step->step_st.bit.dir;
		}
		Step->state_par = JUST_RUN;
		if(Step->step_st.bit.dir) {
			Step->position = pos - Step->steps;
			//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		}
		else {
			Step->position = pos + Step->steps;
			//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		}
	}
#endif
arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);

	if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]) << 1)) {
		Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	}
	else {
		Step->acc_steps = Step->steps >> 1;
	}
#ifdef STEP_MOTOR_DDDM_SUPPORT
	Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed) / Step->acc_steps) : 0;
#endif

	//arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	//myprintf("[%d] %d\r\n", stepno, Step->steps);

	if((Step->moto_remap_config.moto_attr.bit.moto_type_config ==MOTOR_TYPE_FEET )
		&&((Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_PT)
		||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX)))
	{
		if(mode & 0x400) 
		{
			Step->state_par = FEET_STEP_ISWORK;
		//Step->pos_2 = 0;
		//Step->pos_2_main = 0;
			Step->steps_go_temp = 0;
		}
	}

#ifdef ENCODER_SUPPORT
if (encoder_en > 0)
{
	Encoder_setCheck(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, 0);
}
#endif

	Step->step_wait_time = 3;

#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = work;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}
#endif

	arch_StepMotor_Start(stepno);

}

void StepMotor_setpos_2main_postion(unsigned int stepno)
{
	if(stepno >= StepMotor_Count_MAX) return;
	
	STEPMOTOR[stepno].pos_2_main +=1 ;//只要和原来不一样就可以，所以做了一个加一。	

}
void StepMotor_set_LX_ACT_AUTOadj_enable(unsigned short adjenable)
{
	StepMotor_LX_ACTStep_autoadj = adjenable ?1: 0;
}

unsigned int Yarn_motor_input_bits[8];
unsigned short Yarn_motor_bits_ptr=0;

void Yarn_motor_input_bits_init(unsigned int stepno)
{
	if(stepno!=11) return;
	memset(Yarn_motor_input_bits,0,sizeof(Yarn_motor_input_bits));
	Yarn_motor_bits_ptr =0;
}

void Yarn_motor_input_bits_set(unsigned int stepno,unsigned char iszero)
{
	
	if((Yarn_motor_bits_ptr>=256)
		||(stepno!=11))
	{
		return;
	}
	if(iszero)
		Yarn_motor_input_bits[Yarn_motor_bits_ptr/32] |= (1<<(Yarn_motor_bits_ptr%32));
	else
		Yarn_motor_input_bits[Yarn_motor_bits_ptr/32] &=~ ((unsigned int)1<<(Yarn_motor_bits_ptr%32));
	
	Yarn_motor_bits_ptr++;
}

void Yarn_motor_input_bits_send(unsigned int stepno)
{
	int i;
	
	if((stepno!=11)
		||(Yarn_motor_bits_ptr==0))return;
	
	for(i=0;i<(1+((Yarn_motor_bits_ptr-1)/32));i++)
	{
		Message_Send_4halfword(0xFF|(i<<8),Yarn_motor_bits_ptr, Yarn_motor_input_bits[i] & 0xffff, (Yarn_motor_input_bits[i]>>16) & 0xffff);
	}
}


void StepMotor_exec(unsigned int stepno, short pos, int mode,unsigned short otherarg,unsigned char workspeed)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero_work_area;
	int zero;
	//int isleft;
	//int isright;
	int work;
	int ZeroPos_workST;
	//int hp_st_last;
	int encoder_en = 0;
	short mpos;
	unsigned char z1;
	unsigned  char w1;
	
	if(stepno >= StepMotor_Count_MAX) return;
	
	Step = &STEPMOTOR[stepno];
	mpos = Step->position;


	if(is_LX_DENSITY_Step(stepno))
	{
		if(Step->isdoing_with_bind_cmd)
		{			
			Message_Send_4halfword(0xf9|(stepno<<8),Step->position,pos,Step->pos_2_main);
		}
	}
	
	#ifdef LX_DEBUG
	if (is_DENSITY_Step(stepno))
	{
		if ((Step->moto_remap_config.moto_attr.bit.moto_type_exe!=DENSITY_MOTOR_TYPE_LX)
			&&(pos<0))
		alert_push(STEP_ALARM_DEBUG_1,pos);	
	}
	#endif
#if 0	
	if ((Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION)
		&&(pos == 0))
		Message_Send_4halfword(0x9999,stepno,mpos,Step->moto_remap_config.moto_attr.all);
#endif
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config>MOTOR_TYPE_COUNT) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_UNDEF) return;


#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if (((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)&&(enable_log_tosend_LX))	
	{
		unsigned short d[3];
		d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
		d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
		d[0] |= LX_LOG_TYPE_MOVETOZERO_ADD400_2<<8;
		d[0] |= 1<<12;
		d[1] =Step->position;
		d[2] =pos;								
		Message_send_log_LX(d[0],d[1],d[2]);
	}
#endif
	if(Step->moto_dd_config.DD_type)
	{

		if(pos)
		{
			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x7F7F);
			return;
		}
		else
		{
			if(Step->moto_dd_config.DD_cur_cnt_target_pos!=Step->moto_dd_config.DD_Target_POS)
			{
				
			}
		}
		Step->moto_dd_config.DD_type =0;
	}

	
	//myprintf("[%d]StepExec[%d]\r\n", stepno, pos);
#ifdef ENCODER_SUPPORT
	if (1)//判断是否为传感器类型
	{
		encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
		if (encoder_en)
		{
			
			
			Encoder_RunPos(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, &mpos,1);
			//Message_Send_4halfword(0xf2|(stepno<<8),mpos,Step->position,pos);
			
			#ifdef ECODE_USE_MT6813_PWM
			if(Check_is_MT6813PWM_Mode())
			{
				//Get_code_data_from_mem(idx,&encoder);
				#if 1
				if(Check_Ecoder_is_stable(Step->moto_remap_config.moto_attr.bit.moto_ecode_index) && (!Step->step_st.bit.running))
				{
					Step->position = mpos;
				}

				#endif
			}
			else
			#endif	
			{
				if (!Step->step_st.bit.running)
					Step->position = mpos;
			}
		}
		//Message_Send_4halfword(0xFfF1,Step->moto_remap_config.moto_attr.bit.moto_ecode_index,Step->position,encoder_en);
			
	}
#endif /* ENCODER_SUPPORT */

	if (encoder_en == 0)
	{
		zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
		work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
		ZeroPos_workST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
		w1 = work?0xFF:0x00;
		z1 = zero?0xFF:0x00;

		if (is_LX_DENSITY_Step(stepno)  
			|| is_HF_LIFT_Step(stepno)
			||is_EX_OTHER_Step(stepno)
			||is_DOT_ACTION_Step(stepno)
			||is_lx_sinker(stepno)
			||is_lxex_sinker(stepno)
			||is_CX3_Feet_Step(stepno)
			||is_LX_yarn_step(stepno))
		{
			zero_work_area = (Step->moto_zero_width_self+Step->input_errorstep)>>1;//传感器可能亮的范围
			#ifdef IS_CX_ACT
				if(is_DOT_ACTION_Step(stepno))
					zero_work_area = _CX_DOT_ACT_ZERO_ADJ;
			#endif

			if(is_LX_yarn_step(stepno))
			{
				if(Step->chk.bit.check_Right_Now)
					Step->chk.bit.check_Right_Now =0;
			}
		}
		else
		zero_work_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

		zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);


		if (is_HF_LIFT_Step(stepno))
		{
			check_step_pos_isok(pos,0);			
		}
		
		if(is_HP_ACTION_Step(stepno)) 
		{		
			//int hp_st_with_input =0;
			//Step_input_sts_first(stepno);
			//hp_st_with_input = ((zero & 0x01) <<1) |(work & 0x01) ;
		//	isleft = Triangle_is_left_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
		//	isright =Triangle_is_right_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
			zero =Triangle_is_zero_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		//Step->steps_go_temp=0;	
		//Step->state_par = STEP_RESET_HP_ACT;
		//	hp_st_last = Get_st_with_postion(Step->position);
			//if (hp_st_with_input !=hp_st_last)
			//{			
		
			//}
			//hp_st_with_input = 
		}

		if(is_CX3_Feet_Step(stepno))
		{
			if((zero)&&(work!=ZeroPos_workST))
			{
				zero = 0;	
				z1 = zero?0xFF:0x00;
			}
		}		
#ifdef ZERO2_SUPPORT
		if(is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) && zero && (Step->act_is_justcheck_zero==0))
		{
			if(work!=ZeroPos_workST) {
				zero = 0;
			}
		}
#endif
	}
	
	if((mode & 0x1) && (step_resolution[Step->moto_remap_config.moto_attr.bit.moto_type_config-1] != QUARTER)) {
		if(pos < 0) {
			pos *= -1;
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			pos *= -1;
		}
		else {
			pos = stepspeed_Q2R(pos,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		}
	}

	if (pos == 0) 
	{
		if (Step->step_st.bit.running)
			goto step_goto_run;
			
	
		if (!Step->step_st.bit.is_poweron)
		{
			StepMotor_Reset(stepno,otherarg);
			//Message_Send_4halfword(0x1133,stepno,0,0);
			return;
		}
		else if (encoder_en == 0)
		{
			if ((is_LX_DENSITY_Step(stepno))
				||(is_HF_LIFT_Step(stepno))
				||is_LX_yarn_step(stepno))
			{
				goto step_goto_run;
			}

			if (is_Feet_Step(stepno))
			{
				goto step_goto_run;
			}

			if (is_FH_ACTION_Step(stepno))
			{
				goto step_goto_run;
			}
			//#ifdef LX_ACTION_STEP_SPECIAL
			if (is_LX_ACTION_Step(stepno))
			{
				;
			}
			else			
			if(!is_HP_ACTION_Step(stepno)) 
			{
				if (is_LIFT_or_Other_Step(stepno) || is_PT_sinker(stepno))
				{
						unsigned char isright;
						//int zero;	
						int stepdir = Step->step_st.bit.dir_High;
						int needreset=0;

						if (Step->position>=(0-zero_work_area) && Step->position<=(zero_work_area) )
							goto step_goto_run;
							
						//zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);
						isright= ((stepdir==0)&&(Step->position<(0-zero_work_area))) ||((stepdir!=0)&&(Step->position>zero_work_area));
						if (isright)
						{
							if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)&&(!zero))
							{
								needreset=1;
							}
							 if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir==0)&&(zero))
							{
								needreset=1;								
							}							
						}
						else
						{
							if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)&&(zero))
							{
								needreset=1;
							}
							 if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir==0)&&(!zero))
							{
								needreset=1;
							}							
						}

						if ((needreset)&&(Step->alert_delay_max==0))
						{
							StepMotor_Reset(stepno,otherarg);

							//Message_Send_4halfword(0x1122,stepno,0,0);
							return ;
						}					

				}
				else
				{	
					if (is_DENSITY_Step(stepno) ||is_sinker(stepno)||(is_PT_ACTION_Step(stepno) && !is_zero2_support(stepno) )) 
					goto step_goto_run;
					
					if ( (zero && mpos != 0) 
					|| (!zero && mpos == 0))
					{
							Message_Send_4halfword(0xFF04,(stepno<<8)|0x04,Step->position,Step->pos_2);
							Step->st_no_cl.bit.check_zero_work_tmp = Step->st_no_cl.bit.check_zero_work|(1<<4);
							StepMotor_Reset(stepno,otherarg);
							return ;
					}
					
				}
			}
			
		}
	}

	step_goto_run:

		#if 0
	if(is_DENSITY_Step(stepno))
	{
		Message_Send_4halfword(0xFDFF,(stepno<<8)|0x04,Step->position,(Step->step_st.bit.running?1:0)<<15|(Step->pos_2));	
		//if(pos==Step->pos_2_main)
		{
		Message_Send_4halfword(0xFCFF,(stepno<<8)|0x04,Step->pos_2_main,(zero?1:0)<<15|(pos));	
			
		}
	}
	#endif
	if(Step->step_st.bit.running) {
		if(Step->pos_2 != pos)
		{
			Step->pos_2 = pos;	
			Step->pos_2_main = pos;
			if (Step->step_st.bit.running)
			{
				Step->step_st.bit.running = RUNNING_OVER;      //新动作覆盖旧动作
				if (otherarg)
				{
					Step->st_no_cl.bit.report_cnt++;
				}
			//	Message_Send_4halfword(0x9599,stepno,Step->position,Step->pos_2);
				return ;
			}				
		}
		else
		{
			if (otherarg)
			{
				Step->st_no_cl.bit.report_cnt++;
			}
			return ;
		}
		
	}

	if (otherarg)
	{
		Step->st_no_cl.bit.report_cnt++;
	}

	if ((pos==Step->pos_2_main) &&(Step->position == pos))
	{

			//Message_send_log_ecode(Step->pos_2_main,2,0xFFFF);
			if(is_LX_ACTION_Step(stepno)
				||is_CX2_Feet_Step(stepno)
				||is_FH_ACTION_Step(stepno)
				||is_HF_LIFT_Step(stepno)
				||is_EX_OTHER_Step(stepno)
				||is_DOT_ACTION_Step(stepno)
				||is_CX3_Feet_Step(stepno)
				||is_LX_yarn_step(stepno)
				||is_LX_DENSITY_Step(stepno))
			{
				Step->chk.bit.check_sts_enable = 1;
				//return;
			}
			return;
	}
	else
		Step->pos_2_main =pos;
	
	Step->chk.bit.last_work_bit = w1;//?0xff:0x00;
	Step->chk.bit.last_zero_bit = z1;//zero?0xff:0x00;
	Step->pos_2 = pos;			//记录到达的目的地
	Step->pos_2_main =pos;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	#ifdef STEP_DEBUG_HZ
	Step->speed_HZ_idx = 0;
	#endif
	//Step->speed = step_base_speed;
	Step->state_par = JUST_RUN;				//just run
	Step->st_no_cl.bit.checkisleavezero =0;
	// by xhl 2010/05/18
	Step->alarm_step = 0;   
	Step->step_st.bit.IS_Reset_ex =0;
	Step->chk.bit.check_sts_enable = 0;
	Step->chk.bit.check_Right_Now =0;
	Step->step_st.bit.zero = zero;
	Step->chk.bit.HP_check_isover =0;
	Step->chk.bit.HP_check_isok = 0;
	Step->chk.bit.HP_auto_adj =0;
	Step->steps_go_ =0;
	Step->chk.bit.SK_ADJ_ok =0;//清除修正标记
	Step->chk.bit.Zero_Out_ADJ_OK =0;
	Step->HF_ex_cnt =0;
	Step->steps_go_temp =0;
	Step->change_speed =0;
	Step->max_speed_back =0;
	Step->yarn_motor_elapse_steps=0;

	//Step->step_alert_delay_pos2_st.bit.last_needcheck_zero = Step->step_alert_delay_pos2_st.bit.new_needcheck_zero;
	//Step->step_alert_delay_pos2_st.bit.new_needcheck_zero= Step->step_st.bit.dir_High!=Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir?((pos<=0)?1:0):((pos>=0)?1:0);



	if (encoder_en == 0)
	{
		//zero_work_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;

		//zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);

		if(mpos == pos) 
		{

			if (is_LIFT_Step(stepno) && (Step->st_no_cl.bit.check_is_first))
			{
				return ;
			}
			//#ifdef LX_ACTION_STEP_SPECIAL
			if(is_LX_ACTION_Step(stepno)
				||is_CX2_Feet_Step(stepno) 
				||is_FH_ACTION_Step(stepno)
				||is_HF_LIFT_Step(stepno)
				||is_EX_OTHER_Step(stepno)
				||is_DOT_ACTION_Step(stepno)
				||is_CX3_Feet_Step(stepno) 
				||is_LX_yarn_step(stepno))
			{
				Step->chk.bit.check_sts_enable = 1;
				return;
			}

			//#endif

			if (is_HP_ACTION_Step(stepno))
			{
				if(!zero && (Step->position ==0)) {
					#ifdef NEW_ALARM_STYLE
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
					#else
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
					#endif
				}
				if(zero && (Step->position !=0)) {
					#ifdef NEW_ALARM_STYLE
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_VALID);
					#else
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
					#endif
				}	
			}
			else
			{
				if (is_JF_DENSITY_Step(stepno))
				{
					if ((mpos>DM_check_area_max)||(mpos<DM_check_area_min))
					{
						return;
					}
				}
				
				if (is_LX_DENSITY_Step(stepno))
				{
					if(!zero && (mpos <=zero_work_area )&&(mpos >=(0-zero_work_area))) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						#endif
					}
					if(zero && ((mpos > zero_work_area)||(mpos <(0- zero_work_area)))) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_VALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						#endif
					}
				}
				else				
				{
					return;
					#if 0
					if(!zero && (mpos < step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1] )) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001);
						#endif
					}
					if(zero &&  (mpos > zero_work_area)) {
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_VALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002);
						#endif
						#ifdef LX_DEBUG
						{
							alert_push(STEP_ALARM_DEBUG_1,mpos);
							alert_push(STEP_ALARM_DEBUG_2,zero_work_area);
						}
						#endif
					}
					#endif
				}
			}
			return ;
		}
	}

	if(is_HP_ACTION_Step(stepno)) 
	{		
		Step_input_sts_first(stepno);
	}

	if(is_PT_ACTION_Step(stepno) &&(!is_zero2_support(stepno))&&(zero)&&(Step->pos_2==0))
	{
		Step->st_no_cl.bit.checkisleavezero=1;
	}
	


	if (encoder_en > 0)
	{
		Step->st_no_cl.bit.checkisleavezero=0;
	
		goto speed_set;
	}

	if (is_lx_sinker(stepno)||is_lxex_sinker(stepno))/*连兴生克需要考虑一直在传感器内部的问题*/
	{
		if(  ((Step->position !=0 )||( !zero)) &&  (pos == 0)) 
		{
	   		Step->step_st.bit.check_delay_count =0;
			Step->state_par = (!zero)?MOVETO_ZERO:JUST_RUN;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =8;
			#endif
			Step->pos_2 = pos;		
		}
	}
	else
	if(is_HP_ACTION_Step(stepno)  
		|| is_HF_LIFT_Step(stepno)
		||is_EX_OTHER_Step(stepno)
		||is_DOT_ACTION_Step(stepno) 
		||is_CX3_Feet_Step(stepno)
		||is_LX_yarn_step(stepno))
	{	
		if(  ((Step->position !=0 )||( !zero)) &&  (pos == 0)) 
		{
	   		Step->step_st.bit.check_delay_count =0;
			Step->state_par = MOVETO_ZERO;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =9;
			#endif
			Step->pos_2 = pos;		
		}
	}
	else
	{

		if (is_FH_ACTION_Step(stepno))
		{
			if (pos==0)
			{
				if (mpos > zero_work_area /*ZERO_DETECT + step_zero_adj*/ || !zero)  //  回零和普通机一样
				{
					Step->state_par = MOVETO_ZERO;
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =10;
					#endif
					Step->pos_2 = pos;
				}

				if (mpos<0)
				{
					Step->state_par = LEAVE_ZERO;
					Step->pos_2 = pos;
				}
				
			}
			
		}
		//#ifdef LX_ACTION_STEP_SPECIAL
		if (is_LX_ACTION_Step(stepno))
		{
			if (pos==0)
			{
				if ((Step->position>0)&&(!zero))   //
				{
					Step->state_par = MOVETO_ZERO;
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =11;
					#endif
					Step->pos_2 = pos;
				}
				if (((Step->position<0)&&(!zero))||((Step->position<-100)&&(zero)))
				{
					if(F_ACT_Motor_MOVE_zero_type)
					{
						if((Step->position<-100)&&(zero))
						{
							Step->change_speed =1;	
						}
					}
				
					Step->state_par = MOVETO_ZERO_LX_ACT;
					
					Step->pos_2 = pos;
				}
				if(!zero)
				{
					Step->steps_go_temp =4;/*OK*/
				}
			}
			else
			if(StepMotor_LX_ACTStep_autoadj)
			{

			#ifndef LX_ACT_DONOT_ADJ
				if ((Step->position>pos) &&(pos>90))
				{
					Step->state_par = GOTOZERO_LX_ACT;
					Step->pos_2 = pos;
				}
				else
					if ((Step->position<pos) &&(pos>-160)&&(pos<-90))
					{
						Step->state_par = GOTOZERO_LX_ACT;
						Step->pos_2 = pos;
					}
			#endif		
			}

		}
		else
		{

			if (is_LIFT_or_Other_Step(stepno))
			{
				if (pos == 0)
				{
					Step->pos_2 = pos;

					{
						if (!zero)	
						{
							Step->state_par = MOVETO_ZERO;
							#ifdef DEBUG_ALERT_65
							Step->which_gotozero =12;
							#endif
						}
					}
					
				}

				goto speed_set;
			}
			if (is_LX_DENSITY_Step(stepno))
			{
				if (pos==0)
				{
					if(zero)
					{
					Step->state_par = LEAVE_ZERO;
					}
					else
					Step->state_par = MOVETO_ZERO;
					#ifdef DEBUG_ALERT_65
					Step->which_gotozero =13;
					#endif
					Step->pos_2 = pos;
				}
				goto speed_set;

			}

			//#endif
		
			/*
			 * by xhl 2010/09/15
			 * 1) 步进电机从正位置向负位置运动时
			 *    先归零再向负方向运动
			 */
			if(/*(step_run_mode) &&*/
			   (mpos > zero_work_area /*ZERO_DETECT + step_zero_adj*/ || !zero) &&
			   (pos ==/*<*/ 0)) {
				Step->state_par = MOVETO_ZERO;
				#ifdef DEBUG_ALERT_65
				Step->which_gotozero =14;
				#endif
				Step->pos_2 = pos;
				pos = 0;

				//Message_Send_4halfword_debug(0x0909,stepno,0xEEEE,0xffff);
				
			}

			/*
			 * by xhl 2012/07/12
			 * 1) 步进电机从负位置向正位置运动时
			 *    先到正位置再归零
			 */
			if(/*(step_run_mode) && */
			   (mpos < (-5)) &&
			   (pos ==/*>=*/ 0)) {

				#ifdef LX_DEBUG	
				if (is_DENSITY_Step(stepno))
				{
					alert_push(STEP_ALARM_DEBUG_2,mpos);	
				}
				#endif
				
				//Step->position -= zero_work_area;				
				Step->state_par = LEAVE_ZERO;
				//#endif
				//Step->alarm_step = 1;
				Step->pos_2 = pos;
				//Message_Send_4halfword_debug(0x0909,stepno,0xDDDD,0xffff);
			}

			if (Step->alert_delay_max)
			{
				Step->state_par =JUST_RUN;
			}
			//#ifdef LX_ACTION_STEP_SPECIAL
		}
		//#endif

	}

	if((is_Feet_Step(stepno))&&(Step->state_par==MOVETO_ZERO)&&(zero))
	{
		Step->position =Step->pos_2;
		return;
	}

#if 0
		// by xhl 2011/01/01
	if(Step->moto_remap_config.moto_attr.bit.is_fast_mode) 
	{
		if(mpos > pos) {
			Step->steps = mpos - pos;
		}
		else {
			Step->steps = pos - mpos;
		}
		//myprintf("pos[%d]->[%d]\r\n", Step->position, pos);
		//myprintf("steps = %d\r\n", Step->steps);
		if(Step->steps > 400) {
			goto speed_set;
		}
		//fast_mode = 0;				//这里有疑问
	}

	#endif

	
speed_set:

#ifdef LX_DM_MOVETOZERO_FAST_SPEED

	if (is_LX_DENSITY_Step(stepno))
	Step->max_speed = 6000;
	else
#endif	
	//workspeed >
	Step->max_speed = (workspeed==0)? stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1]:(workspeed*100);

	


#ifdef STEP_MOTOR_DDDM_SUPPORT
	
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//2000;
	Step->speed = Step->low_speed;

#else
	Step->speed =step_base_speed;
#endif


//#ifdef LX_ACTION_STEP_SPECIAL
	if(is_LX_ACTION_Step(stepno))
	{
		//unsigned int czw=Step->st_no_cl.bit.check_zero_work;
		Step->chk.all=0;
		//Step->st_no_cl.bit.check_zero_work = czw & 0x0F;
		Step->chk.bit.last_zero_bit = zero?0xff:0x00;
		
		//Step->adj_lost_steps =0;
		//Step->need_check_zero_sts_change= 0;
		Step->chk.bit.last_zero_st = zero?1:0;
		//Step->last_zero_act = zero;
		//Step->need_leave_zero=0;
		if (((mpos>=50) &&(pos<0))||((mpos<=0)&&pos>50))
		{
			Step->chk.bit.chk_gotozero=1;
			Step->chk.bit.chk_st_change=1;
			Step->chk.bit.chk_leavezero=1;
		}
		if ((mpos>=-50)&&(pos<-50))
		{
			Step->chk.bit.chk_st_change=1;
			Step->chk.bit.chk_leavezero=1;
		}
	}
	else
	if (is_FH_ACTION_Step(stepno))
	{
		Step->chk.all=0;
		//Step->st_no_cl.bit.check_zero_work = czw & 0x0F;
		Step->chk.bit.last_zero_bit = zero?0xff:0x00;
		Step->chk.bit.last_work_bit = w1;//zero?0xff:0x00;
		//Step->adj_lost_steps =0;
		//Step->need_check_zero_sts_change= 0;
		Step->chk.bit.last_zero_st = zero?1:0;
		//Step->last_zero_act = zero;
		//Step->need_leave_zero=0;
		if ((mpos>0) &&(pos<=0))
		{
			Step->chk.bit.chk_gotozero=1;
			Step->chk.bit.chk_st_change=1;
			//Step->chk.bit.chk_leavezero=0;
		}
		if ((mpos<=0)&&(pos>0))
		{
			Step->chk.bit.chk_st_change=1;
			Step->chk.bit.chk_leavezero=1;
		}
	}
	else
		if (is_HF_LIFT_Step(stepno)
			||is_EX_OTHER_Step(stepno)
			||is_DOT_ACTION_Step(stepno)
			||is_LX_yarn_step(stepno))
		{
			int zwat;
			Step->chk.all=0;
		//Step->st_no_cl.bit.check_zero_work = czw & 0x0F;
			Step->chk.bit.last_zero_bit = zero?0xff:0x00;
		
		//Step->adj_lost_steps =0;
		//Step->need_check_zero_sts_change= 0;
			Step->chk.bit.last_zero_st = zero?1:0;
			
			zwat =zero_work_area;
			if(is_HF_ex_LIFT_Step(stepno))
			{
				zero_work_area=60;	
			}
			if (((mpos>zero_work_area) && (pos<=0))||
				((mpos<(0-zero_work_area)) && (pos>=0))
				||(((mpos<=zero_work_area)&&(mpos>=(0-zero_work_area)))&&(zero)&&((pos>zero_work_area)||(pos<(0-zero_work_area)))))
			{
				//if (Step->step_st.bit.is_poweron)
				
				Step->chk.bit.chk_st_change=1;
				if((!Step->st_no_cl.bit.YarnStep_can_check)
					&&(is_LX_yarn_step(stepno)))
				{
					Step->chk.bit.chk_st_change=0;
				}
			}
			if((!Step->st_no_cl.bit.YarnStep_can_check)
					&&(is_LX_yarn_step(stepno)))
			{
				Step->st_no_cl.bit.YarnStep_can_check =1;/*下一次就要检查了*/
				
			}

			if((pos>0)&&(is_LX_yarn_step(stepno)))
			{
				Step->chk.bit.chk_st_change =0;
			}
			
			if(is_HF_ex_LIFT_Step(stepno))
			{
				zero_work_area=zwat;	
			}
			
		}
		if (is_lx_sinker(stepno) ||is_lxex_sinker(stepno))
		{
			Step->chk.all=0;		
			Step->chk.bit.last_zero_bit = zero?0xff:0x00;
			Step->chk.bit.last_zero_st = zero?1:0;
			if (!zero)
			{
				Step->chk.bit.chk_gotozero =1;
			}
			if ((mpos>0) &&(pos<(0-zero_work_area)))
			{
				//Step->chk.bit.chk_gotozero=1;
				Step->chk.bit.chk_st_change=1;
				//Step->chk.bit.chk_leavezero=0;
			}
			if ((mpos<=0)&&(pos>zero_work_area))
			{
				Step->chk.bit.chk_st_change=1;
				//Step->chk.bit.chk_leavezero=1;
			}

			//Message_Send_4halfword(0x9999,stepno,Step->chk.bit.chk_st_change,0);
		}

	if(is_LX_yarn_step(stepno))
	{
		if((pos<0) &&(zero) &&(Step->position>0))
		{
			Step->yarn_motor_elapse_steps=1;
		}

		//if(stepno ==9)
		//{
		Yarn_motor_input_bits_init(stepno);
		//}
	}
	
//#endif



	// Half On
	#ifndef STEP_MOTOR_RUN_HALF_MODE
	#ifdef STEP_CUR_HALF_RUN
	arch_StepMotor_Half(stepno, 1);
#else
	arch_StepMotor_Half(stepno, 0);
#endif
	#endif

	if(mpos > pos) {
		//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		Step->steps = mpos - pos;
		Step->step_st.bit.dir = 0;
		if(Step->step_wait_time<=0)
		{
			arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		}
	}
	else {
		
		Step->steps = pos - mpos;
		Step->step_st.bit.dir = 1;
		if(Step->step_wait_time<=0)
		{
			arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		}
	}
	

//	Message_Send_4halfword(0xAABB,Step->moto_remap_config.moto_remap_id_self,Step->step_st.bit.dir,Step->state_par);
	#if 0
	if(is_LX_yarn_step(stepno))
	{
		Step->steps*=3;
	}
	else
	#endif	
	if (is_PT_sinker(stepno))/*注意这里普通生克统一加800步*/
	{
		if ((mpos>0)&&(pos<0)&&(!zero))
		{
			Step->steps +=800;
		}
		if ((mpos<0)&&(pos>0)&&(zero))
		{
			Step->steps +=800;
		}
		
	}else
	if (is_LX_ACTION_Step(stepno))
	{
		#if 1
		if (workspeed ==0)
		{
			if(Step->steps<150)
			{
				Step->max_speed = ACT_SLOW1_SPEED_LX;
			}
			else
			if (Step->steps<=250)
			{
				Step->max_speed = ACT_SLOW2_SPEED_LX;
			}
			else
			if(Step->steps<=350)
			{
				Step->max_speed = ACT_SLOW3_SPEED_LX;
			}
			else
				if(Step->steps<=450)
				{
					Step->max_speed =ACT_SLOW4_SPEED_LX;	
				}
				else
				if(Step->steps<=550)
				{
					Step->max_speed =ACT_SLOW5_SPEED_LX;	
				}
		}	
		#endif
		if (Step->state_par == GOTOZERO_LX_ACT)
		{
			Step->steps +=400;
		}
	}
	
#if 1
	if((Step->moto_remap_config.moto_attr.bit.is_fast_mode) && (Step->steps > 400)) {
		if(Step->steps > 800) {
			Step->steps %= 800;
		}
		else {
			Step->steps = 800 - Step->steps;
			Step->step_st.bit.dir = !Step->step_st.bit.dir;
		}
		//Step->state_par = JUST_RUN;
		if(Step->step_st.bit.dir) {
			Step->position = pos - Step->steps;

			if(Step->step_wait_time<=0)
				arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		}
		else {
			Step->position = pos + Step->steps;
			if(Step->step_wait_time<=0)
				arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		}
		if (is_YARN_Step(stepno)&&(pos==0))
		{
			Step->state_par = MOVETO_ZERO;
			#ifdef DEBUG_ALERT_65
			Step->which_gotozero =15;
			#endif
		}

		
	}
#endif
//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);

	if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]) << 1)) {
		Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	}
	else {
		Step->acc_steps = Step->steps >> 1;
	}
#ifdef STEP_MOTOR_DDDM_SUPPORT
	Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed) / Step->acc_steps) : 0;
	Step->speed_acc_dec =Step->speed_acc_dec>200?200:Step->speed_acc_dec;   /*降低步幅，减少失步可能*/
#endif

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	//myprintf("[%d] %d\r\n", stepno, Step->steps);

	

#if 1
	if((is_HF_ex_LIFT_Step(stepno))&&(Step->state_par == JUST_RUN))
	{
		Step->HF_ex_cnt =1; /*需要自动修正*/
	}

#endif
	if((Step->moto_remap_config.moto_attr.bit.moto_type_config ==MOTOR_TYPE_FEET )
		&&((Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_PT)
		||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX)))
	{
		if(mode & 0x400) {
			Step->state_par = FEET_STEP_ISWORK;
			//Step->pos_2 = 0;
			//Step->pos_2_main = 0;

			Step->steps_go_temp = 0;
		}
	}

#ifdef ENCODER_SUPPORT
	Encoder_setCheck(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, 1);
#endif

	//Step_Zero_Check_Delay[stepno] = 2/*4*/;
#ifdef STEP_TEST_IN_NDL
	Step->needle = Needle_pos_Get();
#endif

	Step->step_wait_time = 0;
	if (Step->step_st.bit.last_dir!=Step->step_st.bit.dir)
	{
		int rettime=0;
		rettime = Check_step_stop_timeout(Step->last_stop_systick);
		if (rettime)
		{
			Step->step_wait_time = rettime;
		}
		Step->dir_is_change_ =2;
	}
	

	if(encoder_en > 0)
	{
		Step->step_wait_time=5;
		Step->dir_is_change_ =0;
		arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);

	}
	else
	{
		if(Step->step_wait_time==0)
		{
			Step->dir_is_change_ =0;
			arch_StepMotor_Dir(stepno, Step->step_st.bit.dir?Step->step_st.bit.dir_High:!Step->step_st.bit.dir_High);

		}
	}

#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = work;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}
#endif
	if(is_CX3_Feet_Step(stepno))
	{
		Step->step_st.bit.zero2 = work;
		if(Step->steps > step_setup_zero2_detect_steps)
			Step->step_st.bit.zero2_count = 0;
	}

	#if 0
	if((is_Feet_Step(stepno))&&(Step->state_par==MOVETO_ZERO))
	      	Message_Send_4halfword(0x99|(stepno<<8),Step->step_st.bit.dir|((zero?1:0)<<8), Step->state_par,Step->position);
	#endif			
	//Message_Send_4halfword(0x99|(stepno<<8),Step->steps,Step->state_par,Step->position);
	arch_StepMotor_Start(stepno);
//Message_Send_4halfword(0x9999,stepno,Step->moto_remap_config.moto_attr.all,Step->moto_remap_config.moto_attr.all>>16);


}

void StepMotor_Reset(unsigned int stepno,unsigned short otherarg)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int rst_speed;
	unsigned int iszero;
	unsigned int isleft;
	unsigned int isright;
	unsigned int iswork;
	int ret, pos;
	int encoder_en;
	
	
	if(stepno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config > MOTOR_TYPE_COUNT) return;

	Step->moto_dd_config.DD_type =0;
	//arch_StepMotor_Stop(stepno);
	encoder_en = 0;

#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if (((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)&&(enable_log_tosend_LX))	
	{
		unsigned short d[3];
		d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
		d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
		d[0] |= LX_LOG_TYPE_MOVETOZERO_ADD400_2<<8;
		d[0] |= 0<<12;
		d[1] =Step->position;
		d[2] =0;								
		Message_send_log_LX(d[0],d[1],d[2]);
	}
#endif
	//memset(step_postion_ecode,0,sizeof(step_postion_ecode));

#ifdef ENCODER_SUPPORT
	if (1)//判断是否为传感器类型
	{
		encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
		if (encoder_en)
		{
		
			ret = Encoder_Reset(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, &pos);/* 获取当前位置 */
			if (ret < 0)
			{
				return;
			}
		}
	}
#endif /* ENCODER_SUPPORT */

	if (otherarg)
	{
		Step->st_no_cl.bit.report_cnt++;
	}

	if(is_LX_DENSITY_Step(stepno))
	{
		//if(Step->isdoing_with_bind_cmd)
		{			
			Message_Send_4halfword((Step->isdoing_with_bind_cmd?0xf7:0xf8)|(stepno<<8),Step->position,Step->step_st.all &0xffff,Step->step_st.all>>16);
		}
	}

	if ((Step->step_st.bit.IS_Reset_ex)&&(Step->step_st.bit.running))
	{	
		return;
	}



	Step->step_st.bit.is_poweron = 1;		//变成1 说明电机动过了.

	if (Step->st_no_cl.bit.check_is_first)
		Step->st_no_cl.bit.check_is_first =0;

	if(!Step->st_no_cl.bit.YarnStep_can_check)
		Step->st_no_cl.bit.YarnStep_can_check=1;
	Step->step_st.bit.IS_Reset_ex = 1;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	#ifdef STEP_DEBUG_HZ
	Step->speed_HZ_idx = 0;
	#endif
	Step->speed = 0;
	Step->pos_2 = 0;
	Step->pos_2_main=0; 
	
	Step->alarm_step = 0;
	Step->steps = Step->step_max;
	Step->state_par = STEP_RESET;
	Step->st_no_cl.bit.checkisleavezero =0;
	Step->change_dir_count =0;
	Step->chk.bit.check_sts_enable = 0;
	Step->chk.bit.check_Right_Now =0;
	Step->moto_zero_width_temp=0;
	Step->chk.bit.HP_check_isover =0;
	Step->chk.bit.HP_check_isok =0;
	Step->chk.bit.HP_auto_adj =0;
	Step->steps_go_ = 0;
	Step->chk.bit.SK_ADJ_ok =0;//清除修正标记
	Step->chk.bit.Zero_Out_ADJ_OK =0;
	Step->HF_ex_cnt =0;
	Step->change_speed =0;
	Step->chk.bit.chk_st_change= 0;
	Step->yarn_motor_elapse_steps=0;

	//Step->step_alert_delay_pos2_st.bit.last_needcheck_zero = 0;
	//Step->step_alert_delay_pos2_st.bit.new_needcheck_zero = 0;

	#ifdef LX_ACT_DONOT_ADJ
	if (is_LX_ACTION_Step(stepno))
	{
		LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]=0;
	}
	#endif
	
	#ifdef DEBUG_STEP_RESET_FAST_SPEED

	Step->reset_dir_change_cnt=0;	
	#endif
	//#ifdef LX_ACTION_STEP_SPECIAL

	if(is_ACTION_Step(stepno))
	{
		if (Step->st_no_cl.bit.check_zero_work_tmp)
		{
			Step->st_no_cl.bit.check_zero_work = Step->st_no_cl.bit.check_zero_work_tmp & 0x0F;	
		}
		else
		{
			Step->st_no_cl.bit.check_zero_work = 0x05;	
		}
		Step->st_no_cl.bit.check_zero_work_tmp =0 ;
	}
	
	if (is_LX_ACTION_Step(stepno)
		||is_HF_LIFT_Step(stepno)
		||is_EX_OTHER_Step(stepno)
		||is_DOT_ACTION_Step(stepno)
		||(is_FH_ACTION_Step(stepno))
		||is_LX_yarn_step(stepno))
	{
		Step->moto_zero_width_temp =0;
		Step->chk.bit.chk_st_change= 0;
		#ifdef ZERO2_SUPPORT
		if (Step->step_st.bit.zero2_mode)
		{
			if (is_FH_ACTION_Step(stepno))
			Step->st_no_cl.bit.check_zero_work =0x0d;
			else					
			Step->st_no_cl.bit.check_zero_work = 0x0F;
		}
		else
		#endif
		Step->st_no_cl.bit.check_zero_work = 0x05;

		#ifdef IS_CX_ACT
		if(is_DOT_ACTION_Step(stepno))
		{
			Step->st_no_cl.bit.check_zero_work =0x05;
			if(Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST)
			{
				Step->st_no_cl.bit.check_zero_work|=0x0A;
			}
		}
		#endif

		if(is_LX_yarn_step(stepno))
		{
			if(Step->chk.bit.check_Right_Now)
				Step->chk.bit.check_Right_Now =0;
		}
	}
	if(is_CX2_Feet_Step(stepno)
		||is_CX3_Feet_Step(stepno))
	{
		Step->moto_zero_width_temp =0;
		Step->chk.bit.chk_st_change= 0;
		if (Step->st_no_cl.bit.check_zero_work_tmp)
		{
			Step->st_no_cl.bit.check_zero_work = Step->st_no_cl.bit.check_zero_work_tmp & 0x0F;	
			Step->st_no_cl.bit.check_zero_work_tmp =0;
		}
		else
			Step->st_no_cl.bit.check_zero_work = 0x05;
		
	}
	
	//#endif

	Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];  // ACC_STEPS - 1;
	rst_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
	#ifdef STEP_MOTOR_DDDM_SUPPORT
	if (rst_speed>stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1])
	#else
	if (rst_speed<stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1])
	#endif	
		rst_speed =stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];	
	Step->max_speed = rst_speed;

	//arch_StepMotor_Half(stepno, 0);//全流
	#ifdef STEP_CUR_HALF_RUN
	arch_StepMotor_Half(stepno, 1);
#else
	arch_StepMotor_Half(stepno, 0);
#endif
	
	if (encoder_en == 0)
	{
		//iszero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);
		iszero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
	//
		Step->chk.bit.last_zero_bit = iszero?0xff:0x00;

#ifdef ZERO2_SUPPORT
		iswork = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);		
	
		Step->chk.bit.last_work_bit = iswork?0xff:0x00;
#endif /* ZERO2_SUPPORT */

	if(is_HP_ACTION_Step(stepno)) 
	{		
		Step_input_sts_first(stepno);	
		isleft = Triangle_is_left_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		isright =Triangle_is_right_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		iszero =Triangle_is_zero_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		Step->steps_go_temp=0;	
		Step->state_par = STEP_RESET_HP_ACT;	
		
		
	}
	else
		Step->state_par = STEP_RESET;

	//Message_Send_4halfword((stepno << 8) | 0xEE,4,iszero,Step->state_par);


	Step->step_st.bit.zero = iszero;

		if(iszero) 
		{		
			Step->step_st.bit.dir = 1; 
			Step->position = 0;	
			//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
			if(is_YARN_Step(stepno))
			{
				Step->state_chi =5;
			}
			else
				Step->state_chi =2;

			#if 0
			if (is_LX_ACTION_Step(stepno))
			{
				Step->step_st.bit.dir = 0; 
				Step->steps = 300;
				Step->state_chi =8;	
			}
			#endif
			
			//Message_Send_4halfword((stepno << 8) | 0xEE,0,Step->position,Step->step_st.bit.dir);
		}
		else 
		{
			if(is_HP_ACTION_Step(stepno)) 
			{
				if (isright) 
				{
					//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
					#ifdef QL_
					Step->step_st.bit.dir = (Step->position<0) ? 1:0;
					#else
					Step->step_st.bit.dir = 0;
					#endif

				//	Message_Send_4halfword((stepno << 8) | 0xEE,1,Step->position,Step->step_st.bit.dir);
					
				}else
				if (isleft)
				{
					//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
					#ifdef QL_
					Step->step_st.bit.dir = (Step->position<0) ? 1:0;
					#else
					
					Step->step_st.bit.dir = 1;
					#endif
				//	Message_Send_4halfword((stepno << 8) | 0xEE,2,Step->position,Step->step_st.bit.dir);
				}
				else				
				{
					//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);

					#ifdef QL_

					Step->step_st.bit.dir = (Step->position<0) ? 1:0;
					#else
					
					Step->step_st.bit.dir = 0;
					#endif	
					//Message_Send_4halfword((stepno << 8) | 0xEE,3,Step->position,Step->step_st.bit.dir);
				}
			
				//Step->step_st.bit.dir = 1;			
			}
			else
			{		
				Step->step_st.bit.dir =0; 
				if (is_HF_LIFT_Step(stepno)
					||is_EX_OTHER_Step(stepno)
					||is_DOT_ACTION_Step(stepno)
					||is_lx_sinker(stepno)
					||is_CX3_Feet_Step(stepno)
					||is_LX_yarn_step(stepno))
				{
					Step->step_st.bit.dir= (Step->position>=0)?0:1;

					#ifdef IS_CX_ACT
					if(is_DOT_ACTION_Step(stepno))
					{
						Step->step_st.bit.dir=((Step->position>=0)&&(Step->position<=400))?0:1;
					}
					#endif

					if(is_CX3_Feet_Step(stepno))
					{
						Step->step_st.bit.dir=((Step->position>=0)&&(Step->position<=400))?0:1;
						Step->steps = 800;
						goto do_TRY_gogo;
					}

					if (is_lx_sinker(stepno))
					{						
						Step->steps = 800;
					}
					else
						if(is_ACTION_Step(stepno))
						{
							Step->steps = 400;//350;	
						}
						else
						Step->steps = 250;	

					#ifdef TRY_ZERO_MIDD

					Step->steps = try_steps_to_zero;//GOTO_TRY_ZERO_STEPS;
					Step->steps_last = Step->steps;
					Step->state_chi = GOTO_ZERO_TRY_ZERO_MIDD;
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];
					goto do_TRY_gogo;

					#endif
				}else
				if (is_LIFT_or_Other_Step(stepno)||is_sinker(stepno))
				{		
					//int work_dir=Step->step_st.bit.dir_High;
					//int zero_dir = Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir;
					
					//if (work_dir==zero_dir?(Step->position>0):(Step->position<=0))	//					
					{					
						Step->step_st.bit.dir = 1;				//往一边走
						Step->steps = 20;				//走50步
						Step->position = 20;								
						Step->state_chi = GOTO_ZERO_TRY;
						goto do_TRY_gogo;
					}	
					
				}
				if (is_DENSITY_Step(stepno))
				{
					if (Step->moto_remap_config.moto_attr.bit.moto_type_exe == DENSITY_MOTOR_TYPE_LX)
					{
						Step->step_st.bit.dir =(Step->position==100)?1:((Step->position>=0)?0:1);
						Step->steps = Step->step_st.bit.dir?(800):(Step->step_max);						
					}
					else
					{
						Step->step_st.bit.dir = 1;				//往一边走
						Step->steps = 50;				//走50步
						Step->position = 50;								
					}
					Step->state_chi = GOTO_ZERO_TRY;
					goto do_TRY_gogo;
				}
			}
			//Step->step_st.bit.dir = 0;			
			Step->position =Step->step_max;
			Step->state_chi =0;
			if (is_zero2_support(stepno) && is_PTorFH_ACTION_Step(stepno))
			{
			//Step->max_speed = step_reset_speed;
				#ifdef DEBUG_ALERT_65
					//Step->which_gotozero =0xff; 
      				#endif
					//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,1,1);
						
				Step->state_par = GOTO_ZERO;
			}
		}

		#if 0
		if(is_LX_ACTION_Step(stepno))
		{
			Message_Send_4halfword((stepno << 8) | 0xEE,Step->position,iszero,Step->state_par|(Step->state_chi<<8));

		}
		#endif
		
	}
	else
	{
		// 在偏差范围比较小的情况下 都往外走 STEPS_LEAVEZERO
		if (pos <= STEPS_ERROR_RANGE)
		{
			Step->step_st.bit.dir = 1;
			Step->state_chi = 0;
			Step->position = pos;
			Step->steps = STEPS_LEAVEZERO - pos;
			Step->state_par = ENCODER_LEAVE_ZERO;
		}
		else
		{
			Step->position = pos;
			if (pos > 0)
			{
				Step->steps = pos;
				Step->step_st.bit.dir = 0;	/**/
			}
			else
			{
				Step->steps = 0 - pos;
				Step->step_st.bit.dir = 1;
			}
			Step->state_chi = 0;
			Step->state_par = ENCODER_JUST_RUN;//JUST_RUN;//
		}
	}

do_TRY_gogo:	

//if(is_Feet_Step(stepno))
//	Message_Send_4halfword((stepno << 8) | 0xEE,Step->position,iszero,Step->state_par);

	if ((is_LIFT_or_Other_Step(stepno)||is_sinker(stepno))&&(!encoder_en))
	{
		char zero_dir=Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir;
		arch_StepMotor_Dir(stepno, Step->step_st.bit.dir? !zero_dir:zero_dir);
	}
	else
	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);
	
#ifdef STEP_MOTOR_DDDM_SUPPORT
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];  //启动频率
	Step->speed_acc_dec =(Step->max_speed>Step->low_speed)?((Step->max_speed -Step->low_speed)  /Step->acc_steps):0 ;	
	Step->speed_acc_dec =Step->speed_acc_dec>200?200:Step->speed_acc_dec;
	Step->speed = (Step->max_speed>Step->low_speed) ? Step->low_speed :Step->max_speed;												//当前速度
#endif

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	

	Step->step_wait_time = 0;
	//Step->done_steps = 0;				//已经执行的步数 针对 三角电机
	//Step->last_rest= 0;	
	Step->change_dir_count = 0;
	//Step->dir_steps =0;
	Step->DD_error_cnt =0;/*归零的时候清除计数值*/
	

	arch_StepMotor_Start(stepno);
	
	if (encoder_en)
		return;
	
#ifdef ZERO2_SUPPORT
if (is_HP_ACTION_Step(stepno))
{
	Step->chk.bit.HP_check_st = 0x03;
	//Triangle_sts[stepno]=0x03;
}
else
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = iswork;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}

	if(is_CX3_Feet_Step(stepno))
	{
		Step->step_st.bit.zero2 = iswork;
		if(Step->steps > step_setup_zero2_detect_steps)
			Step->step_st.bit.zero2_count = 0;
	}
#endif
}


void StepMotor_setMaxSpeed(unsigned int stepno,unsigned int maxspeed)
{ 
	if(stepno >= StepMotor_Count_MAX) return;
	if (maxspeed>7000)
	{
		maxspeed =7000;
	}
	else
		if (maxspeed<500)
		{
			maxspeed =500;	
		}
		
	STEPMOTOR[stepno].max_speed = maxspeed;
	STEPMOTOR[stepno].low_speed	= maxspeed;
	
	return ;

}



#ifdef ZERO2_SUPPORT
int is_zero2_support(unsigned int stepno)
{
	return STEPMOTOR[stepno].step_st.bit.zero2_mode;
}
void StepMotor_Detect_Zero2(unsigned int stepno) /*该函数用于开机修正动作电机的位置*/
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	//int i;
	int steps;
	//int zero;

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

	if (Step->step_st.bit.zero2_mode==0) return;

	
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;

	if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_ACTION) return;
	if(Step->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_DOT) return;

	//zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);

#if 0
	if(Step->position <= sinker_zero_area &&
	   !zero) {
		goto exec_it;
	}
#endif
	//if(arch_StepMotor_Zero(stepno)) {
	//	StepMotor_Reset(stepno);
	//	return ;
	//}

//exec_it:
	arch_StepMotor_Stop(stepno);

	// by xhl 2010/08/26
	Step->step_st.bit.is_poweron =1;

	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	//Step->speed = 0;
	//Step->pos_2 = 0;
	//Step->pos_2_main = 0;
	// by xhl 2010/05/18
	Step->alarm_step = 0;
	// by xhl 2012/03/08
	//Step->speed_div = 0;

	steps = step_setup_zero2_detect_steps_ex;
	Step->steps = steps;
	Step->acc_steps = (steps + 1) >> 1;
	Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//2000;
	Step->speed = Step->low_speed;

	Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed) / Step->acc_steps) : 0;
	Step->speed_acc_dec =Step->speed_acc_dec>200?200:Step->speed_acc_dec;   /*降低步幅，减少失步可能*/



	#ifndef STEP_MOTOR_RUN_HALF_MODE
	#ifdef STEP_CUR_HALF_RUN
	arch_StepMotor_Half(stepno, 1);
#else
	arch_StepMotor_Half(stepno, 0);
#endif
	#endif



	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
	Step->step_st.bit.dir = 1;
	//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
	//Step->step_st.bit.dir = 0;
	
	Step->state_par = DETECT_ZERO2;

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 200/*50*/;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	//myprintf("start dir %d, step %d\r\n", Step->step_st.bit.dir, Step->steps);
	arch_StepMotor_Start(stepno);
	//Step_Zero_Check_Delay[stepno] = 2/*4*/;
#ifdef ENCODER_SUPPORT
	Encoder_deInit(stepno);
#endif
	Step->step_wait_time = 0;
	Step->step_st.bit.zero2 = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
	Step->step_st.bit.zero2_count = 0;
}

#else
int is_zero2_support(unsigned int stepno)
{
	return 0;
}
#endif

void StepMotor_All_Reset(unsigned short otherarg)
{
	unsigned int i;
	STEP_TYPE *Step;
	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!= MOTOR_TYPE_DENSITY)
			continue;			
		StepMotor_Reset(i,((otherarg>>4)&0x0001));
	}
}

void StepMotor_Reset_LeftMotor(unsigned short otherarg)
{
	unsigned int i;
	STEP_TYPE *Step;

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		 Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!= MOTOR_TYPE_DENSITY)
			continue;
		if (Step->moto_remap_config.moto_remap_id_self & 0x01)
			 continue;
		StepMotor_Reset(i,((otherarg>>4)&0x0001));
	}
}

void StepMotor_Reset_RightMotor(unsigned short otherarg)
{
	unsigned int i;
	STEP_TYPE *Step;

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		 Step = &STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!= MOTOR_TYPE_DENSITY)
			continue;
		if (Step->moto_remap_config.moto_remap_id_self & 0x01)				 
			StepMotor_Reset(i,((otherarg>>4)&0x0001));
	}
}
#if 0
int is_triangle(int stepno)
{

	if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION)
		return 1;	
	else
		return 0;
}

#endif

void check_HF_Lift_Work_with_input(STEP_TYPE *Step)
{
	//int i;
	if (Step==0)
		return ;
	else
	{
		//Step= &STEPMOTOR[i];

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_LIFT) 
			return ;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			return ;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			return ;	
		//#ifdef LX_ACTION_STEP_SPECIAL
		if (Step->moto_remap_config.moto_attr.bit.moto_type_exe!=LIFT_MOTOR_TYPE_PT)
		{
				if ((Step->steps == 0) 			
					&& (Step->chk.bit.check_sts_enable)
					&&(Step->step_st.bit.check_delay_count==0)
					&&(Step->step_st.bit.running ==0))
			{		

					char err=0;
					err = check_HF_Lift_input_error(Step);
									
				//if (checkzero||checkwork)					
				if (err){				
						Step->error_count_check++;
						if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
						{							
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)err|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
							Step->error_count_check = 0;
							Step->chk.bit.check_sts_enable = 0;
						}
				}
				else
					{
						Step->error_count_check = 0;
					}			
				
			}

		}
}

}
void check_ACT_Work_with_POS(STEP_TYPE *Step)
{
	//int i;
	//STEP_TYPE *Step;
	//int step_no;
	unsigned int work_t,zero_t;

	//for (i=0; i < StepMotor_Count_MAX; i++)
	if (Step==0)
		return ;
	else
	{
		//Step= &STEPMOTOR[i];
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			return ;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			return ;	
		if ((Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_ACTION) 
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_FEET)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_YARN)
			)
			return ;
		if(is_ACTION_Step_CheckRunTime(Step)
			||is_FEET_Step_CheckRunTime(Step)
			||is_LX_yarn_step_exe(Step))
		{
			char czw=Step->st_no_cl.bit.check_zero_work;
	//#if 0
			if ((((Step->steps == 0) 			
					&& (Step->chk.bit.check_sts_enable)
					&&(Step->step_st.bit.check_delay_count==0)
					&&(Step->step_st.bit.running ==0))||(Step->chk.bit.check_Right_Now))
					&&(czw>>2))
			{		

					char checkzero=0;
					char checkwork =0 ;
					char isyarn = Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_YARN?1:0;
					char isfeet_temp=Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_FEET?1:0;
					work_t = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
					zero_t = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);

					if (((czw>>2) & 0x01) && (zero_t!=(czw & 0x01)))
						checkzero =1;

					if (((czw>>3) & 0x01) && (work_t!=((czw >>1) & 0x01)))
						checkwork =1;
									
				if (checkzero||checkwork)					
				{				
					Step->error_count_check++;
					if(Step->error_count_check> 10)//DEF_TRIANGLE_ERR_CHECK_TIME)
					{
						unsigned short checkerror=0;
						if (checkzero)						
							checkerror=isfeet_temp?0x0001:0x9003;		
						if (checkwork)
							#if 0
								{
							checkerror=work_t?0xC003:0xC002;
							checkerror |=(czw<<4);
							checkerror |=(1<<8);							
						}
							#else
							checkerror=isfeet_temp?0x0002:0xC003;
							#endif
						
						if (checkzero && checkwork)
							checkerror=isfeet_temp?0x0003:0xD003;

						if(isyarn)
						{
							checkerror= checkzero ?LX_YARN_ZEROINPUT_CHECK_ERR:LX_YARN_WORKINPUT_CHECK_ERR;
							alert_push(checkerror, Step->moto_remap_config.moto_remap_id_self);
							
						}
						else
						{
							#ifdef NEW_ALARM_STYLE
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|(0x0001));
							#else
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)checkerror|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
							#endif
						}
						Step->error_count_check = 0;
						//if (act_check_enable)
						//	act_check_enable =0;
						Step->chk.bit.check_sts_enable = 0;
						if(Step->chk.bit.check_Right_Now)
							Step->chk.bit.check_Right_Now =0;
					}
				}
				else
					{
						Step->error_count_check = 0;
					}			
				
			}

		}
		else
		{
		
			if ((Step->steps == 0) 
				#ifndef ACT_ALARM_ALWAYS
					//&& (Step->chk.bit.check_sts_enable)
				#endif	
					&&(Step->step_st.bit.running ==0)
						&&(Step->position == Step->check_work_pos))
			{

					#if 1
						{
							if (!arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg)){
								Step->error_count_check++;
								if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
								{
									#ifdef NEW_ALARM_STYLE
									alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|(0x0001));
									#else
									alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)0x9222|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
									#endif
									Step->error_count_check = 0;
									Step->chk.bit.check_sts_enable = 0;
								}
							}
							else
								{
									Step->error_count_check = 0;
								}
						}
							
					#else
							{
							if (!Triangle_is_right_Sign(i)){
								Step->error_count_check++;
								if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
								{
									#ifdef NEW_ALARM_STYLE
									alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|(0x0002));
									#else
									alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 2);
									#endif
									Step->error_count_check = 0;
									Step->chk.bit.check_sts_enable = 0;
								}
							}
							else
								{
									Step->error_count_check = 0;
								}
							}
					#endif
									
			}
		}	
			
	}

}

void Step_Set_work_check_pos(unsigned char mid,short pos,unsigned char step_type)
{	
	unsigned int stno;
	stno = StepMotor_get_no_with_IDself(mid, step_type);
	if (step_type == MOTOR_TYPE_ACTION)
	{
		STEP_TYPE *Step;	 
		if (stno< StepMotor_Count_MAX)
		{
			Step = &STEPMOTOR[stno];
			Step->check_work_pos = pos;
			if (!act_check_enable)
				act_check_enable =1;
				
		}
	}

}

#if 0
void check_triangle_sts(void)
{
	int i;
	STEP_TYPE *Step;
	//int step_no;

	for (i=0; i < StepMotor_Count_MAX; i++)

	{
		Step= &STEPMOTOR[i];

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_ACTION) 
			continue;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;	
		
		if ((Step->steps == 0) 
				&&(Step->step_st.bit.running ==0)
					&&(Step->chk.bit.check_sts_enable))
		{

			switch (Step->chk.bit.HP_check_st/*Triangle_sts[i] & (0x03)*/){
				case 01:
						{
						if (!Triangle_is_left_Sign(i)){
							Step->error_count_check++;
							if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
							{
								#ifdef NEW_ALARM_STYLE
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|(0x0001));
								#else
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 1);
								#endif
								Step->error_count_check = 0;
								Step->chk.bit.check_sts_enable = 0;
							}
						}
						else
							{
								Step->error_count_check = 0;
							}
						}
						break;
				case 02:
						{
						if (!Triangle_is_right_Sign(i)){
							Step->error_count_check++;
							if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
							{
								#ifdef NEW_ALARM_STYLE
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|(0x0002));
								#else
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 2);
								#endif
								Step->error_count_check = 0;
								Step->chk.bit.check_sts_enable = 0;
							}
						}
						else
							{
								Step->error_count_check = 0;
							}
						}
						break;
				case 03:
						{
						if (!arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index)){
							Step->error_count_check++;
							if(Step->error_count_check> DEF_TRIANGLE_ERR_CHECK_TIME)
							{
								#ifdef NEW_ALARM_STYLE
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
								#else
								alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 3);
								#endif
								Step->error_count_check = 0;
								Step->chk.bit.check_sts_enable = 0;								
							}
						}
						else
							{
							Step->error_count_check = 0;
							}
						}
						break;
				default:
					break;
				}				
		}
	}

}
#endif


void StepMotor_Triangle_Set_Sign(unsigned int sk_no,short pos)
{
		int stepno =0;//sk_no+TRIANGLE_BASE;
		int pos_l;

		stepno =StepMotor_get_no_with_IDself(sk_no, MOTOR_TYPE_ACTION);		

		switch (pos & 0x03){
		case 0x01:
				pos_l = TRIANGLE_LEFT_POSTION;
				break;
		case 0x02:
				pos_l = TRIANGLE_RIGHT_POSTION;
				break;
		case 0x03:
				pos_l = TRIANGLE_ZERO_POSTION;
				break;
		case 0x00:
				pos_l = TRIANGLE_ZERO_POSTION;
				break;
		default:
				return;

		}
		StepMotor_Set_Position(stepno,pos_l);
		return;
}
#if 0
short StepMotor_Triangle_Get_Zerowide(unsigned int ACT_id)
{
		
		int stepno =0;//sk_no+TRIANGLE_BASE;
			
		STEP_TYPE *Step;
		
		
		stepno = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
		if (stepno>=StepMotor_Count_MAX) return 0xff;
		Step = &STEPMOTOR[stepno];
		return Step->moto_zero_width_self;
}





short StepMotor_Triangle_Get_Inputerror(unsigned int ACT_id)
{
		
		int stepno =0;//sk_no+TRIANGLE_BASE;			
		STEP_TYPE *Step;	
		
		stepno = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
		if (stepno>=StepMotor_Count_MAX) return 0xff;
		Step = &STEPMOTOR[stepno];
		return Step->input_errorstep;
}


short StepMotor_Triangle_Set_Zerowide_inputerror(unsigned int ACT_id,short zw,short iner)
{
		
		int stepno =0;//sk_no+TRIANGLE_BASE;			
		STEP_TYPE *Step;		
		stepno = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
		if (stepno>=StepMotor_Count_MAX) return 0xff;
		Step = &STEPMOTOR[stepno];
		Step->moto_zero_width_self=zw;

		if ((iner<=30)&&(iner>=0))			
			Step->input_errorstep=iner;
		return 0;
}

short StepMotor_Lift_Set_Zerowide_inputerror(unsigned int ACT_id,short zw,short iner)
{
		
		int stepno =0;//sk_no+TRIANGLE_BASE;			
		STEP_TYPE *Step;		
		stepno = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_LIFT);
		if (stepno>=StepMotor_Count_MAX) return 0xff;
		Step = &STEPMOTOR[stepno];
		Step->moto_zero_width_self=zw;

		if ((iner<=30)&&(iner>=0))			
			Step->input_errorstep=iner;
		return 0;
}
#endif

void StepMotor_other_Set_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short zw,short iner)
{
		int stepno =0;//sk_no+TRIANGLE_BASE;			
		STEP_TYPE *Step;		
		stepno = StepMotor_get_no_with_IDself(ACT_id,step_type);
		if (stepno>=StepMotor_Count_MAX) return ;
		Step = &STEPMOTOR[stepno];
		Step->moto_zero_width_self=zw;

		if ((iner<=30)&&(iner>=0))			
			Step->input_errorstep=iner;
		return ;
}


void StepMotor_other_Get_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short *zw,short *iner)
{
		int stepno =0;//sk_no+TRIANGLE_BASE;			
		STEP_TYPE *Step;		
		stepno = StepMotor_get_no_with_IDself(ACT_id,step_type);
		if (stepno>=StepMotor_Count_MAX) return ;
		Step = &STEPMOTOR[stepno];
		if (zw!=(void *)0)
		*zw = Step->moto_zero_width_self;
		if (iner!=(void *)0)
		*iner = Step->input_errorstep;
		
		return ;
	
}


short StepMotor_Triangle_Get_Sign(unsigned int ACT_id)
{
		int sign_0 =0;
		int sign_1 =0;
		int stepno =0;//sk_no+TRIANGLE_BASE;
		int pos;

		stepno = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
		sign_0 = arch_StepMotor_Zero_mid_layer(StepMotor_get_zeroID_with_NO(stepno),StepMotor_get_zeroID_cfg_with_NO(stepno));
		sign_1 = arch_StepMotor_Work_mid_layer(StepMotor_get_workID_with_NO(stepno),StepMotor_get_workID_cfg_with_NO(stepno));

			if ((sign_0==sign_1)&& (sign_0==1)){
				pos= 0x03;
			}
			else
			{ 
				if (sign_0!=sign_1) {
					if (sign_0){
						pos = 0x02;
					}
					else
					{
						pos =0x01;
					}
				}
				else{
					pos = 0x00;
				}					
			}
			return pos;
	
}

void StepMotor_Action_Set_Position(unsigned int ACT_id,short pos)
{
	unsigned int st_no;  
	st_no = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
	StepMotor_Set_Position(st_no,pos);
	return ;
}


short StepMotor_Triangle_Get_Position(unsigned int ACT_id)
{
	unsigned int st_no;  
	st_no = StepMotor_get_no_with_IDself(ACT_id,MOTOR_TYPE_ACTION);
	return StepMotor_Get_Position(st_no);
}








#if 0
void StepMotor_Action_Reset(unsigned int act_id,unsigned short otherarg)
{
	unsigned int st_no;  
	st_no = StepMotor_get_no_with_IDself(act_id,MOTOR_TYPE_ACTION);
	 StepMotor_Reset(st_no,((otherarg>>4)&0x0001));
}


void StepMotor_Action_ResetALL(unsigned short otherarg)
{
	int i;
	STEP_TYPE *Step;
	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		Step=&STEPMOTOR[i];
		if(Step->moto_remap_config.moto_attr.bit.moto_type_config !=MOTOR_TYPE_ACTION)
			continue;		
		
		StepMotor_Reset(i,((otherarg>>4)&0x0001));
	}
	

}
#endif


int Get_st_with_postion(short arg)
{
	int ret =0;
	#ifdef QL_
	if (((arg<-50)&&(arg>=-280))||(arg>280))			
	{
		ret =0x02;
	}
	else
		if ((arg>50)||(arg<-280))
		{
			ret =0x01;
		}
		else
		{
			ret =0x03;
		}
	#else
	{
		if (arg<-50){
			ret =0x01;
		}
		else
			if (arg>50)
			{
				ret =0x02;
			}
			else
			{
				ret =0x03;
			}
	}
	#endif
	return ret;
	
	
}


void Exec_Triangle_step(unsigned int Tid,  int Pos,unsigned int alarmenable)
{
	unsigned int stepno;
	//unsigned int localno;
	unsigned int Triangle_sts_loc;
	short arg;
	int alarmenable_local;
	int mode= 0;
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;


	stepno = StepMotor_get_no_with_IDself(Tid, MOTOR_TYPE_ACTION);

	if (stepno >=StepMotor_Count_MAX) return;	

	Step = &STEPMOTOR[stepno];

	//localno = stepno;
	alarmenable_local = alarmenable &0x0003;	
	
	/*Triangle_sts[localno] */Triangle_sts_loc=(alarmenable_local & 0x03);
	mode = Triangle_sts_loc<<2 /*Triangle_sts[localno]<<2*/;
	if ((alarmenable & 0x8000) ==0)
	{		
		alarmenable_local = (Pos & 0x03);
		
		Triangle_sts_loc/*Triangle_sts[localno] */=(alarmenable_local& 0x03);		
		mode = Triangle_sts_loc<<2 /*Triangle_sts[localno]<<2*/;

		
		switch (Pos&0x03) 
		{
		case 0x01:
				arg = TRIANGLE_LEFT_POSTION;
				break;
		case 0x02:
				arg = TRIANGLE_RIGHT_POSTION;
				break;
		case 0x03:
				arg = TRIANGLE_ZERO_POSTION;
				break;
		case 0x00:
				//Triangle_sts[localno] =0x03;		//归零的时候也是需要检测的
				Triangle_sts_loc = 0x03;				
				StepMotor_Reset(stepno,(unsigned short)((alarmenable>>4)&0x0001));
				Step->chk.bit.HP_check_st = Triangle_sts_loc;
				return;
				//break;
		default:
			break;			
		}
		mode |=0x03;

		#ifdef TEST_STEPMOTOR_AUTOREPORT
		StepMotor_exec(stepno, arg, mode,(unsigned short)(0x0001),0);
		#else
		StepMotor_exec(stepno, arg, mode,(unsigned short)((alarmenable>>4)&0x0001),0);
		#endif
		Step->chk.bit.HP_check_st = Triangle_sts_loc;
	}
	else
	{
		arg = (Pos);
		if(is_HP_ACTION_Step(stepno)) 
		{
				alarmenable_local = Get_st_with_postion(arg);
				Triangle_sts_loc /*Triangle_sts[localno]  */=alarmenable_local;
				//Step->chk.bit.HP_check_st = Triangle_sts_loc;	
		}
		else
		{
			if (arg<=20)
			{Triangle_sts_loc /*Triangle_sts[localno]*/  =0x02;}
			else
			if (arg>=20)
			{
			Triangle_sts_loc /*Triangle_sts[localno]  */=0x01;	
			}
			else
				Triangle_sts_loc /*Triangle_sts[localno] */= 0x00;
		}	
	
		
		mode = Triangle_sts_loc <<2;//Triangle_sts[localno]<<2;
		mode |=0x3;
		
		//arch_SendMessage(0, &mode, 1);
		#ifdef TEST_STEPMOTOR_AUTOREPORT
		StepMotor_exec(stepno, arg,mode,(unsigned short)(0x0001),0);
		#else
		StepMotor_exec(stepno, arg,mode,(unsigned short)((alarmenable>>4)&0x0001),0);
		#endif
		Step->chk.bit.HP_check_st = Triangle_sts_loc;
	}
	
	
	
}


//2014-02-17 三角电机复位函数 by hlc

#ifdef DEBUG_TIME3_CC3
void Stepmotor_Check_isrlost(unsigned int stepno)
{
	
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

	if (Step->step_st.bit.running==0) return;

		arch_check_isrlost(stepno); 
	


}

#endif

#if 0
void Exec_Action_step(unsigned int Tid,  short Pos,unsigned short otherarg)
{
	int sk_no;
	sk_no = StepMotor_get_no_with_IDself(Tid,MOTOR_TYPE_ACTION);

	#ifdef TEST_STEPMOTOR_AUTOREPORT
	StepMotor_exec(sk_no, Pos, 1,1);
	#else
	StepMotor_exec(sk_no, Pos, 1,otherarg);

	#endif
}

#endif



//2014-02-17 三角电机复位函数 by hlc

#ifdef STEP_MOTOR_DDDM_SUPPORT

#define DRV8711		1

#endif


void StepMotor_Isr_call(unsigned int stepno)
{	static unsigned short ii=0;
	step_in_isr++;
	StepMotor_Isr(stepno);
	//Message_Send_4halfword(0x9999,stepno,0,ii++);

	step_in_isr--;

}


void Step_check_through_zero_adj(STEP_TYPE *Step)
{
	char work_dir=Step->step_st.bit.dir_High;
	char zero_dir=Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir;
	int step_alert_t;//=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	int tem_go;
	int errsteps_=0;
	#ifndef LOG_DEBUG_FOR_LX_AT_CH
	if (Step->chk.bit.SK_ADJ_ok) return;//已经修正过了，不做修正
	#endif
	if (!Step->step_st.bit.dir)
	{
		if ((Step->chk.bit.last_zero_bit  & 0x1F)==(work_dir==zero_dir?(0x10):(0x0f)))
		{
			step_alert_t=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			tem_go = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]-MOTOR_ZERO_ST_FILTER_BITS+1;

			if (is_lx_sinker_ex(Step))
			{
				tem_go =( (Step->moto_zero_width_self - Step->input_errorstep)>>1 )-MOTOR_ZERO_ST_FILTER_BITS+1;
				//tem_go =MOTOR_ZERO_ST_FILTER_BITS+1
				//step_alert_t =200;
			}
			if((Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER)
				&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_LX_EX ))
			{
				tem_go +=step_zero_adj_sk_lxex;
			}
	#if 0
			if ((Step->moto_remap_config.moto_attr.bit.moto_type_config ==MOTOR_TYPE_SKINER )&&(Step->alert_delay_max))
			{
				step_alert_t =200;
			}
	#endif		
			if (work_dir==zero_dir)
			{
				tem_go +=MOTOR_ZERO_ST_FILTER_BITS+MOTOR_ZERO_ST_FILTER_BITS+ Step->input_errorstep;

				tem_go = 0-tem_go; 
			}

		check_lost_step_liftstep:
			#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if (Step->chk.bit.SK_ADJ_ok)
			{
				
				if(enable_log_tosend_LX)
				{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ADJ_ISOK_BUT:LX_LOG_TYPE_ADJ_ISOK_BUT_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				}
				
				return;
			}

			#endif

			
			if (Step->position<tem_go)
			{
				errsteps_ = tem_go-Step->position;	//显示+	// 2--表示本次失步了
				errsteps_ |=((Step->step_st.bit.dir?3:2)<<12);
			}
			else
			{
				errsteps_ = Step->position-tem_go ;	//显示- // 3 --表表示上一次失步了
				errsteps_ |=((Step->step_st.bit.dir?2:3)<<12);
			}
			/*屏蔽推针电机走多了的报警*/
			if((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
				&&(((errsteps_>>12) & 0x0f)==2))
			{
				errsteps_ =0; /*不报警*/
			}
			
			
			if (((errsteps_&0xFFF)>=step_alert_t)&&(step_alert_t))
			{
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errsteps_|((unsigned int)Step->st_no_cl.bit.return_byte<<16));	
			}
			#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if (((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)&&(enable_log_tosend_LX))
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ZERO_ADJ_LEAVE:LX_LOG_TYPE_ZERO_ADJ_GOTO)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ZERO_ADJ_LEAVE_2:LX_LOG_TYPE_ZERO_ADJ_GOTO_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] = tem_go;								
				Message_send_log_LX(d[0],d[1],d[2]);
			}
	
			#endif
			
			Step->position=tem_go; 
			Step->steps =Step->position>=Step->pos_2?(Step->position-Step->pos_2):(Step->pos_2-Step->position);
			Step->chk.bit.SK_ADJ_ok =1;   
		}
	}
	else
	{
		if ((Step->chk.bit.last_zero_bit  & 0x1F)==(work_dir!=zero_dir?(0x10):(0x0f)))
		{
			step_alert_t=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			tem_go = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]-MOTOR_ZERO_ST_FILTER_BITS+1;
			if (is_lx_sinker_ex(Step))
			{
				tem_go =( (Step->moto_zero_width_self - Step->input_errorstep)>>1 )-MOTOR_ZERO_ST_FILTER_BITS+1;
				//tem_go =MOTOR_ZERO_ST_FILTER_BITS+1
				//step_alert_t =200;
			}
			if((Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER)
				&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_LX_EX ))
			{
				tem_go +=step_zero_adj_sk_lxex;
			}
			
			#if 0
			if ((Step->moto_remap_config.moto_attr.bit.moto_type_config ==MOTOR_TYPE_SKINER )&&(Step->alert_delay_max))
			{
				step_alert_t =200;
			}
			#endif
			if (work_dir!=zero_dir)
			{
				tem_go +=MOTOR_ZERO_ST_FILTER_BITS+MOTOR_ZERO_ST_FILTER_BITS+ Step->input_errorstep;

				//tem_go = 0-tem_go; 
			}
			else
				tem_go = 0-tem_go;

  			goto check_lost_step_liftstep;
		}
	}
		

	

	
}

void Step_check_p2n_step_adj(STEP_TYPE *Step)/*从正到0或负的时候过零修正*/
{

	unsigned char need_adj=0;

	if (Step->step_st.bit.running>RUNNING)
		return;
	if (Step->moto_remap_config.moto_attr.bit.is_fast_mode)
		return;

	//#ifndef LOG_DEBUG_FOR_LX_AT_CH

	if(((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == YARN_MOTOR_TYPE_LX)))
	{
		if((Step->chk.bit.SK_ADJ_ok)
			&&(Step->chk.bit.Zero_Out_ADJ_OK))
			return;
	}
	else
	{	
		if(Step->chk.bit.SK_ADJ_ok) 
			return;
	}
	//#endif

	if((((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)||(Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER))
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe != LIFT_MOTOR_TYPE_PT))
			||((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == ACT_MOTOR_TYPE_DOT))
			)
	{
		if((Step->chk.bit.last_zero_bit  & 0x1F) == 0x0F)
		{
			need_adj=2;
		}
	}
	else
	{
		if(((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == YARN_MOTOR_TYPE_LX)))
		{
			if(Step->step_st.bit.dir==0)
			{
				if(((Step->chk.bit.last_zero_bit  & 0x1F) == 0x0F)	
					&&(!(Step->chk.bit.SK_ADJ_ok)))
				{
					need_adj=3;  /*进入零位的时候修正*/
				
				}
				if(((Step->chk.bit.last_zero_bit  & 0x1F) == 0x10)	
					&&(!(Step->chk.bit.Zero_Out_ADJ_OK)))
				{
					need_adj=4;  /*离开零位的时候修正*/
					if((Step->yarn_motor_elapse_steps>0)
						&&(Step->yarn_motor_elapse_steps<=70))
					{
						need_adj=0;	
					}
					else
					{
						Step->yarn_motor_elapse_steps=0;
					}
				}
			}
		}
		else	
		if((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)
			&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX3))
		{
			if((Step->chk.bit.last_zero_bit  & 0x1F)==0x0F)
			{
				need_adj=1;
			}
		}
		else
		{
			if ((Step->chk.bit.last_zero_bit  & 0x1F)== (Step->step_st.bit.dir?(0x10):(0x0F)))
			{
				need_adj=1;
			}
			if((Step->step_st.bit.dir)
				&&(is_LX_DENSITY_Step_ex(Step)))
			{
				need_adj =0;
			}
		}
	
	}
			
	
	//if (MOTOR_ZERO_IS_EXCHANGE(Step->step_st.bit.dir?0:1,Step->chk.bit.last_zero_bit))	
	//if ((Step->chk.bit.last_zero_bit  & 0x1F)== (Step->step_st.bit.dir?(0x10):(0x0F)))
	if(need_adj)	
	{

	//	if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER)
		//	Message_Send_4halfword(0x9988,0,Step->state_par,Step->chk.bit.last_zero_bit);

			#if 0
			if(Step->chk.bit.SK_ADJ_ok)
			{
				if (enable_log_tosend_LX)
				{
					unsigned short d[3];
					d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
					d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
					d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ADJ_ISOK_BUT:LX_LOG_TYPE_ADJ_ISOK_BUT_2)<<8;
					d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
					d[1] =Step->position;
					d[2] =Step->steps;								
					Message_send_log_LX(d[0],d[1],d[2]);										
				}
				return;
			}
			#endif
	
		//if ()//从正走到负的时候我们才修正
		{
			int errsteps_=0;
			int step_alert_t=step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			int tem_go= step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]-MOTOR_ZERO_ST_FILTER_BITS+1;

			//is_HF_LIFT_Step(int stepno)

			if (is_LX_DENSITY_Step_ex(Step))
				tem_go =((Step->moto_zero_width_self - Step->input_errorstep)>>1)-MOTOR_ZERO_ST_FILTER_BITS+1;
			
			
			if ((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe!= LIFT_MOTOR_TYPE_PT))
			{
				tem_go = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
			}
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			{
				tem_go = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
			}

			if ((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe))
			{
				tem_go = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
			}

			if(((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == ACT_MOTOR_TYPE_DOT)))
			{
				tem_go = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
			}

			if(((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX3)))
			{
				tem_go = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
			}
			if(need_adj==3)
			{
				if(tem_go>4)
					tem_go-=4;
				step_alert_t =0;
				
			}
			else
			if(need_adj==4)
			{				
				tem_go+=4;				
				tem_go =0-tem_go;
				
				step_alert_t =0;
			}
			else
			if(need_adj==2)
			{
				if (Step->step_st.bit.dir)
				{
					tem_go =0-tem_go;
				}
				if (Step->position<tem_go)
				{
					errsteps_ = tem_go-Step->position;		// 2--表示本次失步了
					errsteps_ |=((Step->step_st.bit.dir?3:2)<<12);      /*主控显示+XXX*/
				}
				else
				{
					errsteps_ = Step->position-tem_go ;	// 3 --表表示上一次失步了
					errsteps_ |=((Step->step_st.bit.dir?2:3)<<12);/*主控显示-XXX*/
				}				
			}
			else
			{
				if(((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX3)))
				{
					tem_go -=MOTOR_ZERO_ST_FILTER_BITS;
				}
				else
				{
					if (Step->step_st.bit.dir)
					{				
						tem_go += (Step->input_errorstep)+MOTOR_ZERO_ST_FILTER_BITS;//+MOTOR_ZERO_ST_FILTER_BITS;
					}	
				}
			
				if (Step->position<tem_go)
				{
					errsteps_ = tem_go-Step->position;		// 2--表示本次失步了
					errsteps_ |=((Step->step_st.bit.dir?7:2)<<12);      /*主控显示+XXX*/
				}
				else
				{
					errsteps_ = Step->position-tem_go ;	// 3 --表表示上一次失步了
					errsteps_ |=((Step->step_st.bit.dir?6:2)<<12);/*主控显示-XXX*/
				}
			}

			if (Step->moto_remap_config.moto_attr.bit.is_fast_mode)
			{
				if ((errsteps_&0xFFF)>=700)
					return ;
			}

			
			#if 0
			if (((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)&&(enable_log_tosend_LX))	
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ZERO_ADJ_LEAVE:LX_LOG_TYPE_ZERO_ADJ_GOTO)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				
				//unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (Step->step_st.bit.dir?LX_LOG_TYPE_ZERO_ADJ_LEAVE_2:LX_LOG_TYPE_ZERO_ADJ_GOTO_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] =tem_go;								
				Message_send_log_LX(d[0],d[1],d[2]);
			}
			#endif
	
			if (is_LX_DENSITY_Step_ex(Step))
				errsteps_=0;
			if (((errsteps_&0xFFF)>step_alert_t)&&(step_alert_t))
			{

				//errsteps_ = tem_go;//step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//Step->input_errorstep;
				
				//if (!is_LX_DENSITY_Step_ex(Step))
				if ((Step->moto_remap_config.moto_attr.bit.is_fast_mode) )//&& (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION)&&())
				{
					//bubaoj
				}else					
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errsteps_|((unsigned int)Step->st_no_cl.bit.return_byte<<16));	
			}
			
			Step->position=tem_go; 
			
			if (Step->steps_go_>=2000)/*运行超过2000了。那需要停止了*/
			{
				Step->position=Step->pos_2; 
				Step->steps =0;

			//	Message_Send_4halfword(0xFFFF, Step->moto_remap_config.moto_remap_id_all, Step->position, Step->steps);
			//	Message_Send_4halfword(0xFEFF, Step->moto_remap_config.moto_remap_id_all, Step->chk.bit.last_zero_bit, Step->chk.bit.SK_ADJ_ok);
							
			}
			else
			{				
				
				Step->steps =Step->position>=Step->pos_2?(Step->position-Step->pos_2):(Step->pos_2-Step->position);
				if(need_adj==3)
				{
					Step->position+=Step->steps; 
					Step->steps <<=1;
				}	
				if(is_LX_yarn_step_exe(Step))
					Message_Send_4halfword(0xFF|( Step->moto_remap_config.moto_remap_id_all<<8),need_adj, Step->position, Step->steps);
			}	
			//Message_Send_4halfword(0x06FF|(Step->moto_remap_config.moto_remap_id_all<<12), Step->step_st.bit.dir, Step->position, Step->steps);
		}		

		if(need_adj==4)
		{
			Step->chk.bit.Zero_Out_ADJ_OK=1;
		}
		else
			Step->chk.bit.SK_ADJ_ok=1;
	
	}
}


void Step_check_FH_actstep_adj(STEP_TYPE *Step,int zero,int work)
{
		if ((Step->chk.bit.chk_st_change)&&(Step->chk.bit.last_zero_st !=(zero?1:0))) //状态改变了
		{
			Step->chk.bit.chk_st_change =0;
		}
		if ((Step->chk.bit.chk_gotozero)&&(zero))
		{
			Step->chk.bit.chk_gotozero =0;
		}
		if ((Step->chk.bit.chk_leavezero)&&(!zero))
		{
			Step->chk.bit.chk_leavezero =0;
		}
		Step->chk.bit.last_zero_st = zero?1:0;

}

void Step_zero_st_check_HF_Lift(STEP_TYPE *Step,int zero)
{
	if ((Step->chk.bit.chk_st_change)&&(Step->chk.bit.last_zero_st !=(zero?1:0))) //状态改变了
	{
		Step->chk.bit.chk_st_change =0;
		//if(is_lx_sinker_ex(Step))
		//	Message_Send_4halfword(0x9999,Step->moto_remap_config.moto_remap_id_self,Step->chk.bit.chk_st_change,1);
	}
	Step->chk.bit.last_zero_st = zero?1:0;
		
}

//#define LX_ACT_DONOT_ADJ

void Step_check_LX_actstep_adj(STEP_TYPE *Step,int zero)
{
		short steps_jz;
		short err_steps=0;

		if ((Step->chk.bit.chk_st_change)&&(Step->chk.bit.last_zero_st !=(zero?1:0))) //状态改变了
		{
			Step->chk.bit.chk_st_change =0;
		}
		
		if (Step->state_par==JUST_RUN)
		{
			//int s_e=0;
			//int error_t;
			if (Step->chk.bit.chk_leavezero)
			{					
				//if ((Step->chk.bit.last_zero_st)&&(!zero))/*离开零位*/	
				if ((Step->chk.bit.last_zero_bit & 0x07 )==0x04)/*离开零位*/	
				{
					if ((Step->chk.bit.is_never_leavezero)||( (Step->step_st.bit.dir)?(Step->position>0):(Step->position<0)))
					{

						steps_jz = ((Step->moto_zero_width_self + Step->input_errorstep)>>1)+(Step->step_st.bit.dir?(1):(-1))*LX_ACT_zero_adj;

#if 0	
						if (Step->step_st.bit.dir)
						{

						if (Step->position>=(0-zero_go_))
						{
							error_t = 0x05;
							s_e=abs(Step->position+zero_go_);
							error_t |= (s_e<<8);
						}
						else
						{
							error_t = 0x06;
							s_e=abs((0-zero_go_) - Step->position);
							error_t |= (s_e<<8);
						}
						}
						else
						{

						if (Step->position>=(zero_go_))
						{
							error_t = 0x07;
							s_e=abs(Step->position-zero_go_);
							error_t |= (s_e<<8);
						}
						else
						{
							error_t = 0x08;
							s_e=abs(zero_go_ - Step->position);
							error_t |= (s_e<<8);
						}
						}
						if (s_e>25)
						{
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),error_t);
						}

#endif					
					#ifdef LX_ACT_DONOT_ADJ
						err_steps = abs(Step->position-((Step->step_st.bit.dir?(1):(-1))*steps_jz));
						/*修正值小于10或者偏差大于10的累计次数达到10次*/
						if ((err_steps<=10) || (LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]>=10))
						{
							Step->position = (Step->step_st.bit.dir?(1):(-1))*steps_jz/*0*/;	
							if (Step->position>=Step->pos_2)
							{	
								Step->steps = Step->position-Step->pos_2;
							}
							else
								Step->steps = Step->pos_2-Step->position;
							LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]=0;
						}
						else
						{
							LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]++;
						}
					#else
					#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if(enable_log_tosend_LX)
			{
				short errsteps_ = abs(Step->position-((Step->step_st.bit.dir?(1):(-1))*steps_jz));
				//if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)
				{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_ZERO_ADJ_LEAVE)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_ZERO_ADJ_LEAVE_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] = (Step->step_st.bit.dir?(1):(-1))*steps_jz;								
				Message_send_log_LX(d[0],d[1],d[2]);
				}
			}
			#endif
	
					
						Step->position = (Step->step_st.bit.dir?(1):(-1))*steps_jz/*0*/;	
						if (Step->position>=Step->pos_2)
						{	
							Step->steps = Step->position-Step->pos_2;
						}
						else
							Step->steps = Step->pos_2-Step->position;
					#endif	
						
						Step->chk.bit.chk_leavezero=0;

						//Message_Send_4halfword(0xFFFF, Step->moto_remap_config.moto_remap_id_all, Step->position, 0x0001);
					

					}
					
				}
			}
			if (Step->chk.bit.chk_gotozero)
			{
				//if ((Step->chk.bit.last_zero_st==0)&&(zero))
				if ((Step->chk.bit.last_zero_bit & 0x07 )==0x03)				
				{
					steps_jz = ((Step->moto_zero_width_self - Step->input_errorstep)>>1)+(Step->step_st.bit.dir?(-1):(1))*LX_ACT_zero_adj;
					
					#if 0	
					if (Step->step_st.bit.dir)
					{
					
						if (Step->position>=(0-zero_go_))
						{
							error_t = 0x05;
							s_e=abs(Step->position+zero_go_);
							error_t |= (s_e<<8);
						}
						else
						{
							error_t = 0x06;
							s_e=abs((0-zero_go_) - Step->position);
							error_t |= (s_e<<8);
						}
					}
					else
					{
						
						if (Step->position>=(zero_go_))
						{
							error_t = 0x07;
							s_e=abs(Step->position-zero_go_);
							error_t |= (s_e<<8);
						}
						else
						{
							error_t = 0x08;
							s_e=abs(zero_go_ - Step->position);
							error_t |= (s_e<<8);
						}
					}
					if (s_e>25)
					{
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),error_t);
					}
					
					#endif
					#ifdef LX_ACT_DONOT_ADJ
					err_steps = abs(Step->position-((Step->step_st.bit.dir?(-1):(1))*steps_jz));
					/*修正值小于10或者偏差大于10的累计次数达到10次*/
					if ((err_steps<=10) || (LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]>=10))
					{
						Step->position = (Step->step_st.bit.dir?(-1):(1))*steps_jz/*0*/;	
						if (Step->position>=Step->pos_2)
						{	
							Step->steps = Step->position-Step->pos_2;
						}
						else
							Step->steps = Step->pos_2-Step->position;
						
						LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]=0;
					}
					else
					{
						LX_ACT_check_lost_cnt[Step->moto_remap_config.moto_remap_id_self & 0x01]++;
					}
					#else
					#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if(enable_log_tosend_LX)
			{
				short errsteps_ = abs(Step->position-((Step->step_st.bit.dir?(-1):(1))*steps_jz));
				//if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)
				{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_ZERO_ADJ_GOTO)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_ZERO_ADJ_GOTO_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] = (Step->step_st.bit.dir?(-1):(1))*steps_jz;								
				Message_send_log_LX(d[0],d[1],d[2]);
				}
			}
			#endif
	
					Step->position = (Step->step_st.bit.dir?(-1):(1))*steps_jz/*0*/;	
						if (Step->position>=Step->pos_2)
						{	
							Step->steps = Step->position-Step->pos_2;
						}
						else
							Step->steps = Step->pos_2-Step->position;
					#endif
					Step->chk.bit.chk_gotozero=0;
					//Message_Send_4halfword(0xFFFF, Step->moto_remap_config.moto_remap_id_all, Step->position, 0x0002);
					
				}
			}

			#if 1
			if ((Step->chk.bit.chk_leavezero)&&(Step->steps ==1)&&(Step->chk.bit.is_never_leavezero==0))/*走完了还没离开零位那有问题*/
			{
				//if (Step->steps_go_) 


			#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if(enable_log_tosend_LX)
			{
				//short errsteps_ = abs(Step->position-((Step->step_st.bit.dir?(-1):(1))*steps_jz));
				//if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)
				{
					unsigned short d[3];
					d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
					d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
					d[0] |= (LX_LOG_TYPE_LEAVEZERO_ADD200)<<8;
					d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
					d[1] =Step->position;
					d[2] =Step->steps;								
					Message_send_log_LX(d[0],d[1],d[2]);
				}				
			}
			#endif
				
				Step->steps = 200;
				Step->position = (Step->pos_2+(Step->step_st.bit.dir?(-1):(1))*Step->steps);
				Step->chk.bit.is_never_leavezero=1;
#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if(enable_log_tosend_LX)
			{
				//short errsteps_ = abs(Step->position-((Step->step_st.bit.dir?(-1):(1))*steps_jz));
				//if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)
			
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_LEAVEZERO_ADD200_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
			}
				
			}
			#endif

				//Step->chk.bit.is_adj_need_check =1;
			}
			#endif
			
		}
		Step->chk.bit.last_zero_st = zero?1:0;
}

unsigned int Triangle_input_is_ok(STEP_TYPE *Step)
{
	unsigned int ret = 0;
	switch (Step->chk.bit.HP_check_st)
	{
		case 0x01:
			if (((Step->chk.bit.last_zero_bit & 0x1F)==0)&&((Step->chk.bit.last_work_bit & 0x1F)==0x0F))
			{
				ret =1;		
			}
			else
				ret =0;
			
		break;
		case 0x02:
			if (((Step->chk.bit.last_zero_bit & 0x1F)==0x0F)&&((Step->chk.bit.last_work_bit & 0x1F)==0x00))
			{
				ret =1;		
			}
			else
				ret =0;
		break;
		case 0x03:
			if (((Step->chk.bit.last_zero_bit & 0x0F)==0x0F)&&((Step->chk.bit.last_work_bit & 0x0F)==0x0F))
			{
				ret =1;		
			}
			else
				ret =0;
		break;	
	}

	return ret;
	
}

void StepMotor_Signal_edge_toreport(STEP_TYPE *Step,unsigned char signaltype)
{
	extern void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
	//STEP_TYPE *Step;
	//static unsigned char stepreport_index=0;
	//unsigned char iszero =0;
			
	if(Step==NULL ) return;

	//Step = &STEPMOTOR[stepno];	
	Message_Send_4halfword((CMD_OTHER_SET_MOTOR_SIGNAL_EDG << 8) | 0x01,Step->moto_remap_config.moto_remap_id_self |(Step->moto_remap_config.moto_attr.bit.moto_type_config<<8),Step->position,signaltype);
	return ;
	
}

void StepMotor_afterrun_toreport(unsigned int stepno,unsigned char islxsk)
{
	extern void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
	STEP_TYPE *Step;
	static unsigned char stepreport_index=0;
	unsigned char iszero =0;
			
	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

	if (islxsk)
	{
		Message_Send_4halfword((CMD_STEP_MOTOR_ISOK_RETURN << 8) | 0x01,Step->moto_remap_config.moto_remap_id_self |(Step->moto_remap_config.moto_attr.bit.moto_type_config<<8),Step->position,0xFF|(stepreport_index++<<8));
	}
	else	
	if (Step->st_no_cl.bit.report_cnt)
	{
		Step->st_no_cl.bit.report_cnt--;
		iszero =0;
		if ((Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER) &&
			(Step->moto_remap_config.moto_attr.bit.moto_type_exe != SINKER_MOTOR_TYPE_PT))
		iszero =arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
	//	Message_Send_4halfword((CMD_STEP_MOTOR_ISOK_RETURN << 8) | 0x01,Step->moto_remap_config.moto_remap_id_self |(Step->moto_remap_config.moto_attr.bit.moto_type_config<<8)|((iszero?1:0)<<15),Step->Base_t_250us & 0xffff,Step->Base_t_250us >>16);
		Message_Send_4halfword((CMD_STEP_MOTOR_ISOK_RETURN << 8) | 0x01,Step->moto_remap_config.moto_remap_id_self |(Step->moto_remap_config.moto_attr.bit.moto_type_config<<8)|((iszero?1:0)<<15),Step->position,Step->st_no_cl.bit.report_cnt|(stepreport_index++<<8));
	
	}
	return ;

	
}


void Send_postion_ecode()
{
return ;
#if 0
	int i;
	for (i=0;i<801;)
	{
		Message_Send_4halfword(0xf3|((i/3)<<8),step_postion_ecode[i],step_postion_ecode[i+1],step_postion_ecode[i+2]);
		i+=3;
	}
	i=801;
	Message_Send_4halfword(0xf3|((i/3)<<8),step_postion_ecode[i],step_postion_ecode[i+1],step_postion_ecode[i+2]);
#endif

}




void StepMotor_Isr(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int speed;

	unsigned int step_alert_detect_setup_local;
	unsigned char change_dir=0;
	
	int zero = 0;
	int isleft = 0;
	int isright = 0;
	int work = 0;
	int ZeroPos_workST;
	int WorkPos_workST;
	
	int errno = 0;
	//int step_alert_detect;

	int zero_work_area;
	int zero_work_area_2;
	
	int encoder_en = 0;
	int zero_go_;

	

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

#if 0
	
	//Step->zero_cach_cnt
	if (Step->zero_last ==zero)
	{
	Step->zero_cach_cnt++;	
	}
	else{
	Step->zero_cach_cnt=0;
	Step->zero_last = zero;
	}
	if(Step->zero_cach_cnt==3)
	{
		Step->zero_cach = zero;		
	}
	
	zero = Step->zero_cach;

	#endif

	
	Step->step_is_cnt++;

	if((Step->moto_dd_config.DD_type==1)&&(Step->moto_dd_config.DD_IS_setting))
	{
		//arch_StepMotor_Set_Speed(stepno, 10000);/*遇到中断抢断，延时10us*/
		//goto done;
		arch_StepMotor_Stop(stepno);
		return ;
	}


	if(SYS_is_PowerOn==0)  //断电之后的处理
	{
		goto alarm_and_exit;
	}
	
#ifdef ENCODER_SUPPORT
	encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
#endif /* ENCODER_SUPPORT */

	if (is_HP_ACTION_Step(stepno))
	{
		zero_go_ =(Step->moto_zero_width_self - Step->input_errorstep)>>1;
	}
	else
	zero_go_ = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];

	//#ifdef LX_ACTION_STEP_SPECIAL
	if (is_LX_ACTION_Step(stepno)
		||is_HF_LIFT_Step(stepno)
		||is_EX_OTHER_Step(stepno)
		||is_DOT_ACTION_Step(stepno)
		||is_CX3_Feet_Step(stepno)
		||is_LX_yarn_step(stepno))
	{
		zero_go_ = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
		if (is_LX_ACTION_Step(stepno))
		{
			zero_go_ +=(Step->step_st.bit.dir?(-1):(1))*LX_ACT_zero_adj;
		//if (Step->step_st.bit.dir)

			if (Step->steps_go_ >= 1980)
			{
				errno = 8;
				goto alarm_and_exit;
			}
		}
		#ifdef IS_CX_ACT
		if(is_DOT_ACTION_Step(stepno))
		{
			zero_go_ = Step->step_st.bit.dir?((zero_go_<<1)-_CX_DOT_ACT_ZERO_ADJ):_CX_DOT_ACT_ZERO_ADJ;
		}
		
		#endif
		
	}
	if (is_LX_DENSITY_Step(stepno)||is_lx_sinker(stepno))
	{
		zero_go_ = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
	}
	//#endif

	if (is_lxex_sinker(stepno))
	{
		zero_go_ +=step_zero_adj_sk_lxex;
	}
	

	if (Step->moto_remap_config.moto_attr.bit.moto_type_config != MOTOR_TYPE_UNDEF)
	{
		step_alert_detect_setup_local = step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	}
	else
	{
		step_alert_detect_setup_local = 0;
	}

	if(Step->step_st.bit.running == 0 || Step->steps == 0) {          //说明走完了，该停下来了。
		Step->step_st.bit.running = 0;
		#ifdef MOTOR_FULL_CURR_OUTPUT
		if(is_DENSITY_Step(stepno))
		{
			arch_StepMotor_Half(stepno ,MOTOR_FULL_CURR_OUTPUT);//半流
		}
		else			
		#endif
		arch_StepMotor_Half(stepno ,MOTOR_HALF_OUTPUT);//半流
		//#endif
		arch_StepMotor_Stop(stepno);
		goto exit;
	}
	//Step->step_check_interval = 50;
	if(Step->step_wait_time) {
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		arch_StepMotor_Set_Speed(stepno, 2000/(Step->step_wait_time));
		#else
		arch_StepMotor_Set_Speed(stepno, PULSE_LOW);
		#endif
		goto done;
	}

	
	if(Step->dir_is_change_)
	{
		arch_StepMotor_Dir(stepno, (Step->step_st.bit.dir==0)?!Step->step_st.bit.dir_High:Step->step_st.bit.dir_High);
		//Message_Send_4halfword((0x90+Step->dir_is_change_)|(stepno<<8),Step->pos_2,Step->position,Step->steps);
		//Message_Send_4halfword((0x9A+Step->dir_is_change_)|(stepno<<8),Step->speed,Step->dir,Step->reset);
		Step->dir_is_change_ =0;
	}
	

	if(Step->step_st.bit.level == 0)
	{
#ifdef ENCODER_SUPPORT
		//if (encoder_en)
		//Encoder_RunCheck(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, Step->speed, Step->position);
#endif /* ENCODER_SUPPORT */
			
		Step->step_st.bit.level = 1;
		arch_StepMotor_Pulse(stepno, 1);
#ifndef DRV8711
		arch_StepMotor_Set_Speed(stepno, PULSE_LOW);
		goto done;
#endif
	}

#ifndef DRV8711
	Step->step_st.bit.level = 0;
	arch_StepMotor_Pulse(stepno, 0);
#endif

	zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
	work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
	ZeroPos_workST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
	WorkPos_workST = Step->moto_remap_config.moto_attr.bit.workPos_Work_ST;
	Step->chk.bit.last_zero_bit <<=1;
	Step->chk.bit.last_zero_bit |=zero;

	Step->chk.bit.last_work_bit <<=1;
	Step->chk.bit.last_work_bit |=work;

	//Message_Send_4halfword(0x99|(stepno<<8),Step->chk.all, Step->chk.all>>16,Step->steps);

	Yarn_motor_input_bits_set(stepno,zero);

	

	if(is_HP_ACTION_Step(stepno)) 
	{
		Step_input_sts_new(stepno);
		isleft = Triangle_is_left_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		isright =Triangle_is_right_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		zero =Triangle_is_zero_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
		
		
	}
	
#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) && zero && (Step->act_is_justcheck_zero==0))
	{
		//if ((work)&&(!Step->step_st.bit.IS_Reset_ex) )
		if (work!=ZeroPos_workST)
		{
			zero = 0;
		}
	}
	if(is_PT_ACTION_Step(stepno)) 
	{
		//int zero2 = work;
		if(Step->step_st.bit.zero2 != work) 
		{
			Step->step_st.bit.zero2 = work;		
			Step->step_st.bit.zero2_count ++;		
		}
		if (Step->state_par == DETECT_ZERO2)
		{
			goto pos_end;
		}
	}

	if(is_CX3_Feet_Step(stepno))
	{
		if ((zero)&&(work!=ZeroPos_workST))
		{
			zero = 0;
		}
		if(Step->step_st.bit.zero2 != work) 
		{
			Step->step_st.bit.zero2 = work;		
			Step->step_st.bit.zero2_count ++;		
		}
		if (Step->state_par == DETECT_ZERO2)
		{
			goto pos_end;
		}
	}

#endif

	if (!zero)
	{
		if (Step->step_st.bit.zero_cnt)
		{
			Step->step_st.bit.zero_cnt++;
		}
		else
		{
			if (Step->step_st.bit.zero)
			{
				Step->step_st.bit.zero_cnt++;
			}
		}
	}
	else
	{
		
		Step->step_st.bit.zero_cnt =0;
	}


	


	if (!encoder_en)
	{
	//#ifdef LX_ACTION_STEP_SPECIAL

		/*20191119 check motor signal edgE*/

		if(Step->check_signal_edge & 0x03)/*check zero*/
		{
			if(Step->check_signal_edge & 0x01)
			{
				if((Step->chk.bit.last_zero_bit & 0x07)==0x03)	/*0-->--1*/
				{
					StepMotor_Signal_edge_toreport(Step,0);
				}
			}
			if(Step->check_signal_edge & 0x02)
			{
				if((Step->chk.bit.last_zero_bit & 0x07)==0x04) 	/*1-->--0*/
				{
					StepMotor_Signal_edge_toreport(Step,1);
				}
			}
			
		}
		
		if(Step->check_signal_edge & 0x0C)/*check work*/
		{
			if(Step->check_signal_edge & 0x04)
			{
				if((Step->chk.bit.last_work_bit & 0x07)==0x03)	/*0-->--1*/
				{
					StepMotor_Signal_edge_toreport(Step,2);
				}
			}
			if(Step->check_signal_edge & 0x08)
			{
				if((Step->chk.bit.last_work_bit & 0x07)==0x04) 	/*1-->--0*/
				{
					StepMotor_Signal_edge_toreport(Step,3);
				}
			}
		}
		
	
		if (is_lx_sinker(stepno)||is_lxex_sinker(stepno))
		{
			if (Step->chk.bit.chk_gotozero)
			{
				if ((Step->chk.bit.last_zero_bit & 0x07)==0x03)
				{
					StepMotor_afterrun_toreport(stepno,1);//
					Step->chk.bit.chk_gotozero =0;
				}					
			}
			Step_zero_st_check_HF_Lift(Step,zero);
		}	
	
		if (is_FH_ACTION_Step(stepno))
		{
			Step_check_FH_actstep_adj(Step,zero,work);
		}
		else
		if (is_LX_ACTION_Step(stepno))
		{
			Step_check_LX_actstep_adj(Step,zero);
		}
		else
		if (is_sinker(stepno)
			||is_other_step(stepno)
			||is_LIFT_all_Step(stepno)
			||is_DENSITY_Step(stepno)
			||is_PT_ACTION_Step(stepno)
			||is_DOT_ACTION_Step(stepno)
			||is_CX3_Feet_Step(stepno)
			||is_LX_yarn_step(stepno))
		{

			//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER)
				//Message_Send_4halfword(0x9988,0,Step->state_par,Step->chk.bit.last_zero_bit);

			if(is_JF_DENSITY_Step(stepno))
				goto JFDENSITY_NOTGOZEROCHECK;				
			if (Step->state_par==JUST_RUN)
			{
				if (is_LIFT_or_Other_Step(stepno) || is_sinker(stepno))
				{
					Step_check_through_zero_adj(Step);
				}
				else
				{
					if (is_HF_LIFT_Step(stepno)
						||is_EX_OTHER_Step(stepno)
						||is_DOT_ACTION_Step(stepno)
						||is_CX3_Feet_Step(stepno)
						||is_LX_yarn_step(stepno))
					{
						Step_zero_st_check_HF_Lift(Step,zero);
					}

					#ifdef LX_DM_TEST
					if (is_LX_DENSITY_Step(stepno))
					{
						if (Step->step_st.bit.dir==0)
						{
							goto do_adj_step_all;//Step_check_p2n_step_adj(Step);		
						}
					}
					else
					{
					#endif
				//	do_adj_step_all:
					Step_check_p2n_step_adj(Step);
#ifdef LX_DM_TEST
					}
#endif					
				}	
			}
		}
		else if( is_HP_ACTION_Step(stepno))
		{
			if ((Step->state_par==JUST_RUN)&&(Step->steps==HP_STEP_MOTOR_CHECK_INPUT_AREA))
			{
			
				Step->state_par =	HP_STEP_CHECK_INPUT;		

				//Message_Send_4halfword(0x99|(stepno <<8),Step->position,Step->pos_2,0);
			}
		}
		
			
	}

	JFDENSITY_NOTGOZEROCHECK:
	
//#endif

#if 0
	if (Step->position<0)
	{
		step_postion_ecode[800]++;
	}else
	if (Step->position>=800)	
	{
		step_postion_ecode[801]++;
	}
	else
		step_postion_ecode[Step->position]=Ecode_Get_last_ecode(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
	
	#endif	

	if (Step->steps)
	{
		if(Step->step_st.bit.dir) {
			Step->position ++;
		}
		else {
			Step->position --;
		}
	}
	
#ifdef ZERO2_SUPPORT
pos_end:
#endif

	if (Step->steps)
		Step->steps --;
	Step->steps_go_++;

	if(is_LX_yarn_step(stepno))
	{
		
		if(Step->yarn_motor_elapse_steps)
		{
			Step->yarn_motor_elapse_steps++;
		}
		else
		if(zero)
		{
			Step->yarn_motor_elapse_steps=1;
		}
	}

	switch(Step->state_par) {
		case GOTOZERO_LX_ACT:
			{
				if (is_LX_ACTION_Step(stepno))
				{
					if (((Step->chk.bit.last_zero_bit  & 0xFF)==0x00)&&(Step->chk.bit.LX_ACT_st_ok==0))
					{
						Step->chk.bit.LX_ACT_st_ok=1;
					}
					if (((Step->chk.bit.last_zero_bit  & 0x1F)== 0x0F )&&(Step->chk.bit.LX_ACT_st_ok))
					{
						//short steps_jz=zero_go_-4;
						
						//steps_jz *=(Step->step_st.bit.dir?(-1):(1));
						Step->position = Step->step_st.bit.dir?(4-zero_go_):(zero_go_-4);
						Step->state_par = JUST_RUN;
						goto LX_ACT_gotozero_go;
					
					}
				}
			}			
			break;		
		case JUST_RUN_GO:
			{	
				if (Step->steps==0)
				{	
					Step->steps = abs(Step->pos_2-Step->position);
					//Step->position = STEPS_LEAVEZERO;
					//Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
				#ifdef STEP_MOTOR_DDDM_SUPPORT
					//Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
				#else
					//Step->speed = step_base_speed;
				#endif
					//Step->step_st.bit.phase = 0;
					Step->step_st.bit.dir = !Step->step_st.bit.dir;
					Step->step_wait_time = 3;
					change_dir =1;
					//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
					Step->state_par = JUST_RUN;
				}

			}		
			break;
		
		case JUST_RUN:
		case JUST_RUN_AFTER_ADJ:
		case JUST_RUN_RESET:
			{
			//Message_Send_4halfword(0xAADD,((stepno)<<8)|0x01,Step->position,Step->steps);

			#ifdef ZERO2_SUPPORT
			if((Step->step_st.bit.running ==RUNNING)&&(Step->state_par==JUST_RUN))
			{
				if (((is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) )||(is_CX3_Feet_Step(stepno)))
					&&( (Step->steps == step_setup_zero2_detect_steps)||((Step->steps<step_setup_zero2_detect_steps)&&(zero))))
				{			
				        Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					Step->state_par = GOTO_ZERO2;
					Step->steps *= 2;	

					//Message_Send_4halfword(0xAACC,((stepno)<<8)|0x01,Step->position,Step->steps);
					
				}
			}
			#endif

			if(Step->st_no_cl.bit.checkisleavezero)
			{
				if(!zero)
				{
					//steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
					Step->st_no_cl.bit.checkisleavezero=0;
					Step->steps = 25;//50/*100*/;
					Step->position = 0;
					//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,0,3);
				
					Step->state_par = LEAVE_STEP;
					
				}
			}

			if((is_HF_ex_LIFT_Step(stepno))&&(Step->steps == 20)&&(Step->HF_ex_cnt))
			{
				Step->steps_go_temp=0;
				Step->state_par = 	LEFT_EX_ISWORK;
				Step->steps <<= 1;
				Step->HF_ex_cnt =0;
			}

				
			if(Step->step_st.bit.running == RUNNING_OVER) 
			{
				unsigned char zero_input_is_same=0; /*默认归零方向和工作方向相反，信号在正方向表示两个方向相同*/

				if(Step->moto_dd_config.DD_type)
				{
					if(Step->moto_dd_config.DD_is_goto_turnaway==1)
					{
						Step->max_speed  =Step->speed;
						Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
						if(Step->low_speed>Step->speed)
							Step->low_speed = Step->speed;
						Step->speed_acc_dec = (Step->max_speed - Step->low_speed)/(Step->steps);
						if(Step->speed_acc_dec>STEPMOTOR_ACC_MAX_HZ_P_STEP)
						{
							Step->speed_acc_dec =STEPMOTOR_ACC_MAX_HZ_P_STEP;
						}
						else
						if(Step->speed_acc_dec<STEPMOTOR_ACC_MIN_HZ_P_STEP)
						{
							Step->speed_acc_dec =STEPMOTOR_ACC_MIN_HZ_P_STEP;
						}
						//Step->steps =STEP_GO_SLOW_SPEED_STEPS;
						Step->moto_dd_config.DD_is_goto_turnaway++;
						//Message_Send_4halfword(0x9f|(stepno<<8),Step->position,Step->moto_dd_config.DD_is_goto_turnaway,Step->running);
	
						break;
					}
					else if(Step->moto_dd_config.DD_is_goto_turnaway==2)
					{
						break;
					}				
						
				}


				if ((is_LIFT_or_Other_Step(stepno)||is_sinker(stepno))&&(!encoder_en))
				{
					char zero_dir=Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir;
					if((((Step->step_st.bit.dir_High)?1:0) == Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)
						&&(Step->state_par ==JUST_RUN_RESET))/*上一个动作是归零*/
						zero_input_is_same =1;	
					
				}
				
				if(encoder_en)
					Encoder_clear_restbit(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);

				#ifndef QL_STEPMOTOR_START_LOW
				#if 0
				if (is_sinker(stepno))
				{
					//Message_Send_4halfword(0x8888,stepno,Step->position,Step->pos_2);
					goto sk_runing_over;
				}
				#endif
				#endif

			#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)	
			if(enable_log_tosend_LX){
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_RUNNING_OVER)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_RUNNING_OVER_2)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] =Step->step_st.bit.dir?1:0;								
				Message_send_log_LX(d[0],d[1],d[2]);
				
			}
			#endif
			
LX_ACT_gotozero_go:
				
				if (Step->step_st.bit.dir) // ++
				{
					//Message_Send_4halfword(0xAACC,((stepno)<<8)|0x01,Step->position,Step->pos_2);
					if (Step->position<=Step->pos_2)
					{
						Step->steps = Step->pos_2 - Step->position;
						Step->step_st.bit.running = RUNNING;
						if((Step->moto_dd_config.DD_type)
							&&(Step->moto_dd_config.DD_is_goto_turnaway))
						{
							break;
						}
						goto checkpos2iszero;
					}
					else
					{
						if(Step->pos_2 == 0)
						{
							Step->steps =0; // Step->position - Step->pos_2;
							//goto	checkpos2iszero;
							Step->step_st.bit.running = RUNNING_END;
							goto do_running_end;
						}
						else
						{
							if(Step->moto_dd_config.DD_Stop_immed)
							{
								Step->steps = 0;	
								Step->moto_dd_config.DD_Stop_immed =0;
							}
							else	
								Step->steps = Step->steps>Step->acc_steps?Step->acc_steps :Step->steps;
						//Step->running=0;
						//isexit =1;
							if(encoder_en)
								Step->steps = 0;
							Step->step_st.bit.running = RUNNING_END;
						//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)10|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						}
					}
				}
				else   // -- 
				{
					if(zero_input_is_same==0)
					{
						if (Step->position>=Step->pos_2)
						{
							Step->steps =Step->position - Step->pos_2;//- Step->position;
							Step->step_st.bit.running = RUNNING;
							if((Step->moto_dd_config.DD_type)
							&&(Step->moto_dd_config.DD_is_goto_turnaway))
							{
								break;
							}
							goto checkpos2iszero;
						}
						else
						{
							if(Step->moto_dd_config.DD_Stop_immed)
							{
								Step->steps = 0;	
								Step->moto_dd_config.DD_Stop_immed =0;
							}
							else	
								Step->steps = Step->steps>Step->acc_steps?Step->acc_steps :Step->steps;
							//Step->running=0;
							//isexit =1;
							if(encoder_en)
								Step->steps = 0;
							Step->step_st.bit.running = RUNNING_END;
							//Message_Send_4halfword(0x9699,stepno,Step->position,Step->pos_2);

						}
					}
					else
					{
						//Step->position *=(-1); 
						if ((Step->position * (-1))>=Step->pos_2)
						{
							if(Step->moto_dd_config.DD_Stop_immed)
							{
								Step->steps = 0;	
								Step->moto_dd_config.DD_Stop_immed =0;
							}
							else	
								Step->steps = Step->steps>Step->acc_steps?Step->acc_steps :Step->steps;
							//Step->running=0;
							//isexit =1;
							if(encoder_en)
								Step->steps = 0;
							Step->step_st.bit.running = RUNNING_END;
							//Message_Send_4halfword(0x9699,stepno,Step->position,Step->pos_2);
							
						}
						else
						{
							Step->steps =Step->pos_2- Step->position*(-1);
							Step->position*=(-1);
							Step->step_st.bit.dir =!Step->step_st.bit.dir;
							Step->step_st.bit.running = RUNNING;
							if((Step->moto_dd_config.DD_type)
							&&(Step->moto_dd_config.DD_is_goto_turnaway))
							{
								break;
							}
							//goto checkpos2iszero;
						}
					}
				}

				//Message_Send_4halfword(0x98|(stepno<<8),Step->step_st.bit.running, Step->state_par,Step->steps);
			
				//Step->steps = 0;
				//Step->step_st.bit.running ==0;
				if ((is_LX_ACTION_Step(stepno)||(is_FH_ACTION_Step(stepno)))&&(Step->chk.bit.chk_st_change))			
				{
					Step->chk.bit.chk_st_change =0;
				}

				
				//goto exit;
			}
			else
				if (Step->step_st.bit.running ==RUNNING_END)
				{
					do_running_end:
					if (Step->steps ==0)
					{

					sk_runing_over :

						Step->steps =0;
						Step->step_st.bit.running=0;
						Step->state_par = 0;
						if (is_HP_ACTION_Step(stepno))
						{
							Step->chk.bit.HP_auto_adj =1;
						}
						//isexit =1;
						//Message_Send_4halfword(0x07FF|(stepno<<12),Step->position,Step->pos_2,0);
						//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)11|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						goto exit;
					}					
				}


			if (Step->step_st.bit.running == RUNNING_GO)
			{
				Step->steps =10;
				Step->step_st.bit.running =1;
				Step->state_par =JUST_RUN_GO;
				goto exit;
			}
		
			}
			 break;	

			 checkpos2iszero:
			 {	
			 	Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];
				if((Step->pos_2==0)&&(!encoder_en))			//如果是去零位的，那么就movetozero
				{
					if (is_LX_ACTION_Step(stepno))
					{
						if (Step->position>0)   //
						{
							if (!zero)
							{
								Step->state_par = MOVETO_ZERO;
								#ifdef DEBUG_ALERT_65
								Step->which_gotozero =16;
								#endif
								 Step->chk.bit.chk_gotozero=1;
								Step->chk.bit.chk_leavezero=0;
							}
							else
							{
								//Step->state_par = LEAVE_ZERO;
								if((StepMotor_LX_ACT_CrossZero)&&(!Step->LX_act_just_move_zero))
								{
									Step->state_par = LEAVE_ZERO;	
								}	
								else
								{
									Step->state_par = JUST_RUN;	
								}
								Step->chk.bit.chk_gotozero=0;
								Step->chk.bit.chk_leavezero=1;
							}
							//Step->pos_2 = pos;
						}
						else
						{
							if (!zero)
							{
								Step->state_par = MOVETO_ZERO;
								#ifdef DEBUG_ALERT_65
								Step->which_gotozero =17;
								#endif
								Step->chk.bit.chk_gotozero=1;
								Step->chk.bit.chk_leavezero=0;
							}
							else
							if (Step->position<-100)
							{
								//Step->change_speed
								if(F_ACT_Motor_MOVE_zero_type)
								{
									if((Step->position<-100)&&(zero))
									{
										Step->change_speed =1;	
									}
								}
								Step->state_par = MOVETO_ZERO_LX_ACT;
								
								Step->chk.bit.chk_gotozero=1;
								Step->chk.bit.chk_leavezero=1;
							//Step->pos_2 = pos;
							}
							else
							{
								if((StepMotor_LX_ACT_CrossZero)&&(!Step->LX_act_just_move_zero))
								{
									Step->state_par = LEAVE_ZERO;	
								}	
								else
								{
									Step->state_par = JUST_RUN;	
								}
								Step->chk.bit.chk_gotozero=0;
								Step->chk.bit.chk_leavezero=0;
							}
								
						}
						//Message_Send_4halfword_debug(0x0909,Step->position,Step->steps,Step->state_par);
					}
					else
						if (is_LX_DENSITY_Step_ex(Step) 
							|| (is_HF_LIFT_Step(stepno))
							||is_EX_OTHER_Step(stepno)
							||is_DOT_ACTION_Step(stepno)
							||is_CX3_Feet_Step(stepno)
							||is_LX_yarn_step(stepno))
						{
							Step->state_par = MOVETO_ZERO;
							#ifdef DEBUG_ALERT_65
							Step->which_gotozero =18;
							#endif		
						}
						else
						{				
							if(Step->position > 0 /*ZERO_DETECT + step_zero_adj*/ || !zero) 
							{
								Step->state_par = MOVETO_ZERO;
								#ifdef DEBUG_ALERT_65
								Step->which_gotozero =19;
								#endif
								//Step->pos_2 = pos;
								//pos = 0;

								//Message_Send_4halfword_debug(0x0909,stepno,0xEEEE,0xffff);
								
							}
							if(Step->position < 0)
							{
								//Step->position -= zero_work_area;
								Step->state_par = LEAVE_ZERO;
								//Step->alarm_step = 1;
								//Step->pos_2 = pos;
								//Message_Send_4halfword_debug(0x0909,stepno,0xDDDD,0xffff);
							}
						}
				}
				break;
			 }	

			 
		case HP_STEP_CHECK_INPUT:
			{
				if(Step->step_st.bit.running == RUNNING_OVER) 
				{
					Step->state_par =JUST_RUN;
				}
				else
				{

					//Message_Send_4halfword(0x99|(stepno<<8),Step->chk.all, Step->chk.all>>16,Step->steps);
				
					if (Triangle_input_is_ok(Step)) //到位了。那么再走一点，就停下来吧
					{
						Step->state_par =JUST_RUN;
						Step->steps = zero_go_;
						Step->chk.bit.HP_check_isok=1;
						Step->position = Step->pos_2+(Step->step_st.bit.dir?(-1):(1))*zero_go_;
						
						
					}
					else
					if (Step->steps==0)
					{
						if (Step->chk.bit.HP_check_isover ==0)  //走完了还没检测到位,那么要继续走几步看看喽
						{
							Step->chk.bit.HP_check_isover =1;
							Step->steps = HP_STEP_MOTOR_CHECK_INPUT_AREA;
						}
						else
						{
							Step->position = Step->pos_2;
							Step->state_par =JUST_RUN;
							//报警
							//errno = 
							//goto alarm_and_exit;

						}					
					}
				}
				
			}			
			break;
		case ENCODER_JUST_RUN:
			if(Step->steps <= Step->acc_steps)
			{
				//Encoder_clear_restbit(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
				Step->state_par = JUST_RUN;
			}
			break;
		case ENCODER_LEAVE_ZERO:
			if (Step->steps==0)  //走完了，那么可以调头了
			{
				//int steps = 200;
				//steps = stepspeed_Q2R(steps);

				Step->steps = STEPS_LEAVEZERO;
				Step->position = STEPS_LEAVEZERO;
				Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
				#ifdef STEP_MOTOR_DDDM_SUPPORT
				Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
				#else
				Step->speed = step_base_speed;
				#endif
				Step->step_st.bit.phase = 0;
				Step->step_st.bit.dir = !Step->step_st.bit.dir;
				Step->step_wait_time = Step->step_reset_delay_time;
				change_dir =1;
				//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
				Step->state_par = ENCODER_JUST_RUN;//JUST_RUN;//
			}
			break;

		case STEP_RESET_HP_ACT:
			{
				switch (Step->state_chi)
				{
					case 0:		//第一次进0位
						{
							if (zero)   //找到0位了，那就继续走几步
							{								
								int steps =Step->step_max;								
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
								Step->state_chi_last = Step->state_chi;
								Step->state_chi = 1; 
								Step->steps_go_  =0;
								//Step->done_steps =1;
							}
							else
							{
								if(is_HP_ACTION_Step(stepno)) 
								{
									#if 0
									if (((isleft)&&(!Step->step_st.bit.dir))||((Step->step_st.bit.dir)&&(isright)))
									{
										unsigned int overstepstemp=0;
										if (Step->steps_go_temp==0)	//说明第一次进
										{
											Step->steps_go_temp=1;	
										}
										else
											Step->steps_go_temp++;

										if (Step->steps_go_temp==Step->steps_go_) //说明一开始就在里面
										{
											overstepstemp = STEP_GO_DIR_STEPS;
										}else
										{
											overstepstemp = STEP_GO_DIR_STEPS_MIN;
										}

										
										if (Step->steps_go_temp>=overstepstemp)
										{
											if (Step->state_chi_last==9)
											{
														errno = 0xB001;
														goto alarm_and_exit;
											}
										
											Step->steps = Step->step_max;//200;
											Step->position = 0;
											Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
											Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
											Step->step_st.bit.phase = 0;
											Step->step_st.bit.dir = !Step->step_st.bit.dir ;
											Step->step_wait_time = step_reset_delay_time[stepno];
											arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
											Step->state_chi_last = Step->state_chi;
											Step->state_chi = 9; 
											//Step->moto_zero_width_self = Step->done_steps;
											//Step->done_steps=0;
											Step->steps_go_ =0;
											Step->steps_go_temp=0;
										}
										
									}
									else 
									#endif	
										{
										Step->steps_go_temp++;

										if (Step->steps_go_temp>=800)
										{

											if (Step->steps_go_temp>=1600)
											{
													#ifdef NEW_ALARM_STYLE
													errno = STEP_ALARM_RESET_ERROR;
													#else
													errno = 1;
													#endif
													goto alarm_and_exit;
											}
											else
											{
										
											Step->steps = Step->step_max;//200;
											Step->position = 0;
											Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
											Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
											Step->step_st.bit.phase = 0;
											Step->step_st.bit.dir = !Step->step_st.bit.dir ;
											Step->step_wait_time = Step->step_reset_delay_time;
											change_dir=1;
											//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
											Step->state_chi_last = Step->state_chi;
											Step->state_chi = 0; 
											//Step->moto_zero_width_self = Step->done_steps;
											//Step->done_steps=0;
											Step->steps_go_ =0;
											Step->steps_go_temp=0;
											Step->change_dir_count++;
												}
										}
										
									}
									
									
									
								}
								//else
								{
									if ((Step->steps==0)||(Step->change_dir_count>1)) //走完了，还没找到0位，那么要报警了
									{
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 1;
										#endif
										goto alarm_and_exit;
									}
								}
							}

						}
						break;

						case 9:
						{
							if ((!zero)&&(!isleft)&&(!isright))   //找到0位了，那就继续走几步
							{
								
								int steps =Step->step_max;	
								
								//steps = stepspeed_Q2R(steps);
								Step->steps = steps/*0*/;
								Step->position = steps/*0*/;
								Step->state_chi_last = Step->state_chi;
								Step->state_chi = 0; 
								Step->steps_go_  =0;
								Step->steps_go_temp=0;
								//Step->done_steps =1;
							}
							else
							{								
								if (Step->steps==0) //走完了，还没找到0位，那么要报警了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 1;
									#endif
									
									goto alarm_and_exit;
								}
								
							}
						}
						break;
						
						case 1:     //初始位置就在0位，那么走走看
						{
						
							if (zero)
							{
								if (Step->steps==0)  //走完了，仍然在0位
								{		
								
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 3;
									#endif
									goto alarm_and_exit;
								}	
							}
							else
							{
								
									if (Step->steps_go_<=STEP_GO_ALARM_STEPS) //才走了一点点，就没零位了
									{
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 2;
										#endif	
										goto alarm_and_exit;
									}else							//说明走完了，那就记录下，zerowide
									{
										int steps = STEPS_LEAVEZERO;																
										steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
										Step->steps = steps/*0*/;
										Step->position = 0/*0*/;	
										Step->state_chi_last = Step->state_chi;	
										Step->state_chi = 3; 
										Step->moto_zero_width_self=Step->steps_go_;
										Step->steps_go_ =0;									
									}
								
								
							}
							
						}
						break;
						case 2:				//在0位，那么准备离开0位
						{
							if (!zero)   //说明离开了。那么记录当前的点位
							{
								
								{
									int steps = STEPS_LEAVEZERO;						
									steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
									Step->steps = steps/*0*/;
									Step->position = 0/*0*/;	
									Step->state_chi_last = Step->state_chi;	
									Step->state_chi = 5; 
									Step->steps_go_ =0;
								}
								
							}
							if (Step->steps==0)   //走完了，还没有离开0位，那有问题了
							{
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_RESET_ERROR;
								#else
								errno = 3;
								#endif
								goto alarm_and_exit;
							}

						}
					
						break;
						case 5:
						{
							if (Step->steps==0)  //走完了，那么可以调头了
							{
								Step->steps = Step->step_max;
								Step->position = Step->step_max;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir ;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
								//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi_last = Step->state_chi;
								Step->state_chi = 0; 
								Step->steps_go_temp=0;
								Step->steps_go_ = 0;
								
							}
							if (zero)  //0位信号突然又有了。那要报警了
							{
								if (Step->steps_go_>=STEP_GO_ALARM_STEPS)		//走开了一会儿了，0位又来了，妖怪了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 4;
									#endif
									goto alarm_and_exit;
								}
							}
						}
						break;
						case 3:				//离开0位
						{
							if (Step->steps==0)  //走完了，那么可以调头了
							{
								Step->steps = Step->step_max;
								Step->position = Step->step_max;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir ;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
								//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi_last = Step->state_chi;
								Step->state_chi = 4; 
								Step->steps_go_ = 0;
								
							}
							if (zero)  //0位信号突然又有了。那要报警了
							{
								if (Step->steps_go_>=STEP_GO_ALARM_STEPS)		//走开了一会儿了，0位又来了，妖怪了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 4;
									#endif
									goto alarm_and_exit;
								}
							}
						}
						break;
					case 4:					//最后从非零状态进入零位状态
						{
							if (zero)		//关键的到了//
							{	
								if (Step->steps_go_<STEP_GO_ALARM_STEPS)    //没走几，0位又来了，妖怪了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 4;
									#endif
									goto alarm_and_exit;	
								}
								else
								{
							
								int steps;				
								//steps = stepspeed_Q2R(steps);
								int x=(Step->step_max-Step->steps) - STEPS_LEAVEZERO;
								if ((x<0)||(x>30))
								{										
									//errno=x<0?((0x6<<12)|((0-x)|0xFFF)):((0x07<<12)|(x&0xfff));
									
									//Step->input_errorstep =0;
								}
								else Step->input_errorstep =x;
								
								
								
								steps =(Step->moto_zero_width_self - Step->input_errorstep)>>1;
								
								Step->steps = steps/*0*/;
								
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
									//走回来的步数			- 走出去的步数
								Step->state_par = JUST_RUN;
								Step->steps_go_ =0;
								}

							}
							else
								{
									if (Step->steps==0)	 //走完了，还没找到0位，那么要报警了
									{
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 5;
										#endif
										goto alarm_and_exit;
									}
								}
							
						}
						break;	
				}				
			}	
			break;	
			
		case STEP_RESET:
			{
				#ifdef DEBUG_STEP_RESET_FAST_SPEED
				unsigned int last_chi=Step->state_chi;

				#endif
				switch (Step->state_chi)
				{

				#ifdef TRY_ZERO_MIDD
					case GOTO_ZERO_TRY_ZERO_MIDD:
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x0F)   //找到0位了，那就继续走几步
							if (zero)
							{
								int steps ;
								

								if (is_HF_LIFT_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_LX_yarn_step(stepno))
								{
										
										steps = 400;
										Step->state_chi = 2; 
										Step->moto_zero_width_self = 0;
										Step->moto_zero_width_temp=0;
										#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
								}
								
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
							}
							else
							{											
								if (Step->steps==0) //走完了，还没找到0位，那么要报警了
								{
									if (Step->steps_last<800)  //说明方向反了，需要调头,调头的时候在原来基础上增加步数
									{
										Step->steps = Step->steps_last+try_steps_to_zero;//GOTO_TRY_ZERO_STEPS;//200;
										Step->steps_last = Step->steps;
										//Step->position = 0;
										Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];
										Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
										
										Step->step_st.bit.phase = 0;
										Step->step_st.bit.dir = !Step->step_st.bit.dir;
										Step->step_wait_time = Step->step_reset_delay_time;
										change_dir=1;
										//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
										Step->state_chi = GOTO_ZERO_TRY_ZERO_MIDD;										
									}
									else				/*说明这里已经超出范围了，那么零位可能没有安装*/
									{									
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 1;
										#endif
										goto alarm_and_exit;
									}
								}
							}
						}
						break;
				#endif
					case GOTO_ZERO_TRY:
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x0F)   //说明找到零位了
							if (zero)
							{
								Step->steps = Step->step_max;
								Step->state_chi =2;
								if (is_LX_DENSITY_Step(stepno)||is_lx_sinker(stepno))
								{
									Step->moto_zero_width_temp=1;
								}
							}
							else
							{
								if (Step->steps == 0)  //试着走走，结果走完了，没找到零位,那要调头了
								{
									//if (is_sinker(stepno))
										
									Step->steps = Step->step_max;//200;									
									Step->position = 0;
									Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
									#ifdef STEP_MOTOR_DDDM_SUPPORT
									Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
									#else
									Step->speed = step_base_speed;
									#endif
									Step->step_st.bit.phase = 0;
									Step->step_st.bit.dir = !Step->step_st.bit.dir;
									Step->step_wait_time = Step->step_reset_delay_time;
									change_dir=1;
									//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
									Step->state_chi = 0;
									#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
								}
							}

						}
						break;
				
					case 0:		//第一次进0位
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x0F)   //找到0位了，那就继续走几步
							if (zero)
							{
								int steps ;

								

								if (is_HF_LIFT_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_LX_yarn_step(stepno))
								{
										steps = 400;
										Step->state_chi = 2; 
										Step->moto_zero_width_self = 0;
										Step->moto_zero_width_temp=1;
								}
								else
								{								
									//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
									if(is_YARN_Step(stepno))
									{
										steps = 400;
										Step->state_chi = 2; 
										Step->moto_zero_width_self = 0;
									}else
									if(is_LIFT_or_Other_Step(stepno))
									{
										steps = zero_go_ ;// 这里一定要注意，穿越的电机，不能用这个值，要用更大的值
										Step->state_chi = JUST_RUN_RESET;
									}
									#if 0
									else if(is_LX_ACTION_Step(stepno))
									{
										steps = 400;
										Step->state_chi = 8; 
									}
									#endif
									else
									{
										steps = zero_go_ ;// 这里一定要注意，穿越的电机，不能用这个值，要用更大的值
										Step->state_chi = 1;
									}
									if (is_LX_DENSITY_Step(stepno) || is_lx_sinker(stepno))
									{
										steps = Step->step_max;
										Step->state_chi =2;
										Step->moto_zero_width_temp=1;										
									}
								}
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;

								#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
								#if 0		
								if(is_LX_ACTION_Step(stepno))
								{
									Message_Send_4halfword((stepno << 8) | 0xEA,Step->steps,Step->moto_zero_width_temp,0|(Step->state_chi<<8));

								}	
								#endif
							}
							else
							{											
								if (Step->steps==0) //走完了，还没找到0位，那么要报警了
								{
									if (is_HF_LIFT_Step(stepno)
										||is_EX_OTHER_Step(stepno)
										||is_DOT_ACTION_Step(stepno)
										||is_CX3_Feet_Step(stepno)
										|| is_sinker(stepno)
										||is_LX_yarn_step(stepno))  //说明方向反了，需要调头
									{
										Step->steps = Step->step_max;//200;
										Step->position = 0;
										Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
										#ifdef STEP_MOTOR_DDDM_SUPPORT
										Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
										#else
										Step->speed = step_base_speed;
										#endif
										Step->step_st.bit.phase = 0;
										Step->step_st.bit.dir = !Step->step_st.bit.dir;
										Step->step_wait_time = Step->step_reset_delay_time;
										change_dir=1;
										//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
										Step->state_chi = 7;	

										#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
									}
									else
									{									
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 1;
										#endif
										goto alarm_and_exit;
									}
								}
							}
						}
						break;
					case 1:
						{
							if (Step->steps==0)  //走完了，那么可以调头了
							{
								//int steps = 200;
								//steps = stepspeed_Q2R(steps);
								#if 0  //20170113
								if (!zero)  //0位信号突然没了。那要报警了
								{	
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 2;
									#endif
									goto alarm_and_exit;
								}
								#endif

								
								Step->steps = Step->step_max;//200;
								Step->position = 0;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								#ifdef STEP_MOTOR_DDDM_SUPPORT
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								#else
								Step->speed = step_base_speed;
								#endif
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
							//	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 2;
							#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
								#if 0		
								if(is_LX_ACTION_Step(stepno))
								{
									Message_Send_4halfword((stepno << 8) | 0xEA,Step->steps,Step->moto_zero_width_temp,1|(Step->state_chi<<8));

								}
								#endif
							}
							
							
						}
						break;
					case 2:
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x10)   //说明离开了。那么记录当前的点位
							if (!zero)
							{
								int steps = STEPS_LEAVEZERO;	
								//#ifdef LX_ACTION_STEP_SPECIAL
								if (is_LX_ACTION_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_HF_LIFT_Step(stepno)
									||is_LX_yarn_step(stepno))
								{
									steps = is_LX_ACTION_Step(stepno)?step_N_act_interval_lx:(is_HF_ex_LIFT_Step(stepno)?STEPS_LEAVEZERO_SMALL:(STEPS_LEAVEZERO>>1));
									if (Step->moto_zero_width_temp)   /*说明计算出了零位宽度*/
									{
										Step->moto_zero_width_self = Step->moto_zero_width_temp;
										steps = (is_HF_ex_LIFT_Step(stepno)?STEPS_LEAVEZERO_SMALL:(STEPS_LEAVEZERO>>1));/*中间缺口位置行程改小*/
									}
								}
								//#endif								
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = 0/*0*/;								
								Step->state_chi = 3; 
								#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
								#if 0		
								if(is_LX_ACTION_Step(stepno))
								{
									Message_Send_4halfword((stepno << 8) | 0xEA,Step->steps,Step->moto_zero_width_temp,2|(Step->state_chi<<8));

								}
								#endif
										
							}
							if (is_LX_DENSITY_Step(stepno)||is_lx_sinker(stepno))
							{
								if (Step->moto_zero_width_temp)
									Step->moto_zero_width_temp++;
							}
							if (Step->steps==0)   //走完了，还没有离开0位，那有问题了
							{

							
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_RESET_ERROR;
								#else
								errno = 3;
								#endif
								goto alarm_and_exit;
							}
							//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
							if(is_YARN_Step(stepno))
							{
								Step->moto_zero_width_self++;
							}
							//#ifdef LX_ACTION_STEP_SPECIAL
							else 
								if ((is_LX_ACTION_Step(stepno) 
									||is_EX_OTHER_Step(stepno)
									||is_HF_LIFT_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_LX_yarn_step(stepno))
									&&(Step->moto_zero_width_temp))
								{
									Step->moto_zero_width_temp++;
								}
							
							//#endif
							
						}
						break;

					case 3:
						{
							//#ifdef LX_ACTION_STEP_SPECIAL
							if (is_LX_ACTION_Step(stepno)
								||is_HF_LIFT_Step(stepno)
								||is_EX_OTHER_Step(stepno)
								||is_DOT_ACTION_Step(stepno)
								||is_CX3_Feet_Step(stepno)
								||is_LX_yarn_step(stepno))
							{	
								//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x0F)  /*说明是从空隙走出来的，并且是走到零位去*/
								if (zero)
								{
									int steps ;
										/*400步*/	
									steps = step_N_act_interval_lx<<2; 
																								
									steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
									Step->steps = steps/*0*/;
									Step->position = 0/*0*/;								
									Step->state_chi = 2; 
									if (Step->moto_zero_width_temp)  /*说明进来过了，那就有问题了，需要报警*/
									{
										#ifdef NEW_ALARM_STYLE
										errno = STEP_ALARM_RESET_ERROR;
										#else
										errno = 4;
										#endif
										goto alarm_and_exit;
									}
									else
									Step->moto_zero_width_temp=1;	/*进入真的零位*/		

									#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
									#if 0
									if(is_LX_ACTION_Step(stepno))
									{
										Message_Send_4halfword((stepno << 8) | 0xEA,Step->steps,Step->moto_zero_width_temp,0x3|(Step->state_chi<<8));

									}
									#endif
									break;
								}
							}						
							//#endif

							
							if (Step->steps==0)  //走完了，那么可以调头了
							{

								if ((zero)&&((is_ACTION_Step(stepno ))||(is_Feet_Step(stepno))))  //0位信号突然又有了。那要报警了
								{
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_RESET_ERROR;
								#else
								errno = 4;
								#endif
								goto alarm_and_exit;
								}

								//int steps = 200;
								//steps = stepspeed_Q2R(steps);
								Step->steps = Step->step_max;
								Step->position = Step->step_max;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								#ifdef STEP_MOTOR_DDDM_SUPPORT
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								#else
								Step->speed = step_base_speed;
								#endif
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
							//	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);

								//#ifdef LX_ACTION_STEP_SPECIAL
								if (is_LX_ACTION_Step(stepno) 
									|| is_HF_LIFT_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_LX_yarn_step(stepno))
								{
									if (Step->moto_zero_width_temp)  /**/
									{
										Step->state_chi = 4;
										if((is_LX_ACTION_Step(stepno))&&(StepMotor_LX_ACT_CrossZero))
										{
											if(Step->step_st.bit.dir)
											{
												Step->moto_zero_width_temp=0;
												Step->state_chi =3;  //
											}
										}
										//Step->max_speed = 1500;
									}
									else
									{
										Step->state_chi =3;  //
									}
								}
								else								
								{
									if (is_LX_DENSITY_Step(stepno) ||is_lx_sinker(stepno))
									{
										if (Step->moto_zero_width_temp)  //说明穿越一次了
										{
											Step->state_chi = 4;
										}
										else
											Step->state_chi =0;
									}
									else
									Step->state_chi = 4;
								}

								#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf1f1,stepno,(Step->state_chi<<8)|last_chi,Step->reset_dir_change_cnt++);
										#endif
										#if 0
								if(is_LX_ACTION_Step(stepno))
								{
									Message_Send_4halfword((stepno << 8) | 0xEA,Step->steps,Step->moto_zero_width_temp,0x33|(Step->state_chi<<8));

								}	
								#endif
								
							}
							else
							{
								if ((zero)&&(!is_ACTION_Step(stepno ))&&(!is_Feet_Step(stepno)))  //0位信号突然又有了。那要报警了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 4;
									#endif
									goto alarm_and_exit;
								}
							}
							
						}
						break;						
					case 4:
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x0F)		//关键的到了//
							if (zero)
							{
								int steps = zero_go_ ;	
								int x;
								steps = stepspeed_Q2R(steps ,Step->moto_remap_config.moto_attr.bit.moto_type_config);

								//#ifdef LX_ACTION_STEP_SPECIAL
								if (is_LX_ACTION_Step(stepno)
									||is_HF_LIFT_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_LX_yarn_step(stepno))
								{
									x= (Step->step_max-Step->steps) - (is_HF_ex_LIFT_Step(stepno)?STEPS_LEAVEZERO_SMALL:(STEPS_LEAVEZERO>>1));//- 6;
									//可正可负

									if ((x<0)||(x>50))
									{										
										//errno=x<0?((0x6<<12)|((0-x)|0xFFF)):((0x07<<12)|(x&0xfff));
										
										//Step->input_errorstep =0;
									}
									else Step->input_errorstep =x;

									//Message_Send_4halfword(0xFFDD,stepno,Step->moto_zero_width_self,x);
									
									steps = (Step->moto_zero_width_self - Step->input_errorstep)>>1;		//回到中心位置

									#ifdef IS_CX_ACT
									if(is_DOT_ACTION_Step(stepno))
									{
										steps = Step->step_st.bit.dir?((steps<<1)-_CX_DOT_ACT_ZERO_ADJ):_CX_DOT_ACT_ZERO_ADJ;
									}
									
									#endif
								
									
									#ifdef LOG_DEBUG_FOR_LX_AT_CH
									if(enable_log_tosend_LX)
									{
										unsigned short d[3];
										d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
										d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
										d[0] |= (LX_LOG_TYPE_ZERO_W)<<8;
										d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
										d[1] =Step->moto_zero_width_self;
										d[2] =Step->input_errorstep;								
										Message_send_log_LX(d[0],d[1],d[2]);
										
									}

									#endif
									
								//	Message_Send_4halfword(0x0908,steps,Step->moto_zero_width_self,Step->input_errorstep);
									if (is_LX_ACTION_Step(stepno))
										steps +=(Step->step_st.bit.dir?(-1):(1))*LX_ACT_zero_adj;
									
									Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;

									#if 0
									if(is_LX_ACTION_Step(stepno))
									{
										Message_Send_4halfword((stepno << 8) | 0xEB,steps,Step->moto_zero_width_self,4|(Step->state_chi<<8));

									}
									#endif
									
								}
								if (is_FH_ACTION_Step(stepno))
								{
									if (!work)
									{
										steps = 4;
										Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
									}
									else
										goto	not_gotozero;		//表示还没有走到零位
										
								}
								else
								{
								//#endif								
								//Step->input_errorstep = //可正可负
								x = (Step->step_max-Step->steps) - STEPS_LEAVEZERO;//-6;
								if ((x<0)||(x>100))
								{										
									//errno=x<0?((0x6<<12)|((0-x)|0xFFF)):((0x07<<12)|(x&0xfff));
									
									//Step->input_errorstep =0;
								}
								else Step->input_errorstep =x;
								
								if (is_LX_DENSITY_Step(stepno)||is_lx_sinker(stepno))
								{
									Step->moto_zero_width_self = Step->moto_zero_width_temp;
									steps = (Step->moto_zero_width_self - Step->input_errorstep)>>1;	
									//Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
								}
								//else									
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
								
								//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
								if(is_YARN_Step(stepno))
								{
									//steps =Step->moto_zero_width;
									if(Yarn_Step_Updown)
									{
										steps = (Step->moto_zero_width_self - Step->input_errorstep) >>1;		//回到中心位置
									}
									else
									{
										steps = Step->moto_zero_width_self - Step->input_errorstep -Step->moto_zero_width;		//回到中心位置
									}
									Step->position = 0-steps/*0*/;
								}
								//#ifdef LX_ACTION_STEP_SPECIAL
								}
								//#endif
								Step->steps = steps/*0*/;
								//Step->position = steps/*0*/;
									//走回来的步数			- 走出去的步数		

								Step->state_par = JUST_RUN_RESET;
								#ifdef DEBUG_STEP_RESET_FAST_SPEED			
										Message_Send_4halfword(0xf2|stepno<<8,Step->moto_zero_width_self|(Step->input_errorstep<<8), steps,Step->reset_dir_change_cnt++);
								#endif

							}
							else
							{
								not_gotozero:
							
								if (Step->steps==0)	 //走完了，还没找到0位，那么要报警了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 5;
									#endif
									goto alarm_and_exit;
								}
							}
						}
						break;
						
					case 5:
						{
							if (!zero)   //说明离开了。那么记录当前的点位
							{
								int steps = STEPS_LEAVEZERO;
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = 0/*0*/;
								Step->state_chi = 6;
							}
							if (Step->steps==0)   //走完了，还没有离开0位，那有问题了
							{
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_RESET_ERROR;
								#else
								errno = 3;
								#endif
								goto alarm_and_exit;
							}
						}
						break;
					case 6:
						{
							if (Step->steps==0)  //走完了，那么可以调头了
							{		
								if (zero)  //0位信号突然又有了。那要报警了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 4;
									#endif
									goto alarm_and_exit;
								}
							
								Step->steps = Step->step_max;
								Step->position = Step->step_max;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								#ifdef STEP_MOTOR_DDDM_SUPPORT
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								#else
								Step->speed = step_base_speed;
								#endif
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
								//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 0;
							}
							
						}
						break;
					case 7:
						{							
							if (zero)   //找到0位了，那就继续走几步
							{
								int steps ;

								if (is_HF_LIFT_Step(stepno)
									||is_EX_OTHER_Step(stepno)
									||is_DOT_ACTION_Step(stepno)
									||is_CX3_Feet_Step(stepno)
									||is_sinker(stepno)
									||is_LX_yarn_step(stepno))
								{
										steps = 400;
										Step->state_chi = 2; 
										Step->moto_zero_width_self = 0;
										Step->moto_zero_width_temp=1;
								}
								else
								{								
									//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
									if(is_YARN_Step(stepno))
									{
										steps = 400;
										Step->state_chi = 2; 
										Step->moto_zero_width_self = 0;
									}
									else
									{
										steps = zero_go_ ;
										Step->state_chi = 1;
									}
								}
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = (Step->step_st.bit.dir?(-1):(1))*steps/*0*/;
							}
							else
							{							
							
								if ((Step->steps==0) ) //走完了，还没找到0位，那么要报警了
								{
									#ifdef NEW_ALARM_STYLE
									errno = STEP_ALARM_RESET_ERROR;
									#else
									errno = 1;
									#endif
									goto alarm_and_exit;
								}
							}					

						}
						break;
						#if 0
					case 8:						
						if (is_LX_ACTION_Step(stepno))
						{
							//if ((Step->chk.bit.last_zero_bit & 0x1F) ==0x10)   //说明穿出了。那么记录当前的点位
							if (!zero)
							{
								int steps = STEPS_LEAVEZERO;	
								//#ifdef LX_ACTION_STEP_SPECIAL
								//if (is_LX_ACTION_Step(stepno)||is_HF_LIFT_Step(stepno))
								{
									steps = STEPS_LEAVEZERO;
									if (Step->moto_zero_width_temp)   /*说明计算出了零位宽度*/
									{
										Step->moto_zero_width_self = Step->moto_zero_width_temp;
										steps = STEPS_LEAVEZERO;
									}
								}
								//#endif								
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = 0/*0*/;								
								Step->state_chi = 3; 
							
							}							
							if (Step->steps==0)   //走完了，那要调头了
							{								
								Step->steps = Step->step_max;
								Step->position = Step->step_max;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								#ifdef STEP_MOTOR_DDDM_SUPPORT
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								#else
								Step->speed = step_base_speed;
								#endif
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir;
								Step->step_wait_time = Step->step_reset_delay_time;
								change_dir=1;
								//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 2;
							}								
						}			
						break;
					#endif	
						
					default:
						break;
				}				
			}
			break;
			 
		case GOTO_ZERO:	
			{
				if(Step->steps == 100) {
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					}
				if(zero) 
				{
					
					int steps = zero_go_ ;
					Step->DD_error_cnt =0;

					if (is_FH_ACTION_Step(stepno))
					{
						if (!work)
						{
							steps = 4;
						}
						else
						goto	gotozero_not_zero;	
						

					}

					
					if (is_HP_ACTION_Step(stepno))
					{
						steps = (Step->moto_zero_width_self - Step->input_errorstep)>>1;
					
					}

					//#ifdef LX_ACTION_STEP_SPECIAL
					if (is_LX_ACTION_Step(stepno)
						||is_HF_LIFT_Step(stepno)
						||is_EX_OTHER_Step(stepno)
						||is_DOT_ACTION_Step(stepno)
						||is_CX3_Feet_Step(stepno)
						||is_LX_yarn_step(stepno))
					{
						steps = zero_go_;
					}
					//#endif
				
					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					#ifdef LOG_DEBUG_FOR_LX_AT_CH
					if ((!0/*(errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS*/)&&(enable_log_tosend_LX))	
					{
						unsigned short d[3];
						d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
						d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
						d[0] |= LX_LOG_TYPE_STOP_GO_2<<8;
						d[0] |= (Step->chk.bit.last_zero_bit & 0x07)<<12;
						d[0] |= 0<<15;
						d[1] =Step->position;
						d[2] =Step->steps;								
						Message_send_log_LX(d[0],d[1],d[2]);
					}
					#endif
						
					Step->steps = steps/*0*/;
					Step->position = Step->step_st.bit.dir?(0-steps):steps;
					Step->state_par = JUST_RUN_AFTER_ADJ;

					//Message_Send_4halfword(0x9799,stepno,Step->position,Step->pos_2);

					#ifdef LOG_DEBUG_FOR_LX_AT_CH
					if ((!0/*(errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS*/)&&(enable_log_tosend_LX))	
					{
						unsigned short d[3];
						d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
						d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
						d[0] |= LX_LOG_TYPE_STOP_GO_2<<8;
						d[0] |= (Step->chk.bit.last_zero_bit & 0x07)<<12;
						d[0] |= 1<<15;
						d[1] =Step->position;
						d[2] =Step->steps;								
						Message_send_log_LX(d[0],d[1],d[2]);
					}
					#endif
					


					if (is_LIFT_Step(stepno) && (Step->st_no_cl.bit.check_is_first))
					{
						step_alert_detect_setup_local <<=1;
						Step->st_no_cl.bit.check_is_first =0;
					}

					//#ifdef LX_ACTION_STEP_SPECIAL
					if (is_LX_ACTION_Step(stepno)
						||is_HF_LIFT_Step(stepno)
						||is_EX_OTHER_Step(stepno)
						||is_DOT_ACTION_Step(stepno)
						||is_CX3_Feet_Step(stepno)
						||is_LX_yarn_step(stepno))
					{
						if ((step_alert_detect_setup_local)&&(Step->alarm_step))
						{
							if (Step->alarm_step-1+zero_go_>step_alert_detect_setup_local)
							{
							
								//errno = 2	;//上一次多走了step_alert_detect_setup这么些
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_LOST_NOW;
								#else
								errno = 2;
								#endif
								#ifdef STEP_ALARM_SEND_LOSTSTEPS
								#ifdef NEW_ALARM_STYLE
								errno = STEP_ALARM_LOST_NOW |(Step->alarm_step-1 +zero_go_ );//加入了失步的值
								#else	
								errno = (errno<<12)|(Step->alarm_step-1 +zero_go_ );//加入了失步的值
								#endif		
								#endif
							}
						}
					}
					else						
					//#endif				
					if ((step_alert_detect_setup_local)&&(Step->alarm_step)&&(!is_HP_ACTION_Step(stepno))&&(!is_Feet_Step(stepno)))
					{
						if (Step->alarm_step-1+zero_go_ >step_alert_detect_setup_local)
						{
						
						//errno = 2	;//上一次多走了step_alert_detect_setup这么些
						#ifdef NEW_ALARM_STYLE
						errno = STEP_ALARM_LOST_NOW;
						#else
						errno = 2;
						#endif
						#ifdef STEP_ALARM_SEND_LOSTSTEPS
						#ifdef NEW_ALARM_STYLE
						errno = STEP_ALARM_LOST_NOW |(Step->alarm_step-1 +zero_go_ );//加入了失步的值
						#else	
						errno = (errno<<12)|(Step->alarm_step-1 +zero_go_ );//加入了失步的值
						#endif		
						#endif
						
						//可以自动纠正
						}
					}
					

					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) && (Step->step_st.bit.zero2_count==0) )
					{
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|0x0004);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)3|((unsigned int)Step->st_no_cl.bit.return_byte<<16));  //doing...
						#endif
					}
					#endif
					
		
				}
				else
				{

					gotozero_not_zero:
					if (Step->steps == 0)  //走完了都还没找到0位那说明0位传感器异常
					{
						//errno = 1;			//和复位过程中的错误1 一样，//无法自动纠�
						#ifdef NEW_ALARM_STYLE
						errno = STEP_ALARM_ZERO_INVALID;
						#else
						//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,3,Step->moto_remap_config.moto_attr.bit.moto_type_exe);
						errno =65; // 1; 101--gotozero not ok 
						#endif

						if(stepno<=3)
						{
							Step->DD_error_cnt++;

							if(Step->DD_error_cnt>=STEP_MOTOT_CHECK_ZERO_DELAY_ALERTCNT)
							{
								goto alarm_and_exit;
							}
							else
							{
								errno =0;
								Step->state_par=0;
								goto DO_DD_FILTER_0_8;	
							}
						}
						
						#ifdef DEBUG_ALERT_65
							//Step->which_gotozero=Step->state_par;
						Message_Send_4halfword(0xf9,Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config,Step->which_gotozero);
      						#endif
						goto alarm_and_exit;
					}
					if(Step->alarm_step) 
					{
						Step->alarm_step ++;				
					}
					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno)&&is_PT_ACTION_Step(stepno)&&(Step->alarm_step == step_alert_detect_setup_local) &&(step_alert_detect_setup_local) )
					{
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_LOST_NOW|(Step->alarm_step &0x0FFF));
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)7|((unsigned int)Step->st_no_cl.bit.return_byte<<16));//doing...
						#endif
						
					}
					#endif
					
				}			
			}
			break;
		case MOVETO_ZERO_LIFT:
			{
				if (Step->steps == 0) /*走完了，需要考虑零位状态*/
				{
					if (zero)
					{
						//
					}
					else			/*走完了，但是不在零位，需要走回去*/
					{
						int steps = 200;
						steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

						Step->steps = steps;//200;//Step->step_max;
						Step->step_st.bit.dir = !Step->step_st.bit.dir;
						Step->position = Step->step_st.bit.dir?(0-steps):steps;//200;//Step->step_max;

						Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
						#ifdef STEP_MOTOR_DDDM_SUPPORT
						Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
						#else
						Step->speed = step_base_speed;
						#endif
						Step->step_st.bit.phase = 0;						
						Step->step_wait_time = Step->step_reset_delay_time;
						change_dir=1;
						//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
						//Step->state_par = GOTO_ZERO;
						#ifdef DEBUG_ALERT_65
						//Step->which_gotozero=Step->state_par;
      						#endif
						//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,0,0);
						Step->state_par = GOTO_ZERO;
						
						if(Step->step_st.bit.phase == 2) {
							Step->step_st.bit.phase = 0;
						}
						
					}
				}
			}
			break;
			
		case MOVETO_ZERO:
			{
				if(is_ACTION_Step(stepno))
				{
					if(F_ACT_Motor_MOVE_zero_type)
					{
						if((Step->position ==(-150))&&(Step->is_stop_wait==0))
						{
							Step->is_stop_wait=1;
							Step->step_wait_time = 10;
						}
						else
						{
							if(Step->is_stop_wait)
							{
								Step->max_speed = 3500;//Step->max_speed_back;
								Step->speed = Step->low_speed;
								Step->is_stop_wait =0;
								
								Step->change_speed =0;		
								Step->max_speed_back =0;
							
							}
						}							
					}
				}
			}
			
			if(zero) {
					int steps;
					Step->alert_delay_zerolost_cnt =0;
					Step->DD_error_cnt =0;
					yes_getzero_and_do:

					#ifdef DMSTEP_MOVERTOZERO_DEC_ADV

					if (Step->steps<=SLOWSPEED_STEPS)
					
						Step->max_speed = Step->low_speed;			//改成启动频率
					
					#endif

					if (is_FH_ACTION_Step(stepno))
					{
						if (!work)
						{
							steps = 4;
						}
						else
						goto move_not_true_zero;	
						

					}

					if (is_LIFT_Step(stepno) && (Step->st_no_cl.bit.check_is_first))
					{
						step_alert_detect_setup_local <<=1;
						Step->st_no_cl.bit.check_is_first =0;
					}
	check_step_islost:
					{
							if (Step->moto_remap_config.moto_attr.bit.is_fast_mode)
							{
								step_alert_detect_setup_local=0;
							}
							if (step_alert_detect_setup_local)
							{
								if (Step->steps>=zero_go_ )
								{
									if (Step->steps - zero_go_  >step_alert_detect_setup_local)
									{
										errno = 3;				/*主控显示-XXX*/
										//errno = 3	;//上一次失步了step_alert_detect_setup这么些
										#ifdef STEP_ALARM_SEND_LOSTSTEPS										
										errno = (errno<<12)|(Step->steps - zero_go_ );//加入了失步的值
										#endif
									}
								}
								else
								{
									if (zero_go_ - Step->steps  >step_alert_detect_setup_local)
									{										
										/*主控显示+XXX*/
										errno =  2;//上一次多走了step_alert_detect_setup这么些
										#ifdef STEP_ALARM_SEND_LOSTSTEPS										
										errno = (errno<<12)|(zero_go_-Step->steps);//加入了失步的值
										#endif										
									}
								}
									
							}
							steps = zero_go_;
					}
				#if 0

					//#ifdef LX_ACTION_STEP_SPECIAL
					if(is_LX_ACTION_Step(stepno)||is_HF_LIFT_Step(stepno))
					{						
						steps = zero_go_;
					}
					else
					{
						if (!is_HP_ACTION_Step(stepno)&&(!is_YARN_Step(stepno))&&(!is_Feet_Step(stepno)))					
						{		
							
						}					
					}

				#endif

					if(is_lx_sinker(stepno)||is_lxex_sinker(stepno)) //连兴生克这里不报警
					{
						if(errno)
							errno=0;
					}

					if (is_YARN_Step(stepno))
					{
						if (!Step->step_st.bit.dir)
						{
							if(Yarn_Step_Updown)
							{
								steps = (Step->moto_zero_width_self - Step->input_errorstep)>>1;	
							}
							else
							{							
								steps = Step->moto_zero_width;
							}
							steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
							Step->position = steps/*0*/;
							Step->steps =steps;
							
						}
						else
						{		
							if(Yarn_Step_Updown)
							{
								steps = (Step->moto_zero_width_self - Step->input_errorstep)>>1;	
							}
							else
							{
								steps = Step->moto_zero_width_self - Step->input_errorstep -Step->moto_zero_width;
							}
							if (steps<0)
							{
								steps *= -1;								

								#ifdef NEW_ALARM_STYLE
								errno =STEP_ALARM_OTHER_ERROR|(steps);//加入了失步的值
								#else
								errno = (6<<12)|(steps);//加入了失步的值
								#endif
								steps = 2;
								//steps *= -1;
							}
							steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
							
							Step->position = 0-steps/*0*/;
							Step->steps =steps;
							
						}
						//Step->steps = Step->position;  //steps/*0*/;
					}
					else
						{

				#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if ((!0/*(errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS*/)&&(enable_log_tosend_LX))	
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= LX_LOG_TYPE_ZERO_ADJ_GOTO<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
			}
			#endif
						
							steps = zero_go_;
							steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
							Step->position =Step->step_st.bit.dir?(0-steps):steps;
							Step->steps = steps/*0*/;

			#ifdef LOG_DEBUG_FOR_LX_AT_CH
			if ((!0/*(errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS*/)&&(enable_log_tosend_LX))	
			{
				unsigned short d[3];
				//unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= LX_LOG_TYPE_ZERO_ADJ_GOTO_2<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->pos_2;
				d[2] =Step->position;								
				Message_send_log_LX(d[0],d[1],d[2]);
			}
			#endif
	
						}
					//Step->steps = steps/*0*/;					
					
					Step->state_par = JUST_RUN_AFTER_ADJ;
					//Message_Send_4halfword(0x08FF|(stepno<<12),Step->position,Step->steps,Step->state_par);
					//Message_Send_4halfword(0x09FF|(stepno<<12),Step->pos_2,Step->step_st.bit.dir,Step->step_st.bit.running);
					
					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno) && is_PT_ACTION_Step(stepno) && (Step->step_st.bit.zero2_count==0) )
					{
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_OTHER_ERROR|0x0004);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)(3 + 10)|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						#endif
					}
					#endif					
				
			}
			else {

				move_not_true_zero:	
					#ifdef DMSTEP_MOVERTOZERO_DEC_ADV
					if ((Step->steps <=step_movetozero_adv[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]) )
					{
						Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					}
					#endif

					
				
				if(Step->steps == 0/*ACC_STEP*/) {//说明上一次走多了，这次提前结束了。至于是否要报警，那还要看下面到底还有多少步到达0位
					int steps = 400;

			#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)	
			if(enable_log_tosend_LX)
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_MOVETOZERO_ADD400)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->steps;								
				Message_send_log_LX(d[0],d[1],d[2]);
				
			}
			#endif
	
						
					Step->alert_delay_zerolost_cnt++;
					if (Step->alert_delay_zerolost_cnt<Step->alert_delay_max)
					{
						goto yes_getzero_and_do;
					}
					Step->alert_delay_zerolost_cnt=0;

					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					Step->steps = steps;//200;//Step->step_max;
					Step->position = Step->step_st.bit.dir?(0-steps):steps;//200;//Step->step_max;

					//if(step_alert_detect)
					Step->alarm_step = 1/*step_alert_detect*//*ZERO_DETECT*//*5*/;

			//		Message_Send_4halfword((stepno << 8) | 0xEF,Step->position,zero,Step->state_par);

					
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
				       #ifdef DEBUG_ALERT_65
					//Step->which_gotozero=Step->state_par;
      					#endif
						//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,0,0);
						
					Step->state_par = GOTO_ZERO;
					if(Step->step_st.bit.phase == 2) {
						Step->step_st.bit.phase = 0;
					}
				}
			}
			break;
	//	#ifdef LX_ACTION_STEP_SPECIAL

		case MOVETO_ZERO_LX_ACT_CROSS_ZERO1:
			{
				if(zero)
				{
					if(Step->steps_go_temp++>4)
					{
						Step->state_par = MOVETO_ZERO_LX_ACT_CROSS_ZERO2;
						Step->steps =400;
						Step->steps_go_temp =0;
					}	
				
				}
				else /*alert */
				if(Step->steps==0)
				{
					errno = 41;						
					goto alarm_and_exit;
				}
			}
			break;
		case MOVETO_ZERO_LX_ACT_CROSS_ZERO2:
			{
				if(!zero)
				{
					if(Step->steps_go_temp++>4)
					{
						Step->state_par = MOVETO_ZERO_LX_ACT_CROSS_ZERO3;
						Step->steps =20;
						Step->steps_go_temp =0;
					}		
				
				}
				else /*alert */
				if(Step->steps==0)
				{
					errno = 42;						
					goto alarm_and_exit;
				}
			}
			break;
		case MOVETO_ZERO_LX_ACT_CROSS_ZERO3:
			{
				//if(!zero)
				if(Step->steps==0)
				{
					Step->steps =200;
					Step->position = 200;
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					#ifdef STEP_MOTOR_DDDM_SUPPORT
					Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
					#else
					Step->speed = step_base_speed;
					#endif
					Step->step_st.bit.phase = 0;
					Step->step_st.bit.dir = !Step->step_st.bit.dir;
					Step->alarm_step =0;
					Step->step_wait_time = Step->step_reset_delay_time;
					change_dir=1;
					//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);
					#ifdef DEBUG_ALERT_65
				//	Step->which_gotozero=Step->state_par;
	      				#endif	
					Step->state_par = MOVETO_ZERO;		
				
				}				
				
			}
			break;
			
		case MOVETO_ZERO_LX_ACT:
			{
				if (!zero)
				{
					if((StepMotor_LX_ACT_CrossZero)
						&&(!Step->LX_act_just_move_zero))
					{
						if(Step->steps_go_temp++>4)
						{
							Step->state_par = MOVETO_ZERO_LX_ACT_CROSS_ZERO1;
							Step->steps =400;
							Step->steps_go_temp =0;
						}
					}
					else
					{
						Step->state_par = MOVETO_ZERO;
						#ifdef DEBUG_ALERT_65
						Step->which_gotozero =20;
						#endif
					}
					if(F_ACT_Motor_MOVE_zero_type)
					{
						if(Step->change_speed)
						{
							Step->max_speed_back = Step->max_speed;
							Step->max_speed = Step->low_speed+1;
						}
					}
					
				}
				else
				if(Step->steps == 0/*ACC_STEP*/) {//说明上一次走多了，这次提前结束了。至于是否要报警，那还要看下面到底还有多少步到达0位
						#ifdef NEW_ALARM_STYLE
						errno = STEP_ALARM_ZERO_INVALID;
						#else
						errno = 21;
						#endif
						goto alarm_and_exit;
				}
				
			}
			break;
	//	#endif	
		case LEAVE_ZERO:			
			if(!zero) {
				int steps =(is_HF_ex_LIFT_Step(stepno)?STEPS_LEAVEZERO_SMALL:STEPS_LEAVEZERO);	
			
				#ifndef NEW_ALARM_STYLE

				
				if(Step->alarm_step) //那说明没离开0位的时候就已经走完步数了
				{
					if (Step->alarm_step>(zero_go_ +Step->input_errorstep)) //上次走多了
					{
						if ((Step->alarm_step -(zero_go_+Step->input_errorstep) >step_alert_detect_setup_local)&&(step_alert_detect_setup_local))
						{
							errno =10;//上一次走到负方向的时候走多了，可纠正
						}
						
					}
					else				// 上一次失步了
					{
						if (((zero_go_ +Step->input_errorstep)-Step->alarm_step >step_alert_detect_setup_local)&&(step_alert_detect_setup_local))
						{
							errno =11;//上一次走到负方向的时候走少了，可纠正
						}
					}

				}
				else		//离开了0位还没走完
				{
					if ((Step->steps +(zero_go_ +Step->input_errorstep) >step_alert_detect_setup_local)&&(step_alert_detect_setup_local))
					{
						errno =12;//上一次走到负方向的时候走少了，可纠正，少了很多
					}

				}
				#endif

				#ifdef LX_SET_

				if (is_sinker(stepno))
				{
					steps =10;
				}
				//else	

				#endif

				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

				Step->steps = steps;//50/*100*/;
				Step->position = 0;
				//Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,0,3);
					
				Step->state_par = LEAVE_STEP;

		
			}
			else
			{
				if (Step->steps==0) // 说明走完了还没离开0位 那么上一次有可能走多了，要不要报警，要看下面的
				{
					int steps = 200;

					#ifdef LX_SET_
					if (is_sinker(stepno))
					{
						Step->state_par =0;
					}
					else	
					{
					#endif

					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					Step->steps = steps;//200;//Step->step_max;
					Step->position = (Step->step_st.bit.dir?(-1):(1))*steps;//200;//Step->step_max;					
					Step->alarm_step = 1;
				//	Message_Send_4halfword((stepno << 8) | 0xEF,Step->position,zero|(1<<1),Step->state_par);

					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					#ifdef LX_SET_
					}
					#endif
				}
				if(Step->alarm_step) 
				{
					Step->alarm_step ++;
					if (Step->steps==0)   //又走完了，还没离开0位 ,那对不起，要报警了 不可纠正
					{
						#ifdef NEW_ALARM_STYLE
						errno = STEP_ALARM_ZERO_VALID;
						#else
						errno= 9;
						#endif
						goto alarm_and_exit;
					}
				}

			}
			break;		
		case LEAVE_STEP:
			if(Step->steps == 0) {
				//int steps = 200;
				//steps = stepspeed_Q2R(steps);
				Step->steps =Step->step_max;
				Step->position = Step->step_max;
				Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
				#ifdef STEP_MOTOR_DDDM_SUPPORT
				Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
				#else
				Step->speed = step_base_speed;
				#endif
				Step->step_st.bit.phase = 0;
				Step->step_st.bit.dir = !Step->step_st.bit.dir;
				Step->alarm_step =0;
				Step->step_wait_time = Step->step_reset_delay_time;
				change_dir=1;
				//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);
				#ifdef DEBUG_ALERT_65
			//	Step->which_gotozero=Step->state_par;
      				#endif
				//	Message_Send_4halfword(0x98|(stepno<<8),Step->state_par,0,0);
						
				Step->state_par = GOTO_ZERO;
				//Message_Send_4halfword(0x9899,stepno,Step->position,Step->pos_2);
			
			}
			break;
		case LEFT_EX_ISWORK:
		{
			if(!zero)
			{
				if(Step->steps_go_temp++ > 1) 
				{
					Step->state_par = JUST_RUN;						
					
					Step->steps =2;   /*20190226 by hlc 加大到10步*/
					Step->pos_2 = Step->position+(Step->step_st.bit.dir?1:(-1))*(Step->steps);
				}
			}
			else
			{
				
				if(Step->steps == 0)
				{
					Step->pos_2 = Step->position;
				}
			}
		}
		break;
		case FEET_STEP_ISWORK:
		{		
			#if 1
			switch(Step->moto_remap_config.moto_attr.bit.moto_type_exe)
			{
				case FEET_MOTOR_TYPE_PT:	
				{
					if(work) 				
					{
						if(Step->steps_go_temp++ > 4) 
						{
							Step->state_par = JUST_RUN;						
							
							Step->steps =4;   /*20190226 by hlc 加大到10步*/
							Step->pos_2 = Step->position+(Step->step_st.bit.dir?1:(-1))*(Step->steps);
						}
						//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)8|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
					
					}	
				}
				break;
				case FEET_MOTOR_TYPE_CX:
				{

					//
					if(Step->steps<=10)
					{
						Step->state_par = JUST_RUN;
						//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)9|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
					}
					#if 0
					if(Step->step_st.bit.running==RUNNING_OVER)
					{
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)9|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						Step->step_st.bit.running =RUNNING;	
					}
					#endif
				}
				break;
			}

			#endif
		}
		break;
	#ifdef ZERO2_SUPPORT
		case DETECT_ZERO2:			//	dengxia
			{
				unsigned char whichst;
				Step->step_wait_time = 1;

				if(Step->position<=4)
				{
					whichst = ZeroPos_workST;
				}
				else
					whichst = WorkPos_workST;
			if((work == whichst) && zero && !Step->step_st.bit.dir) {
				//Step->steps = 0;
				//break;
				int steps = 4; ///4/*step_zero_adj*/;
				if(is_CX3_Feet_Step(stepno))
				{
					steps =zero_go_;
				}
				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

				// by xhl 2013/03/21
				//if(zero) {
					//steps = 0;
				//}
				Step->steps = steps/*0*/;
				Step->step_st.bit.dir = 2;
			}
			if((Step->steps == 0) && (work!=ZeroPos_workST) && (Step->step_st.bit.dir != 2)) {
				if(!Step->step_st.bit.dir) break;
				//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
				//Step->step_st.bit.dir = 1;
				change_dir=1;
				//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
				Step->step_st.bit.dir = 0;
				Step->steps = step_setup_zero2_detect_steps_ex<<1;
				Step->step_wait_time = 10;
			}
			if((Step->steps == 0) && (Step->step_st.bit.dir == 2)) {
				Step->position =0;
				Step->steps = 0;
				break;
			}
			break;
			}
		case GOTO_ZERO3:
			{
				
			}
			break;
		case GOTO_ZERO2:
			#if 1
			if(zero) {
				Step->position = 0;
				Step->state_par = 0;
				Step->steps = 0;
				Step->pos_2 = 0;
				break;
			}
			//if((is_FH_ACTION_Step(stepno)&&(zero)&&(!work))||(zero))
			if((zero)&&(is_PT_ACTION_Step(stepno)||is_CX3_Feet_Step(stepno))) 
			{
				int steps =zero_go_;   ////4;//16/*step_zero_adj*/;
				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
				
				Step->steps = steps/*0*/;
				//Step->state_par = JUST_RUN;
				Step->pos_2 = 0;
				if(Step->step_st.bit.dir) {
					//Step->pos_2 = Step->position + steps;
					Step->position = Step->pos_2 - steps;
				}
				else {
					//Step->pos_2 = Step->position - steps;
					Step->position = Step->pos_2 + steps;
				}	
				
				Step->state_par = GOTO_ZERO3;
				
				break;
			}
			#endif
			//if(!arch_StepMotor_Zero(stepno + 2)) { }
			if((work==WorkPos_workST) && ((Step->position > (step_setup_zero2_detect_steps ))||(Step->position <(0- (step_setup_zero2_detect_steps ))))) 
			{
				int steps =4; //zero_go_;   ////4;//16/*step_zero_adj*/;

				if(is_CX3_Feet_Step(stepno))
				{
					steps =zero_go_; 
				}
				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
				
				Step->steps = steps/*0*/;
				Step->state_par = JUST_RUN_AFTER_ADJ;

				#if 0
				if(Step->step_st.bit.dir) {
					Step->pos_2 = Step->position + steps;
				}
				else {
					Step->pos_2 = Step->position - steps;
				}

				#endif

				if(Step->step_st.bit.dir) {
					//Step->pos_2 = Step->position + steps;
					Step->position = Step->pos_2 - steps;
				}
				else {
					//Step->pos_2 = Step->position - steps;
					Step->position = Step->pos_2 + steps;
				}
				//Message_Send_4halfword(0xAABB,((stepno)<<8)|0x01,Step->position,Step->steps);
			}
			break;
	#endif
		default:
			break;
		}

		if (errno)
		{
			if((is_DENSITY_Step(stepno))
				&&(Step->step_st.bit.IS_Reset_ex ==1)
				&&(Step->isdoing_with_bind_cmd))
			{
				
			}
			else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
			errno =0;
		}

DO_DD_FILTER_0_8:
			
		if(Step->steps > 0) {

#ifdef STEP_MOTOR_DDDM_SUPPORT

		if(Step->moto_dd_config.DD_type==0)
		{
			//speed = StepMotor_calc_speed(stepno);	
			speed = StepMotor_calc_speed(stepno);
		}
		else
		{
			do_DD_speed:
			speed = StepMotor_calc_speed_DD(stepno);
		}

		

		
		
#else

		switch (Step->moto_remap_config.moto_attr.bit.moto_type_config)
		{
			case MOTOR_TYPE_DENSITY:
					speed = Step_pulse[Step->speed];
				break;
			case MOTOR_TYPE_SKINER:
					speed = Step_pulse_sinker[Step->speed];
				break;
			case MOTOR_TYPE_ACTION:
					speed = Step_pulse_triangle[Step->speed];
				break;
			case MOTOR_TYPE_FEET:
					speed = Step_pulse[Step->speed];
				break;

			case MOTOR_TYPE_LIFT:
					speed = Step_pulse_lift[Step->speed];
				break;
		}


		switch(Step->step_st.bit.phase) {
		case 0:	// Accelerate
			
			Step->speed ++;
			if(Step->speed >= ACC_STEP) {
				Step->speed = ACC_STEP;
				Step->step_st.bit.phase ++;
			}
			else if(Step->steps < ACC_STEP) {
				Step->step_st.bit.phase ++;
			}
			break;
		case 1: // Isokinetic
			if(Step->steps < ACC_STEP) {
				Step->step_st.bit.phase ++;
			}
			break;
		case 2: // Decelerate
			if(Step->speed > 0)
				Step->speed --;
			break;
		default:
			break;
		}
#ifdef STEPACC_DIV_DEBUG		
set_speed:
#endif
		// by xhl 2011/04/15
		speed += Step->max_speed;

		if ((step_run_mode)&&(Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION))
		{
			speed = stepspeed_R2Q(speed,Step->moto_remap_config.moto_attr.bit.moto_type_config);
		}
		
#endif
		arch_StepMotor_Set_Speed(stepno, speed);

		{
			if((Step->moto_dd_config.DD_Speed_dir==0)?(speed>Step->moto_dd_config.DD_speed_hz_max):(speed<Step->moto_dd_config.DD_speed_hz_max))
			{
				Step->moto_dd_config.DD_speed_hz_max =speed;
				Step->moto_dd_config.DD_speed_max_pos = Step->position;
			}
			Step->moto_dd_config.DD_speed_hz_max =speed;
		}

	}
	else {

		if(Step->moto_dd_config.DD_type)
		{
			if(Step->moto_dd_config.DD_is_goto_turnaway)
			{
				Step->moto_dd_config.DD_is_goto_turnaway=3;
				Message_Send_4halfword(0xF5|(stepno<<8),Step->moto_dd_config.DD_Target_POS, Step->moto_dd_config.DD_cur_cnt_target_pos, Step->position);
			}
			else
			{
				if(Step->position!=Step->moto_dd_config.DD_Target_POS)
				{
					 if(!Step->moto_dd_config.DD_Step_overflow_flag)
					 {
						Step->steps=8;

						if(Step->step_st.bit.dir)/*++*/
					 	{
					 		int t_pos = Step->moto_dd_config.DD_Target_POS - Step->position;
							if(t_pos<0)
							{
								Step->steps= 0;
								Message_Send_4halfword(0xAE|(stepno<<8),Step->moto_dd_config.DD_Target_POS,Step->position,1);
							}
							else
							{
								if (Step->steps > t_pos)
								{
									Step->steps = t_pos;
								}
							}
						}
						else /*--*/
						{
							int t_pos = Step->position - Step->moto_dd_config.DD_Target_POS ;
							if(t_pos<0)
							{
								Step->steps= 0;
								Message_Send_4halfword(0xAE|(stepno<<8),Step->moto_dd_config.DD_Target_POS,Step->position,2);
							}
							else
							{
								if (Step->steps > t_pos)
								{
									Step->steps = t_pos;
								}
							}
						}
						
						Step->moto_dd_config.DD_Step_overflow_flag =1;
						goto do_DD_speed;
					 }	
				}
				else
				if((Step->moto_dd_config.DD_cur_cnt_target_pos==Step->moto_dd_config.DD_Target_POS)
					||(Step->moto_dd_config.DD_cmd_CNT == Step->moto_dd_config.DD_cnt_cur))
				{
					Step->moto_dd_config.DD_type =0;/*标志清零*/
				}
			}
			//Step->moto_dd_config.DD_Step_overflow_flag =0;
		}
		
		Step->step_st.bit.running = 0;

		arch_StepMotor_Stop(stepno);
		#ifdef MOTOR_FULL_CURR_OUTPUT
		if(is_DENSITY_Step(stepno))
		{
			arch_StepMotor_Half(stepno ,MOTOR_FULL_CURR_OUTPUT);//半流
		}
		else			
		#endif
		arch_StepMotor_Half(stepno, MOTOR_HALF_OUTPUT);
		//Step->Base_t_250us = arch_get_ticktime();

		Step->last_stop_systick = arch_get_ticktime();
		Step->step_st.bit.last_dir = Step->step_st.bit.dir;

		if (!encoder_en)
		{
			StepMotor_afterrun_toreport(stepno,0);
		}

		Yarn_motor_input_bits_send(stepno);

		if (is_ACTION_Step(stepno) && act_check_enable)
		{
			Step->chk.bit.check_sts_enable =1;	
		}
		//#ifdef LX_ACTION_STEP_SPECIAL
		if (is_LX_ACTION_Step(stepno) 
			||is_HF_LIFT_Step(stepno)
			||is_FH_ACTION_Step(stepno)
			||is_EX_OTHER_Step(stepno)
			||is_DOT_ACTION_Step(stepno)
			||is_CX3_Feet_Step(stepno)
			||is_LX_yarn_step(stepno)
			||(is_PT_ACTION_Step(stepno)&&!is_zero2_support(stepno)))
		{
			Step->chk.bit.check_sts_enable =1;
			Step->step_st.bit.check_delay_count =( is_LX_yarn_step(stepno)?(0>>1):20)+15;            //35.. 最大63，6个bit位

			if(is_LX_yarn_step(stepno))
			{
				if((Step->position<=(0-zero_go_))
					&&(zero ||(Step->chk.bit.chk_st_change)))
				{
					Step->chk.bit.check_sts_enable=1;
					Step->step_st.bit.check_delay_count =35;
				}
				else
				{
					Step->chk.bit.check_sts_enable=0;
					Step->step_st.bit.check_delay_count =0;
				}
			}
			//Step->check_zero_work_Timer_enable =0;
			//Message_Send_4halfword(0x0104,((stepno)<<8)|0x01,Step->st_no_cl.bit.check_zero_work,0XFBFB);
			goto FH_act_step_ok;
		}
	//	#endif

		if (Step->step_debug_test_st.bit.justrun_test & 0x01)
		{
			if (Step->step_debug_test_st.bit.justrun_test>>1)
			{
				Message_Send_4halfword((0x03 << 8) | 0x0F,(stepno),Step->position,0);
			}
			return;
		}

		#ifdef STEP_MOTOR_DDDM_SUPPORT
		
		if(Step->step_st.bit.step_flags) 
		{			
			void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
			#ifdef STEP_DEBUG_DDM
			Message_Send_4halfword((0x08 << 8) | 0x03,StepMotor_get_IDall_with_no(stepno), Step->position,Step->step_last_postion);
			Step->step_last_postion =0;
			#else
				Message_Send_4halfword((0x08 << 8) | 0x03,StepMotor_get_IDall_with_no(stepno),Step->position,Step->moto_dd_config.DD_speed_hz_max);
			#endif
			Step->step_st.bit.step_flags =0;
		}
		Step->step_is_cnt =0;
		Step->step_is_cnt_old =0;

		//Message_send_log_ecode((0x03 << 8) | 0x0F,(stepno),Step->position);
		#endif

		


		if (((step_work_sign_alarmenable>>Step->moto_remap_config.moto_remap_id_all) & 0x01)){//说明需要判断传感器2是否有效
			if(encoder_en)/*如果是编码器的话需要发送当前位置*/
			{
				StepMotor_afterrun_toreport(stepno,1);
			}
			else
			if (!work)   {//说明没有检测到那就报警
				#ifdef NEW_ALARM_STYLE
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_WORK_INVALID);
				#else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)0x9222|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
				#endif
			}			
			step_work_sign_alarmenable  &=  ~(0x01<<Step->moto_remap_config.moto_remap_id_all) ;//清除检测标志
		}

	#ifdef ENCODER_SUPPORT
		if (1)//判断是否为传感器类型
		{
			if (Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index))
				goto exit;
		}
	#endif /* ENCODER_SUPPORT */
		zero_work_area = zero_go_ + Step->input_errorstep+step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		if ((is_sinker(stepno))&&(!Step->alert_delay_max))
		{
			zero_work_area += sinker_zero_area;
		}

		zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);

#ifdef NEW_ZERO_DETECT_MODE
		if (is_JF_DENSITY_Step(stepno))
		{
			if ((Step->position>DM_check_area_max)||(Step->position<DM_check_area_min))
			{
				Step->step_st.bit.check_delay_count=0;
				errno = 0;
				//Step->pos_2 = Step->position;
				Step->alarm_step = 0;
				goto LX_act_step_ok;
			}
		}

		if (is_LX_DENSITY_Step(stepno)||is_lx_sinker(stepno))
		{
			zero_work_area_2 = zero_go_ - Step->input_errorstep-step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			//if (zero_work_area_2)
			if (((zero&&((Step->position > zero_work_area) ||(Step->position <(0- zero_work_area))))||
				(!zero && (Step->position <=zero_work_area_2)&&(Step->position >=(0-zero_work_area_2))))
				||(Step->chk.bit.chk_st_change))
			{
				Step->step_st.bit.check_delay_count = 20/*10*/;
				Step->step_st.bit.check_delay_count += 15;
				errno = 0;
				//Step->pos_2 = Step->position;
				Step->alarm_step = 0;
				//if(is_lx_sinker_ex(Step))
				//Message_Send_4halfword(0x9999,Step->moto_remap_config.moto_remap_id_self,Step->chk.bit.chk_st_change,2);
	

			}
		}
		else if(is_LIFT_or_Other_Step(stepno))
			{
				Step->step_st.bit.check_delay_count =35;
				errno = 0;
				//Step->pos_2 = Step->position;//
				Step->alarm_step = 0;
			}
		else	if(is_DENSITY_Step(stepno))/*by hlc 20190226 不包括连兴度目*/
			{
				if((zero && ((Step->position > zero_work_area)))||
					((!zero)&&(Step->position <= 0)&&(Density_zero_check?(Step->position >=(0-Density_zero_check)):1)))
				{
					goto check_zero_is_ok_last;
				}
			}
		else		
		if ((zero && ((Step->position > zero_work_area))) ||
		    (!zero && ((Step->position <= 0)))||
		    (is_HP_ACTION_Step(stepno))) {
	
	check_zero_is_ok_last:		
			Step->step_st.bit.check_delay_count = 20/*10*/;
			Step->step_st.bit.check_delay_count += 15;
			errno = 0;
			// by xhl 2012/07/12
			//Step->pos_2 = Step->position;
			Step->alarm_step = 0;
		}

		if(is_Feet_Step(stepno))
		{
			
			if((Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_PT)
			||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX))
			{	
				if (feet_sts & (0x1 << Step->moto_remap_config.moto_remap_id_self))
				{
					if(!work)
					{
					feet_need_check:
						Step->step_st.bit.check_delay_count =35;
						errno = 0;				
						Step->alarm_step = 0;	
					}
				}
				else
				{
					if(!zero)
					{
						goto feet_need_check;
					}
				}
				
			}
			else
			{
				Step->chk.bit.check_sts_enable =1;
				goto feet_need_check;
			}
		}

		if (is_HP_ACTION_Step(stepno))
		{
			if (Step->chk.bit.HP_check_isok)
			{
				Step->step_st.bit.check_delay_count =0;
			}
		}
		
#endif

#ifdef ZERO2_SUPPORT
		if (is_zero2_support(stepno) && is_PT_ACTION_Step(stepno)  && !zero )
		{
			Step->step_st.bit.check_delay_count = 20;
			Step->step_st.bit.check_delay_count += 15;
			if (Step->step_st.bit.zero2_count == 0)
			{
				#ifdef NEW_ALARM_STYLE
				errno = STEP_ALARM_ZERO_INVALID;
				#else
				errno = 3 + 20;
				#endif
			}
		}

#endif
FH_act_step_ok:
#ifdef ZERO2_SUPPORT
		

		if (is_zero2_support(stepno) && is_FH_ACTION_Step(stepno) && (Step->position !=0))
		{			
			if (Step->step_st.bit.zero2_count == 0)
			{			
				errno = 3 + 20;				
			}
		}

		
#endif

//#ifdef LX_ACTION_STEP_SPECIAL
	LX_act_step_ok:
//#endif
		if(errno) {

			if((is_DENSITY_Step(stepno))
				&&(Step->step_st.bit.IS_Reset_ex ==1)
				&&(Step->isdoing_with_bind_cmd))
			{
				
			}
			else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
			
			Step->position = 0;
			Step->pos_2 = 0;
		}

		
		
		goto exit;
	}

done:

{
	unsigned int add_speed_temp;	
	if ((is_sinker(stepno)) && (Step->step_st.bit.IS_Reset_ex ==0))
		add_speed_temp = sinker_add_speed;
	else
		add_speed_temp =0;
	
	arch_StepMotor_Active(stepno,add_speed_temp);
}
exit:

#ifdef DRV8711
	if(Step->step_st.bit.level != 0) {
		Step->step_st.bit.level = 0;
		arch_StepMotor_Pulse(stepno, 0);
	}
#endif

if (change_dir)
{
	if( (is_LIFT_or_Other_Step(stepno)||is_sinker(stepno)) &&(Step->step_st.bit.IS_Reset_ex)&&(!encoder_en))
	{
		char zero_dir=Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir;
		arch_StepMotor_Dir(stepno, Step->step_st.bit.dir? !zero_dir:zero_dir);
	}
	else		
	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);
}
	
	if(Step->steps == 0) {

		#if 0
		if(Step->state_par == FEET_STEP_ISWORK) {
			if(!work) {
				#ifdef NEW_ALARM_STYLE
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),STEP_ALARM_WORK_INVALID);
				#else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)3|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
				#endif
			}
			return ;
		}
		#endif
		
		if(Step->state_par) {
			if(Step->state_par == LEAVE_ZERO ||
			   Step->state_par == GOTO_ZERO) {
				#ifdef NEW_ALARM_STYLE
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),STEP_ALARM_RESET_ERROR);
				#else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)0x8001|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
				#endif
				Step->position = 0;
				Step->pos_2 = 0;
			}
			return ;
		}

		if(Step->moto_dd_config.DD_type==0)
		{
		
		if(((abs(Step->pos_2-Step->position)>(encoder_en?5:1))&&(!is_HP_ACTION_Step(stepno))&&(!is_FH_ACTION_Step(stepno)) /*&&(!is_PT_ACTION_Step(stepno))*/)
			||(is_HP_ACTION_Step(stepno) &&(Step->chk.bit.HP_auto_adj))
			||((abs(Step->pos_2-Step->position)>(encoder_en?5:1)) && encoder_en))
		{

			#if 0
			{	
				unsigned int sendtemp[4]={0,0,0,0};
				sendtemp[0]=3|(stepno<<8);
				sendtemp[1]=Step->position;
				sendtemp[2]=Step->steps;
				sendtemp[3]=Step->step_wait_time;	
				
				arch_SendMessage(4,sendtemp,4);
				sendtemp[0]=4|(stepno<<8);
				sendtemp[1]=Step->pos_2;
				sendtemp[2]=Step->speed;
				sendtemp[3]=Step->max_speed;	
				
				arch_SendMessage(4,sendtemp,4);
				
				
			}
			#endif

	//if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_SKINER)
		//Message_Send_4halfword(0x9999,stepno,Step->position,Step->pos_2);
			#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if ((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)	
			if(enable_log_tosend_LX)
			{
				unsigned short d[3];
				d[0] 	= Step->moto_remap_config.moto_attr.bit.moto_type_config;
				d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
				d[0] |= (LX_LOG_TYPE_STOP_GO)<<8;
				d[0] |= (Step->chk.bit.last_zero_bit & 0x0F)<<12;
				d[1] =Step->position;
				d[2] =Step->pos_2;								
				Message_send_log_LX(d[0],d[1],d[2]);
				
			}
			#endif
			
			//StepMotor_runingover(stepno, Step->pos_2, 0);
			//alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),(unsigned int)12|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
					
			Step->step_wait_time = 0 ;//
			//Step->dir_is_change_ =1;
			Step->need_2_pos_after_reset = Step->pos_2;
			
			Step->last_stop_systick = arch_get_ticktime();
			Step->step_st.bit.last_dir = Step->step_st.bit.dir;
			Step->Had_next_cmd =1;
				#if 0
				{
					unsigned int add_speed_temp;	
					if (is_sinker(stepno))
						add_speed_temp = sinker_add_speed;
					else
						add_speed_temp =0;
	
				arch_StepMotor_Active(stepno,add_speed_temp);
				}
				#endif
			// by xhl 2012/07/12
			//Step->step_wait_time = 10;
		

			
		}
		else 
		{
			if(Step->step_st.bit.step_flags) {
				Step->step_st.bit.step_flags =0;
			}
			#ifdef ENCODER_SUPPORT
			Encoder_setCheck(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, 1);
			//Message_Send_4halfword(0x8888,stepno,Step->position,Step->pos_2);
		     	#endif
			
		}
		}
		

		if ((Step->step_st.bit.IS_Reset_ex)&& is_YARN_Step(stepno)/*(Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)*/) 
		{
			if (Step->need_2_pos_after_reset !=0)
			{
				//extern void arch_StepMotor_Active(unsigned int stepno,unsigned int  add_speed);
				
				//StepMotor_exec(stepno, Step->need_2_pos_after_reset, 1,0,0);
				
				//arch_StepMotor_Active(stepno,0);
			
				//Step->step_wait_time =STEP_DIR_CHANGE_DELAY_MS;

				Step->last_stop_systick = arch_get_ticktime();
				 //= Step->pos_2;
				Step->pos_2 =Step->need_2_pos_after_reset;
				Step->step_st.bit.last_dir = Step->step_st.bit.dir;
				Step->Had_next_cmd =1;
				Step->step_wait_time =0;
			}
			else
			{
				//int stepidself= Step->moto_remap_config.moto_remap_id_self;
				emf_status[2] |=(0x03<<Step->moto_remap_config.moto_remap_id_self*2);	
			}
		}
	}

	return ;

alarm_and_exit:
#ifdef DRV8711
	if(Step->step_st.bit.level != 0) {
		Step->step_st.bit.level = 0;
		arch_StepMotor_Pulse(stepno, 0);
	}
#endif
	
		Step->step_st.bit.running = 0;
		arch_StepMotor_Stop(stepno);
		#ifdef MOTOR_FULL_CURR_OUTPUT
		if(is_DENSITY_Step(stepno))
		{
			arch_StepMotor_Half(stepno ,MOTOR_FULL_CURR_OUTPUT);//半流
		}
		else			
		#endif
		arch_StepMotor_Half(stepno, MOTOR_HALF_OUTPUT);

		if (errno)
		{
				if((is_DENSITY_Step(stepno))
				&&(Step->step_st.bit.IS_Reset_ex ==1)
				&&(Step->isdoing_with_bind_cmd))
			{
				
			}
			else
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
		}
		return ;
	
}


void Motor_Get_All_postion(unsigned int ispos2,unsigned short *posdata)
{
	STEP_TYPE *Step;
	short postemp[4];
	int i;
	for (i=0;i<4;i++)
	{
		Step = &STEPMOTOR[i];		
		postemp[i] =((ispos2?Step->pos_2:Step->position)&0x0FFF);///* |((Step->step_st.bit.running?1:0)<<11*/);
		
		//*posdata = postemp;		
	}
	memset((char *)posdata,0,6);
	*posdata=(postemp[0] & 0x0FFF);
	*posdata |= (postemp[1] & 0x000F)<<12;
	*(posdata+1)=(postemp[1] & 0x0FF0)>>4;
	*(posdata+1) |= (postemp[2] & 0x00FF)<<8;
	*(posdata+2)  = (postemp[2] & 0x0F00)>>8;
	*(posdata+2) |= (postemp[3] & 0x0FFF)<<4;
	
}
#if 0
void StepMotor_Halt(int stepno)
{
	STEP_TYPE *Step;
	
	if (stepno >= StepMotor_Count_MAX)
		return;
	
	Step = &STEPMOTOR[stepno];
	
	Step->step_st.bit.running = 0;
	arch_StepMotor_Stop(stepno);
	arch_StepMotor_Half(stepno, 1);
	
	if (Step->step_st.bit.level != 0)
	{
		Step->step_st.bit.level = 0;
		arch_StepMotor_Pulse(stepno, 0);
	}
}



void StepMotor_Set_waittime(int stepno,unsigned int timems)
{
	STEPMOTOR[stepno].step_wait_time = timems;
	return;
	
}

#endif

void StepMotor_Poll(void)
{
#ifdef ENCODER_SUPPORT
	Encoder_Poll();
#endif /* ENCODER_SUPPORT */


#if 0
	int i;
	int check_area/* =  step_zero_adj + ZERO_DETECT*/;
	int err = 0;
	STEP_TYPE *Step;
	for (i = 0; i < STEP_NUM; i ++) {
		Step = &STEPMOTOR[i];	
		myprintf("\r\n==111 moto_config[%d][%d] 111 ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);

		}
	#endif
}


#ifdef MOTOR_ST_AUTO_REPORT
unsigned long int step_al_st;

int  check_step_alert_st(unsigned int stepno,unsigned char *al_st)
{
	unsigned char ret=0;	
	unsigned char td=0;
	unsigned char sid;
	STEP_TYPE *Step;
	if (stepno>StepMotor_Count_MAX) return ret;	
	Step = &STEPMOTOR[stepno];
	if ((Step->step_alert_st.all & 0x0f)!=((Step->step_alert_st.all>>4)&0x0f))
	{
		td = Step->step_alert_st.all<<4;
		td |= (Step->step_alert_st.all & 0x0f);
		Step->step_alert_st.all = td;
		td = Step->step_alert_st.all & 0x0f;
		ret =1;
		sid = Step->moto_remap_config.moto_remap_id_self;
		switch (Step->moto_remap_config.moto_attr.bit.moto_type_config)
		{
			case  MOTOR_TYPE_DENSITY:
				*(al_st+(sid>>1)) &=(0xF0 >>((sid & 0x01)?4:0));				
				*(al_st+(sid>>1)) |= (td <<((sid & 0x01)?4:0));
				break;
			case MOTOR_TYPE_SKINER:		
				*(al_st+2) &=(0xF0 >>((sid & 0x01)?4:0));
				*(al_st+2) |= (td <<((sid & 0x01)?4:0));				
				break;
			case MOTOR_TYPE_ACTION:
				*(al_st+3) &=(0xF0 >>((sid & 0x01)?4:0));
				*(al_st+3) |= (td <<((sid & 0x01)?4:0));				
				break;
			case MOTOR_TYPE_FEET:
				*(al_st+4) &=(0xF0 >>((sid & 0x01)?4:0));
				*(al_st+4) |= (td <<((sid & 0x01)?4:0));				
				break;
			case MOTOR_TYPE_YARN:  /*沙嘴电机另外处理*/
				
				break;
			case MOTOR_TYPE_LIFT:
				*(al_st+5) &=(0xF0 >>((sid & 0x01)?4:0));
				*(al_st+5) |= (td <<((sid & 0x01)?4:0));				
				break;
			default:

			break;
		}
	}
	//ret = Step->step_alert_st.all;
	return ret;	
}

#endif

void check_step_goon(unsigned int stepno)
{
	//int i;
	int needtosendit=0;
	STEP_TYPE *Step;
	if (stepno>StepMotor_Count_MAX) return;
	
	Step = &STEPMOTOR[stepno];
	if ((Step->st_no_cl.bit.report_cnt==0))
		return;
	needtosendit=0;	
	if(Step->step_st.bit.running==0) 
	{
		if(Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index))
		{
			if (Encoder_rechk_over(Step->moto_remap_config.moto_attr.bit.moto_ecode_index))
			{
				needtosendit=1;
			}
			else
				needtosendit=0;
		}
		else
			needtosendit=1;
		
		if ((Step->st_no_cl.bit.report_cnt)&&(needtosendit))
		{
			StepMotor_afterrun_toreport(stepno,0);			
		}

		if((Step->moto_dd_config.DD_last_wp)||(Step->moto_dd_config.DD_last_speed[0]))
		{
			//arch_StepMotor_DD_printf(Step);
			Step->moto_dd_config.DD_last_wp =0;
			Step->moto_dd_config.DD_last_speed[0] =0;
			
		}

		if((Step->moto_remap_config.moto_attr.bit.moto_type_exe== YARN_MOTOR_TYPE_LX)
				&&(Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_YARN))
		{
			if(Step->chk.bit.check_Right_Now)
			{
				check_ACT_Work_with_POS(Step);	
			}
		}
			
		
	}
	else 
		return;
	

}


int check_HF_Lift_input_error(STEP_TYPE *Step)
{
	int err=0;
	//if ((Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_LIFT)
	//		&&(Step->moto_remap_config.moto_attr.bit.moto_type_exe==LIFT_MOTOR_TYPE_HF))
	{
		int zero;
		//int isleft;
		//int isright;
		int check_area,check_area_;
		unsigned char needcheck=1;		
				
		{
			//check_area = (Step->moto_zero_width_self-Step->input_errorstep)>>1;
			if((Step->moto_remap_config.moto_attr.bit.moto_type_exe== LIFT_MOTOR_TYPE_HF_EX)
				&&(Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_LIFT))
			{
				check_area = 60;
				check_area_ = check_area;

			}
			else
			{
				check_area = (Step->moto_zero_width_self+Step->input_errorstep)>>1;//传感器可能亮的范围
				check_area_ = (Step->moto_zero_width_self-Step->input_errorstep)>>2;// 1;
				check_area += step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			}
			zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);

			if ((Step->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)&&(!Step->alert_delay_max))
			{
				check_area += sinker_zero_area;
				check_area_ -= step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			}

			

			#ifdef IS_CX_ACT
			if(is_DOT_ACTION_Step(Step->moto_remap_config.moto_remap_id_all))
			{
				int check_area_s,check_area_s_;
			
				check_area = Step->input_errorstep+_CX_DOT_ACT_ZERO_ADJ;
				check_area_s = _CX_DOT_ACT_ZERO_ADJ;
				check_area_= Step->moto_zero_width_self-_CX_DOT_ACT_ZERO_ADJ;
				check_area_s_=check_area_-Step->input_errorstep;
				
				if (zero)
				{
					if ((Step->position>check_area)||(Step->position<(0-check_area_)))
					{
						//Step->alarm_step ++;							
						err = 0x0002;						
					}
					else
					{
						//Step->alarm_step = 0;
						//Step->step_st.bit.check_delay_count = 0;
					}	
				}
				else
				{
					if ((Step->position>=(0-check_area_s_))&&(Step->position<=Step->input_errorstep))
					{
						//Step->alarm_step ++;							
						err = 0x0001;							
					}
					else
					{
						//Step->alarm_step = 0;
						//Step->step_st.bit.check_delay_count = 0;
					}
				}
				return err;
			}
			
			#endif

			if((Step->moto_remap_config.moto_attr.bit.moto_type_exe== YARN_MOTOR_TYPE_LX)
				&&(Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_YARN))
			{
				if(Step->position>check_area)
				{
					needcheck=0;
				}
			}

			if(needcheck)	
			{
				if (zero)
				{
					if ((Step->position>check_area)||(Step->position<(0-check_area)))
					{
						//Step->alarm_step ++;							
						err = 0x0002;						
					}
					else
					{
						//Step->alarm_step = 0;
						//Step->step_st.bit.check_delay_count = 0;
					}	
				}
				else
				{
					if ((Step->position>=(0-check_area_))&&(Step->position<=check_area_))
					{
						//Step->alarm_step ++;							
						err = 0x0001;							
					}
					else
					{
						//Step->alarm_step = 0;
						//Step->step_st.bit.check_delay_count = 0;
					}
				}
			}

		}
	}
	return err;
		
}

//#define  TEST_ACT_ZERO_CHANGE   /*检查动作电机零位信号的变化*/

void check_step_motor_loop_()
{
#ifdef TEST_ACT_ZERO_CHANGE
	static unsigned short step_zero_input_cach=0;
	static unsigned short sendidx=0;
	unsigned short step_zero_input_temp=0;
#endif	
	int i;
	unsigned short need_step_power_zero=0xFFFF;//默认可以关闭
	unsigned char  idall_power_zero[MOTOR_TYPE_COUNT];
	extern unsigned short Sys_power_lost_bit;
	#ifdef MOTOR_ST_AUTO_REPORT
	int needtosendit=0;
	#endif

		
	STEP_TYPE *Step;
	for (i = 0; i < StepMotor_Count_MAX; i ++) 
	{
		Step = &STEPMOTOR[i];
		if ((Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_UNDEF)
			||(Step->moto_remap_config.moto_attr.bit.moto_type_config>=MOTOR_TYPE_MAX))
			continue;

#ifdef TEST_ACT_ZERO_CHANGE
			//if(Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION)
			{
				//if (arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index))
				if (arch_StepMotor_Zero(16+i))
				{
					step_zero_input_temp |=(0x01<<i);
				}				
			}
#endif		
			if(Step->Had_next_cmd)
			{
				Step->Had_next_cmd=0;
				StepMotor_exec(i,Step->pos_2,1,0,0);
			}

			if (Sys_power_lost_bit)
			{
				unsigned char stid_temp=Step->moto_remap_config.moto_attr.bit.moto_type_config-1;
				
				if(Step->step_st.bit.running)
				{
					need_step_power_zero &=~(0x0001<<(stid_temp));
				}
				else
					if(Sys_power_lost_bit & (0x0001<<stid_temp))
					{
						arch_StepMotor_Disable_onestepmoto(Step->moto_remap_config.moto_remap_id_all);		
					}
				//idall_power_zero[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]=Step->moto_remap_config.moto_remap_id_all;
			}

			check_step_goon(i);
			#ifdef MOTOR_ST_AUTO_REPORT
			needtosendit |= check_step_alert_st(i,(unsigned char *)&step_al_st);	
			#endif
	}

	#ifdef MOTOR_ST_AUTO_REPORT
	if (needtosendit)
	{
		Message_Send_4halfword((CMD_STEP_ALERT_ST_REPORT << 8) | 0x01, (step_al_st & 0xFFFF), ((step_al_st>>16) & 0xFFFF), ((step_al_st >>32)& 0xFFFF));
	}
	#endif
#ifdef TEST_ACT_ZERO_CHANGE
	if (step_zero_input_temp!=step_zero_input_cach)
	{
		Message_Send_4halfword((0xCD << 8) | 0xAB,step_zero_input_temp,step_zero_input_cach, sendidx++);
		step_zero_input_cach = 	step_zero_input_temp;
		
	}
#endif	

	if (Sys_power_lost_bit)
	{
		for (i=MOTOR_TYPE_DENSITY;i<MOTOR_TYPE_MAX;i++)
		{
			if((Sys_power_lost_bit & (0x0001<<(i-1)))
				&&(need_step_power_zero & (0x0001<<(i-1))))
			{
				//Exec_Set_Motor_Curr_with_stepno(i,idall_power_zero[i-1],0);

				Sys_power_lost_bit &= ~(0x0001<<(i-1)); 
			}
		}
	}
	
}



void StepMotor_timer()
{
	extern volatile unsigned char alert_runtime_check;
	int i;
	int check_area/* =  step_zero_adj + ZERO_DETECT*/;/*零位可能有效的范围*/
	int check_area_;/*零位必须有效的范围*/
	//int check_area_range[4];/*0-传感器无效正值 1-传感器无效负值
							//  2-传感器有效正值 3-传感器有效负值*/
	
	int err = 0;
	int encoder_en = 0;
	unsigned char stepdir=0;
	STEP_TYPE *Step;
	for (i = 0; i < StepMotor_Count_MAX; i ++) {
		Step = &STEPMOTOR[i];	
		stepdir = Step->step_st.bit.dir_High;
		if ((Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_UNDEF)
			||(Step->moto_remap_config.moto_attr.bit.moto_type_config>=MOTOR_TYPE_MAX)
			||(!Step->moto_remap_config.moto_attr.bit.is_activestep_enable))
		continue;

		if(Step->chk.bit.check_Right_Now)
		{
			if(is_ACTION_Step(i)||is_Feet_Step(i))
				check_ACT_Work_with_POS(Step);				
		}
		
		if(Step->step_wait_time) {
			Step->step_wait_time --;
			if(Step->step_wait_time == 0) {
				Step->step_check_interval = 10;
			}
		}
		if(Step->step_st.bit.running) {
			//Step->step_stop_long =0;
			if(Step->step_wait_time) {
				continue;
			}
			if(Step->step_check_interval <= 0) {
				continue;
			}
			Step->step_check_interval--;
			if(Step->step_check_interval > 0)
				continue;
			Step->step_check_interval = 2;
			if(Step->step_check_pos == Step->steps && Step->steps != 0) {				
				//arch_StepMotor_Start(i);
			}
			else {
				Step->step_check_pos = Step->steps;
				if(Step->steps == 0) {
					arch_StepMotor_Stop(i);					
					Step->step_st.bit.running = 0;					
				}
			}
			continue;
		}
		else
		{
			

#ifdef NEW_ZERO_DETECT_MODE
	#ifdef ENCODER_SUPPORT
		if (1)//判断是否为传感器类型
		{
			encoder_en = Encoder_Work(Step->moto_remap_config.moto_attr.bit.moto_ecode_index);
			if (encoder_en)
				continue;
		}
	#endif /* ENCODER_SUPPORT */
	
		if(Step->step_st.bit.check_delay_count > 0) {
			int zero;
			
			Step->step_st.bit.check_delay_count --;
			if(Step->step_st.bit.check_delay_count > 15) continue;

			if (Step->input_errorstep<0)
				Step->input_errorstep =0;			

			
			check_area = step_zero_adj[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]  + Step->input_errorstep;
			//check_area+=step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			check_area_ = 800;
			if (is_HF_LIFT_Step(i)
				||is_EX_OTHER_Step(i) 
				||is_DOT_ACTION_Step(i)
				||is_CX3_Feet_Step(i)
				||is_LX_DENSITY_Step(i)
				||is_lx_sinker(i)
				||is_LX_yarn_step(i))
			{	
				if(is_HF_ex_LIFT_Step(i))
				{
					check_area = 60;
					check_area_ = 60;//传感器必须亮的范围
				
				}
				else
				{
					check_area = (Step->moto_zero_width_self+Step->input_errorstep)>>1;
					check_area_ = (Step->moto_zero_width_self-Step->input_errorstep)>>1;//传感器必须亮的范围
					#ifdef IS_CX_ACT
					if(is_DOT_ACTION_Step(i))
					{
						
					}					
					#endif
				}
			}
			else		
			{				
				check_area+=step_zero_detect[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
			}
			if(is_lxex_sinker(i))
			{
				check_area += step_zero_adj_sk_lxex;
			}
			
			if (is_sinker(i)&&(!Step->alert_delay_max))
			{
				check_area += sinker_zero_area;
			}

		
			
			
			check_area = stepspeed_Q2R(check_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			check_area_ = stepspeed_Q2R(check_area_,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			

			{
				
				int work;
				int ZeroPos_WorkPos;
				int WorkPos_WorkST;
				int isleft;
				int isright;
				//#ifdef LX_ACTION_STEP_SPECIAL

				zero = arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg);
				work = arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg); 
				ZeroPos_WorkPos = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
				WorkPos_WorkST = Step->moto_remap_config.moto_attr.bit.workPos_Work_ST;
				if ((Step->chk.bit.chk_st_change) && ((Step->chk.bit.last_zero_st !=(zero?1:0))))
				{
					Step->chk.bit.chk_st_change =0;
				}
				
				if (is_HF_LIFT_Step(i) 
					||is_EX_OTHER_Step(i)
					||is_DOT_ACTION_Step(i)
					||is_CX3_Feet_Step(i)
					||is_LX_DENSITY_Step(i)
					||is_lx_sinker(i)
					||is_LX_yarn_step(i))
				{
					err = check_HF_Lift_input_error(Step);
				//	if(is_LX_yarn_step(i)) /*连兴纱嘴电机不判断信号*/
					//	err=0;
					//check_area = (Step->moto_zero_width_self-Step->input_errorstep)>>1;
					#ifdef SHIELD_LIFT_ALARM
					if(is_HF_LIFT_Step(i) )
					{
						err=0;	
					}
					#endif
					if(err)
					{
						Step->alarm_step++;
					}

					if ((is_HF_LIFT_Step(i)
						||is_EX_OTHER_Step(i)
						||is_DOT_ACTION_Step(i)
						||is_CX3_Feet_Step(i)
						||is_LX_yarn_step(i)
						||is_lx_sinker(i)) && (!err))
					{
						if (Step->chk.bit.chk_st_change)
						{
							Step->alarm_step++;
							err=0x9004;
							//if(is_lx_sinker_ex(Step))
							//	Message_Send_4halfword(0x9999,Step->moto_remap_config.moto_remap_id_self,Step->chk.bit.chk_st_change,Step->alarm_step|(3<<8));
	
						}
						else
						{
							
							//Step->step_st.bit.check_delay_count = 0;
						}
					}

					if (!err)
					{
						Step->alarm_step = 0;
						Step->step_st.bit.check_delay_count=0;
					}
					
					goto step_timmer_check_isalerm;
					
				}
				
				if (is_LX_ACTION_Step(i)
					||(is_CX2_Feet_Step(i))
					||is_CX3_Feet_Step(i)
					||is_FH_ACTION_Step(i)
					||(is_PT_ACTION_Step(i)&&!is_zero2_support(i)))
				{
					char czw=Step->st_no_cl.bit.check_zero_work;
					char checkzero=0;
					char checkwork =0 ;
					char isfeet_tmp=Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_FEET ?1:0;

					#if 0
					if(!Step->alarm_step)
					{
						Message_Send_4halfword(0x0104,(i<<8)|0x01,Step->st_no_cl.bit.check_zero_work,0xFCFC);
						Message_Send_4halfword(0x0104,(i<<8)|0x01,(zero?1:0)|((work?1:0)<<8),0xFdFd);
							
					}
					#endif
					
					if (((czw>>2) & 0x01) && (zero!=(czw & 0x01)))
						checkzero =1;

					if (((czw>>3) & 0x01) && (work!=((czw >>1) & 0x01)))
						checkwork =1;
									
					if (checkzero||checkwork)
					{				
						Step->alarm_step++;

						if (checkzero)						
							err=isfeet_tmp?0x0001:0x9003;		
						if (checkwork)
							err=isfeet_tmp?(0x0002):0xC003;//work|(czw<<8)
						#if 0
						if (checkwork)
						{
							err=work?0xC003:0xC002;
							err |=(czw<<4);
							err |=(0<<8);							
						}
						#endif
							

						if (checkzero && checkwork)
							err=isfeet_tmp?0x0003:0xD003;
					}
					else
					if (Step->chk.bit.chk_st_change)
					{
						//Step->alarm_step++;
					//	err=0x9004;
					}
					else
					{
						Step->alarm_step = 0;
						Step->step_st.bit.check_delay_count = 0;
					}
					if(is_PT_ACTION_Step(i))
					{
						if (err) 
						{
							goto step_timmer_check_isalerm;
						}	
						else
						{
							goto checkother_alert_pt_actionstep;
						}
					}
					else					
					goto step_timmer_check_isalerm;
				}
				else
				{				
				//#endif
				checkother_alert_pt_actionstep:
				if (is_HP_ACTION_Step(i))
				{
					zero = Triangle_is_zero_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
					isleft = Triangle_is_left_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
					isright =Triangle_is_right_Sign(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg,Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg);
					if (/*(Triangle_sts[i] & (0x03))*/Step->chk.bit.HP_check_st==0x01)
					{
						if (!isleft)
						{
							Step->alarm_step ++;
							err = 0xA001;
						}
						else
						{
							Step->alarm_step = 0;
							Step->step_st.bit.check_delay_count = 0;
						}
					}
					else
						if (/*(Triangle_sts[i] & (0x03))*/Step->chk.bit.HP_check_st==0x02)
						{
							if (!isright)
							{
								Step->alarm_step ++;
								err = 0xA002;
							}
							else
							{
								Step->alarm_step = 0;
								Step->step_st.bit.check_delay_count = 0;
							}
						}
						else
							if (/*(Triangle_sts[i] & (0x03))*/Step->chk.bit.HP_check_st==0x03)
							{
								if (!zero)
								{
									Step->alarm_step ++;
									err = 0xA003;
								}
								else
								{
									Step->alarm_step = 0;
									Step->step_st.bit.check_delay_count = 0;
								}
							}
						goto step_timmer_check_isalerm;	

				}
				else
				{
					if(is_zero2_support(i) && is_PT_ACTION_Step(i)&&(work!=WorkPos_WorkST) && (Step->position>check_area))//????这里有疑问
					{
						#ifdef NEW_ALARM_STYLE
						err = STEP_ALARM_OTHER_ERROR;
						#else
						err = 3 + 30;
						#endif
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), err|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						Step->alarm_step = 0;
						Step->step_st.bit.check_delay_count = 0;
						continue;
					}

					

					if (is_LIFT_or_Other_Step(i)||is_sinker(i))
					{	
						unsigned char isright;
						//int zero;		

						if (Step->position<check_area && Step->position >(0-check_area))
							goto lift_end_alert;
						//zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);
						isright= ((stepdir==0)&&(Step->position<0)) ||((stepdir!=0)&&(Step->position>0));
						if (isright)
						{
							if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)&&(!zero))
							{
								zero_in_alert:
								Step->alarm_step ++;						
								err = 0x0001;	
							}
							else if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir==0)&&(zero))
							{
								zero_no_alert:
								Step->alarm_step ++;						
								err = 0x0002;									
							}
							else
							{
								zero_is_ok:
								if (zero) 
								{
									if (Step->alert_delay_cnt)
										Step->alert_delay_cnt =0;
								}
								Step->alarm_step =0;
							}
							
						}
						else
						{
							if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir)&&(zero))
							{
								goto zero_no_alert;
							}
							else if ((Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir==0)&&(!zero))
							{
								goto zero_in_alert;
							}
							else
							{
								goto zero_is_ok;
							}
							
						}

						lift_end_alert:
							goto step_timmer_check_isalerm;
						
					}

					if(!is_Feet_Step(i))
					{
						//int zero;
						//zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index);

						//if (stepdir==0)
					if(Step->position > check_area) {
						 
					#ifdef ZERO2_SUPPORT
						if(is_zero2_support(i) && is_PT_ACTION_Step(i) && zero && (Step->act_is_justcheck_zero==0))  
						{
							if(work!=ZeroPos_WorkPos) 
							{
								zero = 0;
							}
						}
					#endif
						if(zero) 
						{
							Step->alarm_step ++;
							#ifdef NEW_ALARM_STYLE
							err = STEP_ALARM_ZERO_VALID;
							#else
							err = 0x0002;
							#endif
						}
						else
							{
								Step->alarm_step = 0;
								Step->step_st.bit.check_delay_count = 0;
							}
					}
					else if(Step->position <= 0) {
						
					#ifdef ZERO2_SUPPORT
						if(is_zero2_support(i) && is_PT_ACTION_Step(i) && zero  && (Step->act_is_justcheck_zero==0))  
						{
							if(work!=ZeroPos_WorkPos) 
							{
								zero = 0;
							}
						}
					#endif
						if(!zero) 
						{
							Step->alarm_step ++;
							#ifdef NEW_ALARM_STYLE
							err = STEP_ALARM_ZERO_INVALID;
							#else
							err = 0x0001;
							#endif
						}
						else {
							Step->alarm_step = 0;
							Step->step_st.bit.check_delay_count = 0;
						}
						}
					}
					}
				
//#ifdef LX_ACTION_STEP_SPECIAL
				}
//#endif
			}

		step_timmer_check_isalerm:	

			#ifdef NOTALART_ACTSTEP
			if((Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION) &&(err))
			{
				err=0;
			}
			#endif	
				
			if((Step->alarm_step > 10) && err) {
				Step->step_st.bit.check_delay_count = 0;
				Step->alarm_step = 0;
				if(is_ACTION_Step(i)
					||is_CX2_Feet_Step(i)
					||is_CX3_Feet_Step(i))
				{
					if(Step->chk.bit.check_Right_Now)
						Step->chk.bit.check_Right_Now =0;
					Step->chk.bit.check_sts_enable =0;
				}
				#if 1
				if ((err==0x0001)&&(Step->alert_delay_max))/*该亮不亮的报警*/
				{
					//Message_Send_4halfword(0XAAAA,Step->step_alert_delay_pos2_st.all,Step->alert_delay_cnt,i);
					//if((Step->step_alert_delay_pos2_st.bit.last_needcheck_zero==0)
						//&&(Step->step_alert_delay_pos2_st.bit.new_needcheck_zero))
						Step->alert_delay_cnt++;
					
					if (Step->alert_delay_cnt<Step->alert_delay_max)
					{
						err=0;
					}
					else
					{
						Step->alert_delay_cnt=0;
					}
				}
				//
				#endif
				if (err)
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),err|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
				if(is_LX_yarn_step(i))
				{
					Message_Send_4halfword(0XAA|(i<<8)
						,Step->position,Step->pos_2,
						(Step->chk.bit.last_zero_st<<15)|((zero?1:0)<<14)|(Step->chk.bit.SK_ADJ_ok<<13)|(Step->chk.bit.SK_ADJ_ok<<12)|(Step->chk.bit.last_zero_bit));
						
				}
			}
		}
		else
		{
			if(!Step->chk.bit.check_Right_Now)
			{
				if(Step->chk.bit.check_sts_enable)
				{
					#ifndef NO_CHECKINPUT_FOR_LXACT_REALTIME
					if(is_ACTION_Step(i)
						||is_CX2_Feet_Step(i)
						||is_CX3_Feet_Step(i))
					#else
					if(is_CX2_Feet_Step(i)
						||is_CX3_Feet_Step(i))
					#endif	
					{
						check_ACT_Work_with_POS(Step);	
					}
				}
			}
		}
		
#endif

			if(alert_runtime_check)/*可以实时检查*/
			{
				//char i_stepno = Step->moto_remap_config.moto_remap_id_self;
				if(feet_enable & (0x01<<Step->moto_remap_config.moto_remap_id_self) )
					check_feet_sts(Step);
				if ((act_check_enable)/*||(Head_Mode_ == HEAD_MODE_LX_ACT )*/||(Head_Mode_ == HEAD_MODE_FH_ACT ))
					check_ACT_Work_with_POS(Step);
				if (is_HF_LIFT_Step(i)
					||is_EX_OTHER_Step(i)
					||is_DOT_ACTION_Step(i)
					||is_LX_yarn_step(i))
				{
					check_HF_Lift_Work_with_input(Step);
				}
			}
		}
	}


	
	//#ifndef LX_ACTION_STEP_SPECIAL
	

	


}


//volatile int feet_run_max_step = 500;
//变量声明必需放在程序头上
void StepMotor_Set_Feet_Num(int num)
{
	if((num == 2) || ((num == 4)))
	{
		feet_used_num = num;
	}
	return;
}

unsigned int step_get_feet_steps()
{
	return Motor_run_max_step[MOTOR_TYPE_FEET-1];
}


void StepMotor_Feet_Setup(int cmd, unsigned int arg)
{
	int i;
	//int feet_count;
	STEP_TYPE *Step;		
	switch(cmd) {
	case 0:
		Motor_run_max_step[MOTOR_TYPE_FEET-1] = arg;
		{
			for (i=0;i<MAX_MOTOR_COUNT_FEET;i++)
			{
				Motor_run_max_step_Feet[i] = arg;
				feet_enable |=(0x01<<i);		
			}
		}

		break;
	case 2:
		if ((arg & 0xFF) == 0xFE) {
			
			feet_alarm_enable  = 1;

			for (i = 0; i < StepMotor_Count_MAX; i++)
			{
				Step = &STEPMOTOR[i];
				if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_FEET) 
					continue;
				if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
					continue;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if(!(feet_enable & (0x1 << Step->moto_remap_config.moto_remap_id_self)))
				{
					continue;	
				}

				if (feet_sts & (0x1 << Step->moto_remap_config.moto_remap_id_self))
				{
				
					if (!arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg))
					{
						StepMotor_Feet_exec_no(i,Step->moto_remap_config.moto_remap_id_self, 0x4000,0);
					}
				}
				else
				{
					if (!arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
					{
						Step->position = 20;
						StepMotor_Feet_exec_no(i,Step->moto_remap_config.moto_remap_id_self, 0,0);
					}
				}			
							
			}
		}
		if ((arg & 0xFF) == 0xFD) {
			
			feet_alarm_enable  = 0;
		}
		if (((arg & 0xFF) >= 0x40) && ((arg & 0xFF) <= 0x4F))
		{
			
			feet_sts = (arg & 0xFF);			//根据压脚的特性
			
			for (i = 0; i < StepMotor_Count_MAX; i++)
			{
				Step = &STEPMOTOR[i];
				if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_FEET) 
					continue;
				if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
					continue;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if(!(feet_enable & (0x1 << Step->moto_remap_config.moto_remap_id_self)))
				{
					continue;	
				}
		
				if (feet_sts & (0x1 << Step->moto_remap_config.moto_remap_id_self))
				{
				
					if (!arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg))
					{
						if (feet_alarm_enable)
							#ifdef NEW_ALARM_STYLE
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_WORK_INVALID);
							#else
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)4|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
							#endif
					}
				}
				else
				{
					if (!arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
					{
						if (feet_alarm_enable)
						#ifdef NEW_ALARM_STYLE
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
						#else
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)3|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
						#endif
					}
				}
			}
		}
		break;
	case 3:
		StepMotor_Set_Feet_Num(arg);
		break;		
	default:
		break;
	}
}


void StepMotor_Feet_exec_no(int stno,int stidself, int arg,unsigned short otherarg)
{
	//myprintf("stno[%d],arg[%d]\r\n",stno,arg);
	//Message_Send_4halfword(0x7799,0,stno ,stidself);

	if (stno>=StepMotor_Count_MAX) return;

	if(stidself >=MAX_MOTOR_COUNT_FEET) return;

	if(!(feet_enable & (0x1 << stidself)))
	{
		return;	
	}

	if(arg & 0x8000) {									//复位
		
		feet_sts &= ~(0x1 << stidself);
		StepMotor_Reset(stno,0);
	}
	else if(arg & 0x4000) {						
		if(arch_StepMotor_Zero_mid_layer(StepMotor_get_zeroID_with_NO(stno),StepMotor_get_zeroID_cfg_with_NO(stno))) 
		{
			feet_sts |= (0x1 << stidself);
			StepMotor_exec(stno, (short)Motor_run_max_step_Feet[stidself], 0x1 | 0x400,otherarg,0);
		}
	}
	else {
		feet_sts &= ~(0x1 << stidself);
		StepMotor_exec(stno, 0, 0x1,otherarg,0);
	}
}

void check_feet_sts(STEP_TYPE *Step)
{
	//int i;
	//int feet_count;
	//int step_no;
	//STEP_TYPE *Step;	
	
	if (Step==0) 
		return;
	else
	{	
		//Step = &STEPMOTOR[i];

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_FEET) 
			return;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			return;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			return;	
		//step_no = i;
		if((Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_PT)
			||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == FEET_MOTOR_TYPE_CX))
		{
			if ((Step->step_st.bit.running == 0)&&(Step->steps==0))
			{
				if (feet_sts & (0x1 << Step->moto_remap_config.moto_remap_id_self))
				{
					if (!arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg))
					{
						Step->error_count_check++;
						if(Step->error_count_check > DEF_FEET_ERR_CHECK_TIME)
						{
							if (feet_alarm_enable)
							#ifdef NEW_ALARM_STYLE
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_WORK_INVALID);
							#else
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)2|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
							#endif
						}
					}else
					{
						Step->error_count_check = 0;
					}
				}
				else
				{
					if (!arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))
					{
						Step->error_count_check++;
						if(Step->error_count_check > DEF_FEET_ERR_CHECK_TIME)
						{	
							if (feet_alarm_enable)
							#ifdef NEW_ALARM_STYLE
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), STEP_ALARM_ZERO_INVALID);
							#else	
							alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (unsigned int)1|((unsigned int)Step->st_no_cl.bit.return_byte<<16));
							#endif
						
						}
					}else
					{
						Step->error_count_check = 0;					
					}
				}
			}
		}
	}
}
//unsigned int arch_Get_DirStatus(void);
//#ifndef PLATFORM_TMS2812
#if 0
short StepMotor_Feet_Get_Position(unsigned int feetid)
{
	int no;
	no =  StepMotor_get_no_with_IDself(feetid,MOTOR_TYPE_FEET);
	return StepMotor_Get_Position(no);
}

#endif
/*----------------------------------------------------------------------------*/
//#ifdef LX_ACTION_STEP_SPECIAL
void StepMotor_Type_exec(int mid, short arg, unsigned char step_type,unsigned char check,unsigned char  workspeed,unsigned char justcheckalert,unsigned char justmovezero)
{
	unsigned int stno;
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	stno = StepMotor_get_no_with_IDself(mid, step_type);
	//
	if(stno >= StepMotor_Count_MAX) return;	
	Step = &STEPMOTOR[stno];
	Step->st_no_cl.bit.return_byte = check;

	if(justcheckalert)
	{
		if(is_ACTION_Step_CheckRunTime(Step)||is_FEET_Step_CheckRunTime(Step))
		{				
			if((Step->step_st.bit.running)&&(arg == 1))
			{
				unsigned short step_count;
				step_count = abs(Step->pos_2 - Step->position);
				if(step_count > 4)
				{
					alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), (0x0E << 12) | step_count);
				}
				else
					goto check_it_right_now;	
			}
			else
			{
				check_it_right_now:
				Step->st_no_cl.bit.check_zero_work = check & 0x0f;
				if(arg==0)
				{
					Step->chk.bit.check_Right_Now=0;
					Step->chk.bit.check_sts_enable=1;
				}	
				else	if(arg==1)
				{						
					Step->chk.bit.check_Right_Now=1;
					check_ACT_Work_with_POS(Step);	
				}	
				
			}
		}
	}
	else
	{
		if ((step_type == MOTOR_TYPE_FEET)
			&&((Step->moto_remap_config.moto_attr.bit.moto_type_exe==FEET_MOTOR_TYPE_PT)
			||(Step->moto_remap_config.moto_attr.bit.moto_type_exe==FEET_MOTOR_TYPE_CX)))
			StepMotor_Feet_exec_no(stno, mid, arg,(unsigned short)((check>>4)&0x01));
		else
		{
		
			if (step_type == MOTOR_TYPE_ACTION)
			{
				#if 0
				STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
				if(stno >= StepMotor_Count_MAX) return;	
				Step = &STEPMOTOR[stno];
				#endif

				
				
				if ((Step->moto_remap_config.moto_attr.bit.moto_type_exe== ACT_MOTOR_TYPE_LX)
				||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == ACT_MOTOR_TYPE_FH)
				||(Step->moto_remap_config.moto_attr.bit.moto_type_exe == ACT_MOTOR_TYPE_DOT))
				{
					Step->st_no_cl.bit.check_zero_work = check & 0x0f;
					//Message_Send_4halfword(0x0104,(stno<<8)|0x01,Step->st_no_cl.bit.check_zero_work,0xFAFA);
				}
				else if(Step->moto_remap_config.moto_attr.bit.moto_type_exe == ACT_MOTOR_TYPE_PT)
				{
					unsigned char checktemp;
					checktemp = (arg<=0)?5:((arg>step_zero_adj[step_type-1])?4:0);	/*普通动作电机默认负数方向检测零位有信号*/
					if((check>>2) & 0x03) /*说明主控带下来检查了*/
					{
						if((check>>2) & 0x01) /*检查0位*/
						{
							if(check & 0x01)
							{
								checktemp |=((0x01<<2)|(0x01<<0));
							}
							else
							{
								checktemp |=(0x01<<2);
								checktemp &=~(0x01<<0);
							}
						}
						else
						{
							if(check & 0x02)
							{
								checktemp |=((0x01<<3)|(0x01<<1));
							}
							else
							{
								checktemp |=(0x01<<3);
								checktemp &=~(0x01<<1);
							}
						}
					}	
					Step->st_no_cl.bit.check_zero_work =checktemp & 0x0f;
				}
					

				
				Step->LX_act_just_move_zero =justmovezero?1:0;
				
			}
			if((step_type == MOTOR_TYPE_FEET) 
				&&((Step->moto_remap_config.moto_attr.bit.moto_type_exe ==FEET_MOTOR_TYPE_CX3 )
					||(Step->moto_remap_config.moto_attr.bit.moto_type_exe ==FEET_MOTOR_TYPE_CX2 )))
			{
				Step->st_no_cl.bit.check_zero_work = check & 0x0f;				
			}
			#ifdef TEST_STEPMOTOR_AUTOREPORT
			StepMotor_exec(stno, arg, 1,(unsigned short)(0x0001),workspeed);
			#else
			//Message_Send_4halfword(0xFFEE,(stno<<8)|0x01,step_type,0xFAFA);
			StepMotor_exec(stno, arg, 1,(unsigned short)((check>>4)&0x01),workspeed);
			#endif
		}
	}
}
//#else
#if 0
void StepMotor_Type_exec(int mid, short arg, unsigned char step_type)
{
	unsigned int stno;
	stno = StepMotor_get_no_with_IDself(mid, step_type);
	if (step_type == MOTOR_TYPE_FEET)
		StepMotor_Feet_exec_no(stno, mid, arg);
	else
		StepMotor_exec(stno, arg, 1);
}
#endif
void StepMotor_Type_Reset(unsigned int mid, unsigned char step_type,unsigned short otherarg)
{
	unsigned int st_no;  
	st_no = StepMotor_get_no_with_IDself(mid, step_type);
	StepMotor_Reset(st_no,((otherarg>>4)&0x0001));
}

void StepMotor_Type_ResetALL(int step_type,unsigned short otherarg)
{
	int i;
	STEP_TYPE *Step;
	
	for (i = 0; i < StepMotor_Count_MAX; i++)
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config != step_type)
			continue;
		
		StepMotor_Reset(i,((otherarg>>4)&0x0001));
	}
}

void StepMotor_Type_Set_Position(unsigned int sk_id, short pos, unsigned char step_type)
{
	int no;
	no = StepMotor_get_no_with_IDself(sk_id,step_type);
	StepMotor_Set_Position(no, pos);
}

/*----------------------------------------------------------------------------*/
#if 0
int StepMotor_Sinker_Get_Busy()
{

	int i;
	int ret = 0;
	STEP_TYPE *Step;

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		Step = &STEPMOTOR[i];

		if(Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_SKINER)
			continue;
		if(Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if(!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			continue;		

		if(StepMotor_Get_Busyidx(i)) {
			ret = 0xFFFF;
			break;
		}
	}
	return ret;
}
#endif

#ifdef E480_BOARD_V10
int is_YARN_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== YARN_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;

}

int is_LX_yarn_step(int stepno)  /*连兴沙嘴，类似于推针*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== YARN_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;
}

int is_LX_yarn_step_exe(STEP_TYPE *Step_ex)  /*连兴沙嘴，类似于推针*/
{
	if (Step_ex==NULL ) return 0;
	
		if ((Step_ex->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_YARN)
			&&(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe== YARN_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;
}


#endif




int is_Feet_Step(int stepno)		/*压脚电机*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)
			return 1;
		else
			return 0;

}





int is_EX_OTHER_Step(int stepno)		/*紧编电机，零位一个点*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe!= LIFT_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;

}


int is_DOT_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== ACT_MOTOR_TYPE_DOT))
			return 1;
		else
			return 0;

}



int is_HF_LIFT_Step(int stepno)		/*浩丰的推针电机*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe!= LIFT_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;

}

int is_HF_ex_LIFT_Step(int stepno)		/*新型的推针电机，中间很宽*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== LIFT_MOTOR_TYPE_HF_EX))
			return 1;
		else
			return 0;

}



int is_LIFT_or_Other_Step(int stepno)		/*推针电机--做连杆电机用，只有一个零位传感器，负方向一直有效*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
			||(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER))
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== LIFT_MOTOR_TYPE_PT)
			)
			return 1;
		else
			return 0;

}



int is_LIFT_Step(int stepno)		/*推针电机--做连杆电机用，只有一个零位传感器，负方向一直有效*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe== LIFT_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;

}



int is_LIFT_all_Step(int stepno)		/*推针电机*/
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_LIFT)
			
			return 1;
		else
			return 0;

}




int is_DENSITY_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_DENSITY)
			return 1;
		else
			return 0;

}

int is_JF_DENSITY_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_DENSITY)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==DENSITY_MOTOR_TYPE_JF))
			return 1;
		else
			return 0;
}


int is_LX_DENSITY_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_DENSITY)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe == DENSITY_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;

}

int is_LX_DENSITY_Step_ex(STEP_TYPE *Step_ex)
{
	if ((Step_ex->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_DENSITY)
			&&(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe == DENSITY_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;
}

int is_ACTION_Step_CheckRunTime(STEP_TYPE *Step_ex)
{
	if ((Step_ex->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_ACTION) 
		&&((Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_LX)
			||(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_FH)
			||(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_PT)
			||(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_DOT)))
		return 1;
	else
		return 0;
}

int is_FEET_Step_CheckRunTime(STEP_TYPE *Step_ex)
{
	if ((Step_ex->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_FEET) 
		&&((Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==FEET_MOTOR_TYPE_CX2)
		||(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe==FEET_MOTOR_TYPE_CX3)))
		return 1;
	else
		return 0;
	
}

int is_CX2_Feet_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe ==FEET_MOTOR_TYPE_CX2))
			return 1;
		else
			return 0;
}

int is_CX3_Feet_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_FEET)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe ==FEET_MOTOR_TYPE_CX3))
			return 1;
		else
			return 0;
}


int is_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			return 1;
		else
			return 0;

}

int is_PT_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;
}

int is_PTorFH_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_PT)
			||(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_FH)))
			return 1;
		else
			return 0;
}




int is_LX_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;
}



int is_HP_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_HP))
			return 1;
		else
			return 0;
}

int is_FH_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe==ACT_MOTOR_TYPE_FH))
			return 1;
		else
			return 0;
}


int is_sinker(int stepno)
{
		if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			return 1;
		else
			return 0;
}


int is_PT_sinker(int stepno)
{
		if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_PT))
			return 1;
		else
			return 0;
}

int is_lxex_sinker(int stepno)
{
		if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_LX_EX))
			return 1;
		else
			return 0;

}


int is_lx_sinker(int stepno)
{
		if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			&&(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;

}

int is_lx_sinker_ex(STEP_TYPE *Step_ex)
{
	if ((Step_ex->moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			&&(Step_ex->moto_remap_config.moto_attr.bit.moto_type_exe == SINKER_MOTOR_TYPE_LX))
			return 1;
		else
			return 0;
}

#if 0
int is_sinker_or_other(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_SKINER)
			||(STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER))
			return 1;
		else
			return 0;
}

#endif

int is_other_step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_OTHER)
			return 1;
		else
			return 0;
}

int check_sinker_done(void)
{
	int i;
	int step_count;
	STEP_TYPE *Step;


	for(i = 0; i < StepMotor_Count_MAX; i++)	
	{
		Step = &STEPMOTOR[i];

		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_SKINER)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			continue;
		
		if(Step->step_st.bit.running)
		{
			if(Step->pos_2 > Step->position)
				step_count = Step->pos_2 - Step->position;
			else
				step_count = Step->position - Step->pos_2;
			if(step_count > 4)
				alert_push(SINKER_DONE_ERR, (Step->moto_remap_config.moto_remap_id_self << 12) | step_count);
		}
	}
	return 0;
}

/*
* left_right: 0 left, 0-2-4-6; 1 right, 1-3-5-7
*/
int check_sti_done(int left_right)     
{
	int i;
	int sts = 0;
	unsigned int stepid;
	STEP_TYPE *Step;

	for(i = 0; i < StepMotor_Count_MAX; i++)
	{
		Step = &STEPMOTOR[i];		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_DENSITY)
			continue;
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			continue;
		
		stepid = Step->moto_remap_config.moto_remap_id_all;

		if(left_right)    //奇数
		{
			if (stepid & 0x01) 
			{
				if( Step->step_st.bit.running)
				sts |= (0x1 << stepid);
			}
			else
				continue;
		}
		else
		{
			if ( !(stepid & 0x01))	//偶数 
			{
				if(Step->step_st.bit.running)
				sts |= (0x1 << stepid);
			}
			else
				continue;
		}
	}
	if(sts)
		alert_push(STI_DONE_ERR, sts);

	return 0;
}





#ifdef E480_BOARD_V10

void arch_YARN_Step_Setup(int yno, int onoff)
{
	unsigned int step_id;
	unsigned short tmp_stus;

	unsigned short last_stus;

	int moto_pos;	

	/*---2020-02-27 by hlc 慈星沙嘴电机做上下动作。----*/
	
	if(Yarn_Step_Updown) 
	{
		return;
	}
	/*------*/

	step_id=(yno>>1);		   
	{
		tmp_stus = emf_status[2];
		last_stus =emf_status[2];
		if(onoff&0xFFFF)
		{
			tmp_stus|=(1<<yno);
		}
		else
		{
			tmp_stus&=(~(1<<yno));
		}
		if(tmp_stus!=last_stus)
		{
		       int twoyarnsts;		
			emf_status[2] =	tmp_stus;
			twoyarnsts=(tmp_stus>>(2*step_id))&0x03;
			switch(twoyarnsts)
			{
			case 0x0:
			{
				int justselfyarn = (last_stus>>(2*step_id))&0x03;   //之前的状态
				switch(justselfyarn)
				{
				case 0x01:
				{
					{
				       	moto_pos=800-POS_Yarn_DO;
				     	}
					emf_status[2] |=(1<<(yno+1));	
				}
				break;
				case 0x02:
				{
					
				     	{
				        	moto_pos=POS_Yarn_DO;
				    	}
					emf_status[2] |=(1<<(yno-1));	
				}
				break;
				default:
					moto_pos=400;								
				break;
				}				    
			}
			break;
			case 0x1:					
			{
				moto_pos=POS_Yarn_DO;
			}						
			break;	
			case 0x2:				    
			{
				moto_pos=800-POS_Yarn_DO;
			}					 
			break;
			case 0x3:
				moto_pos=0;
				break;					 
			default:
				break;
			}

			step_id = StepMotor_get_no_with_IDself(step_id,MOTOR_TYPE_YARN);	
			
			if(STEPMOTOR[step_id].step_st.bit.is_poweron ==0)
			{
			     
			         StepMotor_Reset(step_id,0);
				 STEPMOTOR[step_id].need_2_pos_after_reset = moto_pos;
			}
			else
			{
			         StepMotor_exec(step_id,moto_pos,1,0,0);    
				      					   
			}
		}
	}
	

}


void Exec_SET_DCT_sts(int arg2,int arg3)
{
	//extern  int stepspeed_Q2R(int Q,int whichstep);
	if (((arg2 & 0x00ff)==(arch_board_id+0x10))&&(Yarn_use_Step))
	{
								unsigned short i;
								unsigned short st_id;
								emf_status[2] = arg3 & 0xFFFF;
               						//根据dct_stus 设置电机pos值
               						for (i = 0;i<4;i++)
               						{
               							//st_id = i+8;
									st_id = StepMotor_get_no_with_IDself(i,MOTOR_TYPE_YARN);		
									switch((emf_status[2]>>(i*2)) & 0x03)
									{	
										case 0:
											STEPMOTOR[st_id].position= stepspeed_Q2R(800>>1,MOTOR_TYPE_YARN);
											STEPMOTOR[st_id].pos_2 = stepspeed_Q2R(800>>1,MOTOR_TYPE_YARN);											
											break;
										case 1:											
				     							{
				          							STEPMOTOR[st_id].position= stepspeed_Q2R(POS_Yarn_DO,MOTOR_TYPE_YARN);
												STEPMOTOR[st_id].pos_2 = stepspeed_Q2R(POS_Yarn_DO,MOTOR_TYPE_YARN);	
				     							}											
				     							break;
										case 2:											
				     							{
				          							STEPMOTOR[st_id].position= stepspeed_Q2R(800-POS_Yarn_DO,MOTOR_TYPE_YARN);
												STEPMOTOR[st_id].pos_2 = stepspeed_Q2R(800-POS_Yarn_DO,MOTOR_TYPE_YARN);		
				     							}											
											break;
										case 3:	
											 STEPMOTOR[st_id].position= 0;
											STEPMOTOR[st_id].pos_2 = 0;											 
											break;
									}
									STEPMOTOR[st_id].step_st.bit.is_poweron =1;
								}									
	   }
}

void Exec_GET_Moto_Zero_Width(int arg2)
{
	unsigned int tx_data[4]={0,0,0,0};
	if (((arg2 & 0x00ff)==(arch_board_id+0x10))&&(Yarn_use_Step))
	{
		 int sk_no;
		 unsigned short whichyarn =((arg2>>8) & 0xFF);
		
		 if (whichyarn>=8) return;
		 sk_no = StepMotor_get_no_with_IDself(whichyarn>>1,MOTOR_TYPE_YARN);	
		
               tx_data[0]=STEPMOTOR[sk_no].moto_zero_width;
               tx_data[1]=((arch_board_id+0x10)<<8) | whichyarn;

		//arch_SendMessage(1,&tx_data[0],2); 	
		Message_Send_4halfword(0x04|(0x0A<<8),tx_data[0],tx_data[1],0);
           //can_fifo_Push(&can_tx_fifo, &tx_data);
	}
}

void Exec_SET_Moto_Zero_Width(int arg2,int arg3)
{

	if (((arg2 & 0x00ff)==(arch_board_id+0x10))&&(Yarn_use_Step))
	{
		int sk_no;
		 unsigned short whichyarn =((arg2>>8) & 0xff);
		 
		if (whichyarn>=8) return;
		sk_no = StepMotor_get_no_with_IDself(whichyarn>>1,MOTOR_TYPE_YARN);	

		if ((arg3<=50)&&(arg3>0))		
              	STEPMOTOR[sk_no].moto_zero_width = arg3;
               					
	}
}


void Exec_Reset_step_moto_one(int arg2,unsigned short otherarg)
{
	int sk_no;
	if (((arg2 & 0x00ff)==(arch_board_id+0x10))&&(Yarn_use_Step))
	{
		unsigned short whichyarn =((arg2>>8) & 0xff);
		
		if (whichyarn>=8) return;
		sk_no = StepMotor_get_no_with_IDself(whichyarn>>1,MOTOR_TYPE_YARN);	
		
              StepMotor_Reset(sk_no,((otherarg>>4)&0x0001));               					
	}

}

void Exec_Reset_step_moto_all(int arg2,unsigned short otherarg)
{

	unsigned short i;
	STEP_TYPE *Step;
	if (Yarn_use_Step)
	{
		for (i=0;i<StepMotor_Count_MAX;i++)
		{
			
			Step=&STEPMOTOR[i];
			//myprintf("\r\n==     Step-is YARN[%d][%d]                       ==",i,Step->moto_remap_config.moto_attr.bit.moto_type_config);
			if(Step->moto_remap_config.moto_attr.bit.moto_type_config !=MOTOR_TYPE_YARN)
			continue;
			StepMotor_Reset(i,((otherarg>>4)&0x0001));
		}   
	}

}


void StepMotor_Setup_Yarnstep_Active(unsigned int yarnstep_active)
{
	unsigned short i;
	unsigned char stepid;
	STEP_TYPE *Step;
	if (Yarn_use_Step)
	{
		for (i=0;i<StepMotor_Count_MAX;i++)
		{
			Step = &STEPMOTOR[i];
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_YARN)
			continue;
			stepid =Step->moto_remap_config.moto_remap_id_self;
			Step->moto_remap_config.moto_attr.bit.is_activestep_enable = (yarnstep_active>>stepid) & 0x01;
		}   
	}	
}

int arch_get_yarnstep_sign_()
{
	unsigned int ret = 0;
	int i;
	unsigned char stepid;
	STEP_TYPE *Step;
	if (Yarn_use_Step)
	{
		for (i=0;i<StepMotor_Count_MAX;i++)
		{
			Step = &STEPMOTOR[i];
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_YARN)
				continue;
			stepid =Step->moto_remap_config.moto_remap_id_self;
			ret |= ((!arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg))<<stepid);
		}
	}
	return ret;
}


#endif


void Exec_Set_Step_check_Type(unsigned int whichstep,unsigned int stepno,unsigned char c_type)
{
	//unsigned int ret = 0;
	int i;
	//unsigned char stepid;
	STEP_TYPE *Step;
	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];

		if (whichstep !=0xff)
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=whichstep)
				continue;
		}
		if (stepno !=0xff)
		{
			if (Step->moto_remap_config.moto_remap_id_self !=stepno)
				continue;
		}
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config != MOTOR_TYPE_UNDEF)
			Step->moto_remap_config.moto_attr.bit.check_type = c_type;
	}
	
	return ;
	
}

void Exec_Set_DDM_need_Check(unsigned int ischeck)
{
	Sys_Step_need_check_ddm = (ischeck==0)?0:ischeck;	
}

#if 0
void Exec_Set_LX_DM_special(unsigned int issp)
{
	unsigned int ret = 0;
	int i;
	unsigned char stepid;
	STEP_TYPE *Step;
	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_DENSITY)
			continue;
		
		Step->moto_remap_config.moto_attr.bit.moto_type_exe = issp?DENSITY_MOTOR_TYPE_LX:DENSITY_MOTOR_TYPE_PT;	
		
	}
	
	return ;

}

#endif
#ifdef QL_DEBUG_STEP_MOTOR_ECORD

unsigned char stepno_debug=0;
short steps_debug_pos2=0;
unsigned char test_begin=0;




void Step_debug_set_par(unsigned char stepno_id,short pos)
{
	stepno_debug = stepno_id;
	steps_debug_pos2 = pos;
	test_begin =1;
}


void Step_debug_goto_()
{
	static unsigned char step_debug_case=0;
	short ss;
	STEP_TYPE *Step;
	if (test_begin)
	{
		if (stepno_debug<StepMotor_Count_MAX)
		{
			Step = &STEPMOTOR[stepno_debug];
			if (Step->step_st.bit.running)
				return;
			if (step_debug_case<5)
			{
				ss = (short)((steps_debug_pos2 *  (step_debug_case+1)) /5);
				StepMotor_exec(stepno_debug,ss,1,0,0);
				step_debug_case++;
			}
			else
			{
				step_debug_case =0;
				test_begin =0;
			}		
		}
	}

}

#endif

#if 0
void Exec_Set_ACT_Motor_current_half(unsigned char stepno,unsigned char ishalf)
{
	if ((STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_ACTION)
		&&(Head_Mode_ == HEAD_MODE_LIFT_EX))
	{
		if (ishalf)
		{
			DAC_SetVoltage_channel2(800);
		}
		else
		{
			DAC_SetVoltage_channel2(stepmotor_current[MOTOR_TYPE_ACTION-1]);	
		}
	}
}

#endif

void check_Lift_step_iswork_(unsigned char stepno)
{
	int i;	
	STEP_TYPE *Step;
	//int step_count;

	//alert_push(LIFT_STEP_DONE_ERR, (Step->moto_remap_config.moto_remap_id_self << 12) | step_count);
	//return;
	
	i = StepMotor_get_no_with_IDself(stepno,MOTOR_TYPE_LIFT);
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_LIFT)
			return ;	
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			return ;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			return ;
		
		//if (Step->step_st.bit.running)
		{
			short whichpos;
			unsigned short ac;
			whichpos = Step->step_st.bit.running?Step->pos_2:Step->position;
			ac =STEP_CHECK_WORKPOS+(Step->step_st.bit.running?1:0);
			if(!((whichpos > 70) || (whichpos <(0-70))))
			//if(step_count > 4)
			{
				alert_push(ac, (Step->moto_remap_config.moto_remap_id_self << 12) | whichpos);
				//continue;
			}
		}
	
		
	}
	
	return ;
	
}

void check_lift_step_isstop()
{
	int i;	
	STEP_TYPE *Step;
	int step_count;

	//alert_push(LIFT_STEP_DONE_ERR, (Step->moto_remap_config.moto_remap_id_self << 12) | step_count);
	//return;
	
	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_LIFT)
			continue;	
		if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
			continue;
		if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
			continue;
		
		if (Step->step_st.bit.running)
		{
			//if(Step->pos_2 > Step->position)
				step_count = abs(Step->pos_2 - Step->position);
			//else
				//step_count = Step->position - Step->pos_2;
			if(step_count > 4)
			{
				alert_push(LIFT_STEP_DONE_ERR, (Step->moto_remap_config.moto_remap_id_self << 12) | step_count);
				continue;
			}
		}	

		if ((Step_check_is_check >>i)& 0x01)	/*说明设置了检查*/
		{
			step_count = abs(Step_postion_set[i][Step->check_pos_index] - Step->position);
			if(step_count > 4)
				alert_push(STEP_CHECK_ALERT, (Step->moto_remap_config.moto_remap_id_self << 12) | step_count);
			Step_check_is_check &=~(0x01<<i);
		}
		
	}
	
	return ;
}


void Set_step_alert_st_ecoder(unsigned char stepno,unsigned char st)
{
	if(stepno >= StepMotor_Count_MAX) return;
	if (STEPMOTOR[stepno].step_alert_st.bit.ecord_input_st!= (st?1:0))
	{
		STEPMOTOR[stepno].step_alert_st.bit.ecord_input_st = (st?1:0);
	}
}

void Exec_Set_Step_ResetDir_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short dirmask=mask_;
	unsigned short dirdata=data_;
	unsigned char whichbitno;
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((dirmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichbitno = Step->moto_remap_config.moto_remap_id_all;
			//Step->step_st.bit.dir_High = ((dirdata >> whichbitno) & 0x01)?0:1;	
			Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =  ((dirdata >> whichbitno) & 0x01)?1:0;
			if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
			{
				Step->moto_remap_config.moto_attr.bit.zero_dir_isset = 1;
			}
			#ifdef ENCODER_SUPPORT
			Encoder_setDir(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, !Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir,0);
			#endif /* ENCODER_SUPPORT */
			
		}
		else
		{	
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
				continue;	
			whichbitno = Step->moto_remap_config.moto_remap_id_self;
			if ((dirmask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir =  ((dirdata >> whichbitno) & 0x01)?1:0;
				if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
				{
					Step->moto_remap_config.moto_attr.bit.zero_dir_isset = 1;
				}
				#ifdef ENCODER_SUPPORT
				Encoder_setDir(Step->moto_remap_config.moto_attr.bit.moto_ecode_index, !Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir,0);
				#endif /* ENCODER_SUPPORT */
			}
			else
				continue;
		}
		
	}
	
	return ;
	
}

void Exec_Set_Step_RunDir_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short dirmask=mask_;
	unsigned short dirdata=data_;
	unsigned char whichbitno;
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((dirmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichbitno = Step->moto_remap_config.moto_remap_id_all;
			Step->step_st.bit.dir_High = ((dirdata >> whichbitno) & 0x01)?0:1;				
			if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
			{
				Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
			}
			
		}
		else
		{	
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
				continue;	
			whichbitno = Step->moto_remap_config.moto_remap_id_self;
			if ((dirmask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				Step->step_st.bit.dir_High = ((dirdata >> whichbitno) & 0x01)?0:1;		
				if (!Step->moto_remap_config.moto_attr.bit.zero_dir_isset)
				{
					Step->moto_remap_config.moto_attr.bit.zero_is_positive_dir = !Step->step_st.bit.dir_High;
				}
			}
			else
				continue;
		}
		
	}
	
	return ;
	
}

void Exec_Set_Step_Enable_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short enablemask=mask_;
	unsigned short enabledata=data_;
	unsigned char whichbitno;
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];

		if (motortype_h==MOTOR_TYPE_0XFF) //说明是指定某几个物理电机
		{
			if (!((enablemask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichbitno = Step->moto_remap_config.moto_remap_id_all;
			Step->moto_remap_config.moto_attr.bit.is_activestep_enable = ((enabledata >> whichbitno) & 0x01)?1:0;
		}
		else
		{
				
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
			continue;	
	
		whichbitno = Step->moto_remap_config.moto_remap_id_self;

		if ((enablemask >> whichbitno) & 0x01) /*yes you are mask enable*/
		{
			Step->moto_remap_config.moto_attr.bit.is_activestep_enable = ((enabledata >> whichbitno) & 0x01)?1:0;
		}
		else
			continue;
		}
		
	}
	
	return ;
	
}




void Exec_Set_Step_Fastmode_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short fastmask=mask_;
	unsigned short fastdata=data_;
	unsigned char whichbitno;
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h == MOTOR_TYPE_0XFF)
		{
			if (!((fastdata>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichbitno = Step->moto_remap_config.moto_remap_id_all;
			Step->moto_remap_config.moto_attr.bit.is_fast_mode= ((fastdata >> whichbitno) & 0x01)?1:0;
		}
		else
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
				continue;	
		
			whichbitno = Step->moto_remap_config.moto_remap_id_self;

			if ((fastmask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				Step->moto_remap_config.moto_attr.bit.is_fast_mode= ((fastdata >> whichbitno) & 0x01)?1:0;
			}
			else
				continue;
		}
		
	}
	
	return ;
	
}

void Exec_Set_Step_Poweron_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short fastmask=mask_;
	unsigned short fastdata=data_;
	unsigned char whichbitno;
	unsigned char whichno;
	
	int i;	
	STEP_TYPE *Step;



	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h==MOTOR_TYPE_0XFF) //说明是指定某几个物理电机
		{
			if (!((fastmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichbitno = Step->moto_remap_config.moto_remap_id_all;
			
			 if ((fastdata >> whichbitno) & 0x01) 
			 {
				arch_StepMotor_Enable_onestepmoto(whichbitno);
			 }
			 else
			 {
				arch_StepMotor_Disable_onestepmoto(whichbitno);
			 }
			
		}
		else
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
				continue;	
			whichbitno = Step->moto_remap_config.moto_remap_id_self;
			if ((fastmask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				whichno = Step->moto_remap_config.moto_remap_id_all;
				 if ((fastdata >> whichbitno) & 0x01) 
				 {
					arch_StepMotor_Enable_onestepmoto(whichno);
				 }
				 else
				 {
					arch_StepMotor_Disable_onestepmoto(whichno);
				 }
			}
			else
				continue;
		}
	}
	
	return ;
}



void Exec_set_Step_Current_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	extern void Exec_Set_Motor_Curr_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short cruu);
	unsigned short currmask=mask_;
	unsigned short currdata=data_;
	unsigned char whichbitno;
	unsigned char whichno;
	
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];

		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((currmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichno = Step->moto_remap_config.moto_remap_id_all;
			whichbitno = Step->moto_remap_config.moto_attr.bit.moto_type_config;
			Exec_Set_Motor_Curr_with_stepno(whichbitno,whichno,currdata);
		}
		else
		{
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
			continue;	
			whichbitno = Step->moto_remap_config.moto_remap_id_self;
			if ((currmask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				whichno = Step->moto_remap_config.moto_remap_id_all;
			 	Exec_Set_Motor_Curr_with_stepno(motortype_h,whichno,currdata);
			}
			else
			continue;
		}
		
	}
	
	return ;
}


void Exec_set_Step_ResetSpeed_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	extern void Exec_Set_Motor_RestSpeed_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short speed);
	unsigned short rsmask=mask_;
	unsigned short rsdata=data_;
	unsigned char whichbitno;
	unsigned char whichno;
	
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((rsmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichno = Step->moto_remap_config.moto_remap_id_all;
			whichbitno = Step->moto_remap_config.moto_attr.bit.moto_type_config;
			 Exec_Set_Motor_RestSpeed_with_stepno(whichbitno,whichno,rsdata);
		}
		else
		{
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
			continue;	
	
		whichbitno = Step->moto_remap_config.moto_remap_id_self;
		

		if ((rsmask >> whichbitno) & 0x01) /*yes you are mask enable*/
		{
			whichno = Step->moto_remap_config.moto_remap_id_all;
			 Exec_Set_Motor_RestSpeed_with_stepno(motortype_h,whichno,rsdata);
		}
		else
			continue;
		}	
		
	}
	
	return ;
}


void Exec_set_Step_RunSpeed_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	extern void Exec_Set_Motor_RunSpeed_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short speed);
	unsigned short rsmask=mask_;
	unsigned short rsdata=data_;
	unsigned char whichbitno;
	unsigned char whichno;
	
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];

		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((rsmask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichno = Step->moto_remap_config.moto_remap_id_all;
			whichbitno = Step->moto_remap_config.moto_attr.bit.moto_type_config;
			 Exec_Set_Motor_RunSpeed_with_stepno(whichbitno,whichno,rsdata);
		}
		else
		{
		
		
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
			continue;	
	
		whichbitno = Step->moto_remap_config.moto_remap_id_self;
		

		if ((rsmask >> whichbitno) & 0x01) /*yes you are mask enable*/
		{
			whichno = Step->moto_remap_config.moto_remap_id_all;
			 Exec_Set_Motor_RunSpeed_with_stepno(motortype_h,whichno,rsdata);
		}
		else
			continue;
		}
		
	}
	
	return ;
}

void Exec_Set_Step_LostAlert_with_motortype(unsigned char motortype_h,unsigned short mask_,unsigned short data_)
{
	unsigned short lamask=mask_;
	unsigned short ladata=data_;
	unsigned char whichbitno;
	unsigned char whichno;
	int i;	
	STEP_TYPE *Step;
	

	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = &STEPMOTOR[i];
		if (motortype_h==MOTOR_TYPE_0XFF)
		{
			if (!((lamask>>i) & 0x0001))   /*没有使能*/
			{
				continue;
			}
			whichno = Step->moto_remap_config.moto_attr.bit.moto_type_config;
			Step_Motor_Set_Check_all(whichno,ladata);
		}
		else
		{		
			if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=motortype_h)
				continue;	
		
			whichbitno = Step->moto_remap_config.moto_remap_id_self;
			

			if ((lamask >> whichbitno) & 0x01) /*yes you are mask enable*/
			{
				whichno = Step->moto_remap_config.moto_remap_id_all;
				 //Exec_Set_Motor_RunSpeed_with_stepno(motortype_h,whichno,otherdata);
				 Step_Motor_Set_Check_all(motortype_h,ladata);
			}
			else
				continue;
		}
		
	}
	
	return ;

}


void Exec_Set_Step_Parameter_with_motortype(unsigned char para_type,unsigned char motortype_main,unsigned short mask_,unsigned short data_)
{
	unsigned char motortype_head ;
	if (motortype_main==MOTOR_TYPE_0XFF)
		motortype_head = motortype_main;
	else
		motortype_head= motor_type_remap[motortype_main];
	
	switch (para_type)
	{
		case 0x00:
			Exec_Set_Step_Poweron_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x01:
			Exec_Set_Step_RunDir_with_motortype(motortype_head,mask_,data_);				
			break;
		case 0x02:
			Exec_Set_Step_Enable_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x03:
			Exec_Set_Step_Fastmode_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x04:
			Exec_Set_Step_LostAlert_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x05:
			Exec_set_Step_Current_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x06:
			Exec_set_Step_ResetSpeed_with_motortype(motortype_head,mask_,data_);
			break;	
		case 0x07:
			Exec_set_Step_RunSpeed_with_motortype(motortype_head,mask_,data_);
			break;
		case 0x08:
			Exec_Set_Step_ResetDir_with_motortype(motortype_head,mask_,data_);
			break;
		default:
			
			break;		
	}
	
}

#ifdef DEBUG_STEP_OPERATE_GONO_ERROR

unsigned char goton_cnt=0;
unsigned char goton_no=0;


void debug_motor_moveto_0_and_goto_N_dir(unsigned char whichstepno)
{
		goton_cnt = 10;
		goton_no = whichstepno;
		StepMotor_Reset(whichstepno,0);
}

void check_gono_cnt()
{
	if (goton_cnt)
	{
		goton_cnt--;
		if (goton_cnt) 
			return;
		StepMotor_exec(goton_no,200,0,1,0);
		
	}
	else
		return;
}

#endif


void Set_Motor_steps_for_ACC_DEC(unsigned short acc_steps)
{
	if (acc_steps) 
	Motor_acc_steps=acc_steps;
}


void Set_Step_Motor_postion_ex(unsigned char steptype,unsigned char stepno,unsigned char dataindex,short data_pos )
{
	if ((steptype>0)&&(steptype<MOTOR_TYPE_MAX) && (dataindex<8))
	{
		unsigned int stepid;
		stepid = StepMotor_get_no_with_IDself(stepno,steptype);
		if (stepid<STEP_NUM)
			Step_postion_set[stepid][dataindex] = data_pos;

	}	
	
}

void Set_Step_Motor_TO_Which_Postion(unsigned char steptype,unsigned short dataindex)
{
	if ((steptype>0)&&(steptype<MOTOR_TYPE_MAX))
	{
		unsigned int stepid;
		STEP_TYPE *Step;
		int i;
		unsigned short idx;
		for (i=0;i<((steptype==MOTOR_TYPE_YARN)?4:2);i++)
		{
			stepid = StepMotor_get_no_with_IDself(i,steptype);
			if (stepid<STEP_NUM)
			Step = (STEP_TYPE *)Get_STEPMOTO_add(stepid);
			idx = (dataindex>>(i<<2)) & 0x000F;
			Step->check_pos_index = idx&0x07; 
			Step_check_is_check |=(0x01<<stepid);
		}
	}
}

#ifdef LOG_DEBUG_FOR_LX_AT_CH
void time_250us_zero_change(unsigned int bt,unsigned int dir)
{
	static unsigned int zero_cach=0;
	static unsigned short idx=0;
	unsigned int zero_temp=0;
	int i;
	unsigned short D[3];
			
	for (i=0;i<20;i++)
	{
		if(arch_StepMotor_Zero(i))
		{
			zero_temp |=(1<<i);
		}
	}
	if(zero_cach!=zero_temp)
	{
		if(enable_log_zero_change)
		{
			D[0]=LX_LOG_TYPE_ZERO_CHANGE<<8;
			//D[0]|=basetime_tick & 0xFF;
			D[0]|=dir?1:0;
			D[1]=zero_temp & 0xFFFF;
			D[2]=bt & 0xFFFF;//(zero_temp>>16) & 0xFF;
			//D[2]|=(idx++)<<8;
			//D[]=
			Message_send_log_LX(D[0],D[1],D[2]);
			
		}
		zero_cach = zero_temp;
	}
	

}
#endif


void Set_LX_ex_adjData(unsigned char steptype,unsigned short data)
{
	if(steptype==MOTOR_TYPE_SKINER)
	{
		step_zero_adj_sk_lxex = data;
	}
	if(steptype==MOTOR_TYPE_ACTION)
	{
		if((data>10)&&(data<800))
		{
			step_N_act_interval_lx = data;
		}
	}	
}


void StepMotor_Speed_report(unsigned short retcmd,unsigned char typemain,unsigned char steptype)
{
	if ((steptype>0)&&(steptype<MOTOR_TYPE_MAX))
	{
		Message_Send_4halfword(retcmd,typemain | (stepmotor_AccSteps[steptype-1]<<8),step_base_speed[steptype-1],stepmotor_speed[steptype-1][1]);
	}	
}


void StepMotor_Set_Density_check_zero_data(unsigned short czd)
{
	Density_zero_check = czd;	
}


void arch_StepMotor_DD_printf(STEP_TYPE * Step)
{
	int i;
	if(Step->moto_dd_config.DD_last_speed[Step->moto_dd_config.DD_last_wp]==0)/*没有兜圈*/
	{
		for(i=0;i<((Step->moto_dd_config.DD_last_wp+2) /3);i++)
		{
			unsigned short data_temp[3];
			data_temp[0]= Step->moto_dd_config.DD_last_speed[i*3+0];
			data_temp[1]= Step->moto_dd_config.DD_last_speed[i*3+1];
			data_temp[2]= Step->moto_dd_config.DD_last_speed[i*3+2];
			Message_Send_4halfword(0xEE|(i<<8),data_temp[0],data_temp[1],data_temp[2]);
		}
	}
	else{
		
		int ptr=Step->moto_dd_config.DD_last_wp;
		for(i=0;i<6;i++)
		{
			unsigned short data_temp[3];
			int xx ;
			int k;

			for (k=0;k<3;k++)
			{
				xx= i*3+ptr+k;
				if(xx>=16)
				{
					xx-=16;
				}

				if(xx>=16)
				{
					data_temp[k] =0;
				}	
				else
					data_temp[k] =  Step->moto_dd_config.DD_last_speed[xx];
				
			}			
			
			Message_Send_4halfword(0xEE|(i<<8),data_temp[0],data_temp[1],data_temp[2]);
		
		}

	}

	
}


void StepMotor_set_maxspeed_DD_HZ(unsigned short maxspeed)
{
	if((maxspeed>100)&&(maxspeed<10000))
		stepmotor_speed_max_DD = maxspeed;

	#ifdef SSTI_PULS_SMOOTH
	if(stepmotor_speed_max_DD > DEFSTEPMOTOR_MAX_SPEED_HZ)
	{
		scsim_max_acc = DEF_STEPMOTOR_MAX_ACC_SPEED_HZ+(stepmotor_speed_max_DD-DEFSTEPMOTOR_MAX_SPEED_HZ)	/10;	
		if(scsim_max_acc>250)
			scsim_max_acc=250;
		
		scsim_max_dec = DEF_STEPMOTOR_MAX_DEC_SPEED_HZ -(stepmotor_speed_max_DD-DEFSTEPMOTOR_MAX_SPEED_HZ)	/6;
		if(scsim_max_dec<-250)
			scsim_max_dec=-250;
	}
	#endif
}



void StepMotor_Set_MAXSpeed_DD(unsigned short speed_hz)
{
#ifdef STEP_MOTOR_DDDM_SUPPORT

	
	if (speed_hz<SPET_MOTOR_MIN_SPEED_DD_HZ)
	{
		speed_hz= SPET_MOTOR_MIN_SPEED_DD_HZ;
	}
	if (speed_hz>SPET_MOTOR_MAX_SPEED_DD_HZ)
	{
		speed_hz= SPET_MOTOR_MAX_SPEED_DD_HZ;
	}
	stepmotor_speed_max_DD=speed_hz;
	
#else
	return;
#endif

}

void Motor_is_binding_exec_set(unsigned char motorno,unsigned char isset)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	if(motorno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[motorno];
	Step->isdoing_with_bind_cmd= isset;
}

unsigned char Motor_is_doing_bind_cmd(unsigned char motorno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	if(motorno >= StepMotor_Count_MAX) return 0;
	Step = &STEPMOTOR[motorno];
	return Step->isdoing_with_bind_cmd;
}


void Motor_bind_cmd_set(unsigned short idx,unsigned char motor1,unsigned char motor2,short pos1,short pos2)
{
	unsigned char i;
	
	STEP_BIND_cmd *bindcmd;
	//if(motor1==motor2) return;
	if((motor1>3)||(motor2>3)||(motor1==motor2))return;
	for(i=0;i<2;i++)
	{
		bindcmd = &DM_motor_bind_cmd[i];
		if((bindcmd->Cmd_step==0)||(bindcmd->Cmd_step==0xff))
		{
			break;	
		}
	}
	if(i>=2)
	{
			/*报警*/
		//if(error_no_bind)
		if(motor1<4)
			alert_push(MOTOR_BIND_CMD_ERR+motor1,0x03);		
	
	}
	else
	{
		bindcmd = &DM_motor_bind_cmd[i];
		bindcmd->cmdID = idx;
		bindcmd->Motor_NO1 = motor1;
		bindcmd->Motor_NO2 = motor2;
		bindcmd->Targer_pos_1= pos1;
		bindcmd->Targer_pos_2= pos2;
		bindcmd->Exec_TimeOut = 1000>>1;/*1000ms 用2ms中断做计时*/
		bindcmd->Delay_timeout_2ms=0;
		bindcmd->Cmd_step=1;		
	}
}

void Motor_bind_timer_isr()
{
	if(DM_motor_bind_cmd[0].Delay_timeout_2ms)
		DM_motor_bind_cmd[0].Delay_timeout_2ms--;
	if(DM_motor_bind_cmd[1].Delay_timeout_2ms)
		DM_motor_bind_cmd[1].Delay_timeout_2ms--;
	if(DM_motor_bind_cmd[0].Exec_TimeOut)
		DM_motor_bind_cmd[0].Exec_TimeOut--;
	if(DM_motor_bind_cmd[1].Exec_TimeOut)
		DM_motor_bind_cmd[1].Exec_TimeOut--;
	
}


/*
现在的接口是这样的：
		电机1，电机2，电机1步数，电机2步数
		那么执行的时候，
		第一步，电机1启动走零；
		第二步，等待几个毫秒；
		第三步，电机2启动走零;
		第四步，等待电机1、电机2走零完成；
		第五步，电机2走步数；
		第六步，等待几个毫秒；
		第七步，电机1走步数；
		第八步，等待电机1、电机2动作完成；
		第九步，结束。
*/
void Motor_bind_exec_loop()
{
	unsigned char i;
	STEP_BIND_cmd *bindcmd;
	unsigned short error_no_bind=0; 
	for(i=0;i<2;i++)
	{
		error_no_bind=0;
		bindcmd = &DM_motor_bind_cmd[i];
		if(bindcmd->Cmd_step==0)
			continue;
		switch(bindcmd->Cmd_step)
		{
			case 1:
				Motor_is_binding_exec_set(bindcmd->Motor_NO1,1);
				//Motor_is_binding_exec_set(bindcmd->Motor_NO2,1);
				StepMotor_isr_exec(bindcmd->Motor_NO1,0, 1,0);
				//StepMotor_isr_exec(bindcmd->Motor_NO2, 0, 1,0);
				bindcmd->Cmd_step=2;
				bindcmd->Delay_timeout_2ms=6>>1;// 24 ms
				break;
			case 2:
				{
					if(bindcmd->Delay_timeout_2ms==0)
					{
						Motor_is_binding_exec_set(bindcmd->Motor_NO2,1);
						StepMotor_isr_exec(bindcmd->Motor_NO2, 0, 1,0);
						bindcmd->Cmd_step=3;
					}

				}
				break;
			case 3:
				{
					unsigned char MotorInputST=0;	
					MotorInputST |= (StepMotor_Get_Busyidx(bindcmd->Motor_NO1)?0:1)<<0;
					MotorInputST |= (StepMotor_Get_Busyidx(bindcmd->Motor_NO2)?0:1)<<1;
					MotorInputST |= (StepMotor_Get_Zero_ST(bindcmd->Motor_NO1)?1:0)<<2;
					MotorInputST |= (StepMotor_Get_Zero_ST(bindcmd->Motor_NO2)?1:0)<<3;

					if(MotorInputST==0x0f)  /*完成并零位有信号*/
					{
						bindcmd->Cmd_step=4;	
						bindcmd->Delay_timeout_2ms=8>>1;/*8ms 用2ms中断*/
						
					}
					if(bindcmd->Exec_TimeOut==0)
					{
						/*超时报警*/
						error_no_bind = 0x01;
						bindcmd->Cmd_step=0xff;	
					}
						
				}
				break;
			case 4:
				if(bindcmd->Delay_timeout_2ms==0)
				{
					bindcmd->Cmd_step=5;
					bindcmd->Exec_TimeOut=1000>>1;
					//StepMotor_isr_exec(bindcmd->Motor_NO1,bindcmd->Targer_pos_1, 1,0);
					StepMotor_isr_exec(bindcmd->Motor_NO2,bindcmd->Targer_pos_2, 1,0);
					bindcmd->Delay_timeout_2ms = 10>>1;
				}
				break;
			case 5:
				{
					if(bindcmd->Delay_timeout_2ms==0)
					{
						StepMotor_isr_exec(bindcmd->Motor_NO1,bindcmd->Targer_pos_1, 1,0);
						bindcmd->Cmd_step=6;
					}					
				}
				break;
			
			case 6:
				{
					unsigned char MotorInputST=0;	
					MotorInputST |= (StepMotor_Get_Busyidx(bindcmd->Motor_NO1)?0:1)<<0;
					MotorInputST |= (StepMotor_Get_Busyidx(bindcmd->Motor_NO2)?0:1)<<1;

					if(MotorInputST==0x03)  /*完成*/
					{
						bindcmd->Cmd_step=0xff;	
						//bindcmd->Delay_timeout_2ms=8>>1;/*8ms 用2ms中断*/
						
					}
					if(bindcmd->Exec_TimeOut==0)
					{
						/*超时报警*/
						error_no_bind = 0x02;
						bindcmd->Cmd_step=0xff;	
					}
				}
				break;
			case 0xff:
				{
					Motor_is_binding_exec_set(bindcmd->Motor_NO1,0);
					Motor_is_binding_exec_set(bindcmd->Motor_NO2,0);
					bindcmd->Cmd_step=0;	
				}
				break;
		}	

	if(error_no_bind)
		alert_push(MOTOR_BIND_CMD_ERR+bindcmd->Motor_NO1,error_no_bind);		
		
		
	}
}

void Motor_set_input_checkarg(unsigned char mid,unsigned char step_type,unsigned char arg)
{
	unsigned int stno;
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	stno = StepMotor_get_no_with_IDself(mid, step_type);
	//
	if(stno >= StepMotor_Count_MAX) return;	
	Step = &STEPMOTOR[stno];
	Step->st_no_cl.bit.check_zero_work = arg & 0x000F;
	Step->chk.bit.check_Right_Now = (arg & 0x0020) ?(( arg & 0x0010)?1:0):0;
	//Step->st_no_cl.bit.YarnStep_check_runtime =(arg & 0x0020) ?( arg & 0x0010):0;
}


void Motor_Check_Zeroinput_and_alert(unsigned char idself,unsigned char ty,unsigned short argother)
{
	unsigned char st;
	if((argother & 0x0020)==1)/*开*/
	{
	
		if((argother & 0x0010)==0) /*只一次*/
		{
			if(argother & 0x0004) /*检查零位*/
			{		
				st = arch_StepMotor_Zero_mid_layer(StepMotor_get_zeroID_with_IDself(idself,ty),StepMotor_get_zeroID_cfg_with_IDself(idself,ty));		
				if((st?1:0) != (argother & 0x0001))
				{
					alert_push(LX_YARN_ZEROINPUT_CHECK_ERR, idself);
				}
			}
			if(argother & 0x0008) /*检查工作位*/
			{		
				st = arch_StepMotor_Work_mid_layer(StepMotor_get_workID_with_IDself(idself,ty),StepMotor_get_workID_cfg_with_IDself(idself,ty));		
				if((st?1:0) != (argother & 0x0002))
				{
					alert_push(LX_YARN_WORKINPUT_CHECK_ERR, idself);
				}
			}
		}
		else
		{
			Motor_set_input_checkarg(idself,ty,(argother & 0xff));
		}
	}
	else
	{
		Motor_set_input_checkarg(idself,ty,(argother & 0xff));
	}
}

