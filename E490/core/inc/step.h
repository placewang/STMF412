#ifndef __STEP_H__
#define __STEP_H__

#include "config.h"



//#define IS_CX_FEET_3_INPUT_LIKE_SAMPLE   /*慈星新的压脚系统，类似简易三角，两个信号，一个用于零位，一个用于工作位计数*/

//#define IS_CX_ACT
#ifdef IS_CX_ACT
#define _CX_DOT_ACT_ZERO_ADJ	8

#endif


//#define DDM_MOTOR_FULLCURR    /*定义度目电机全流锁定*/

#define STEPMOTOR_HALF_ENABLE

#ifdef STEPMOTOR_HALF_ENABLE
/*半流使能*/
#define MOTOR_HALF_OUTPUT	1		
#else
/*半流禁止(电机锁定也是全流模式)*/
#define MOTOR_HALF_OUTPUT	0

#endif



//#define MOTOR_FULL_CURR_OUTPUT	0

/*电机变化平缓模式*/
#define SSTI_PULS_SMOOTH

#ifdef SSTI_PULS_SMOOTH

//#define V_FLOAT float 
#define V_FLOAT int 
typedef struct {
//	int mot_id;
	//int mot_cpos;
//	int mot_mpos;		// moving destination?
//	int mot_dest;
	//int mot_dir;
	//int mot_status;
	//int mot_flags;
//	int mot_sno;
	int mot_steps;
	//
	int mot_phase;		// 0: boot
				// 1: running
				// 2: decelerating..
	int mot_phase_dec_ex;		 //0--正常
							// 1--加速一次
	//
	// 
	V_FLOAT mot_speed;
	V_FLOAT mot_estspd;	// speed wanted..
	V_FLOAT mot_acc;		// accelerator
	//
	//long mot_period;	// 0.001ms..
	//
	// runtime stuff..
	//float mot_inispd;	// initial speed (before each step)
	V_FLOAT mot_thyspd;	// theory speed? (= estspd)?
	//...
} STEP_MOTOR_SMOOTH;


#endif




typedef struct {
	unsigned char DD_type;		/*动态类型:0--根据给定时间和目标位置，修正电机速度
										      1--根据段数，每次的目标值，最终目标值，来跟随*/
	unsigned char DD_cmd_CNT;/*动态命令条数>0,0表示全速跑*/
	unsigned char DD_cnt_cur; 	/*当前是第几条*/

	unsigned char DD_cnt_cur_nextidx;/*记录下一条正确的ID序号*/
	
	unsigned char DD_err_cnt;  /*记录出错次数*/

	unsigned char DD_dont_act;/*不需要做当前动作*/

	unsigned char DD_dec_speed;/*0--正常处理，1--缓慢加减速*/

	unsigned char DD_cmd_err_cnt;	/*记录命令错误的情况*/
	unsigned char DD_Step_overflow_flag;/*步数溢出标志*/
	unsigned char DD_Speed_dir;/*速度加减速方向,0--加速，1--减速*/
	unsigned char DD_IS_stope;/*判断是不是重新起来*/
	unsigned char DD_IS_High_speed;/*判断是不是高速模式*/
	unsigned char DD_IS_dir_first;/*判断是不是调头之后第一次*/
	unsigned char DD_IS_setting;/*正在设置*/
	unsigned char DD_Needle_idx; /*针位置*/

	unsigned char DD_Dont_check_idx;/*断电续织起来第一次不做检查*/

	unsigned char DD_Stop_immed;	/*立刻停止*/
	unsigned char DD_is_goto_turnaway;/*减速调头过程0--普通，1--正在减速，2--减速完成*/
	unsigned char DD_is_overlap_area;/*重叠区域*/
	

	unsigned char DD_Next_con_enable;/*下一次的配置命令来了，先保存*/
	unsigned char DD_Next_cmd_CNT;/* 对应DD_cmd_CNT*/
	unsigned char DD_Next_Needle_idx; /*针位置DD_Needle_idx*/
	unsigned short DD_Next_Target_POS;/* 对应DD_Target_POS*/
	unsigned short DD_Next_last_cmd_time_100us;/*对应DD_last_cmd_time_100us*/

	short DD_New_Target_POS;	  /*新的0x28 命令的*/
	
	
	short DD_cur_cnt_target_pos;/*当前这条命令对应的目标位置*/
	short DD_Target_POS;		  /*当前的动态度目的最终目标位置*/

	short DD_Target_arr[3];/*0--当前目标值，1--上次目标值，2--上上次目标值*/
	
	short DD_last_POS;		/*上一次的目标位置*/
	short DD_cur_steps;		/*当前步数*/
	unsigned short DD_last_cmd_time_100us;		/*时间点*/
	unsigned short DD_last_interval_time_100us;/*间隔时间(时长)*/
	unsigned short DD_last_interval_time_100us_for_dir;/*调头前的那次时间*/

	unsigned short DD_last_i_t_buffer[4];
	unsigned short DD_last_Buf_P;
	
	unsigned short DD_speed_hz_max;/*当前最大速*/
	unsigned short DD_speed_max_pos;/*当前最大速对应位置*/
	unsigned short DD_last_speed[16];	/*最后写进的速度值*/
	unsigned short DD_last_wp;			/*最后写进的速度对应序号*/

	/*time = n*b+(0+1+2+3...n-1)d*/
	unsigned int DD_speed_last_part_B;	/*最后一段的算法中的BASE  (us)*/
	unsigned int DD_speed_last_part_D;	/*最后一段的算法中的d   (us)*/
	unsigned int DD_last_step_idx;
	unsigned int DD_last_steps;
	#ifdef SSTI_PULS_SMOOTH
	unsigned int DD_last_steps_cal_HZ;		/*上一个周期的步数*/
	unsigned int DD_last_time_cal_HZ;	/*上一个周期的时间*/
	STEP_MOTOR_SMOOTH DD_Smooth;
	#endif
	

	
	
}Step_DD_CFG;



#ifdef NEW_ALARM_STYLE
#define STEP_ALARM_ZERO_INVALID		(0x0002)
#define STEP_ALARM_ZERO_VALID			(0x0001)
#define STEP_ALARM_RESET_ERROR		(0x0003)
#define STEP_ALARM_WORK_INVALID		(0x1002)
#define STEP_ALARM_WORK_VALID			(0x1001)
#define STEP_ALARM_LOST_NOW			(0x2000)
#define STEP_ALARM_LOST_LAST			(0x3000)
#define STEP_ALARM_OTHER_ERROR		(0x4000)
#endif


struct MOTOR_ATTR_BITS {
   unsigned int is_verystep_enable		:1;		// 0-普通电机，1--外挂式电机，那么这里就不需要继续执行动作
   unsigned int is_activestep_enable	:1;		// 0--未使能。1--使能
   unsigned int is_fast_mode			:1;		//0-普通模式，1--最快模式 
   unsigned int check_type				:2;		//0--1--表示普通传感器模式，2--编码器模式，3--传感器+编码器模式
   unsigned int moto_ecode_index		:4;		//20190926之前5，改成4// //电机对应的编码器序号
   unsigned int moto_type_config		:4;		////0--未定义，1--度目，2--生克，3--动作，4--压脚, 5--抬针
   unsigned int	moto_work_input_index	:5;		//
   unsigned int	moto_zero_input_index	:5;   
   unsigned int moto_type_exe			:3;	//20190926之前2，改成3	/*针对不同电机类型，该参数含义不同0--简易三角电机，1--HP款三角电机,2--LX三角*/
   unsigned int zero_is_positive_dir		:1;	/*表示零位是否在正方向(0-默认在负方向，1-正方向)*/
   unsigned int zero_dir_isset			:1;	
   unsigned int zeroPos_Work_ST		:1;				/*零位位置的时候，工作位状态值*/
   unsigned int workPos_Work_ST		:1;				/*工作位置的时候，工作位状态值*/
   unsigned int zero_input_NC_cfg	:1;	  /*零位传感器信号常开-0，常闭-1*/
   unsigned int work_input_NC_cfg	:1;	  /*工作位传感器信号常开-0，常闭-1*/
   
   
};

union MOTOR_ATTR_REG {
   unsigned int	all;
   struct MOTOR_ATTR_BITS bit;
};




typedef struct {
/*
*	说明，机头箱上面的电机号是stepno(0-5)物理定义的，0xff无效
*			 主控对于度目电机的命令中对应的电机号为 ID_all (0-15)。因为有些生克或压脚的命令需要走度目的命令
*			主控对于其它电机的命令(有独立命令的电机，比如生克，压脚，三角，之类的)中对应的电机号为ID_self(0-15)。
*/


	unsigned char moto_remap_id_all;    //主控id号_度目的接口
	unsigned char moto_remap_id_self;    //主控id号_其它电机自己的接口
	union MOTOR_ATTR_REG moto_attr;
	#if 0
	unsigned char moto_zero_input_index; 	//
	unsigned char moto_work_input_index; 	//
	unsigned char moto_type_config;		
	unsigned char is_verystep_enable;		
	unsigned char is_activestep_enable;	
	unsigned char is_fast_mode;			
	unsigned char check_type;				
	unsigned char moto_ecode_index;		
	#endif
	
	
} STEP_REMAP_DEF;

typedef struct{
	short		position;
	short		difpos;
	unsigned short	speed;
	unsigned short	step_max;

}STEP_DDM_DEBUG_MSG;


/*传感器状态过滤次数(1-7)*/
#define MOTOR_ZERO_ST_FILTER_BITS		4
#define ZERO_ST_BIT_MASK			(~(0xFF<<(MOTOR_ZERO_ST_FILTER_BITS+1)))
#define ZERO_ST_1_BIT_DATA			(~(0xFF<<MOTOR_ZERO_ST_FILTER_BITS))
#define ZERO_ST_0_BIT_DATA			(0x01<<MOTOR_ZERO_ST_FILTER_BITS)

#if 0
#define MOTOR_ZERO_IS_EXCHANGE(iszero_,stbit_) (iszero_?((stbit_ & ZERO_ST_BIT_MASK)==ZERO_ST_1_BIT_DATA)\
													      :((stbit_ &  ZERO_ST_BIT_MASK)==ZERO_ST_0_BIT_DATA))  
#endif
#define MOTOR_ZERO_IS_EXCHANGE(iszero_,stbit_) (iszero_?((stbit_ & 0x1F)==0x0F)\
													      :((stbit_ &  0x1F)==0x10))  

struct STEP_CHK_BITS {
	unsigned int last_zero_bit			:8;	/*8位用于存放最后8次的零位状态*/
	unsigned int last_zero_st			:1; 	/*上一次的零位状态*/
	unsigned int chk_gotozero			:1; 	/*进零位*/
	unsigned int chk_leavezero		:1;		/*离开零位*/ 
	unsigned int chk_st_change		:1;		/*状态变化*/
	unsigned int is_never_leavezero	:1;		/*走完了还没离开零位*/
	//unsigned int check_zero_work		:4;		/*状态检查标志*/
	unsigned int check_sts_enable		:1;
	unsigned int check_is_first			:1;			/*断电续织起来第一次执行*/
	unsigned int HP_check_st			:2;			/*需要查询的状态*/
	//unsigned int HP_input_st			:2;			/*HP款电机的输入传感器状态*/
	//unsigned int HP_input_st_last		:2;			/*HP款电机动作之前的状态*/
	unsigned int last_work_bit			:8;	/*8位用于存放最后8次的工作位状态*/
	unsigned int HP_check_isover		:1;		/*判断是否上一次检查了没到位*/
	unsigned int HP_check_isok		:1;		/*判断是否OK*/
	unsigned int HP_auto_adj			:1;		/*是否需要自动接续下一个命令*/
	unsigned int LX_ACT_st_ok			:1;		/**/
	unsigned int SK_ADJ_ok				:1;		/*生克修正完成*/
	//unsigned int is_adj_need_check		:1;/*动作电机加了200的修正之后，还是需要过零修正*/
	unsigned int check_Right_Now		:1;		
	unsigned int Zero_Out_ADJ_OK		:1;		/*离开零位修正*/ 
	
};

union STEP_CHK_REG {
   unsigned int all;
   struct STEP_CHK_BITS bit;
};




struct STEP_CHK_NOT_CLEAR_BITS {
	unsigned int check_is_first			:1;			/*断电续织起来第一次执行*/
	unsigned int check_zero_work		:4;		/*状态检查标志*/
	unsigned int report_after_run		:1;		/*电机运行完之后是否自动上报*/
	unsigned int report_cnt				:8;		/*电机运行完之后自动上报计数*/
	unsigned int check_zero_work_tmp	:5;		/*状态检查标志*/
	unsigned int return_byte			:8;		/*20190130 报警的时候返回给主控的数据*/
	unsigned int checkisleavezero		:1;		/*动作电机负数到0的时候，需要检查是否离开零位，做修正用
												避免长期负数到零走多了就误差累积*/
	unsigned int YarnStep_can_check		:1;		/*表示电机纱嘴可以执行检查信号命令*/
	//unsigned int YarnStep_check_runtime	:1;		
	unsigned int res					:2;		/*预留*/ 
};

union STEP_CHK_NO_CLEAR_REG {
   unsigned int all;
   struct STEP_CHK_NOT_CLEAR_BITS bit;
};


struct STEP_ALERT_ST_BITS {
	unsigned char zero_input_st			:1;		/*零位信号异常*/
	unsigned char work_input_st			:1;		/*工作位信号异常*/
	unsigned char ecord_input_st			:1;		/*编码器输入状态异常*/
	unsigned char other_alarm_st			:1;		/*电机产生其它报警*/
	unsigned char res						:4;		/*预留*/ 
};

union STEP_ALERT_ST_REG {
   unsigned char all;
   struct STEP_ALERT_ST_BITS bit;
};


struct STEP_ALERT_DELAY_BITS {
	unsigned char new_needcheck_zero			:1;		/*当前需要检查零位信号*/
	unsigned char last_needcheck_zero			:1;		/*上一次需要检查零位信号*/
	unsigned char res						:6;		/*预留*/ 
};


union STEP_ALERT_DELAY_REG {
   unsigned char all;
   struct STEP_ALERT_DELAY_BITS bit;
};



	
struct STEP_ST_BITS {
	unsigned int is_poweron			:1; 	/*是否运行过*/
	unsigned int dir					:2; /*方向*/
	unsigned int dir_High				:1; /*方向对应的电机正反转*/
	unsigned int step_flags			:1; 	/*动态度目应答*/
	unsigned int level 					:1;   /*高低电平*/
	unsigned int	running				:3;  /*运行标志*/
	unsigned int	phase				:2;  /*加减速状态*/
	unsigned int	zero				:1; /* 上一次的零位状态*/
	unsigned int	zero2_mode			:1;
	unsigned int	zero2_count			:5;/*最大31*/
	unsigned int	zero2				:1;/*工作位状态*/
	unsigned int	zero_cnt			:2; /*最大15*/ 
	unsigned int	check_delay_count	:8;/* 最大支持255*///18
	unsigned int IS_Reset_ex			:1;/*复位状态*/
	unsigned int last_dir				:2; /**/
	//unsigned int res					:1;		/*预留*/ 
};

union STEP_ST_REG {
   unsigned int all;
   struct STEP_ST_BITS bit;
};



	
struct STEP_Debug_Test_BITS {
	unsigned char justrun_test			:2; /**/
	//unsigned int res					:1;		/*预留*/ 
};

union STEP_Debug_Test_REG {
   unsigned char all;
   struct STEP_Debug_Test_BITS bit;
};

	

	



typedef struct {
	short		position;
	short		pos_2;
	short 		pos_2_main;
	unsigned char check_pos_index;
	unsigned char alert_delay_max;/*报警延时次数，默认为0，表示立即报警*/
	unsigned char alert_delay_cnt;/*报警延时次数，该值超过上面这个max则报警*/
	unsigned char alert_delay_zerolost_cnt;/*报警延时次数，该值超过上面这个max则报警*/

	unsigned char LX_act_just_move_zero;/*by hlc 20190701 连兴动作电机走0 不做修正*/

	unsigned char act_is_justcheck_zero;/*简易机才用到，只查零位信号*/

	unsigned char check_signal_edge;/*20191119 */

	unsigned char is_stop_wait;
	unsigned char change_speed;/**/

	unsigned char isdoing_with_bind_cmd;/*20220927 置成1表示正在执行绑定命令，其它命令屏蔽*/
	unsigned int	max_speed_back;   	//上一次最大频率

	unsigned int yarn_motor_elapse_steps;
	unsigned int	steps;
	
	#ifdef TRY_ZERO_MIDD
	unsigned int	steps_last;
	#endif
	
	unsigned int	step_max;
	unsigned int	speed;
#ifdef STEP_DEBUG_HZ
	unsigned int	speed_HZ_idx;
#endif	
	//unsigned int	speed_div;
	//unsigned int	speed_base;
	unsigned int	max_speed;   	//最大频率


	

#ifdef STEP_MOTOR_DDDM_SUPPORT
	 int speed_acc_dec;				//步长频率	(每次加减多少频率)
	unsigned int low_speed;			//启动频率
	//unsigned int max_speed;			//最大频率
	//unsigned int step_flags;
	unsigned int step_is_cnt;
	unsigned int step_is_cnt_old;
#endif

	
	unsigned int	alarm_step;
	
	//unsigned int	dir;
	
	
	unsigned int	state_par;	// 0 => NORMAL; 1 => Go Zero; 2 => Leave Zero

#ifdef DEBUG_ALERT_65
	unsigned int  which_gotozero;
#endif
	int last_stop_systick;
	unsigned int	acc_steps;
	unsigned int	step_wait_time;
	unsigned int 	dir_is_change_;
	unsigned int	step_reset_delay_time;
	//unsigned int	Triangle_sts;
	
	//unsigned int	Base_t_250us; 
	
	unsigned int	step_check_pos;
#ifdef STEP_TEST_IN_NDL
	unsigned int	needle;
#endif
	unsigned int	step_check_interval;

#ifdef ZERO2_SUPPORT
	//unsigned int	zero2_mode;
	//unsigned int	zero2;
	//unsigned int	zero2_count;
#endif
	//unsigned int step_stop_long;

	//unsigned int	work_position_count;


//#ifdef TRIANGLE_STEP_SUPPORT
	//unsigned int done_steps;
	//unsigned int steps_go_run;

	unsigned int change_dir_count;
	
	//unsigned int last_rest;
	//unsigned int dir_steps;

	#ifdef DEBUG_STEP_RESET_FAST_SPEED
	unsigned int reset_dir_change_cnt;/**/
	#endif
		
//#endif


#ifdef E480_BOARD_V10
	unsigned int moto_zero_width;          /*主控设置下来的值，就是到了零位之后再走多少算中间,纱嘴电机比较特殊*/
	unsigned int moto_zero_width_self;		/*电机复位的时候自动计算的一个值*/
	unsigned int moto_zero_width_temp;	
	unsigned int need_2_pos_after_reset;		/*先复位，再走到这个位置*/
#endif
	//unsigned int need_2_pos_maxspeed;

	//unsigned int is_poweron;
	unsigned int error_count_check;
	int input_errorstep;		//传感器误差(进出传感器的时候，步数差)
	
	unsigned int steps_go_;			//走了多少步了
	unsigned int steps_check_zero;	//零位检查范围(人为加一个范围值，默认为0)
	//unsigned int steps_go_leftright;	//走了多少步了到左边或右边
	unsigned int steps_go_temp;		//内部计数器
	
	
	unsigned int state_chi;			//0,第一次进0位，1-离开0位，2?,第二次进0位
	unsigned int state_chi_last;			//上一次的状态
	
	unsigned int last_new_input_sts;			//传感器，最后的状态和最新的状态，分别占两位bit0-1,last,bit2,3new 0,2:zero.1-3:work,bit4 表示是否变化

	
	STEP_REMAP_DEF moto_remap_config;
	#ifdef STEP_DEBUG_DDM
	unsigned int step_poslistid;
	STEP_DDM_DEBUG_MSG Stepdebugmsg[4];
	#endif
	short check_work_pos;
	short step_input_nc_cfg;
	short HF_ex_cnt;/*20190507*/
	//#ifdef LX_ACTION_STEP_SPECIAL
	
	//unsigned char check_zero_work;
	//unsigned char check_zero_work_Timer_enable;
	//unsigned char adj_lost_steps;
	//unsigned char need_check_zero_sts_change;
	//unsigned char need_leave_zero;
	//unsigned char last_zero_act;
	unsigned char DD_error_cnt;/*度目电机0-8报警滤波计数器*/
	unsigned char Had_next_cmd;
	Step_DD_CFG moto_dd_config;
	union STEP_CHK_REG chk;
	union STEP_ST_REG  step_st;	
	union STEP_CHK_NO_CLEAR_REG st_no_cl;
	union STEP_ALERT_ST_REG step_alert_st;/*这个用于主动上报电机状态*/
	union STEP_ALERT_DELAY_REG step_alert_delay_pos2_st;
	union STEP_Debug_Test_REG step_debug_test_st;
	//#endif
	


}STEP_TYPE;


typedef struct {
	unsigned short cmdID;	/*命令序号*/
	unsigned char Motor_NO1;/*电机号1*/
	unsigned char Motor_NO2;/*电机号2*/
	short Targer_pos_1;	/*电机号1对应目标位置*/	
	short Targer_pos_2;	/*电机号2对应目标位置*/
	unsigned short Delay_timeout_2ms;/*延时时间*/
	unsigned short Exec_TimeOut;	/*动作执行超时时间*/
	unsigned char Cmd_step;/*当前执行到第几步:
							0--无命令
							1--收到命令
							2--正在归零
							3--归零完成
							4--正在执行
							5--执行完成*/ 
							
	

}STEP_BIND_cmd;




/*度目电机零位检查报警滞后几次报警*/
#define STEP_MOTOT_CHECK_ZERO_DELAY_ALERTCNT 1


#define SPET_MOTOR_MIN_SPEED_HZ	(500)
#define SPET_MOTOR_MAX_SPEED_HZ	(20000)


#define SPET_MOTOR_MIN_SPEED_DD_HZ	(500)
#define SPET_MOTOR_MAX_SPEED_DD_HZ	(10000)




/*
#ifdef TRIANGLE_STEP_SUPPORT
#define TRIANGLE_UNKNOW_ZERO	(1)
#define TRIANGLE_ZERO_ZERO		(2)
#define TRIANGLE_NEGATIVE_ZERO	(3)
#define TRIANGLE_PLUS_ZERO		(4)
#define TRIANGLE_ZERO_NEGATIVE	(5)
#define TRIANGLE_ZERO_PLUS		(6)
#define TRIANGLE_NEGATIVE_PLUS 	(7)
#define TRIANGLE_PLUS_NEGATIVE 	(8)


#endif
*/

// STEP reset flag 
#define JUST_RUN 				(0)
#define GOTO_ZERO				1
#define LEAVE_ZERO				2
#define LEAVE_STEP				3
#define MOVETO_ZERO			4
#define DETECT_ZERO2			5
#define GOTO_ZERO2	 			6
#define GOTO_ZERO3	 			61

#define CHECK_SIGN_WIDTH 		7
#define STEP_RESET 				8
#define ENCODER_LEAVE_ZERO	9
#define STEP_RESET_HP_ACT 		10
#define JUST_RUN_GO 			11
//#ifdef LX_ACTION_STEP_SPECIAL
#define MOVETO_ZERO_LX_ACT	12
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO1	121
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO2	122
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO3	123

//#endif

#define GOTO_ZERO_TRY 			13
#define MOVETO_ZERO_LIFT 		14
#define CHECK_SIGN_DEVIATION	(15)
#define FEET_STEP_ISWORK		(16)
#define HP_STEP_CHECK_INPUT	(17)

#define GOTOZERO_LX_ACT		(18)
#define LEFT_EX_ISWORK          (161)
//#define LEFT_EX_ISWORK2          (162)



#ifdef TRY_ZERO_MIDD
#define GOTO_ZERO_TRY_ZERO_MIDD	(19)		/*来回检查电机零位在什么位置*/

#define GOTO_TRY_ZERO_STEPS	50			/*每次尝试左右拓展寻找零位位置的步数*/

#endif

#define JUST_RUN_RESET 				(20)
#define ENCODER_JUST_RUN			21

#define JUST_RUN_AFTER_ADJ 			30

//#ifdef TRIANGLE_STEP_SUPPORT

/*无论如何要走这么些步数才认为是报警来了*/
#define STEP_GO_ALARM_STEPS	4

/*无论如何要走这么些步数才认为是可以调头了*/
#define STEP_GO_DIR_STEPS	80
#define STEP_GO_DIR_STEPS_MIN	10

/*电机加减速步数*/
#ifdef FOR_BW_DM_SLOWLYRUN
#define STEP_DDM_STEPS		(32)  //(16)  
#else
#define STEP_DDM_STEPS		(16)  
#endif
//#ifdef LX_ACTION_STEP_SPECIAL

//#define STEPS_LEAVEZERO_FOR_LX	(100)  
/*20180723 修改成150 慈星特殊*/
#define STEPS_LEAVEZERO_FOR_LX	(200)    
/*连兴特殊的值*/
/*从正到负再走10个点，从负到正少走10个点*/
#ifdef TEST_CX_ACT
#define STEP_ZERO_GOON_STEPS	(-1)
#else
#define STEP_ZERO_GOON_STEPS	(3)
#endif

#define HP_STEP_MOTOR_CHECK_INPUT_AREA	(100)   /*HP款动作电机，每次动作提前检查到位的步数，±这个值*/

//#endif

#define ALERT_DELAY_CNT_DEFAULT		5 /*默认几次报警之后才报警*/

#define STEPS_LEAVEZERO	(30)//默认离开0位的步数 

#define STEPS_LEAVEZERO_SMALL	(25)//默认离开0位的步数 


#define STEPS_ERROR_RANGE	(5)	//零位偏移误差最大步


#define STEPS_ZERO_GOBACK	(0)  /*电机编码器自动找零之后往回走多少步,默认0*/

#define GOTO_LEFT	8		//走到01位置(-300)
#define GOTO_RIGHT	9		//走到10位置(300)
#define CHECK_IS_LEFT 10     //检查01位置是不是真的到了
#define CHECK_IS_RIGHT 11   //检查11位置是不是真的到了
#define CHECK_ISNOT_LR 12     //检查是不是真的离开了两边

#define TEST_SIGN_WIDTH  0x20   			//检测传感器宽度
#define TEST_SIGN_WIDTH_LEAVE  0x21   //检测传感器宽度离开中间
#define TEST_SIGN_WIDTH_IN 0x22   		//回到传感器中间
#define TEST_SIGN_WIDTH_LEAVE_EX 0x23  



#define TRIANGLE_LEFT_POSTION 	(0-240)
#define TRIANGLE_RIGHT_POSTION 	(240)
#define TRIANGLE_ZERO_POSTION 	(0)

#define DEF_CHECK_WORK_POS  (2000)

//#endif


enum {
	HEAD_MODE_DEFAULT = 0,		/*4度目+2三角+2生克+4纱嘴(如果有电机沙嘴的话)*/
	HEAD_MODE_SIMPLE,			/*4度目+2三角+2空白+4纱嘴(如果有电机沙嘴的话)*/
	HEAD_MODE_SKMODE2,			/*4度目+2生克+2空白+4纱嘴(如果有电机沙嘴的话)*/
	HEAD_MODE_SKMODE4,			/*4度目+4生克+4纱嘴(如果有电机沙嘴的话)*/
	HEAD_MODE_LIFT2,			/*4度目+2三角+2抬针+4纱嘴(如果有电机沙嘴的话)*/	// ADD by zhuqiwei 160601
	HEAD_MODE_LIFT_EX,		/*4度目+2三角+2生克+4纱嘴(如果有电机沙嘴的话)+2抬针*/ //E490_v1.0
	HEAD_MODE_FEET,			/*4度目+2压脚+2生克+4纱嘴(如果有电机沙嘴的话)    国光*/
	HEAD_MODE_LIFT_HP,		/*4度目+2三角(HP)+2生克+4纱嘴(如果有电机沙嘴的话)+2抬针*/ //20160816
	HEAD_MODE_LX_ACT,			/*4度目+2三角(LX)+2生克+4纱嘴(如果有电机沙嘴的话)+2抬针*/ // 2016 11 29 
	HEAD_MODE_FH_ACT,		/*4度目+2三角(FH)+2生克+4纱嘴(如果有电机沙嘴的话)+2抬针*/ // 2016 11 29 
	//HEAD_MODE_LIFT3,			/*4度目+2三角+2生克+2推针+4纱嘴(如果有电机沙嘴的话)*/
	HEAD_MODE_FEET_CX,		/**4度目+2压脚+2生克+4纱嘴(如果有电机沙嘴的话)    +2抬针---慈星20171122*/
	HEAD_MODE_ALL_STEP_CX,	/**4度目+2三角+2压脚+4纱嘴(如果有电机沙嘴的话)    +2抬针+扩展2生克---慈星20181129*/
	HEAD_MODE_MAX,
};

/*E480--最多支持12个电机，默认0模式支持0,1,2,3 四种模式，E475--最多支持6个电机，默认2模式，支持1,2两种模式*/

enum {
	MOTOR_TYPE_UNDEF = 0,	//0-未定义 
	MOTOR_TYPE_DENSITY,	// 1-度目
	MOTOR_TYPE_SKINER,	// 2-申克
	MOTOR_TYPE_ACTION,	// 3-动作
	MOTOR_TYPE_FEET,		// 4-压脚
	MOTOR_TYPE_YARN,		// 5-纱嘴
	MOTOR_TYPE_LIFT,		// 6-抬针
	MOTOR_TYPE_OTHER,		//7-其它电机(这类电机工作模式类似与生克电机)
	MOTOR_TYPE_UPDOWN,	//8-慈星沙嘴上下运动电机，每系统4个，接原先沙嘴电机接口
	MOTOR_TYPE_MAX,		//最大类型号
};
#define MOTOR_TYPE_COUNT	(MOTOR_TYPE_MAX-1)			//所有电机类型数量
#define MOTOR_TYPE_0XFF	(0xFF)				/* 电机类型为0XFF的时候直接对物理ID操作*/

#define MOTOR_TYPE_EXE_PT		0

enum{
	DENSITY_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,
	DENSITY_MOTOR_TYPE_JF,
	DENSITY_MOTOR_TYPE_LX,
	DENSITY_MOTOR_TYPE_OTHER,	
};

enum{
	ACT_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,   /*0: 普通的半边亮半边不亮*/
	ACT_MOTOR_TYPE_HP,		/*1: HP款，两个信号组合用*/
	ACT_MOTOR_TYPE_LX,		/*2: LX款，翻针位有缺口的(F型)*/
	ACT_MOTOR_TYPE_FH,		/*3: FH款，在连兴基础上，翻针位另外加了一个信号检查*/
	ACT_MOTOR_TYPE_DOT,		/*4: 信号在中间一个点的模式*/
};

enum{
	LIFT_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	LIFT_MOTOR_TYPE_HF,                /*支持浩丰模式*/	
	LIFT_MOTOR_TYPE_HF_EX,	 /*中间没信号，两边有信号，找零的时候需要计算中间空白宽度大小*/
};

enum{
	SINKER_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	SINKER_MOTOR_TYPE_LX,                /*支持连兴模式*/	
	SINKER_MOTOR_TYPE_LX_EX,		/*连兴模式基础上又增加一种*/

};



enum{
	FEET_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	FEET_MOTOR_TYPE_CX,                /*支持慈星模式就是工作位走完再检测*/	
	FEET_MOTOR_TYPE_CX2,		  /*支持慈星模式--按照位置走步数*/
	FEET_MOTOR_TYPE_CX3		/*支持慈星模式--按照信号走，工位需要修正*/	
};


enum{
	YARN_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	YARN_MOTOR_TYPE_LX,                /*支持连兴模式的电机纱嘴，类似推针电机，中间亮(信号在中间)，两头是位置(工作位和机械零位)*/	
};






// STEP running flag 
#define RUNNING		1
#define RUNNING_OVER	2
#define RUNNING_GO	3
#define RUNNING_END	4

#ifdef LOG_DEBUG_FOR_LX_AT_CH
enum{
	LX_LOG_TYPE_ZERO_ADJ_GOTO=0, /*零位修正进 0*/
	LX_LOG_TYPE_ZERO_ADJ_GOTO_2, /*零位修正进1*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE, /*零位修正出2*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE_2, /*零位修正出3*/
	
	LX_LOG_TYPE_MOVETOZERO_ADD400, /*走零位加走400步4*/
	LX_LOG_TYPE_MOVETOZERO_ADD400_2, /*走零位加走400步5*/
	LX_LOG_TYPE_LEAVEZERO_ADD200, /*离开零位加走200步6*/
	LX_LOG_TYPE_LEAVEZERO_ADD200_2, /*离开零位加走200步7*/
	LX_LOG_TYPE_RUNNING_OVER,	/*两个命令接续了8*/
	LX_LOG_TYPE_RUNNING_OVER_2,	/*两个命令接续了9*/
	LX_LOG_TYPE_STOP_GO,	/*停止了但是位置值和目10标值步相等，继续走*/
	LX_LOG_TYPE_STOP_GO_2,	/*停止了但是位置值和11目标值步相等，继续走*/
	LX_LOG_TYPE_ZERO_CHANGE,/*零位信号变化12*/	
	LX_LOG_TYPE_ZERO_W,/*零位信号宽度13*/	
	LX_LOG_TYPE_ADJ_ISOK_BUT,/*零位来了14切薜奶跫掷戳恕?*/
	LX_LOG_TYPE_ADJ_ISOK_BUT_2,/*零位来了，15但是修的条件又来了。*/
};
#define ADJ_NEED_LOG_STEPS	(20)		/*修正超过这个步数才报警*/

#endif



void StepMotor_Init(void);
void StepMotor_Set_ZeroAdj(unsigned int adj);
void StepMotor_Set_ZeroDetect(unsigned int area);
void StepMotor_Setup_Active(unsigned int active,unsigned char islog);
void StepMotor_Set_VeryMode(unsigned int verymask,unsigned char islog);
void StepMotor_Setup_Direction(unsigned int dir);
void StepMotor_Setup_Resolution(unsigned int mototype,unsigned int rlt);
void StepMotor_Set_Mode(unsigned int mode);
void StepMotor_Set_FastMode(unsigned int stepno,unsigned char islog);
void StepMotor_Set_Reset_Speed(unsigned int speed,unsigned int mototype);
void StepMotor_Set_Speed(unsigned int speed,unsigned int steptype);
unsigned int StepMotor_Set_SinkerSpeed(unsigned int speed);
void StepMotor_Reset(unsigned int stepno,unsigned short otherarg);
void StepMotor_Reset_LeftMotor(unsigned short otherarg);
void StepMotor_Reset_RightMotor(unsigned short otherarg);
void StepMotor_All_Reset(unsigned short otherarg);
void StepMotor_Start(unsigned int stepno);
void StepMotor_Set_Position(unsigned int stepno, short pos);
void StepMotor_Modfiy_Position(unsigned int stepno, short pos, int power);
short StepMotor_Get_Position_2(unsigned int stepno);
short StepMotor_Get_Position(unsigned int stepno);
unsigned int StepMotor_Get_Busyidx(unsigned int stepno);

unsigned int StepMotor_get_IDall_with_IDself(unsigned int step_IDself,unsigned char step_type);


void StepMotor_exec(unsigned int stepno, short pos, int mode,unsigned short otherarg,unsigned char workspeed);
void StepMotor_isr_exec(unsigned int stepid,short pos,int mode,unsigned short otherarg);

void StepMotor_Set_Check(unsigned int step);

void StepMotor_Feet_Setup(int cmd, unsigned int arg);
//int StepMotor_Sinker_Get_Busy();
//#ifdef LX_ACTION_STEP_SPECIAL
void StepMotor_Type_exec(int mid, short arg, unsigned char step_type,unsigned char check,unsigned char wspeed,unsigned char justcheckalert,unsigned char mz);
//#else
//void StepMotor_Type_exec(int mid, short arg, unsigned char step_type);
//#endif
void StepMotor_Type_Reset(unsigned int mid, unsigned char step_type,unsigned short otherarg);
void StepMotor_Type_ResetALL(int step_type,unsigned short otherarg);
void StepMotor_Type_Set_Position(unsigned int sk_id, short pos, unsigned char step_type);

void StepMotor_Set_Reset_Delay_Time(int stepno, unsigned int delay);

void StepMotor_Setup_Para(int cmd, int para);

// 用于测试电机是否到位
void StepMotor_debug(void);

int check_sinker_done(void);
int check_sti_done(int leftOrRight);
void StepMotor_Poll(void);
void StepMotor_Isr(unsigned int stepno);
void StepMotor_yarn_Isr(unsigned int stepno);

void StepMotor_Set_with_Head_Mode(unsigned int mode);
unsigned char Stepmotor_get_Head_Mode(void);
void Exec_Set_Step_check_Type(unsigned int whichstep,unsigned int stepno,unsigned char c_type);

void StepMotor_Set_work_steps(unsigned char steptype,unsigned  int steps,unsigned char whichstepid);

void Step_Set_work_check_pos(unsigned char mid,short pos,unsigned char step_type);
void StepMotor_afterrun_toreport(unsigned int stepno,unsigned char islxsk);
void Exec_Reset_step_moto_one(int arg2,unsigned short otherarg);
void Exec_Reset_step_moto_all(int arg2,unsigned short otherarg);

#ifdef STEP_MOTOR_DDDM_SUPPORT
void StepMotor_exec_new_dd(unsigned int stepno, short pos, int time, int mode, int maxspd,char isnewcmd);
void StepMotor_isr_exec_new_dd(unsigned int stepid,short pos,int time,int mode,int maxspeed,char isnewcmd);
#endif

void StepMotor_justrun(unsigned int stepno,unsigned char needreturn, short pos);


void StepMotor_Isr_call(unsigned int stepno);

void Motor_Set_DM_Type(unsigned char steptype,unsigned char dm_type);

void StepMotor_Set_zero_dir(unsigned char whichtype,unsigned short stepno,unsigned short dirdata);

int is_Feet_Step(int stepno);

void Set_step_alert_st_ecoder(unsigned char stepno,unsigned char st);

void StepMotor_other_Get_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short *zw,short *iner);

void StepMotor_other_Set_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short zw,short iner);

void Stepmotor_Set_ID_Type_with_NO(unsigned char stepno,unsigned int idself,unsigned int type);
void StepMotor_Set_Zero_go_Steps_ex(unsigned int steptype, short gos);

void check_lift_step_isstop(void);
unsigned char Get_StepMotor_isveryenable_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_zeroID_with_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_workID_with_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_no_with_IDself(unsigned int step_IDself,unsigned char step_type);
void check_step_motor_loop_(void);
void Exec_Set_Step_Parameter_with_motortype(unsigned char para_type,unsigned char motortype_main,unsigned short mask_,unsigned short data_);
void Set_Motor_steps_for_ACC_DEC(unsigned short acc_steps);
void Get_Step_data(void);
void Stepmotor_Set_inputID_with_NO(unsigned char stepno,unsigned char in_zero,unsigned int id_work,unsigned int zw_mask);
unsigned char Get_StepMotor_isveryenable_IDall(unsigned int step_idall);
void StepMotor_Set_Input_error(unsigned int stepno,unsigned int inerr);
void StepMotor_Setup_Direction_log(unsigned int dir);
void Step_Motor_Set_Check_all(unsigned char whichtype,unsigned int step);
void StepMotor_Set_Zero_go_Steps(unsigned int steptype,unsigned int gos);
void StepMotor_Set_zero_type_(unsigned int steptype,unsigned char whichstep,unsigned int zt);
void StepMotor_Set_zero_detect(unsigned char whichtype,unsigned int step);
void StepMotor_Set_movetozero_adv_steps(unsigned short steps,unsigned short motortype );

void StepMotor_Set_Speed_EX(unsigned char motortype,unsigned char whichspeed,unsigned int speed_hz);
void Exec_Set_DDM_need_Check(unsigned int ischeck);

void Motor_Set_minsteps_ecode_rang(unsigned char steptype,unsigned int data);
void Motor_Set_DM_check_area( short max_a, short min_a);
void Motor_Get_All_postion(unsigned int ispos2,unsigned short *posdata);
void StepMotor_Other_SetSpeed(unsigned char steptype,unsigned char whichspeed,unsigned int speeddata);
void StepMotor_Other_Set_isfast(unsigned char steptype,unsigned int isfast);
void StepMotor_Other_Set_Work_Enable(unsigned char steptype,unsigned int isenable);
void StepMotor_Setup_ACTION_Para(int cmd, int para);

void Exec_Triangle_step(unsigned int Tid,  int Pos,unsigned int alarmenable);
short StepMotor_Triangle_Get_Position(unsigned int ACT_id);
short StepMotor_Triangle_Get_Sign(unsigned int ACT_id);

void StepMotor_Action_Set_Position(unsigned int ACT_id,short pos);
void StepMotor_Triangle_Set_Sign(unsigned int sk_no,short pos);
void Motor_Set_maxsteps_cw(unsigned char steptype,unsigned int data);

void Set_Step_Motor_postion_ex(unsigned char steptype,unsigned char stepno,unsigned char dataindex,short data_pos );
void Set_Step_Motor_TO_Which_Postion(unsigned char steptype,unsigned short dataindex);
void StepMotor_Set_ZeroPos_WorkST(unsigned int steptype,unsigned char wst);
void StepMotor_Add_curr(unsigned int stepno);
void StepMotor_setMaxSpeed(unsigned int stepno,unsigned int maxspeed);
int StepMotor_Get_Running(unsigned int stepno);
void StepMotor_reback_curr(unsigned int stepno);
void StepMotor_setpos_2main_postion(unsigned int stepno);

void Get_Step_debug_msg(unsigned short whichstep);

void StepMotor_Setup_Yarnstep_Active(unsigned int yarnstep_active);

void arch_YARN_Step_Setup(int yno, int onoff);
int is_zero2_support(unsigned int stepno);
int is_LX_DENSITY_Step(int stepno);
int is_HF_LIFT_Step(int stepno);
int is_EX_OTHER_Step(int stepno);
int is_ACTION_Step(int stepno);
int is_HP_ACTION_Step(int stepno);
int is_PT_ACTION_Step(int stepno);
int is_LX_ACTION_Step(int stepno);
int is_LIFT_or_Other_Step(int stepno);
int Get_st_with_postion(short arg);
int is_FH_ACTION_Step(int stepno);
int is_DENSITY_Step(int stepno);
int is_LIFT_Step(int stepno);
int is_JF_DENSITY_Step(int stepno);
int is_lx_sinker(int stepno);
int is_YARN_Step(int stepno);
int is_PTorFH_ACTION_Step(int stepno);
int check_HF_Lift_input_error(STEP_TYPE *Step);
int is_lx_sinker_ex(STEP_TYPE *Step_ex);
int is_LX_DENSITY_Step_ex(STEP_TYPE *Step_ex);
int is_LIFT_all_Step(int stepno);
void Set_LX_ex_adjData(unsigned char steptype,unsigned short data);
void StepMotor_Speed_report(unsigned short retcmd,unsigned char typemain,unsigned char steptype);
void StepMotor_input_NCorNO_set(unsigned char steptype,unsigned char stepno,unsigned char ncno);
void StepMotor_Set_Ecoder_dir_withzerodir();

#endif

