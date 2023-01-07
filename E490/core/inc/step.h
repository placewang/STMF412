#ifndef __STEP_H__
#define __STEP_H__

#include "config.h"



//#define IS_CX_FEET_3_INPUT_LIKE_SAMPLE   /*�����µ�ѹ��ϵͳ�����Ƽ������ǣ������źţ�һ��������λ��һ�����ڹ���λ����*/

//#define IS_CX_ACT
#ifdef IS_CX_ACT
#define _CX_DOT_ACT_ZERO_ADJ	8

#endif


//#define DDM_MOTOR_FULLCURR    /*�����Ŀ���ȫ������*/

#define STEPMOTOR_HALF_ENABLE

#ifdef STEPMOTOR_HALF_ENABLE
/*����ʹ��*/
#define MOTOR_HALF_OUTPUT	1		
#else
/*������ֹ(�������Ҳ��ȫ��ģʽ)*/
#define MOTOR_HALF_OUTPUT	0

#endif



//#define MOTOR_FULL_CURR_OUTPUT	0

/*����仯ƽ��ģʽ*/
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
	int mot_phase_dec_ex;		 //0--����
							// 1--����һ��
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
	unsigned char DD_type;		/*��̬����:0--���ݸ���ʱ���Ŀ��λ�ã���������ٶ�
										      1--���ݶ�����ÿ�ε�Ŀ��ֵ������Ŀ��ֵ��������*/
	unsigned char DD_cmd_CNT;/*��̬��������>0,0��ʾȫ����*/
	unsigned char DD_cnt_cur; 	/*��ǰ�ǵڼ���*/

	unsigned char DD_cnt_cur_nextidx;/*��¼��һ����ȷ��ID���*/
	
	unsigned char DD_err_cnt;  /*��¼�������*/

	unsigned char DD_dont_act;/*����Ҫ����ǰ����*/

	unsigned char DD_dec_speed;/*0--��������1--�����Ӽ���*/

	unsigned char DD_cmd_err_cnt;	/*��¼�����������*/
	unsigned char DD_Step_overflow_flag;/*���������־*/
	unsigned char DD_Speed_dir;/*�ٶȼӼ��ٷ���,0--���٣�1--����*/
	unsigned char DD_IS_stope;/*�ж��ǲ�����������*/
	unsigned char DD_IS_High_speed;/*�ж��ǲ��Ǹ���ģʽ*/
	unsigned char DD_IS_dir_first;/*�ж��ǲ��ǵ�ͷ֮���һ��*/
	unsigned char DD_IS_setting;/*��������*/
	unsigned char DD_Needle_idx; /*��λ��*/

	unsigned char DD_Dont_check_idx;/*�ϵ���֯������һ�β������*/

	unsigned char DD_Stop_immed;	/*����ֹͣ*/
	unsigned char DD_is_goto_turnaway;/*���ٵ�ͷ����0--��ͨ��1--���ڼ��٣�2--�������*/
	unsigned char DD_is_overlap_area;/*�ص�����*/
	

	unsigned char DD_Next_con_enable;/*��һ�ε������������ˣ��ȱ���*/
	unsigned char DD_Next_cmd_CNT;/* ��ӦDD_cmd_CNT*/
	unsigned char DD_Next_Needle_idx; /*��λ��DD_Needle_idx*/
	unsigned short DD_Next_Target_POS;/* ��ӦDD_Target_POS*/
	unsigned short DD_Next_last_cmd_time_100us;/*��ӦDD_last_cmd_time_100us*/

	short DD_New_Target_POS;	  /*�µ�0x28 �����*/
	
	
	short DD_cur_cnt_target_pos;/*��ǰ���������Ӧ��Ŀ��λ��*/
	short DD_Target_POS;		  /*��ǰ�Ķ�̬��Ŀ������Ŀ��λ��*/

	short DD_Target_arr[3];/*0--��ǰĿ��ֵ��1--�ϴ�Ŀ��ֵ��2--���ϴ�Ŀ��ֵ*/
	
	short DD_last_POS;		/*��һ�ε�Ŀ��λ��*/
	short DD_cur_steps;		/*��ǰ����*/
	unsigned short DD_last_cmd_time_100us;		/*ʱ���*/
	unsigned short DD_last_interval_time_100us;/*���ʱ��(ʱ��)*/
	unsigned short DD_last_interval_time_100us_for_dir;/*��ͷǰ���Ǵ�ʱ��*/

	unsigned short DD_last_i_t_buffer[4];
	unsigned short DD_last_Buf_P;
	
	unsigned short DD_speed_hz_max;/*��ǰ�����*/
	unsigned short DD_speed_max_pos;/*��ǰ����ٶ�Ӧλ��*/
	unsigned short DD_last_speed[16];	/*���д�����ٶ�ֵ*/
	unsigned short DD_last_wp;			/*���д�����ٶȶ�Ӧ���*/

	/*time = n*b+(0+1+2+3...n-1)d*/
	unsigned int DD_speed_last_part_B;	/*���һ�ε��㷨�е�BASE  (us)*/
	unsigned int DD_speed_last_part_D;	/*���һ�ε��㷨�е�d   (us)*/
	unsigned int DD_last_step_idx;
	unsigned int DD_last_steps;
	#ifdef SSTI_PULS_SMOOTH
	unsigned int DD_last_steps_cal_HZ;		/*��һ�����ڵĲ���*/
	unsigned int DD_last_time_cal_HZ;	/*��һ�����ڵ�ʱ��*/
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
   unsigned int is_verystep_enable		:1;		// 0-��ͨ�����1--���ʽ�������ô����Ͳ���Ҫ����ִ�ж���
   unsigned int is_activestep_enable	:1;		// 0--δʹ�ܡ�1--ʹ��
   unsigned int is_fast_mode			:1;		//0-��ͨģʽ��1--���ģʽ 
   unsigned int check_type				:2;		//0--1--��ʾ��ͨ������ģʽ��2--������ģʽ��3--������+������ģʽ
   unsigned int moto_ecode_index		:4;		//20190926֮ǰ5���ĳ�4// //�����Ӧ�ı��������
   unsigned int moto_type_config		:4;		////0--δ���壬1--��Ŀ��2--���ˣ�3--������4--ѹ��, 5--̧��
   unsigned int	moto_work_input_index	:5;		//
   unsigned int	moto_zero_input_index	:5;   
   unsigned int moto_type_exe			:3;	//20190926֮ǰ2���ĳ�3	/*��Բ�ͬ������ͣ��ò������岻ͬ0--�������ǵ����1--HP�����ǵ��,2--LX����*/
   unsigned int zero_is_positive_dir		:1;	/*��ʾ��λ�Ƿ���������(0-Ĭ���ڸ�����1-������)*/
   unsigned int zero_dir_isset			:1;	
   unsigned int zeroPos_Work_ST		:1;				/*��λλ�õ�ʱ�򣬹���λ״ֵ̬*/
   unsigned int workPos_Work_ST		:1;				/*����λ�õ�ʱ�򣬹���λ״ֵ̬*/
   unsigned int zero_input_NC_cfg	:1;	  /*��λ�������źų���-0������-1*/
   unsigned int work_input_NC_cfg	:1;	  /*����λ�������źų���-0������-1*/
   
   
};

union MOTOR_ATTR_REG {
   unsigned int	all;
   struct MOTOR_ATTR_BITS bit;
};




typedef struct {
/*
*	˵������ͷ������ĵ������stepno(0-5)������ģ�0xff��Ч
*			 ���ض��ڶ�Ŀ����������ж�Ӧ�ĵ����Ϊ ID_all (0-15)����Ϊ��Щ���˻�ѹ�ŵ�������Ҫ�߶�Ŀ������
*			���ض����������������(�ж�������ĵ�����������ˣ�ѹ�ţ����ǣ�֮���)�ж�Ӧ�ĵ����ΪID_self(0-15)��
*/


	unsigned char moto_remap_id_all;    //����id��_��Ŀ�Ľӿ�
	unsigned char moto_remap_id_self;    //����id��_��������Լ��Ľӿ�
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


/*������״̬���˴���(1-7)*/
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
	unsigned int last_zero_bit			:8;	/*8λ���ڴ�����8�ε���λ״̬*/
	unsigned int last_zero_st			:1; 	/*��һ�ε���λ״̬*/
	unsigned int chk_gotozero			:1; 	/*����λ*/
	unsigned int chk_leavezero		:1;		/*�뿪��λ*/ 
	unsigned int chk_st_change		:1;		/*״̬�仯*/
	unsigned int is_never_leavezero	:1;		/*�����˻�û�뿪��λ*/
	//unsigned int check_zero_work		:4;		/*״̬����־*/
	unsigned int check_sts_enable		:1;
	unsigned int check_is_first			:1;			/*�ϵ���֯������һ��ִ��*/
	unsigned int HP_check_st			:2;			/*��Ҫ��ѯ��״̬*/
	//unsigned int HP_input_st			:2;			/*HP���������봫����״̬*/
	//unsigned int HP_input_st_last		:2;			/*HP��������֮ǰ��״̬*/
	unsigned int last_work_bit			:8;	/*8λ���ڴ�����8�εĹ���λ״̬*/
	unsigned int HP_check_isover		:1;		/*�ж��Ƿ���һ�μ����û��λ*/
	unsigned int HP_check_isok		:1;		/*�ж��Ƿ�OK*/
	unsigned int HP_auto_adj			:1;		/*�Ƿ���Ҫ�Զ�������һ������*/
	unsigned int LX_ACT_st_ok			:1;		/**/
	unsigned int SK_ADJ_ok				:1;		/*�����������*/
	//unsigned int is_adj_need_check		:1;/*�����������200������֮�󣬻�����Ҫ��������*/
	unsigned int check_Right_Now		:1;		
	unsigned int Zero_Out_ADJ_OK		:1;		/*�뿪��λ����*/ 
	
};

union STEP_CHK_REG {
   unsigned int all;
   struct STEP_CHK_BITS bit;
};




struct STEP_CHK_NOT_CLEAR_BITS {
	unsigned int check_is_first			:1;			/*�ϵ���֯������һ��ִ��*/
	unsigned int check_zero_work		:4;		/*״̬����־*/
	unsigned int report_after_run		:1;		/*���������֮���Ƿ��Զ��ϱ�*/
	unsigned int report_cnt				:8;		/*���������֮���Զ��ϱ�����*/
	unsigned int check_zero_work_tmp	:5;		/*״̬����־*/
	unsigned int return_byte			:8;		/*20190130 ������ʱ�򷵻ظ����ص�����*/
	unsigned int checkisleavezero		:1;		/*�������������0��ʱ����Ҫ����Ƿ��뿪��λ����������
												���ⳤ�ڸ��������߶��˾�����ۻ�*/
	unsigned int YarnStep_can_check		:1;		/*��ʾ���ɴ�����ִ�м���ź�����*/
	//unsigned int YarnStep_check_runtime	:1;		
	unsigned int res					:2;		/*Ԥ��*/ 
};

union STEP_CHK_NO_CLEAR_REG {
   unsigned int all;
   struct STEP_CHK_NOT_CLEAR_BITS bit;
};


struct STEP_ALERT_ST_BITS {
	unsigned char zero_input_st			:1;		/*��λ�ź��쳣*/
	unsigned char work_input_st			:1;		/*����λ�ź��쳣*/
	unsigned char ecord_input_st			:1;		/*����������״̬�쳣*/
	unsigned char other_alarm_st			:1;		/*���������������*/
	unsigned char res						:4;		/*Ԥ��*/ 
};

union STEP_ALERT_ST_REG {
   unsigned char all;
   struct STEP_ALERT_ST_BITS bit;
};


struct STEP_ALERT_DELAY_BITS {
	unsigned char new_needcheck_zero			:1;		/*��ǰ��Ҫ�����λ�ź�*/
	unsigned char last_needcheck_zero			:1;		/*��һ����Ҫ�����λ�ź�*/
	unsigned char res						:6;		/*Ԥ��*/ 
};


union STEP_ALERT_DELAY_REG {
   unsigned char all;
   struct STEP_ALERT_DELAY_BITS bit;
};



	
struct STEP_ST_BITS {
	unsigned int is_poweron			:1; 	/*�Ƿ����й�*/
	unsigned int dir					:2; /*����*/
	unsigned int dir_High				:1; /*�����Ӧ�ĵ������ת*/
	unsigned int step_flags			:1; 	/*��̬��ĿӦ��*/
	unsigned int level 					:1;   /*�ߵ͵�ƽ*/
	unsigned int	running				:3;  /*���б�־*/
	unsigned int	phase				:2;  /*�Ӽ���״̬*/
	unsigned int	zero				:1; /* ��һ�ε���λ״̬*/
	unsigned int	zero2_mode			:1;
	unsigned int	zero2_count			:5;/*���31*/
	unsigned int	zero2				:1;/*����λ״̬*/
	unsigned int	zero_cnt			:2; /*���15*/ 
	unsigned int	check_delay_count	:8;/* ���֧��255*///18
	unsigned int IS_Reset_ex			:1;/*��λ״̬*/
	unsigned int last_dir				:2; /**/
	//unsigned int res					:1;		/*Ԥ��*/ 
};

union STEP_ST_REG {
   unsigned int all;
   struct STEP_ST_BITS bit;
};



	
struct STEP_Debug_Test_BITS {
	unsigned char justrun_test			:2; /**/
	//unsigned int res					:1;		/*Ԥ��*/ 
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
	unsigned char alert_delay_max;/*������ʱ������Ĭ��Ϊ0����ʾ��������*/
	unsigned char alert_delay_cnt;/*������ʱ��������ֵ�����������max�򱨾�*/
	unsigned char alert_delay_zerolost_cnt;/*������ʱ��������ֵ�����������max�򱨾�*/

	unsigned char LX_act_just_move_zero;/*by hlc 20190701 ���˶��������0 ��������*/

	unsigned char act_is_justcheck_zero;/*���׻����õ���ֻ����λ�ź�*/

	unsigned char check_signal_edge;/*20191119 */

	unsigned char is_stop_wait;
	unsigned char change_speed;/**/

	unsigned char isdoing_with_bind_cmd;/*20220927 �ó�1��ʾ����ִ�а����������������*/
	unsigned int	max_speed_back;   	//��һ�����Ƶ��

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
	unsigned int	max_speed;   	//���Ƶ��


	

#ifdef STEP_MOTOR_DDDM_SUPPORT
	 int speed_acc_dec;				//����Ƶ��	(ÿ�μӼ�����Ƶ��)
	unsigned int low_speed;			//����Ƶ��
	//unsigned int max_speed;			//���Ƶ��
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
	unsigned int moto_zero_width;          /*��������������ֵ�����ǵ�����λ֮�����߶������м�,ɴ�����Ƚ�����*/
	unsigned int moto_zero_width_self;		/*�����λ��ʱ���Զ������һ��ֵ*/
	unsigned int moto_zero_width_temp;	
	unsigned int need_2_pos_after_reset;		/*�ȸ�λ�����ߵ����λ��*/
#endif
	//unsigned int need_2_pos_maxspeed;

	//unsigned int is_poweron;
	unsigned int error_count_check;
	int input_errorstep;		//���������(������������ʱ�򣬲�����)
	
	unsigned int steps_go_;			//���˶��ٲ���
	unsigned int steps_check_zero;	//��λ��鷶Χ(��Ϊ��һ����Χֵ��Ĭ��Ϊ0)
	//unsigned int steps_go_leftright;	//���˶��ٲ��˵���߻��ұ�
	unsigned int steps_go_temp;		//�ڲ�������
	
	
	unsigned int state_chi;			//0,��һ�ν�0λ��1-�뿪0λ��2�,�ڶ��ν�0λ
	unsigned int state_chi_last;			//��һ�ε�״̬
	
	unsigned int last_new_input_sts;			//������������״̬�����µ�״̬���ֱ�ռ��λbit0-1,last,bit2,3new 0,2:zero.1-3:work,bit4 ��ʾ�Ƿ�仯

	
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
	unsigned char DD_error_cnt;/*��Ŀ���0-8�����˲�������*/
	unsigned char Had_next_cmd;
	Step_DD_CFG moto_dd_config;
	union STEP_CHK_REG chk;
	union STEP_ST_REG  step_st;	
	union STEP_CHK_NO_CLEAR_REG st_no_cl;
	union STEP_ALERT_ST_REG step_alert_st;/*������������ϱ����״̬*/
	union STEP_ALERT_DELAY_REG step_alert_delay_pos2_st;
	union STEP_Debug_Test_REG step_debug_test_st;
	//#endif
	


}STEP_TYPE;


typedef struct {
	unsigned short cmdID;	/*�������*/
	unsigned char Motor_NO1;/*�����1*/
	unsigned char Motor_NO2;/*�����2*/
	short Targer_pos_1;	/*�����1��ӦĿ��λ��*/	
	short Targer_pos_2;	/*�����2��ӦĿ��λ��*/
	unsigned short Delay_timeout_2ms;/*��ʱʱ��*/
	unsigned short Exec_TimeOut;	/*����ִ�г�ʱʱ��*/
	unsigned char Cmd_step;/*��ǰִ�е��ڼ���:
							0--������
							1--�յ�����
							2--���ڹ���
							3--�������
							4--����ִ��
							5--ִ�����*/ 
							
	

}STEP_BIND_cmd;




/*��Ŀ�����λ��鱨���ͺ󼸴α���*/
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
#define GOTO_ZERO_TRY_ZERO_MIDD	(19)		/*���ؼ������λ��ʲôλ��*/

#define GOTO_TRY_ZERO_STEPS	50			/*ÿ�γ���������չѰ����λλ�õĲ���*/

#endif

#define JUST_RUN_RESET 				(20)
#define ENCODER_JUST_RUN			21

#define JUST_RUN_AFTER_ADJ 			30

//#ifdef TRIANGLE_STEP_SUPPORT

/*�������Ҫ����ôЩ��������Ϊ�Ǳ�������*/
#define STEP_GO_ALARM_STEPS	4

/*�������Ҫ����ôЩ��������Ϊ�ǿ��Ե�ͷ��*/
#define STEP_GO_DIR_STEPS	80
#define STEP_GO_DIR_STEPS_MIN	10

/*����Ӽ��ٲ���*/
#ifdef FOR_BW_DM_SLOWLYRUN
#define STEP_DDM_STEPS		(32)  //(16)  
#else
#define STEP_DDM_STEPS		(16)  
#endif
//#ifdef LX_ACTION_STEP_SPECIAL

//#define STEPS_LEAVEZERO_FOR_LX	(100)  
/*20180723 �޸ĳ�150 ��������*/
#define STEPS_LEAVEZERO_FOR_LX	(200)    
/*���������ֵ*/
/*������������10���㣬�Ӹ���������10����*/
#ifdef TEST_CX_ACT
#define STEP_ZERO_GOON_STEPS	(-1)
#else
#define STEP_ZERO_GOON_STEPS	(3)
#endif

#define HP_STEP_MOTOR_CHECK_INPUT_AREA	(100)   /*HP��������ÿ�ζ�����ǰ��鵽λ�Ĳ����������ֵ*/

//#endif

#define ALERT_DELAY_CNT_DEFAULT		5 /*Ĭ�ϼ��α���֮��ű���*/

#define STEPS_LEAVEZERO	(30)//Ĭ���뿪0λ�Ĳ��� 

#define STEPS_LEAVEZERO_SMALL	(25)//Ĭ���뿪0λ�Ĳ��� 


#define STEPS_ERROR_RANGE	(5)	//��λƫ��������


#define STEPS_ZERO_GOBACK	(0)  /*����������Զ�����֮�������߶��ٲ�,Ĭ��0*/

#define GOTO_LEFT	8		//�ߵ�01λ��(-300)
#define GOTO_RIGHT	9		//�ߵ�10λ��(300)
#define CHECK_IS_LEFT 10     //���01λ���ǲ�����ĵ���
#define CHECK_IS_RIGHT 11   //���11λ���ǲ�����ĵ���
#define CHECK_ISNOT_LR 12     //����ǲ�������뿪������

#define TEST_SIGN_WIDTH  0x20   			//��⴫�������
#define TEST_SIGN_WIDTH_LEAVE  0x21   //��⴫��������뿪�м�
#define TEST_SIGN_WIDTH_IN 0x22   		//�ص��������м�
#define TEST_SIGN_WIDTH_LEAVE_EX 0x23  



#define TRIANGLE_LEFT_POSTION 	(0-240)
#define TRIANGLE_RIGHT_POSTION 	(240)
#define TRIANGLE_ZERO_POSTION 	(0)

#define DEF_CHECK_WORK_POS  (2000)

//#endif


enum {
	HEAD_MODE_DEFAULT = 0,		/*4��Ŀ+2����+2����+4ɴ��(����е��ɳ��Ļ�)*/
	HEAD_MODE_SIMPLE,			/*4��Ŀ+2����+2�հ�+4ɴ��(����е��ɳ��Ļ�)*/
	HEAD_MODE_SKMODE2,			/*4��Ŀ+2����+2�հ�+4ɴ��(����е��ɳ��Ļ�)*/
	HEAD_MODE_SKMODE4,			/*4��Ŀ+4����+4ɴ��(����е��ɳ��Ļ�)*/
	HEAD_MODE_LIFT2,			/*4��Ŀ+2����+2̧��+4ɴ��(����е��ɳ��Ļ�)*/	// ADD by zhuqiwei 160601
	HEAD_MODE_LIFT_EX,		/*4��Ŀ+2����+2����+4ɴ��(����е��ɳ��Ļ�)+2̧��*/ //E490_v1.0
	HEAD_MODE_FEET,			/*4��Ŀ+2ѹ��+2����+4ɴ��(����е��ɳ��Ļ�)    ����*/
	HEAD_MODE_LIFT_HP,		/*4��Ŀ+2����(HP)+2����+4ɴ��(����е��ɳ��Ļ�)+2̧��*/ //20160816
	HEAD_MODE_LX_ACT,			/*4��Ŀ+2����(LX)+2����+4ɴ��(����е��ɳ��Ļ�)+2̧��*/ // 2016 11 29 
	HEAD_MODE_FH_ACT,		/*4��Ŀ+2����(FH)+2����+4ɴ��(����е��ɳ��Ļ�)+2̧��*/ // 2016 11 29 
	//HEAD_MODE_LIFT3,			/*4��Ŀ+2����+2����+2����+4ɴ��(����е��ɳ��Ļ�)*/
	HEAD_MODE_FEET_CX,		/**4��Ŀ+2ѹ��+2����+4ɴ��(����е��ɳ��Ļ�)    +2̧��---����20171122*/
	HEAD_MODE_ALL_STEP_CX,	/**4��Ŀ+2����+2ѹ��+4ɴ��(����е��ɳ��Ļ�)    +2̧��+��չ2����---����20181129*/
	HEAD_MODE_MAX,
};

/*E480--���֧��12�������Ĭ��0ģʽ֧��0,1,2,3 ����ģʽ��E475--���֧��6�������Ĭ��2ģʽ��֧��1,2����ģʽ*/

enum {
	MOTOR_TYPE_UNDEF = 0,	//0-δ���� 
	MOTOR_TYPE_DENSITY,	// 1-��Ŀ
	MOTOR_TYPE_SKINER,	// 2-���
	MOTOR_TYPE_ACTION,	// 3-����
	MOTOR_TYPE_FEET,		// 4-ѹ��
	MOTOR_TYPE_YARN,		// 5-ɴ��
	MOTOR_TYPE_LIFT,		// 6-̧��
	MOTOR_TYPE_OTHER,		//7-�������(����������ģʽ���������˵��)
	MOTOR_TYPE_UPDOWN,	//8-����ɳ�������˶������ÿϵͳ4������ԭ��ɳ�����ӿ�
	MOTOR_TYPE_MAX,		//������ͺ�
};
#define MOTOR_TYPE_COUNT	(MOTOR_TYPE_MAX-1)			//���е����������
#define MOTOR_TYPE_0XFF	(0xFF)				/* �������Ϊ0XFF��ʱ��ֱ�Ӷ�����ID����*/

#define MOTOR_TYPE_EXE_PT		0

enum{
	DENSITY_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,
	DENSITY_MOTOR_TYPE_JF,
	DENSITY_MOTOR_TYPE_LX,
	DENSITY_MOTOR_TYPE_OTHER,	
};

enum{
	ACT_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,   /*0: ��ͨ�İ������߲���*/
	ACT_MOTOR_TYPE_HP,		/*1: HP������ź������*/
	ACT_MOTOR_TYPE_LX,		/*2: LX�����λ��ȱ�ڵ�(F��)*/
	ACT_MOTOR_TYPE_FH,		/*3: FH������˻����ϣ�����λ�������һ���źż��*/
	ACT_MOTOR_TYPE_DOT,		/*4: �ź����м�һ�����ģʽ*/
};

enum{
	LIFT_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	LIFT_MOTOR_TYPE_HF,                /*֧�ֺƷ�ģʽ*/	
	LIFT_MOTOR_TYPE_HF_EX,	 /*�м�û�źţ��������źţ������ʱ����Ҫ�����м�հ׿�ȴ�С*/
};

enum{
	SINKER_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	SINKER_MOTOR_TYPE_LX,                /*֧������ģʽ*/	
	SINKER_MOTOR_TYPE_LX_EX,		/*����ģʽ������������һ��*/

};



enum{
	FEET_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	FEET_MOTOR_TYPE_CX,                /*֧�ִ���ģʽ���ǹ���λ�����ټ��*/	
	FEET_MOTOR_TYPE_CX2,		  /*֧�ִ���ģʽ--����λ���߲���*/
	FEET_MOTOR_TYPE_CX3		/*֧�ִ���ģʽ--�����ź��ߣ���λ��Ҫ����*/	
};


enum{
	YARN_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	YARN_MOTOR_TYPE_LX,                /*֧������ģʽ�ĵ��ɴ�죬�������������м���(�ź����м�)����ͷ��λ��(����λ�ͻ�е��λ)*/	
};






// STEP running flag 
#define RUNNING		1
#define RUNNING_OVER	2
#define RUNNING_GO	3
#define RUNNING_END	4

#ifdef LOG_DEBUG_FOR_LX_AT_CH
enum{
	LX_LOG_TYPE_ZERO_ADJ_GOTO=0, /*��λ������ 0*/
	LX_LOG_TYPE_ZERO_ADJ_GOTO_2, /*��λ������1*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE, /*��λ������2*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE_2, /*��λ������3*/
	
	LX_LOG_TYPE_MOVETOZERO_ADD400, /*����λ����400��4*/
	LX_LOG_TYPE_MOVETOZERO_ADD400_2, /*����λ����400��5*/
	LX_LOG_TYPE_LEAVEZERO_ADD200, /*�뿪��λ����200��6*/
	LX_LOG_TYPE_LEAVEZERO_ADD200_2, /*�뿪��λ����200��7*/
	LX_LOG_TYPE_RUNNING_OVER,	/*�������������8*/
	LX_LOG_TYPE_RUNNING_OVER_2,	/*�������������9*/
	LX_LOG_TYPE_STOP_GO,	/*ֹͣ�˵���λ��ֵ��Ŀ10��ֵ����ȣ�������*/
	LX_LOG_TYPE_STOP_GO_2,	/*ֹͣ�˵���λ��ֵ��11Ŀ��ֵ����ȣ�������*/
	LX_LOG_TYPE_ZERO_CHANGE,/*��λ�źű仯12*/	
	LX_LOG_TYPE_ZERO_W,/*��λ�źſ��13*/	
	LX_LOG_TYPE_ADJ_ISOK_BUT,/*��λ����14������޵����������ˡ�*/
	LX_LOG_TYPE_ADJ_ISOK_BUT_2,/*��λ���ˣ�15�����޵����������ˡ�*/
};
#define ADJ_NEED_LOG_STEPS	(20)		/*����������������ű���*/

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

// ���ڲ��Ե���Ƿ�λ
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

