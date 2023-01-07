#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "arch.h"	 
#include "config.h"
#include "command.h"
#include "alert.h"
#include "step.h"
#include "massage.h"
#include "encoder.h"
#include "Eeprom.h"
#include "Platform_config.h"
//#endif

//#define DEBUG_DATA_STORE 


typedef uint16_t		b16_t;


#define CHECK_LIFT_STEP_IS_STOP

//#define CHECK_LIFT_IS_WORK

extern volatile unsigned char sys_max_jqd_cnt;

//unsigned int timer6_isr_cnt=0;

unsigned char JQD_overload_alerted =0;


/*
 * 新选针控制模式支持
 * 针中断命令可以随意组合选针器
 */
#define JAQ_WORK_MODE
/*
 * 纱嘴电磁铁动作执行两次工作模式支持 
 */
//#define EMF_TIME2_SUPPORT

#define YARN_NEEDLE_STATUS_CHECK
//#define TEST_SELF
volatile unsigned int error_active;
volatile unsigned int work_mode;

volatile unsigned char DA_is_out;
volatile unsigned char LocalSys_is_enable =1;

volatile unsigned char Yarn_use_Step=0;
volatile unsigned char TZ_use_Step=0;
volatile unsigned char SK_use_Step=1;
volatile unsigned char Step_use_exBoard=0;  /*使用电机扩展板*/
volatile unsigned char Yarn_Step_Updown=0;/*将沙嘴电机用于上下运动，接口从电机命令引入*/

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
volatile unsigned char Needle_Selector_is_tenBlade=0;/*判断当前板是不是10段选针板*/
extern volatile unsigned char sys_max_blade_phi;
#endif

volatile unsigned short JQD_do_delay[2]={0,0};/*20190404 连兴选针开关区别执行,需要有延时*/

volatile unsigned int CAN_error_cnt=0;

volatile unsigned int CAN_error_cnt_err[5]={0,0,0,0,0};

volatile unsigned int act_check_enable=0;

//volatile unsigned char TZ_use_Step=0;		//主要是抬针三角

volatile unsigned char Send_check_online=0;
volatile unsigned int Send_delay_online=0;


volatile unsigned int JQD_halfwork_mode;		//0,正常模式，4选针，8刀，!0,模式 8选针，4刀.

volatile unsigned char SYS_Test_Mode_ISPWM=0; // 是否使用PWM模式测试系统阻值

//#define Needle_isok_mask_test (1)


unsigned short Data_store_key_sys=0;
unsigned short Sys_power_lost_bit =0;

#ifdef DEBUG_DATA_STORE

#define DEBUG_ST_READ		0
#define DEBUG_ST_WRITE		1
#define DEBUG_ST_READ_ERR		2
#define DEBUG_ST_WRITE_ERR	3
#define DEBUG_ST_FREE		0xffff
unsigned char Debug_data_st =DEBUG_ST_WRITE;


#endif


#ifdef Needle_isok_mask_test
volatile unsigned int Needle_isok_mask;  //测试
#endif


#ifdef JQD_ISR_TIMER_ONE
volatile unsigned char jqd_last_blade[MAX_JACQUARD];
volatile unsigned int jqd_last_blade_st[MAX_JACQUARD];

volatile unsigned short needresetjqd=0;
volatile unsigned char jqd_isfirst_do[MAX_JACQUARD][MAX_BLADE];/*确认是否是第一次动作0--第一次动*/

#endif


#ifdef JAQ_WORK_MODE
volatile unsigned int jqd_blade[MAX_JACQUARD/*8*/];
#else
volatile unsigned int jqd_blade;
volatile unsigned int jqd_blade_front;
volatile unsigned int jqd_blade_back;
#endif
volatile unsigned int jqd_max_blade;
volatile unsigned int jqd_clear_blade[MAX_BLADE/*8*/][MAX_JACQUARD/*8*/];
volatile unsigned int jqd_start_blade[MAX_JACQUARD/*8*/];
volatile unsigned int blade_group_keep_time[MAX_BLADE/*8*/][MAX_JACQUARD];

volatile unsigned int blade_group_delay_time[MAX_BLADE/*8*/][MAX_JACQUARD];


#ifdef JQD_XZQ_NEEDLE_REDO_MODE
volatile unsigned short jqd_blade_redo_cnt[MAX_JACQUARD][MAX_BLADE];

volatile unsigned short jqd_blade_redo_cnt_set=0;
volatile unsigned short jqd_blade_redo_time_set=0;


#endif
//volatile unsigned int blade_group_last_setup_systick[MAX_BLADE/*8*/][MAX_JACQUARD];

//#define JQD_CTR_CMD671_670_MUTEX	/*671命令和670命令互斥*/

#ifdef JQD_CTR_CMD671_670_MUTEX

volatile b16_t jqd_start_inited=0;	//选针器起始刀设置标志位

#endif


#ifdef JAQ_WORK_MODE
volatile unsigned int blade_group_active_jqd[MAX_BLADE/*8*/];
#else
volatile unsigned int blade_group_keep_time_F[MAX_BLADE/*8*/];
volatile unsigned int blade_group_keep_time_B[MAX_BLADE/*8*/];
#endif

#ifdef DO_XZQ_ALL_2STEP
/*本来是每一位对应一个选针器，现在要每两位对应一个选针器*/
volatile unsigned int jqd_keep_time_test2[MAX_JACQUARD/*8*/];
volatile unsigned int jqd_test_mode2;	//
volatile unsigned int jqd_2step_is_do;	// by hlc 选针起的分段动作是否启动
volatile unsigned int jqd_status2[MAX_JACQUARD/*8*/];
volatile unsigned int blade_2step_keep_time[MAX_JACQUARD][XZQ_KEEPTIME_STS_CNT];//每个选针器 分段处理的两个参数，1、间隔时间，2、状态

#endif
volatile unsigned int jqd_keep_time_test[MAX_JACQUARD/*8*/];


volatile unsigned int jqd_test_mode;	// by xhl 2011/09/20


volatile unsigned char alert_runtime_check=1;/*报警实时检查(只针对探针和翻针导块)*/

volatile unsigned int jqd_status[MAX_JACQUARD/*8*/];
volatile unsigned int jqd_status_last[MAX_JACQUARD/*8*/];

volatile unsigned int jqd_keep_time_setup;
volatile unsigned int jqd_keep_time_setup_us;	
volatile unsigned int jqd_doitmust=0;

volatile unsigned int jqd_blade_next_do[MAX_JACQUARD][5];/*0--mask,1--st,2--nextid,3--isr-can do ，4--flag to irs_loop-getnext*/



volatile unsigned int jqd_keep_time_setup_2;

//volatile unsigned int jqd_pwm_time_setup;
volatile unsigned int jqd_direction;
volatile unsigned char JQD_Return_reset=0;
volatile unsigned char JQD_cmd_index=0;
volatile unsigned char JQD_cmd_index_next=0;
volatile unsigned int can_jqdcmd_repeat_count=0;/*选针命令重复*/

volatile unsigned char jqd_dir_back=0;

#ifdef CHECK_LIFT_IS_WORK
volatile unsigned char check_start=0x00;
volatile unsigned char yarn_is_do=0x00;

#endif

#ifdef CHECK_LIFT_STEP_IS_STOP
volatile unsigned char need_c_f_g_knitarea;
volatile unsigned char whichJQD=0xFF;
volatile unsigned short jqd_cmd_cnt ;//=jqd_max_blade>>1;
volatile unsigned short lift_step_after_jqd_arg=0;

#endif
volatile unsigned int jqd_safe_time;

#ifdef NEEDLE_POS_SUPPORT
// by xhl 2011/07/27
volatile unsigned int needle_pos;
volatile unsigned int needle_pos_mask;
#endif

volatile unsigned int emf_safe_time;
volatile unsigned int emf_keep_time[20];
#ifdef EMF_TIME2_SUPPORT
// by xhl 2012/04/17
volatile unsigned int Yemf_time2_flag;	// just for yarn emf
volatile unsigned int Yemf_time2_flag2;	// just for yarn emf
volatile unsigned int Yemf_time2_delay;	// just for yarn emf
#endif
volatile unsigned int emf_status[3];
volatile unsigned int emf_status_isdone;

volatile unsigned char need_check_rs=0;
volatile unsigned char need_check_rs_each=0;
volatile unsigned char test_delay_2ms =0; /*默认0毫秒*/
volatile unsigned char nowtestemfindex=0;

//volatile unsigned int yarn_status;

volatile unsigned char Test_Step_enable	=0;





volatile unsigned int 	jqd_blade_sts_check_mainloop[MAX_JACQUARD][MAX_BLADE];  
volatile unsigned int  	jqd_blade_sts_check_01ms[MAX_JACQUARD][MAX_BLADE];


volatile unsigned int main_loop_cnt;
volatile unsigned int ms01_loop_cnt;

volatile unsigned char MainID_Get=0;/*bit 0表示mainid低命令位收到，bit1表示mainid高位命令收到*/





volatile unsigned int emf_on_keep_time_setup;
volatile unsigned int emf_off_keep_time_setup;
volatile unsigned int emfYARN_on_keep_time_setup;
volatile unsigned int emfYARN_off_keep_time_setup;
volatile unsigned long emf_work_mode;


#ifdef LX_JQD_MASK_ENABLE
volatile unsigned char jqd_time_is_test_ex[MAX_JACQUARD/*8*/][MAX_BLADE]; //by hlc
#endif
volatile unsigned int jqd_keep_time_test_ex[MAX_JACQUARD/*8*/][MAX_BLADE]; //by hlc
 volatile unsigned int jqd_test_mode_ex[MAX_JACQUARD];	//by hlc


extern volatile unsigned char jqd_pwmsts[MAX_JACQUARD][MAX_BLADE];
extern volatile unsigned char jqd_clear_sts[MAX_JACQUARD][MAX_BLADE];

extern volatile unsigned int stepmotor_current[];
extern volatile unsigned char motor_type_remap[];

extern volatile unsigned int DC24_P_CURR_Zero;
extern volatile unsigned int DC24_N_CURR_Zero;
extern volatile unsigned char StepMotor_Count_MAX;
extern volatile unsigned int whichOverload_in_progress_mask;
extern volatile unsigned char which_part_close_power;

extern volatile unsigned char Head_Mode_;

#ifdef NEW_ALARM_STYLE
extern unsigned int sys_type[];

#endif
#if 0
volatile unsigned int emf_setup_on_keep_time[40];
volatile unsigned int emf_setup_off_keep_time[40];
#endif
#ifdef STM32_BOARD
volatile unsigned int tanzhen_alarm[2];
volatile unsigned int tanzhen_delay[2];
volatile unsigned int tanzhen_active[2];
volatile unsigned int tanzhen_alarm_delay_count[2];
volatile unsigned int tanzhen_alarm_delay_num[2];
#endif



volatile unsigned int BootVer[5] __attribute__((at(0x2001F000)));
volatile unsigned int BootVer_int;



volatile unsigned int check_dir_still_zero;
void Check_DirStatus_new(void);

void Exec_EMF_isr(unsigned int which,unsigned int dctno,unsigned int sts,unsigned short times_ms);
void Exec_alarm_shock(unsigned int whichdata,unsigned int CNdata,unsigned int whichshock);
void Exec_alarm_check_start(unsigned int ischeckdir);
void Exec_alarm_check_end(unsigned int maskdata,unsigned int data_a);
int Exec_EMF_isr_EX(unsigned int which,unsigned int dctno,unsigned int sts);
unsigned int Exec_EMF_GET_EX(unsigned int which,unsigned int dctno,unsigned int sts);
void Jacquard_blade_remap(int s_blade,int maxbladecnt,char is0_7);
unsigned short send_check_mainloop_and_01ms(unsigned char jqdno,unsigned char blandno);
extern void arch_shock_ctr(int isenable);

#ifdef E480_BOARD_V10
void Exec_Yarn_step_cmd(unsigned short arg1,unsigned short arg2,unsigned short arg3);
#endif


extern void LIFT_step_PWMDA_Set_val(unsigned int Step_A);
extern void Time_stop_time5(void);
extern unsigned int Get_Build_Time(unsigned int *buf);
extern void DAC_SetVoltage_channel1(long);
extern void DAC_SetVoltage_channel2(long);
extern void ADC_start_main(void);
extern void Alert_Poll(void);
extern int alert_push_again(void);
extern void Exec_Set_Motor_Curr(unsigned char steptype,unsigned short cruu);
extern unsigned short RegularConvData_Tab[];
extern void arch_check_Overload_longTime(unsigned char is250us);
extern unsigned char arch_is_shock_board(void);
#ifdef NEW_ALARM_STYLE
extern void Set_stepmotor_alarm_id(unsigned int sysadd,unsigned int *systype);
#endif
// //选针器动作，下一针的序号，原来为1，HP3G改为2
static volatile unsigned short needle_work_step;
#ifdef LOGOUT_ENABLE
volatile unsigned short logout_enable;
volatile unsigned short logout_needle_pos;
#endif

extern int Decrypt_can_data(unsigned short *data_buff,unsigned short syskey);
extern int Encryption_can_data(unsigned short *data_buff,unsigned short syskey);

//#ifdef STEP_WORK_SIGN_SUPPORT
extern volatile unsigned int step_work_sign_alarmenable;
//#endif

extern volatile int feet_enable;

static volatile unsigned int build_time;





#ifdef JQD_INIT_DELAY_ENABLE
volatile unsigned int Jacquard_Init_delay_ms;//刀头初始化延时(3-5ms)
#endif





extern volatile unsigned char arch_check_JQD_YARN_ACT_Bit;
extern volatile unsigned int DC24_PN_data_Arry[];//MAX_DC24_DATA_COUNT];
#ifndef MAX_DC24_DATA_COUNT
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		#define MAX_DC24_DATA_COUNT (80)
	#else
		#define MAX_DC24_DATA_COUNT (72)
	#endif
#endif

#define CMD_RETURN  ((cmd->cmd_data << 8) | cmd->cmd_type)
#define CMD_RETURN_CHECKONLINE	 (arch_Get_ID()<<8 | cmd->cmd_type)
#define CMD_RETURN_CHECKONLINE_D	 (arch_Get_ID()<<8 | CMDTYPE_CHECKONLINE)

#define CMD_RETURN_DATA_STORE_READ  ( CMD_DATA_STORE_READ_DATA<<8 | CMDTYPE_DATASTORE)
#define CMD_RETURN_DATA_STORE_WRITE  ( CMD_DATA_STORE_WRITE_DATA<<8 | CMDTYPE_DATASTORE)

#define CMD_RETURN_CHECKIN_SET  ( CMD_OTHERSET_SET_CHECKIN_TIME<<8 | CMDTYPE_OTHERSET)


#ifdef YARN_ZERO_CHECK
#define CMD_RETURN_CHECKYARNZERO	 (CMD_OTHERSET_RETURN_YARN_ZERO_ST<<8 | CMDTYPE_OTHERSET)

#endif

#define CMD_CHECK_ISSUPPORT()	if (Check_CMD_is_Support(cmd)) \
								{	\
									break;\
								}


unsigned int Get_Version(void);

void arch_init(void);
void arch_LED_Setup(unsigned long time);
int arch_time_500_pulse(void);

void WatchDog_Kick(void);
void Get_base_DC24_zero(void);

void Shell_Poll(void);
void Overload_Poll(void);
void StepMotor_Poll(void);
int arch_setup_Jacquard_blade_map(int blade, int remap);
void Time_2ms_check_binding(void);

extern unsigned long Get_STEPMOTO_add(unsigned int stepno);
extern unsigned int StepMotor_get_no_with_IDall(unsigned int);
//extern void arch_StepMotor_Enable_single(unsigned int whichstep);
extern void alert_set_Step_PWM_A(int Step_A);
extern void Yarn_step_PWMDA_Set_val(unsigned int Step_A);
extern unsigned int arch_get_ticktime(void);

extern void StepMotor_timer(void);
extern void alert_time_poll(void);

extern void SKER_step_PWMDA_Set_val(unsigned int Step_A);

#if 0
void clear_all_EMF()
{



	int i;

	for(i = 0; i < 12; i ++) {
		if(emf_work_mode & (0x1 << i)) {
			continue;
		}
		arch_ACTEMF_Clear(i);
	}

	for(i = 0; i < 8; i ++) {
		if(emf_work_mode & (0x1 << (i+12))) {
			continue;
		}
		arch_YARNEMF_Clear(i);
	}
}
#endif

void JQD_Data_Init()
{
	unsigned int i;

#ifdef NEEDLE_POS_SUPPORT
	needle_pos = 0;
	needle_pos_mask = 0x1;
#endif


	jqd_safe_time = 0;
	jqd_keep_time_setup = XZQ_TIME;
	#ifdef JQD_IS_FAST_MODE_2_STEPS
	jqd_keep_time_setup_us = 520;//us
	#else
       jqd_keep_time_setup_us = 1500;
	#endif
	//isfastjqd=0;   

	
	jqd_keep_time_setup_2 = XZQ_TIME_2;
	//jqd_pwm_time_setup = XZQ_BL;
#ifndef JAQ_WORK_MODE
	jqd_blade = 0;
	jqd_blade_front = 0;
	jqd_blade_back = 0;
#endif
	jqd_direction = 0xFFFF;
	jqd_max_blade = 8;  /*工作中用的最大刀数*/

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	sys_max_blade_phi =arch_is_tenBladeSelector_Board()?MAX_BLADE_HARDWARE_10:MAX_BLADE_HARDWARE_8; /*硬件办卡支持的最大刀数*/ 
#endif

	JQD_halfwork_mode = 0;	//正常模式

	jqd_test_mode = 0;

#ifdef DO_XZQ_ALL_2STEP
       jqd_test_mode2 =0;
	jqd_2step_is_do =0;
#endif
	
	for(i = 0; i < MAX_JACQUARD; i++) {
		int j;
		jqd_status[i] = 0xFFFF;
		jqd_status_last[i] = jqd_status[i] ;
		jqd_start_blade[i] = 0;
		jqd_blade_next_do[i][2]=0xff;
		jqd_blade_next_do[i][3]=0;
		jqd_blade_next_do[i][4]=0;
		for(j = 0; j < MAX_BLADE; j++)
			{
			blade_group_keep_time[i][j] = 0;
			jqd_keep_time_test_ex[i][j] = 0;
			#ifdef LX_JQD_MASK_ENABLE
			jqd_time_is_test_ex[i][j] = 0;
			#endif
			blade_group_delay_time[i][j]=0;
			//blade_group_last_setup_systick[j][i]=arch_get_ticktime();

			
			}
#ifdef JAQ_WORK_MODE
		jqd_blade[i] = 0;
		blade_group_active_jqd[i] = 0;
#else
		blade_group_keep_time_F[i] = 0;
		blade_group_keep_time_B[i] = 0;
#endif
		jqd_keep_time_test[i] = 0;


		jqd_test_mode_ex[i] = 0;
		
#ifdef DO_XZQ_ALL_2STEP 
		jqd_keep_time_test2[i] = 0;
		blade_2step_keep_time[i][XZQ_KEEP_TIME_INDEX]=0;
		blade_2step_keep_time[i][XZQ_STS_INDEX]=0;
		
#endif
	}

		needle_work_step = 1;

}

void Alarm_Init()
{
	unsigned int i;
	error_active = 0xFFFF;
	work_mode = 0;


#ifdef STM32_BOARD
	for(i = 0; i < 2; i++) {
		tanzhen_alarm[i] = 0;
		tanzhen_delay[i] = 0;
		tanzhen_active[i] = 1;
		tanzhen_alarm_delay_num[i] = 0;
		tanzhen_alarm_delay_count[i] = 0;
	}
#endif


	check_dir_still_zero = 0;

#ifdef LOGOUT_ENABLE
	logout_enable = 0;
	logout_needle_pos = 1000;
#endif


}

void EMF_Init()
{
	int i;

	emf_work_mode = 0x00000000;
	emf_on_keep_time_setup = DCT_TIME;
	emf_off_keep_time_setup = DCT_TIME;

	emfYARN_on_keep_time_setup = 0;
	emfYARN_off_keep_time_setup = 0;

	emf_safe_time = 0;
	
for(i = 0; i < 20; i++) {
		emf_keep_time[i] = 0;
	}

#ifdef EMF_TIME2_SUPPORT
	Yemf_time2_flag = 0;
	Yemf_time2_flag2 = 0;
	Yemf_time2_delay = 2/*5*/;
#endif

	for(i = 0; i < 3; i++) {
		emf_status[i] = 0;
		
	}
	emf_status_isdone=0;
	emf_status[EMF_INDEX_YARN] = 0xffff;	//wf,20131104, 默认为0时，三系统右边系统会报错
	
	//emf_status[4] = 0xf000;	//wf,20131104, 默认为0时，三系统右边系统会报错

	//yarn_status=emf_status[2];
}

#ifdef JQD_XZQ_NEEDLE_REDO_MODE
void JQD_redo_cnt_init()
{
	//int i;
	if (!jqd_blade_redo_cnt_set) return;
	
	memset((char *)&jqd_blade_redo_cnt[0][0],0xff,sizeof(jqd_blade_redo_cnt));
	//jqd_blade_redo_cnt
		

}



void JQD_redo_cnt_add(unsigned int jqdno)
{
	int i;
	if (!jqd_blade_redo_cnt_set) return;
	
	for (i=0;i<MAX_BLADE;i++)
	{
		if (jqd_blade_redo_cnt[jqdno][i]<0xffff)
			jqd_blade_redo_cnt[jqdno][i]++;
	}
	
}


void JQD_redo_cnt_check_redo(unsigned int jqdno)
{
	
	int i;
	unsigned int bld_i;
	if (!jqd_blade_redo_cnt_set) return;
	
	for (i=0;i<MAX_BLADE;i++)
	{
		bld_i = jqd_clear_blade[i][jqdno];
		if (jqd_blade_redo_cnt[jqdno][bld_i]==jqd_blade_redo_cnt_set)
		{
			arch_Jacquard_Setup(jqdno, bld_i,(jqd_status[jqdno]>>bld_i)&0x01,1);
			blade_group_active_jqd[bld_i/*i*/] |= (0x1 << jqdno);
			if (jqd_blade_redo_time_set)
				blade_group_keep_time[bld_i/*i*/][jqdno] =jqd_blade_redo_time_set;
			else
				blade_group_keep_time[bld_i/*i*/][jqdno] = jqd_keep_time_setup;
			jqd_blade_redo_cnt[jqdno][bld_i]++;
		}
		
	}


}


void Send_debug_jqd_redo_cnt()
{
	int i,j;
	for (i=0;i<sys_max_jqd_cnt;i++)
	{
		for (j=0;j<MAX_BLADE;j++)
		{
			Message_Send_4halfword(0x09|(0x07<<8),i|(j<<8), jqd_blade_redo_cnt[i][j],jqd_clear_blade[j][i]);
		}
	}

}


#endif


/************************************************
 * Start Jacquard Ctrl
 ************************************************/
void Jacquard_Init(void)
{
	unsigned int i, j;

	#ifdef JQD_XZQ_NEEDLE_REDO_MODE
	JQD_redo_cnt_init();
	#endif

	for (j = 0; j < sys_max_jqd_cnt/*8*/; j++)
	{
		#ifdef JQD_CTR_CMD671_670_MUTEX

		if (jqd_start_inited & (1 << j))
		{
			jqd_start_inited &= ~(1 << j);
			continue;
		}
		#endif
		
		
		jqd_blade[j] = 0;
		for (i = 0; i < jqd_max_blade; i++)
			jqd_clear_blade[i][j] = (jqd_start_blade[j] + i) % jqd_max_blade;
	}
}




#define JQD_INIT_DO_MASK	0x55AA
void Jacquard_Init_blade(unsigned int jqdno, unsigned short arg)
{
	unsigned int i;

#ifdef JQD_INIT_DELAY_ENABLE
	if (arg ==JQD_INIT_DO_MASK)
	{
		#ifdef JQD_CTR_CMD671_670_MUTEX
		if (jqd_start_inited & (1 << jqdno))
		{
			jqd_start_inited &= ~(1 << jqdno);
			
		}
		#endif
		return ;
	}
	
	if (arg == 0)
	{		
		Jacquard_Init_delay_ms = 0;  //重新开始计时
	}
	else
	{
		#ifdef JQD_CTR_CMD671_670_MUTEX
		jqd_start_inited |= (1 << jqdno);
		#endif

	#ifdef JQD_XZQ_NEEDLE_REDO_MODE
	JQD_redo_cnt_init();
	#endif
		
		#ifdef JAQ_WORK_MODE
			jqd_blade[jqdno] = 0;
		#else
			jqd_blade = 0;
			jqd_blade_front = 0;
			jqd_blade_back = 0;
		#endif
		
		for (i = 0; i < jqd_max_blade; i ++)
		{
			jqd_clear_blade[i][jqdno] = (jqd_start_blade[jqdno] + i) % jqd_max_blade;
		}

		
	}
#else
	#ifdef JAQ_WORK_MODE
	jqd_blade[jqdno] = 0;
	#else
	jqd_blade = 0;
	jqd_blade_front = 0;
	jqd_blade_back = 0;
	#endif

	for(i = 0; i < jqd_max_blade; i ++) {
		jqd_clear_blade[i][jqdno] = (jqd_start_blade[jqdno] + i) % jqd_max_blade;
	}
#endif
}

// by xhl 2010/08/09
unsigned int Jacquard_Get_jqdBlade(unsigned int jqdno)
{


	return 0;
}

// by xhl 2010/08/09
unsigned int Jacquard_Get_Blade()
{
#ifdef JAQ_WORK_MODE
	return jqd_blade[0];
#else
	//return jqd_blade;
	return (jqd_blade_front << 8) | jqd_blade_back;
#endif
}

void Jacquard_Set_Blade(unsigned int blade)
{
#ifdef JAQ_WORK_MODE
	int i;
	for(i = 0; i < sys_max_jqd_cnt/*8*/; i ++) {
		jqd_blade[i] = blade;
	}
#else
	jqd_blade_front = blade >> 8;
	jqd_blade_back = blade & 0xFF;

	jqd_blade = jqd_blade_front;
#endif
}

void Jacquard_blade_remap(int s_blade,int maxbladecnt,char is0_7)
{
	int i;
	

#ifdef JAQ_WORKMODE_8_16	
	if (jaq_work_mode_8_16)
	{
		for(i = 0; i < 4; i ++) {
			s_blade %= 4;
			arch_setup_Jacquard_blade_map(i, s_blade);
			s_blade ++;
		}
	}
	else
	{
		for(i = 0; i < MAX_BLADE; i ++) {
			s_blade %= MAX_BLADE;
			arch_setup_Jacquard_blade_map(i, s_blade);
			s_blade ++;
		}

	}
#else
	//if (!is0_7)

		if (maxbladecnt>MAX_BLADE)
		{
			maxbladecnt= MAX_BLADE;
		}

		for(i = 0; i < maxbladecnt; i ++) 
		{
			s_blade %= maxbladecnt;
			arch_setup_Jacquard_blade_map(i, s_blade);
			if (!is0_7)
			{
				s_blade ++;
			}
			else
			{
				if (!s_blade)
				{
					s_blade = maxbladecnt-1;
				}
				else
				{
					s_blade --;
				}
			}
		}
	
	
#endif
	
}

unsigned int Needle_pos_Get()
{
#ifdef NEEDLE_POS_SUPPORT
	return needle_pos;
#else
	return 0;
#endif
}


void Exec_Jacquard_ex(unsigned int jqdno,unsigned int bladeno,  unsigned int sts,unsigned int times)
{
	//unsigned int i;
	unsigned int maxblade_tmp = jqd_max_blade;
	unsigned int maxblades_hw =MAX_BLADE_HARDWARE;
	static unsigned short indextest=0;

	//DC24_PN_EnableBIT |=0x01<<0;

	arch_check_JQD_YARN_ACT_Bit |=(0x01<<(sts?0:3));

	if(jqdno >= sys_max_jqd_cnt/*8*/) return;

	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	maxblades_hw =sys_max_blade_phi;
	#endif
	
	if (bladeno>=maxblades_hw) return;
		
	jqd_safe_time = CLR_JQD_DELAY_TIME;
		//for(i = 0; i <( maxblade_tmp); i ++) {
		if(sts) {
			//if(!(jqd_status[jqdno] & (0x1 << bladeno)))
			{
				arch_Jacquard_Setup(jqdno, bladeno, 1,0,0);				
			}
			jqd_status[jqdno] |= (0x1 << bladeno);
		}
		else {
			//if(jqd_status[jqdno] & (0x1 << bladeno)) 
			{
				arch_Jacquard_Setup(jqdno, bladeno, 0,0,0);				
			}
			jqd_status[jqdno] &= ~ (0x1 << bladeno);
		}
		//arch_Jacquard_ClearBlade(jqdno, bladeno);

		//Message_Send_4halfword(0xFFFF,jqdno|bladeno<<8,sts?1:0,indextest++);
	//}

	//jqd_status[jqdno] =sts;
	if ((times>0)&&(times<20))
	{
		jqd_keep_time_test_ex[jqdno][bladeno] =times;
	}
	else
	jqd_keep_time_test_ex[jqdno][bladeno] = XZQ_TIME_TEST;

	#ifdef LX_JQD_MASK_ENABLE
	jqd_time_is_test_ex[jqdno][bladeno] = 1;	
	#endif
	
	jqd_test_mode_ex[jqdno] |= 0x1 << (bladeno);   
	jqd_pwmsts[jqdno][bladeno]	=0;
		//#endif
	

}


unsigned int Exec_JQD_GET_EX(unsigned int jqdno,unsigned int bladeno)
{
	unsigned int emfid;
	unsigned int maxblade_hw=MAX_BLADE_HARDWARE;
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	maxblade_hw= sys_max_blade_phi;
	#endif

	if(jqdno >= sys_max_jqd_cnt/*8*/) return 0xffff;
	if (bladeno>=maxblade_hw) return 0xffff;

	emfid = jqdno*MAX_BLADE_HARDWARE_8+bladeno;

	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if(bladeno >=MAX_BLADE_HARDWARE_8)
	{
		emfid = 72+(jqdno<<1)+(bladeno-MAX_BLADE_HARDWARE_8);	
	}
	#endif
	
	if (emfid<MAX_DC24_DATA_COUNT)
	{
		return DC24_PN_data_Arry[emfid];
	}else
	return 0xffff;
}

#ifdef JQD_ISR_TIMER_50US

//extern volatile unsigned short jqd_isr_isdo;
//extern volatile unsigned short jqd_isr_isdo_EX_cct[MAX_JACQUARD];

extern void arch_start_jqd_timer(unsigned int whichjqd,unsigned int timercnt_us,unsigned char isfirst);


int Exec_Jqd_blade_next(unsigned int jqdno,unsigned int bladno,unsigned int st,unsigned char redo)
{
	int i=bladno;
	int isdo=0;
	if(st) 
	{
		if((!(jqd_status_last[jqdno] & (0x1 << i)))||(jqd_doitmust)||(redo)) 
		{
			#ifdef JQD_ISR_TIMER_CNT_50US
			{
				jqd_status_last[jqdno] |=(0x01<<i);				
				blade_group_delay_time[i][jqdno] = JQD_do_delay[st?1:0]/50;
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup_us;
				blade_group_active_jqd[i] |= (0x1 << jqdno);		
				isdo =1;
			}
			#else	
			if (arch_Jacquard_Setup(jqdno, i, 1,1,2)==0)			
			{
				#ifdef LX_JQD_MASK_ENABLE
				jqd_status_last[jqdno] |=(0x01<<i);
				//jqd_keep_time_test_ex[jqdno][i] = jqd_keep_time_setup_2;//jqd_keep_time_setup; //20180416
				//jqd_time_is_test_ex[jqdno][i] = 0;
				//jqd_test_mode_ex[jqdno] |=(0x1 << i);   

					//if (isdo)

				
				blade_group_active_jqd[i] |= (0x1 << jqdno);
				#ifdef JQD_ISR_TIMER_50US
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup_us;
				#else
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup_2;
				#endif				
				#endif

				isdo =1;
				//return 1;
			}
			#endif	
		}
	}
	else 
	{
		if((jqd_status_last[jqdno] & (0x1 << i))||(jqd_doitmust)||(redo)) 
		{
			#ifdef JQD_ISR_TIMER_CNT_50US
			{
				jqd_status_last[jqdno] &=~(0x01<<i);
				blade_group_delay_time[i][jqdno] = JQD_do_delay[st?1:0]/50;
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup_us;
				blade_group_active_jqd[i] |= (0x1 << jqdno);	
				isdo =1;
			}
			#else
			if (arch_Jacquard_Setup(jqdno, i, 0,1,3)==0)
			{
				#ifdef LX_JQD_MASK_ENABLE
				jqd_status_last[jqdno] &=~(0x01<<i);
				//jqd_keep_time_test_ex[jqdno][i] = jqd_keep_time_setup; //20180416
				//jqd_time_is_test_ex[jqdno][i] = 0;
				//jqd_test_mode_ex[jqdno] |=(0x1 << i); 
				blade_group_active_jqd[i] |= (0x1 << jqdno);
				#ifdef JQD_ISR_TIMER_50US
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup_us;
				#else
				blade_group_keep_time[i][jqdno] =jqd_keep_time_setup;
				#endif
				#endif
				
				isdo =1;
				//return 2;
			}
			#endif
			
		}
	}

	if (isdo)
	{	
		return 1;	
	}
	else	
		return 0;
	
}


void Exec_check_and_do_next_blade(unsigned int jqdno)
{
	if (jqdno>=sys_max_jqd_cnt)
		return;
	if ((jqd_blade_next_do[jqdno][0] )&&(jqd_blade_next_do[jqdno][2]<MAX_BLADE))
	{
		unsigned char nextbladno;
		unsigned int st;
		unsigned int bmask;
		int ret;
		donext:
		if((!jqd_blade_next_do[jqdno][0]))//||(jqd_blade_next_do[jqdno][2]>=MAX_BLADE))
		{
			jqd_blade_next_do[jqdno][2]=0xff;
			return;
		}
		nextbladno = jqd_blade_next_do[jqdno][2];	
		
		bmask= jqd_blade_next_do[jqdno][0] & ((unsigned short)0x1 << nextbladno);
		jqd_blade_next_do[jqdno][0] &=~((unsigned short)0x01<<nextbladno);
		if (nextbladno>=MAX_BLADE)	
		{
			goto yesdonext;
		}
		if(bmask)
		{
			st = jqd_blade_next_do[jqdno][1] & ((unsigned short)0x1 << nextbladno); 
			//ret = arch_Jacquard_Setup(jqdno, nextbladno, st,1,1);
			ret = Exec_Jqd_blade_next(jqdno,nextbladno,st,0);
			if (ret)
			{
				#ifndef JQD_ISR_TIMER_CNT_50US
					#ifdef JQD_ISR_TIMER_ONE
					jqd_last_blade[jqdno] = nextbladno;	
					jqd_last_blade_st[jqdno] = st;
					//jqd_isr_isdo_EX_cct[jqdno]=0;
					arch_start_jqd_timer(jqdno,jqd_keep_time_setup_us,0);
					#endif
				#endif
			}
			else
			{
			goto yesdonext;
			}
				
		}
		else
		{
			yesdonext:
			jqd_blade_next_do[jqdno][2]++;
			if(jqd_blade_next_do[jqdno][2]>=MAX_BLADE)/*超出从0开始*/
				jqd_blade_next_do[jqdno][2] =0;	
			goto donext;
		}

	}
 	return;

}
#if 0

void arch_jqd_first_do()
{
	int i;
	unsigned short blad_mask ;
	for(i=0;i<4;i++)
	{
		if((jqd_isr_isdo >> i ) & 0x01)
		{
			continue;
		}
		else
		{
			if((needresetjqd>>i) & 0x01)
			{
				arch_8844_Reset_one(i);
				needresetjqd &=~(0x01<<i);
			}
			blad_mask = ~(0xFFFF<<(jqd_max_blade));
			if((jqd_isfirst_do[i] & blad_mask)!=blad_mask)
			{
				Exec_check_and_do_next_blade(i);	
			}
			else
				continue;
		}
		
	}
	
}

#endif

#ifdef JQD_NEXT_DO_FIFO
extern unsigned char jqd_timer_isbusy[MAX_JACQUARD];

void arch_do_next_jqd()
{
	int jqdno=0;
	
	JQD_BLADE_TYPE *nextjqd=NULL;
	extern JQD_BLADE_TYPE *jqd_operate_fifo_pop(unsigned whichjqd);
	for (jqdno=0;jqdno<sys_max_jqd_cnt;jqdno++)
	{
		if (!jqd_timer_isbusy[jqdno])
		{
			#ifdef FILTER_1ST_2ND_JQD_OVERLOAD
			if((needresetjqd >> jqdno )& 0x01)
			{
				arch_8844_Reset_one(jqdno);
				needresetjqd &=~((unsigned short)0x01<<jqdno);
			}
			#endif
			nextjqd =NULL;
			nextjqd = jqd_operate_fifo_pop(jqdno);		
			if(nextjqd== NULL)
			continue;
			if (nextjqd->jqd_no!=jqdno)
			continue;		
			{
				unsigned char is_first=0;	
				arch_Jacquard_Setup(jqdno, nextjqd->bladeno, nextjqd->do_st,1,8);
				jqd_last_blade[jqdno] =nextjqd->bladeno;
				jqd_last_blade_st[jqdno] = nextjqd->do_st;
				jqd_timer_isbusy[jqdno] =1;
				//jqd_isr_isdo_EX_cct[jqdno]=0;
				#ifdef FILTER_1ST_2ND_JQD_OVERLOAD
				{
					unsigned char jqdno_do_cnt;
					jqdno_do_cnt =jqd_isfirst_do[jqdno][jqd_last_blade[jqdno]];
					if(jqdno_do_cnt<=2)  /*小于3次，就要加上去*/
					{
						jqdno_do_cnt++;
						//jqd_isfirst_do[jqdno]&=~(0x000fL<<(jqd_last_blade[jqdno]<<2));
						//jqd_isfirst_do[jqdno]|=((unsigned long)jqdno_do_cnt<<(jqd_last_blade[jqdno]<<2));
						if(jqdno_do_cnt<=2)
						{
							is_first =1;
						}
						jqd_isfirst_do[jqdno][jqd_last_blade[jqdno]] = jqdno_do_cnt;
					}
				}
				#endif
				arch_start_jqd_timer(jqdno,jqd_keep_time_setup_us,is_first);
			}
		}
	}
}


void Exec_Jacquard(unsigned int jqdno, unsigned int sts)   /*6.3*/
{
	unsigned int i;
	unsigned int maxblade_tmp = jqd_max_blade;
	unsigned short sts_blade;
	unsigned short mask_blade;/*0--动作，1--不动作*/
	unsigned char doit=0;

#ifdef LX_JQD_MASK_ENABLE
	mask_blade = (sts>>16)& 0xFFFF;
#else
	mask_blade = 0;
#endif
	sts_blade = sts & 0xFFFF;

	if(jqdno >=sys_max_jqd_cnt/*8*/) return;

	jqd_safe_time = CLR_JQD_DELAY_TIME;	
	jqd_status_last[jqdno] = jqd_status[jqdno] ;
	for(i = 0; i <( maxblade_tmp); i ++) 
	{
		if (((mask_blade>>i) & 0x1)==0)
		{
			if(sts_blade & ((unsigned short)0x1 << i))
			{
				jqd_status[jqdno] |=((unsigned short)0x01<<i);
			}
			else
			{
				jqd_status[jqdno] &=~((unsigned short)0x01<<i);
			}				
			{
				int ret;
				int doitmust=0;
				if((sts_blade & ((unsigned short)0x1 << i)) &&(!(jqd_status_last[jqdno] & ((unsigned short)0x1 << i))))
				{
					doitmust=2;
				}
				else
				if(!(sts_blade & ((unsigned short)0x1 << i)) &&(jqd_status_last[jqdno] & ((unsigned short)0x1 << i)))
				{
					doitmust=1;
				}
				if (doitmust)
				{
					ret = jqd_operate_fifo_push(jqdno, i, doitmust-1);
					if (ret<0)
					{
						alert_push(0x75,(jqdno<<8)|i);
					}
				}
			}
		}
		else
			continue;
	}	
	
}

#endif

#ifdef JQD_ISR_TIMER_CNT_50US

void Exec_Jacquard(unsigned int jqdno, unsigned int sts)   /*6.3*/
{
	unsigned int i;
	unsigned int maxblade_tmp = jqd_max_blade;
	unsigned short sts_blade;
	unsigned short mask_blade;/*0--动作，1--不动作*/
	unsigned char doit=0;

#ifdef LX_JQD_MASK_ENABLE
	mask_blade = (sts>>16)& 0xFFFF;
#else
	mask_blade = 0;
#endif
	sts_blade = sts & 0xFFFF;

	if(jqdno >=sys_max_jqd_cnt/*8*/) return;

	jqd_blade_next_do[jqdno][3] =1; /*50us 中断不能打进来*/ 

	if(jqd_blade_next_do[jqdno][0])  /*说明还有队列没做完*/
		doit =1;
		
	jqd_safe_time = CLR_JQD_DELAY_TIME;		
	//jqd_blade_next_do[jqdno][0] = 0;
	//jqd_blade_next_do[jqdno][1] = sts_blade;  
	//jqd_blade_next_do[jqdno][2] = 0xff;	
	
	//jqd_status_last[jqdno] = jqd_status[jqdno] ;
	for(i = 0; i <( maxblade_tmp); i ++) 
	{
		if (((mask_blade>>i) & 0x1)==0)
		{
			if(sts_blade & ((unsigned short)0x1 << i))
			{
				jqd_status[jqdno] |=((unsigned short)0x01<<i);
				jqd_blade_next_do[jqdno][1] |=((unsigned short)0x01<<i);
			}
			else
			{
				jqd_status[jqdno] &=~((unsigned short)0x01<<i);
				jqd_blade_next_do[jqdno][1] &=~((unsigned short)0x01<<i);
			}
			
			if(doit)
			{
				jqd_blade_next_do[jqdno][0] |= ((unsigned short)0x01<<i);/*第几刀需要动*/

				if(jqd_blade_next_do[jqdno][2]==0xff)
					jqd_blade_next_do[jqdno][2] = i; /*下一刀要做的*/
				continue;
			}
			else
			{
				if(Exec_Jqd_blade_next(jqdno,i,sts_blade & ((unsigned short)0x1 << i),0))
				{
					doit =1;						
				}	
			}
		}
		else
			continue;
	}	

	jqd_blade_next_do[jqdno][3]=0;
	
}

#endif

#else
void Exec_Jacquard(unsigned int jqdno, unsigned int sts)   /*6.3*/
{
	unsigned int i;
	unsigned int maxblade_tmp = jqd_max_blade;
	unsigned short sts_blade;
	unsigned short mask_blade;

#ifdef LX_JQD_MASK_ENABLE
	mask_blade = (sts>>16)& 0xFFFF;
#else
	mask_blade = 0;
#endif
	sts_blade = sts & 0xFFFF;


	if(jqdno >=sys_max_jqd_cnt/*8*/) return;

	#ifdef DO_XZQ_ALL_2STEP
	if (1){
		blade_2step_keep_time[jqdno][XZQ_KEEP_TIME_INDEX] = XZQ_SECOND_STEP_TIME_DEFAULT;
		blade_2step_keep_time[jqdno][XZQ_STS_INDEX] = sts;
		jqd_status2[jqdno]=jqd_status[jqdno] ;
		maxblade_tmp = jqd_max_blade>>1;
		jqd_2step_is_do |= 0x1 << (jqdno); 
		
		}
	#endif	
		
		jqd_safe_time = CLR_JQD_DELAY_TIME;
		for(i = 0; i <( maxblade_tmp); i ++) 
		{
			if (((mask_blade>>i) & 0x1)==0)
			{
				
				if(sts_blade & ((unsigned short)0x1 << i)) 
				{
						if(!(jqd_status[jqdno] & ((unsigned short)0x1 << i))) 
						{
							if (arch_Jacquard_Setup(jqdno, i, 1,1)==0)
							{
								#ifdef LX_JQD_MASK_ENABLE
								jqd_status[jqdno] |=(0x01<<i);
								jqd_keep_time_test_ex[jqdno][i] = jqd_keep_time_setup_2;//jqd_keep_time_setup; //20180416
								jqd_time_is_test_ex[jqdno][i] = 0;
								jqd_test_mode_ex[jqdno] |=((unsigned short)0x1 << i);   								
								#endif
							}							
#ifdef LOGOUT_ENABLE
						if(logout_enable & LOGOUT_XZQ_MASK)
							Message_Send_Log_Dct(LOG_OUT_XZQ, jqdno, i, 1);
#endif					
						continue;
						}
				}
				else 
				{
					if(jqd_status[jqdno] & ((unsigned short)0x1 << i)) 
					{
						if (arch_Jacquard_Setup(jqdno, i, 0,1)==0)
						{
							#ifdef LX_JQD_MASK_ENABLE
							jqd_status[jqdno] &=~((unsigned short)0x01<<i);
							jqd_keep_time_test_ex[jqdno][i] = jqd_keep_time_setup; //20180416
							jqd_time_is_test_ex[jqdno][i] = 0;
							jqd_test_mode_ex[jqdno] |=((unsigned short)0x1 << i); 							
							#endif
						}
#ifdef LOGOUT_ENABLE
						if(logout_enable & LOGOUT_XZQ_MASK)
							Message_Send_Log_Dct(LOG_OUT_XZQ, jqdno, i, 0);
#endif					
						continue;
					}
				}
				//arch_Jacquard_ClearBlade(jqdno, i);
				//alert_push(JQD_OPERATE_ERROR+1,(jqdno <<8)|i);
				
			}
			else
				continue;
		}
    	//mask_blade ~=mask_blade;
    	
    	
    	#ifndef LX_JQD_MASK_ENABLE
	jqd_status[jqdno] =  sts_blade ;	
	jqd_keep_time_test[jqdno] = jqd_keep_time_setup;
	jqd_test_mode |= ((unsigned short)0x1 << (jqdno)); 	
	#endif
		
	
	
}


#endif


#ifdef JAQ_WORK_MODE
//#define TEST_JQD_DATA_DW

#ifdef TEST_JQD_DATA_DW
int last_start_blad_set=0;
int last_needle_count=0;
int last_blad_id=0;

#endif

#ifdef CHECK_LIFT_IS_WORK
extern void check_Lift_step_iswork_(unsigned char);
#endif




void Needle_isr(unsigned int Needle_dat, unsigned int jqd_mask)  /*6.2*/
{
	unsigned int i;
	volatile unsigned int pblade_clr;
	volatile unsigned int *pblade;
	//volatile unsigned int *pkeep_time;
	unsigned int active = jqd_mask;
	unsigned int mask;
	char isdo=0;
	char isonoff=0;
	char opentime=0;
	

	jqd_safe_time = CLR_JQD_DELAY_TIME;

#ifdef NEEDLE_POS_SUPPORT
	if(needle_pos_mask & 0x1) {
		needle_pos ++;
	}
#endif
	for(i = 0; i < sys_max_jqd_cnt/*8*/; i ++) {
		mask = 0x1 << i;
		if(active & mask/*(0x1 << i)*/) {

			isdo = 0;
			pblade = &jqd_blade[i];//逻辑刀片
			//pkeep_time = &blade_group_keep_time[*pblade][i];
			//blade_group_active_jqd[*pblade] |= mask/*(0x1 << i)*/;//??
			pblade_clr = jqd_clear_blade[*pblade][i];   //实际物理刀片
			mask = 0x1 << pblade_clr;
			//*pkeep_time = jqd_keep_time_setup;
			//blade_group_keep_time[*pblade][i] = jqd_keep_time_setup;
			#ifdef JQD_XZQ_NEEDLE_REDO_MODE
			JQD_redo_cnt_add(i);
			#endif			
			
			if(Needle_dat & (0x1 << i)) {
				if(!(jqd_status[i] & mask/*(0x1 << pblade_clr)*/)) 
				{

					#ifdef JQD_NEXT_DO_FIFO
						int ret;
						ret = jqd_operate_fifo_push(i, pblade_clr, 1);
						if (ret<0)
						{
							alert_push(0x76,(i<<8)|pblade_clr);
						}
					
					#else
					#ifdef JQD_XZQ_NEEDLE_REDO_MODE
					jqd_blade_redo_cnt[i][pblade_clr] =0;
					#endif	
					#ifndef JQD_ISR_TIMER_CNT_50US
					arch_Jacquard_Setup(i, pblade_clr, 1,1,4);
					#endif

					#endif
					jqd_status[i] |= mask/*(0x1 << pblade_clr)*/;
					isdo =1;
					isonoff =1;
					opentime =1;
					
				}
			}
			else {
				if(jqd_status[i] & mask/*(0x1 << pblade_clr)*/) 
				{					
					#ifdef JQD_NEXT_DO_FIFO
						int ret;
						ret = jqd_operate_fifo_push(i, pblade_clr, 0);
						if (ret<0)
						{
							alert_push(0x76,(i<<8)|pblade_clr);
						}					
					#else
					{
					#ifdef JQD_XZQ_NEEDLE_REDO_MODE
					jqd_blade_redo_cnt[i][pblade_clr] =0;
					#endif		
					#ifndef JQD_ISR_TIMER_CNT_50US   //只记录标记
					arch_Jacquard_Setup(i, pblade_clr, 0,1,5);
					#endif
					}
					#endif
					jqd_status[i] &= ~mask/*(0x1 << pblade_clr)*/;
					isdo =1;
					isonoff =0;
					opentime=1;
					
				
				}
				#ifdef CHECK_LIFT_IS_WORK
					if ((check_start) && (yarn_is_do))
					{
						if (jqd_direction?((i==0)||(i==2)):((i==1)||(i==3)))
						{							
							if ((check_start>>i)&0x01)
							{
								check_Lift_step_iswork_((i<2)?0:1);
								check_start &=~(0x01<<i);
							}								
						}
						else
						{
							check_start &=~(0x01<<i);
						}
					}

					#endif
			}

			#ifdef  JQD_ISR_TIMER_CNT_50US
			if (isdo)
			{	
				
				blade_group_delay_time[pblade_clr][i]=JQD_do_delay[isonoff?1:0]/50;
				blade_group_keep_time[pblade_clr][i] =jqd_keep_time_setup_us;	
				blade_group_active_jqd[pblade_clr] |= (0x1 << i);
			}
			#endif
			
			#ifdef JQD_XZQ_NEEDLE_REDO_MODE

			JQD_redo_cnt_check_redo(i);


			#endif


			
#if 0
			if(jqd_direction) {
				(*pblade) ++;
				if((*pblade) == jqd_max_blade) {
					(*pblade) = 0;
				}
			}
			else {
				if((*pblade) == 0) {
					(*pblade) += jqd_max_blade - 1;
				}
				else {
					(*pblade) --;
				}
			}
#else
#ifdef TEST_JQD_DATA_DW
			if (((arch_Get_ID()==0) &&(i==3))||((arch_Get_ID()==1)&&(i==2)))	
			{
				last_needle_count++;
				last_blad_id = (*pblade) ;
				
			}
#endif

			if(jqd_dir_back?(!jqd_direction):jqd_direction) {
				//(*pblade) ++;	//2013.08.27 选针进距改为支持2针
				(*pblade) += needle_work_step;
				if((*pblade) == jqd_max_blade) {
					(*pblade) = 0;
				}
			}
			else {
				if((*pblade) == 0) {
					//(*pblade) += jqd_max_blade - 1;
					(*pblade) += jqd_max_blade - needle_work_step;
				}
				else {
					//(*pblade) --;
					(*pblade) -= needle_work_step;
				}
			}

			#ifdef CHECK_LIFT_STEP_IS_STOP   /* 2017 11 27 by hlc 检查推针电机是否完成*/
			if ((whichJQD!=0xff)&&(lift_step_after_jqd_arg))
			{
				if (i==whichJQD)
				{
					if (jqd_cmd_cnt)
					{
						if(!(--jqd_cmd_cnt))
						{
							whichJQD=0xff;
							check_lift_step_isstop();
						}
					}
				}
			}	

			#endif
			
#endif
		}
	}
}

#else

void Needle_isr(unsigned int Needle_dat, int mode)
{
	unsigned int i;
	volatile unsigned int *pblade_clr;
	volatile unsigned int *pblade;
	volatile unsigned int *pkeep_time;
	unsigned int active;

	jqd_safe_time = CLR_JQD_DELAY_TIME;
	if(mode == 1) {
		active = jqd_active_front;
		pblade = &jqd_blade_front;
		pkeep_time = &blade_group_keep_time_F[*pblade];
	}
	else if(mode == 2) {
#ifdef NEEDLE_POS_SUPPORT
		needle_pos ++;
#endif
		active = jqd_active_back;
		pblade = &jqd_blade_back;
		pkeep_time = &blade_group_keep_time_B[*pblade];
	}
	else {
#ifdef NEEDLE_POS_SUPPORT
		needle_pos ++;
#endif
		active = jqd_active;
		pblade = &jqd_blade;
		pkeep_time = &blade_group_keep_time[*pblade][0];
	}

	pblade_clr = &jqd_clear_blade[*pblade][0];

	*pkeep_time = jqd_keep_time_setup;

	for(i = 0; i < sys_max_jqd_cnt/*8*/; i ++) {
		if(active & (0x1 << i)) {
			if(Needle_dat & (0x1 << i)) {
				if(!(jqd_status[i] & (0x1 << pblade_clr[i]))) {
					//
					arch_Jacquard_Setup(i, pblade_clr[i], 1,0);
					jqd_status[i] |= (0x1 << pblade_clr[i]);
				}
			}
			else {
				if(jqd_status[i] & (0x1 << pblade_clr[i])) {
					//
					arch_Jacquard_Setup(i, pblade_clr[i], 0,0);
					jqd_status[i] &= ~(0x1 << pblade_clr[i]);
				}
			}
		}
	}

	//*pkeep_time = jqd_keep_time_setup;
	if(jqd_direction) {
		(*pblade) ++;
		if((*pblade) == jqd_max_blade) {
			(*pblade) = 0;
		}
	}
	else {
		if((*pblade) == 0) {
			(*pblade) += jqd_max_blade - 1;
		}
		else {
			(*pblade) --;
		}
	}
}
#endif

unsigned int Jacquard_Get_Status(int jqdno)
{
#ifdef JAQ_WORKMODE_8_16	
	if (jaq_work_mode_8_16)
	{
		return jqd_status[jqdno & 0xF];
	}
	else
	{
		return jqd_status[jqdno & 0x7];
	}
#else
	return jqd_status[jqdno & 0x7];
#endif

	
}

/************************************************
 * End Jacquard Ctrl
 ************************************************/



void tanzhen_alarm_delay_Timer_2ms()
{
#ifdef STM32_BOARD
	unsigned int i;
	for (i=0;i<2;i++)
	{
		if(tanzhen_delay[i] && tanzhen_alarm[i])
		{
			tanzhen_alarm_delay_count[i] ++;		
		}
	}
#endif
}

void EMF_autoclear_Timer_2ms()			//安位优化
{
	unsigned int i=0;
	//unsigned int j=0;
	
	//i = 12;
	if (emf_status_isdone)
	{
		do{
			if (emf_status_isdone & (0x01<<(i)))
			{				
				if(emf_work_mode & (0x1 << i)) {
					continue;
				}
				if(emf_keep_time[i] == 0) {
					continue;
				}
				emf_keep_time[i] --;
				if(emf_keep_time[i] == 0) 
				{
					//emf_keep_time[i] = 10; 			
					if (i<12)	//action
						arch_ACTEMF_Clear(i);
					else
						arch_YARNEMF_Clear(i - 12);
					emf_status_isdone &=~(0x01<<i);
				}
				
				//if ((emf_status_isdone>>i)==0)
				//break;	
			}
			//i++;	
		}	while(emf_status_isdone>>(++i));
		
	}
}

void JQD_autoclear_Timer_2ms()				//可以优化
{
	return ;
#if 0
	if(jqd_safe_time) {
		jqd_safe_time --;
		if(jqd_safe_time == 0) {
			arch_Jacquard_AllClear();
		}
	}
	else {
		jqd_safe_time = 3;
	}
	#endif
}

void JQD_Init_delay_Timer_2ms()
{
 #ifdef JQD_INIT_DELAY_ENABLE
 	{
		if (Jacquard_Init_delay_ms <= 3) {
			
			Jacquard_Init_delay_ms += 2 ;
			if (Jacquard_Init_delay_ms > 3)
			{
				Jacquard_Init();
			}		
		}
 	}	
 #endif
}

void time_2ms_prog()
{

//by wf 20131105; tanzhen delay counter;

	

	tanzhen_alarm_delay_Timer_2ms();
	EMF_autoclear_Timer_2ms();
	
	JQD_autoclear_Timer_2ms();

	//alert_time_poll();
	//WatchDog_Kick();

	StepMotor_timer();
	JQD_Init_delay_Timer_2ms();
	
	arch_shock_timer(0);
	arch_check_Overload_longTime(0);
	#ifdef DEBUG_STEP_OPERATE_GONO_ERROR
	check_gono_cnt();
	#endif
	Time_2ms_check_binding();
	CheckIn_Timer();
	Motor_bind_timer_isr();

	//arch_check_dct_shock_enable();
}

void JQD_clearblade_timer_250us()
{

	register unsigned int i,j;
	j = jqd_max_blade;		
	do {
		j --;
		i = sys_max_jqd_cnt;
		#if 0
		if (JQD_halfwork_mode)
		{
			i = MAX_JACQUARD/*8*/;
		}
		else
		{
			i = MAX_JACQUARD >>1/*4*/;		
		}
		#endif
		do {
			i --;
			if(blade_group_keep_time[j][i] == 0)
				continue;
			blade_group_keep_time[j][i] --;
			if(blade_group_keep_time[j][i] > 0)
				continue;

			if(blade_group_active_jqd[j] & (0x1 << i)) {
				blade_group_active_jqd[j] &= ~(0x1 << i);
				arch_Jacquard_ClearBlade(i, j /*jqd_clear_blade[j][i]*/);
				Exec_check_and_do_next_blade(i);
			}
		} while(i > 0);
		////blade_group_active_jqd[j] = 0;
	} while(j > 0);

}

unsigned int Get_blade_group_keep_time(unsigned char d,unsigned char x)
{
return blade_group_keep_time[d][x];
}





void time_250us_prog()
{
	register unsigned int i,j;
	
//#ifdef YUEFA_TEST
	if(check_dir_still_zero)
	{
		Check_DirStatus_new();
	}
//#endif
	// for needle isr
#ifndef JQD_ISR_TIMER_CNT_50US

#ifndef JQD_ISR_TIMER_50US

#ifdef JAQ_WORK_MODE
	JQD_clearblade_timer_250us();
#else
	
#endif
#endif
#endif

#ifndef JQD_ISR_TIMER_ONE

	if(jqd_test_mode) {
		// for Exec Jaquard
		j = sys_max_jqd_cnt/*8*/;
		do {
			j --;
			if(jqd_keep_time_test[j] == 0)
				continue;
			jqd_keep_time_test[j] --;
			if(jqd_keep_time_test[j] != 0)
				continue;
			//arch_Jacquard_Clear(j);
		
			#ifndef DO_XZQ_ALL_2STEP
				arch_Jacquard_Clear(j);
			#else
				arch_Jacquard_Clear_Step(j,0);
			#endif
			jqd_test_mode &= ~(0x1 << j);
		} while(j > 0);
	}
#endif

#if 1

	j = sys_max_jqd_cnt;
	do
	{
		j--;
		if(jqd_test_mode_ex[j]) {

			i = sys_max_blade_phi;//jqd_max_blade;
			do
			{
				i--;
				if (jqd_test_mode_ex[j] & (0x01<<i))
				{
					if (jqd_keep_time_test_ex[j][i] == 0)
						continue;
					jqd_keep_time_test_ex[j][i] --;					
					if (jqd_keep_time_test_ex[j][i] !=0)
					{
					#ifdef LX_JQD_MASK_ENABLE
						if (jqd_time_is_test_ex[j][i])					
					#endif
						//PWM 形式输出,相当于降压处理
						arch_Jacquard_PWMBlade(j,i);
						continue;
					}
					
					arch_Jacquard_ClearBlade(j,i);
					jqd_test_mode_ex[j] &= ~(0x1 << i);
				}
				
			}
			while(i);

			}
		else
			continue;
	}
	while(j);

#endif

#ifndef JQD_ISR_TIMER_ONE


	#ifdef DO_XZQ_ALL_2STEP
	if(jqd_test_mode2) {
		// for Exec Jaquard
		j = sys_max_jqd_cnt/*8*/;
		do {
			j --;
			if(jqd_keep_time_test2[j] == 0)
				continue;
			jqd_keep_time_test2[j] --;
			if(jqd_keep_time_test2[j] != 0)
				continue;
			arch_Jacquard_Clear_Step(j,1);
			jqd_test_mode2 &= ~(0x1 << j);
		} while(j > 0);
	}

	if (jqd_2step_is_do)
	{
		j = sys_max_jqd_cnt/*8*/;
		do {
			j --;
			if(blade_2step_keep_time[j][XZQ_KEEP_TIME_INDEX] == 0)
			{
				jqd_2step_is_do &= ~(0x1 << j);
				continue;
			}
			blade_2step_keep_time[j][XZQ_KEEP_TIME_INDEX] --;
			if(blade_2step_keep_time[j][XZQ_KEEP_TIME_INDEX] != 0)
				continue;
			{
				int i;
				int sts = blade_2step_keep_time[j][XZQ_STS_INDEX];
				#ifdef LX_JQD_MASK_ENABLE
				unsigned char sts_mask = (blade_2step_keep_time[j][XZQ_STS_INDEX]>>8)&0xff;
				#else
				unsigned char sts_mask =0;
				#endif
				jqd_safe_time = CLR_JQD_DELAY_TIME;
				for(i = jqd_max_blade>>1; i <( jqd_max_blade); i ++) 
				{
					if (((sts_mask>>i) & 0x1)==0)
					{
						if(sts & (0x1 << i)) {
							if(!(jqd_status2[j] & (0x1 << i))) {
								arch_Jacquard_Setup(j, i, 1,1);
								#ifdef LX_JQD_MASK_ENABLE
								jqd_status[j] |=(0x01<<i);
								jqd_keep_time_test_ex[j][i] = jqd_keep_time_setup_2;
								jqd_time_is_test_ex[j][i] = 0;
								jqd_test_mode_ex[j] |=(0x1 << i);   	
								#endif
								
								continue;
							}
						}
						else {
								if(jqd_status2[j] & (0x1 << i)) {
									arch_Jacquard_Setup(j, i, 0,1);
									#ifdef LX_JQD_MASK_ENABLE
									jqd_status[j] &=~(0x01<<i);
									jqd_keep_time_test_ex[j][i] = jqd_keep_time_setup;
									jqd_time_is_test_ex[j][i] = 0;
									jqd_test_mode_ex[j] |=(0x1 << i); 
									#endif
								continue;
								}
							}
						//arch_Jacquard_ClearBlade(j, i);
					}
					else
						continue;
				}
				//jqd_status[j] = sts;	
				//jqd_status[j] =(jqd_status[j] & (0x0F)) | (sts & (0xF0));	
				#ifndef LX_JQD_MASK_ENABLE
				jqd_keep_time_test2[j] = jqd_keep_time_setup;
				jqd_test_mode2 |= 0x1 << (j);   
				#endif
			}		
			jqd_2step_is_do &= ~(0x1 << j);
		} while(j > 0);
	}

	#endif
	
#ifdef EMF_TIME2_SUPPORT
	// by xhl 2012/06/18
	for(i = 24; i < 40; i ++) {
		j = i - 24;
		if(((Yemf_time2_flag & (0x1 << j)) != 0) &&
		   ((Yemf_time2_flag2 & (0x1 << j)) == 0)) {
			if(emf_keep_time[i] > 0) {
				emf_keep_time[i] --;
			}
		}
	}
#endif

#endif

	WatchDog_Kick();
}



/************************************************
 * Start EMF Ctrl
 ************************************************/
 #define CHECK_DC24_YARN 1
 #define CHECK_DC24_ACT 2
 extern unsigned char arch_Get_Test_act_id_(unsigned int whichc,unsigned int no1,unsigned int no2);
void Exec_EMF(unsigned int emfno, unsigned int onoff,unsigned char istest,unsigned short times_ms)
{
	unsigned int offset, idx;
	unsigned int mask;

	//emfno 的范围(0-19)，前12个为动作三角，后8个为纱嘴

	emf_safe_time = CLR_EMF_DELAY_TIME;

	if(emfno >= 20) return ;

	if(emfno < 12) {	// act
		idx = emfno;
		//arch_ACTEMF_Setup(emfno, onoff);
		offset = emfno / 6;
		mask = 0x1 << (emfno % 6);

		if(onoff)
		{
			emf_status[offset] |= mask;
			emf_keep_time[emfno] = emf_on_keep_time_setup;
			
		}
		else 
		{
			emf_status[offset] &= ~mask;
			emf_keep_time[emfno] = emf_off_keep_time_setup;			
		}

		if(times_ms>>1)
		{
			emf_keep_time[emfno] =times_ms>>1;
		}

		if (istest)
		{
			emf_keep_time[emfno] =EMF_ACT_TIME_TEST;
			if (emf_work_mode & (0x1 << emfno))
			{
				DC24_PN_data_Arry[arch_Get_Test_act_id_(CHECK_DC24_ACT,onoff?1:0,idx)]=999;
				return;
			}
		}

		arch_shock_ctr(0);	
		
		arch_ACTEMF_Setup(idx, onoff,1);
		
		
	}
	else
	{	// yarn
		idx = emfno - 12;
		//arch_YARNEMF_Setup(idx, onoff);
		offset = EMF_INDEX_YARN;
		mask = 0x1 << idx;

		#ifdef CHECK_LIFT_IS_WORK
			yarn_is_do++;
			
		#endif

		#ifdef E480_BOARD_V10

		if (Yarn_use_Step)
		{
			{
				arch_YARNEMF_Setup(idx, onoff,0);
				return;  //纱嘴保持原来的位置不动
			}				
		}
		
		#endif
	
		if(onoff) 
		{
			emf_status[offset] |= mask;
			emf_keep_time[emfno] = emf_on_keep_time_setup;
			if(emfYARN_on_keep_time_setup ) 
			{
				emf_keep_time[emfno] = emfYARN_on_keep_time_setup;
			}
		}	
		else 
		{
		
			emf_status[offset] &= ~mask;
			emf_keep_time[emfno] = emf_off_keep_time_setup;
			if(emfYARN_off_keep_time_setup ) 
			{
				emf_keep_time[emfno] = emfYARN_off_keep_time_setup;
			}
		
		}

		if(times_ms>>1)
		{
			emf_keep_time[emfno] =times_ms>>1;
		}
		if (istest)
		{
			emf_keep_time[emfno] =EMF_TIME_TEST;
			if (emf_work_mode & (0x1 << emfno))
			{
				DC24_PN_data_Arry[arch_Get_Test_act_id_(CHECK_DC24_YARN,onoff?1:0,idx)]=999;
				return;
			}
		}
		

#ifdef EMF_TIME2_SUPPORT
		Yemf_time2_flag2 |= mask;
		Yemf_time2_flag |= mask;
#endif
		arch_YARNEMF_Setup(idx, onoff,1);


	}
	emf_status_isdone |=(0x01<<emfno);
	
}


void exec_EMF_YARN_(unsigned short yarnst)
{
	int i;
	unsigned short yh=emf_status[2]^yarnst;
	for (i=0;i<8;i++)
	{	
		if ((yh>>i)&0x01)
			Exec_EMF(i+12,((emf_status[2]>>i)&0x01)?0:1,0,0);
	}
}



unsigned int EMF_Get_ACT_Status(int grp)
{
	return emf_status[grp & 0x1];
}

unsigned int EMF_Get_YARN_Status()
{
	return emf_status[2];
}


unsigned int Get_ZeroWorkStatus_StepType_sig(unsigned char st,unsigned int idself)  // whichinput =0 表示0位，1表示工作位
{
	unsigned int ret=0xff;
	if(Get_StepMotor_isveryenable_IDself(idself,st))
		return ret;

	if (arch_StepMotor_Zero_mid_layer(StepMotor_get_zeroID_with_IDself(idself,st),StepMotor_get_zeroID_cfg_with_IDself(idself,st)))
		ret &= ~(0x01);
	if (arch_StepMotor_Work_mid_layer(StepMotor_get_workID_with_IDself(idself,st),StepMotor_get_workID_cfg_with_IDself(idself,st)))
		ret &= ~(0x02);

	return ret;
								

}


unsigned int Get_Status_StepType_sig(unsigned char whichinput,unsigned char st,unsigned int idself)  // whichinput =0 表示0位，1表示工作位
{
	extern short StepMotor_Get_input_error(unsigned int stepno);
	if(Get_StepMotor_isveryenable_IDself(idself,st))
	{
		if (whichinput>=3)
			return 0;
		return 0xFF;
	}
	switch (whichinput)
	{
	case 0:
		return !arch_StepMotor_Zero_mid_layer(StepMotor_get_zeroID_with_IDself(idself,st),StepMotor_get_zeroID_cfg_with_IDself(idself,st));		
	case 1:
		return !arch_StepMotor_Work_mid_layer(StepMotor_get_workID_with_IDself(idself,st),StepMotor_get_workID_cfg_with_IDself(idself,st));		
	case 2:
		return !StepMotor_Get_Busyidx(StepMotor_get_no_with_IDself(idself,st));	
	case 3:
		return StepMotor_Get_Position(StepMotor_get_no_with_IDself(idself,st));	
	case 4:
		return StepMotor_Get_input_error(StepMotor_get_no_with_IDself(idself,st));
	default:
		break;
	}

	return 1;
}

unsigned int Get_Status_StepType(unsigned char whichinput,unsigned char st)  // whichinput =0 表示0位，1表示工作位
{	unsigned int i,ri;
	unsigned int status = 0xFFFF;
	STEP_TYPE *Step;

	switch (whichinput)
	{
		case 0:
			for (i=0;i<StepMotor_Count_MAX;i++)
			{
				Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
				
				if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=st)
					continue;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
					continue;		
				ri = Step->moto_remap_config.moto_remap_id_self;			
				if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg)) 
				{
					status &= ~(0x1 << (ri));
				}		
			}
			//return status;
			break;
		case 1:	
			for (i=0;i<StepMotor_Count_MAX;i++)
			{
				Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
				if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=st)
					continue;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
					continue;		
				ri = Step->moto_remap_config.moto_remap_id_self;	
				if(arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg)) 
				{
					status &= ~(0x1 << (ri));
				}		
			}
			//return status;
			break;
		case 2:	
			for (i=0;i<StepMotor_Count_MAX;i++)
			{
				Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
				if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=st)
					continue;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable)
					continue;		
				ri = Step->moto_remap_config.moto_remap_id_self;	
				if(Step->step_st.bit.running) 
				{
					status &= ~(0x1 << (ri));
				}		
			}
			//return status;
			break;
			
		default:
				
			break;
	}	

return status;
}


#ifdef E490_V10_BOARD

unsigned char zero_remap[]={0,1,2,3,4,5,6,7,8,9,10,11,16,17,12,13,14,15,18,19,20};

unsigned char get_zeromapid(char id)
{
	return zero_remap[id];
}

unsigned int Get_SIG_PH_Status()
{	unsigned int i;
	unsigned int status = 0xFFFFFFFF;	
	unsigned int maxinput =21;



	for (i=0;i<maxinput;i++)
	{
		if(arch_StepMotor_Zero_mid_layer(zero_remap[i],0)) 
		{
			status &= ~(0x1 << (i));
		}		
	}	

	return status;
	
}


#else

unsigned int Get_SIG_PH_Status()
{	unsigned int i;
	unsigned int status = 0xFFFF;	
	unsigned int maxinput =12;

	#ifdef E480_BOARD_V10
	maxinput = 16;
	#endif

	for (i=0;i<maxinput;i++)
	{
		if(arch_StepMotor_Zero_mid_layer(i,0)) 
		{
			status &= ~(0x1 << (i));
		}		
	}	

	return status;
	
}

#endif

unsigned int Get_ZeroStatus(unsigned char whichinput)  // whichinput =0 表示0位，1表示工作位
{	unsigned int i,ri;
	unsigned int status = 0xFFFF;
	STEP_TYPE *Step;

	switch (whichinput)
	{
		case 0:
			for (i=0;i<StepMotor_Count_MAX;i++)
			{
				Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
				ri = Step->moto_remap_config.moto_remap_id_all;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
		
				if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg)) 
				{
					status &= ~(0x1 << (ri));
				}		
			}
			
			break;
		case 1:	
			for (i=0;i<StepMotor_Count_MAX;i++)
			{
				Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
				ri = Step->moto_remap_config.moto_remap_id_all;
				if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable)
					continue;
				if(arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg)) 
				{
					status &= ~(0x1 << (ri));
				}		
			}
			//return status;
			break;
		default:
				//return status;
			break;
	}
	return status;

}


unsigned short Get_ZeroStatus_log(unsigned char whichinput)
{
	unsigned int i;
	unsigned int status = 0xFFFF;
	STEP_TYPE *Step;
	unsigned char whichbitno=0;
	extern volatile char step_log_remap[];
	
	for (i=0;i<StepMotor_Count_MAX;i++)
	{
		Step = (STEP_TYPE *)Get_STEPMOTO_add(i);
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

		if (whichinput ==0)
		{		
			if(arch_StepMotor_Zero_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_zero_input_index,
				Step->moto_remap_config.moto_attr.bit.zero_input_NC_cfg)) 
			{
				status &= ~(0x1 << (whichbitno));
			}
		}
		else
		{
			if(arch_StepMotor_Work_mid_layer(Step->moto_remap_config.moto_attr.bit.moto_work_input_index,
				Step->moto_remap_config.moto_attr.bit.work_input_NC_cfg)) 
			{
				status &= ~(0x1 << whichbitno);
			}	
		}
					
				
	}
	return status;
			
}








unsigned int arch_Get_DirStatus(void);

void Check_DirStatus()
{
	if(jqd_direction) {
		if(arch_Get_DirStatus() != 0x00) {
			#ifdef NEW_ALARM_STYLE
			alert_push(DIR_ERR_CODE_ARG(0));
			#else
			alert_push(DIR1_ERR, 0);
			#endif
		}
	}
	else {
		if(arch_Get_DirStatus() != 0x03) {
			#ifdef NEW_ALARM_STYLE
			alert_push(DIR_ERR_CODE_ARG(1));
			#else
			alert_push(DIR2_ERR, 0);
			#endif
		}
	}
}

//#ifdef YUEFA_TEST
/*static*/ volatile unsigned int check_dir_1count;
/*static*/ volatile unsigned int check_dir_2count;
void Check_DirStatus_new(void)
{
	if(arch_get_dir1())
	{
		if(check_dir_1count < 20000)
			check_dir_1count ++;
	}
	if(arch_get_dir2())
	{
		if(check_dir_2count < 20000)
			check_dir_2count ++;
	}
}


unsigned int Get_ErrorStatus(void);
void Upgrade_Start(void);
extern void arch_Power_On(void);

//#define MLOCK
#ifdef MLOCK
void UART0Send(char *BufferPtr, unsigned long Length );
int UART0Rcv(char *rcv);
unsigned char keybuf[256];
unsigned char key_count = 0;
void arch_LED_On(void);
void arch_LED_Off(void);

void Keydog_Read()
{
	unsigned char rcv;
	int len;
   
	arch_LED_Setup(0);
	arch_LED_Off();
	UART0Send("KEYDOG_READ\0", 12);
	key_count = 0;
	len = 0;
	while(1) {

		while(UART0Rcv((char*)&rcv) == 0);

		if(len == 0) {
			if(rcv == 0) {
				Message_Send(0);
				break;
			}	   	 
			len = rcv;	   
		}
		else {
			keybuf[key_count ++] = rcv;
			if(key_count >= len) {
				int i;
				unsigned short *pKey = (unsigned short *)keybuf;
								   
				arch_LED_On();
				Message_Send(len);
				len = (len + 1) >> 1;
				for(i = 0; i < len; i ++) {
					wait_us(10); 
					Message_Send(pKey[i]);
				}	   
				arch_LED_Off();

				break;
			}
		}
	}
	arch_LED_Setup(50);
}

void Keydog_Check()
{
	char rcv;
	UART0Send("KEYDOG_CHECK\0", 13);
	while(UART0Rcv(&rcv));
	Message_Send(rcv);
}

#endif
#if 0
void Work_Para_Setup(int arg1, int arg2)
{
	switch(arg1) {
	case 0x10:
		Jacquard_blade_remap(arg2,10,0);
		break;
	case 0x18:
		emfYARN_on_keep_time_setup = arg2;
		break;
	case 0x19:
		emfYARN_off_keep_time_setup = arg2;
		break;
	}
}

#endif
#ifdef STEP_TEST_IN_NDL
unsigned int StepMotor_Get_Needle(unsigned int stepno);
#endif


static unsigned int Get_Func_Type(void)
{
	unsigned int data = 0;
	
data = arch_Get_ID();

#ifdef ENCODER_SUPPORT
	data |= (0x1 << 2);
#endif


	data |= (0x1 << 3);

	return data;
}

static void tanzhen_zt_get(unsigned short rtn)
{
	extern unsigned int Get_TZ_error_st(unsigned char whichtz);
	unsigned short tzst=0;
	tzst |= Get_TZ_error_st(0)?1:0;
	tzst |= (Get_TZ_error_st(1)?1:0)<<1;
	Message_Send_4halfword(rtn,tzst,0,0);		
}

static void tanzhen_delay_set(int setmask,int isenable, int time)
{
	//int i;
	extern unsigned int Get_VER_data();
#if 0
	if (Get_VER_data()>=0x85)
	{
		int stm=0;
		stm = (setmask >> 0x01)&0x01;
		stm |= (setmask<<0x01);
		setmask =stm;
	}

#endif	
	if(setmask & 0x2) {
		if(!isenable) {			
			tanzhen_delay[0] = 0;
			//tanzhen_delay[2] = 0;
			tanzhen_active[0] = 0;
			//tanzhen_active[2] = 0;
		}
		else
		{		
			tanzhen_delay[0] =  (time-1)*50;
			//tanzhen_delay[2] = tanzhen_delay[0];
			tanzhen_active[0] = 1;
			//tanzhen_active[2] = 1;
			
		}
		tanzhen_alarm_delay_num[0] = tanzhen_delay[0];
		tanzhen_alarm_delay_count[0] = 0;
		tanzhen_alarm[0] = 0;

		//tanzhen_alarm_delay_num[2] = tanzhen_delay[2];
		//tanzhen_alarm_delay_count[2] = 0;
		//tanzhen_alarm[2] = 0;		
	}
	if(setmask & 0x1) {
		if(!isenable) {			
			tanzhen_delay[1] = 0;
			//tanzhen_delay[3] = 0;
			tanzhen_active[1] = 0;
			//tanzhen_active[3] = 0;
		}
		else 
		{
			tanzhen_delay[1] = ( (time-1)*50);
			//tanzhen_delay[3] =tanzhen_delay[1] ;
			tanzhen_active[1] = 1;
			//tanzhen_active[3] = 1;
			
		}
		tanzhen_alarm_delay_num[1] = tanzhen_delay[1];
		tanzhen_alarm_delay_count[1] = 0;
		tanzhen_alarm[1] = 0;
		//tanzhen_alarm_delay_num[3] = tanzhen_delay[3];
		//tanzhen_alarm_delay_count[3] = 0;
		//tanzhen_alarm[3] = 0;
		
		
	}

}



unsigned int Check_CMD_is_Support(NEWCMDMASSAGE  *cmd)
{
	
	if (cmd->cmd_type & 0x80)
	{
		Message_Send_4halfword(CMD_RETURN,cmd->arg1,cmd->arg2,cmd->arg3);
		return 1;
	}
	return 0;
}


#if 0
unsigned int Get_adc_data(unsigned int whichdata)
{
				extern volatile unsigned int DC24_P_CURR_A;
				extern volatile unsigned int DC24_N_CURR_A;
				extern volatile unsigned int DC24_P_CURR_V;
				extern volatile unsigned int DC24_N_CURR_V ;
				extern volatile unsigned int DC12_P_CURR_V ;
				extern volatile unsigned int Vbat_CURR_V ;
				extern volatile unsigned int Vrf_CURR_V ;
				extern volatile unsigned int TempS_CURR_V ;
				//extern volatile unsigned int TempS_CURR_v_o;
				unsigned short d =0;
				 unsigned short dt;
				switch(whichdata)
				{
				case 0:
					{    
						dt = DC24_P_CURR_A;
						
						if (dt>=1650)
						{
							d= (dt*10-16500)/55;
						}
						else
							d =0;
						//return d[1];
					}
					break;
				case 1:
					{
						dt = DC24_N_CURR_A;
						
						if (dt>=1650)
						{
							d = (dt*10-16500)/55;
						}
						else
							d =0;						
						//return d;	
					}
					break;
				case 2:
					{
						d=DC24_P_CURR_V *21/100;
						//return [1];
					}
					break;
				case 3:
					{
						d=DC24_N_CURR_V *21/100;
						//return d[2];
					}
					break;
				case 4:
					{
						d=DC12_P_CURR_V *215/1500;
						//return [1];
					}
				break;
				case 5:
					{
						d=Vbat_CURR_V * 2;
					//	return [2];
					}
					break;
				case 6:	
					{
						d=Vrf_CURR_V;
					}
				break;
				case 7:	
					{
						dt = TempS_CURR_V;						
						d= 250L -((long)(760-dt) * 100L)/25L ;	
						//unsigned int tmp_ = 250L -((long)(760-TempS_CURR_V) * 100L)/25L ;			
						
					}
					break;	
					
				default:
						break;
						
				}
				//Message_Send_4halfword(d[0],d[1],d[2],d[3]);	
	
	return d;

}

#endif
#ifdef JQD_DO_CAN_ISR
int arch_do_jqdcmd_incan_isr(unsigned short *pData)
{
	
	if ((!LocalSys_is_enable)||(pData==NULL)) return;	
	switch(*pData)
	{
		case 0x0401:
		{
			jqd_direction = *(pData+1);
		}
		break;
		case 0x0106:
		{
			Jacquard_Init();
		}
		break;
		case 0x0206:
		{
			
			unsigned char id=arch_Get_ID();
			unsigned int needledata;
			unsigned int jqdmask;
			

			if (!arch_check_power_isok(POWER_CTR_P_24))	
			{
				alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
				break;
			}
			if (!arch_check_power_isok(POWER_CTR_N_24))	
			{
				alert_push(HEAD_POWER_ISOFF,(POWER_CTR_N_24<<8)|which_part_close_power);
				break;
			}
			
			if (JQD_halfwork_mode)//8选针模式
			{
				switch (id)
				{
					case 	0:
					case	3:
							{
								needledata =*(pData+1) & 0xff ;
								jqdmask =*(pData+1) >>8 ;
							}
							break;
					case 	4:		
					case	1:
							{
								needledata =*(pData+2) & 0xff ;
								jqdmask = *(pData+2) >>8;
							}
							break;
					case 	5:		
					case	2:
							{
								needledata =*(pData+3) & 0xff ;
								jqdmask = *(pData+3) >>8;
							}
							break;
					
						default:
							break;
										
				}
				
			}
			else				// 4选针 模式
			{
				switch (id)
				{
					case 	0:
							{
								needledata =(*(pData+1) ) & 0x0F;
								jqdmask =( *(pData+1) >>4 )& 0x0F;
							}
							break;
					case	1:
							{
								needledata =(*(pData+1) >>8) & 0x0F;
								jqdmask =( *(pData+1) >>12 )& 0x0F;
							}
							break;
					case	2:
							{
								needledata =(*(pData+2) ) & 0x0F;
								jqdmask =( *(pData+2) >>4 )& 0x0F;
							}
							break;
					case	4:
							{
								needledata =(*(pData+2) >>8) & 0x0F;
								jqdmask =( *(pData+2) >>12 )& 0x0F;
							}
							break;
					case	5:
							{
								needledata =(*(pData+3) ) & 0x0F;
								jqdmask =( *(pData+3) >>4 )& 0x0F;
							}
							break;
					case	6:
							{
								needledata =(*(pData+3) >>8) & 0x0F;
								jqdmask =( *(pData+3) >>12 )& 0x0F;
							}
							break;
					default:
							break;
				}
			}
			Needle_isr(needledata, jqdmask);			
				
		}
		break;
		case 0x0706:
		{
			jqd_start_blade[*(pData+1) & 0x07] = *(pData+2);
			Jacquard_Init_blade(*(pData+1) & 0x07, *(pData+3));
		}
		break;
		case 0x0906:
		{
			Jacquard_Set_Blade(*(pData+1));
		}
		break;
		default:
			return -1;	
	}
	return 0;

	
}

#endif
#ifdef YARN_ZERO_CHECK

void Send_yarn_zero_st_change(unsigned short yst)
{
	Message_Send_4halfword(CMD_RETURN_CHECKYARNZERO, arch_Get_ID()?(yst<<8):yst, 0,0);
}

void Get_yarn_zero_st(unsigned short reth)
{
	extern unsigned short yarn_zero_st;
	Message_Send_4halfword(reth, arch_Get_ID()?(yarn_zero_st<<8):yarn_zero_st, 0,0);
	
}

#endif


void Command_exec_undefine(NEWCMDMASSAGE *cmd)
{
	if (cmd==NULL) return;
	if (!LocalSys_is_enable) return;
	
	return ;
}




void Get_can_error_msg(unsigned short whichone)
{
		unsigned int x=0;	
		extern void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);

		if (whichone==6)
		{
			extern unsigned int Get_CAN_err_cnt(void);
			x=Get_CAN_err_cnt();
		}
		else
		{
		
		if (whichone > 4)
		{
			x = CAN_error_cnt ;	
		}
		else
		{
			x =CAN_error_cnt_err[whichone];
		}
		}
		arch_SendMessage(4,&x,2);
		//Message_Send_4halfword(CMD_RETURN,x&0xFFFF,(x>>16)&0xFFFF,0xEEEE);
}

#ifdef CHECK_XZQ_ISR_INIT
unsigned int xzq_cnt=0;
unsigned int max_xzq_cnt[2];
#endif


#ifdef CHECK_LIFT_STEP_IS_STOP

void check_first_getinto_Knit_area(char jqd_data,char jqd_mask)
{
	if (need_c_f_g_knitarea && lift_step_after_jqd_arg)
	{
		int j;	
		for (j=0;j<sys_max_jqd_cnt;j++)
		{
			if (((jqd_mask>>j) & 0x01)
				&&(((jqd_data>>j)&0x01) ==0x00))
			{
				whichJQD =j;//第几个选针器需要真的动作
				jqd_cmd_cnt =lift_step_after_jqd_arg;
				need_c_f_g_knitarea =0;
				//Message_Send_4halfword(0xAABB,whichJQD,jqd_cmd_cnt,0);
				return;				
			}
			else
				continue;
		}	
		//Message_Send_4halfword(0xAACC,jqd_data,jqd_mask,0);
		return;
	}
	else
		return;
	//check_step:
	//check_lift_step_isstop();
		
		
}

//void check_first_getinto_Knit_area


#endif

extern volatile unsigned short Main_Head_Binding_flag;
extern volatile unsigned short Main_Head_Binding_is_fail;		/*表示当前状态为绑定失败的,默认成功的*/

extern volatile unsigned int wait_time_ms_for_check_binding;
extern volatile unsigned short head_tryout_time_par;

														/*  2*60*500*/

void Time_2ms_check_binding()
{
	if (wait_time_ms_for_check_binding)
	{
		wait_time_ms_for_check_binding--;
		if (Main_Head_Binding_flag)
		{
			if (!wait_time_ms_for_check_binding)	
			{
				Exec_check_is_binding_ok(1,0);

				//wait_time_ms_for_check_binding = CHECK_BINGD_STATUS_BLANK_TIME_DEF;
			}
		}
		else
			wait_time_ms_for_check_binding=0;
	}
		
}



void Command_exec_peripheral(NEWCMDMASSAGE *cmd)
{	
	//char ischeckcmd=0;
	static char isfirst_poweron=0;

	if (cmd==NULL) return;
#if 0
#define CMD_UNDEFINE				(0x00) //无定义
#define CMD_POWER				(0x01)// 外设加电 断电arg1(0- 断电!0 加电)
#define CMD_STEP_ENABLE			(0x02)// 步进电机上电断电,arg1(0- 断电!0 加电)
#define CMD_GET_DIR					(0x05)// 取机头换向传感器状态
#define CMD_DIR_ISR					(0x06)// 设置机头工作方向
#define CMD_GET_STEP_INPUT			(0x07)//取机头电机的零位状态arg1l表示电机类型(0-,1 度目,2 生克,3 三角 ,4 压脚),arg1h(0-0位，!0--工作位)
#define CMD_KEYDOG_READ			(0x09)	//		
#define CMD_KEYDOG_CHECK			(0x0A)	//
#define CMD_ENCODER_SETUP			(0x0B)	//	电机编码器功能设置,ARG1=0 禁用，=1启用
#define CMD_ENCODER_CHECK			(0x0C)	//	 返回各编码器的状态
#define CMD_STEP_ENCODER_CHECK	(0x0D) /* 返回指定电机编码器的状态arg1=电机号(0-15) */
#endif
	
	if ((!LocalSys_is_enable) &&(cmd->cmd_data !=CMD_SET_SYS_ENABLE)) return;

	//ischeckcmd = (cmd->cmd_type & 0x80)?1:0;	

	
	switch (cmd->cmd_data)
	{
		case CMD_UNDEFINE:
			CMD_CHECK_ISSUPPORT();
			break;
		case CMD_POWER:
			{
				CMD_CHECK_ISSUPPORT();
				if (cmd->arg1)
				{
					arch_Power_On();
					if(!isfirst_poweron)
					{
						Set_check_mainbinding_timedelay(head_tryout_time_par);
						isfirst_poweron++;
					}
				}
				else
				{
					arch_Power_Off();
					arch_StepMotor_Disable();
				}				
			}
			break;
		/*case 0x0E:
			{
				if (cmd->arg1)
				{
					arch_StepMotor_Enable_single(cmd->arg2);
				}
				else
				{
					arch_StepMotor_Disable();
				}				
			}			
			break;	
			*/
		case CMD_STEP_ENABLE:
			{
				CMD_CHECK_ISSUPPORT();
				if (cmd->arg1)
				{
					arch_StepMotor_Enable();
				}
				else
				{
					arch_StepMotor_Disable();
				}				
			}			
			break;
		case CMD_GET_DIR:
			{
				CMD_CHECK_ISSUPPORT();
				Message_Send_4halfword(CMD_RETURN,arch_Get_DirStatus(),0,cmd->arg3);
			}				
			break;
		case CMD_DIR_ISR:
			CMD_CHECK_ISSUPPORT();
			
			jqd_direction = cmd->arg1;

			JQD_Return_reset =0;/*表示换向了*/

			#ifdef 	CHECK_LIFT_IS_WORK
			check_start=0xFF;
			yarn_is_do=0;
			#endif
			#ifdef  CHECK_LIFT_STEP_IS_STOP
	
			need_c_f_g_knitarea =1;
			whichJQD = 0xff;
			#endif
			#ifdef DEBUG_STEP_DDM_CNT
				Check_pos_debug_cnt();
			#endif

			if (Main_Head_Binding_is_fail && Main_Head_Binding_flag)  /*要绑定，并且绑定失败*/
			{
				alert_push(HEAD_BINDING_MAIN_ERR,0);	
			}
		
#if 0 //def CHECK_XZQ_ISR_INIT
			
			if ((xzq_cnt)&&(max_xzq_cnt[jqd_direction?0:1])&&(xzq_cnt!=max_xzq_cnt[jqd_direction?0:1]))
			{
				alert_push(0xED,abs(xzq_cnt-max_xzq_cnt[jqd_direction?0:1]));
			}
			max_xzq_cnt[jqd_direction?0:1] =xzq_cnt;
			xzq_cnt=0;
#endif
			break;
		case CMD_GET_STEP_INPUT:
			{
				CMD_CHECK_ISSUPPORT();
				Message_Send_4halfword(CMD_RETURN,Get_ZeroStatus(cmd->arg1?1:0),0,cmd->arg3);
			}
			break;
		case CMD_GET_STEP_INPUT_LOG:
			{
				CMD_CHECK_ISSUPPORT();
				Message_Send_4halfword(CMD_RETURN,Get_ZeroStatus_log(0),Get_ZeroStatus_log(1),cmd->arg3);

			}
			break;
		case CMD_KEYDOG_READ:
				
			break;

		case CMD_KEYDOG_CHECK:
				
			break;

	#ifdef ENCODER_SUPPORT	
		case CMD_ENCODER_SET_GOBACK_STEPS:/*目前不起作用*/
			CMD_CHECK_ISSUPPORT();
			Encoder_Set_Goback_Steps(cmd->arg1,cmd->arg2);
			
			break;
		case CMD_ENCODER_SET_MIN_ECODE_RANG:
			CMD_CHECK_ISSUPPORT();
			Encoder_Set_minsteps_rang(cmd->arg1,cmd->arg2);
			
			break;		
		case CMD_ENCODER_GET_MAX_STEPS:
			CMD_CHECK_ISSUPPORT();
			Message_Send_4halfword(CMD_RETURN,cmd->arg1,Encoder_Get_Max_Steps(cmd->arg1),cmd->arg3);
			break;
		case CMD_ENCODER_ENABLE_MASK:
			CMD_CHECK_ISSUPPORT();
			Encoder_Enable_mask(cmd->arg1);
			StepMotor_Set_Ecoder_dir_withzerodir();/*20190513 */
			#ifdef ECODE_USE_MT6813_PWM
			if((cmd->arg1)&&(Check_is_MT6813PWM_Mode()))
			{
				extern void Ecode_CS_for_MT6813();
				Ecode_CS_for_MT6813();
			}
			#endif
			
			break;
		case CMD_ENCODER_SETUP:
			CMD_CHECK_ISSUPPORT();
			Encoder_Enable(cmd->arg1);
			StepMotor_Set_Ecoder_dir_withzerodir();
			#ifdef ECODE_USE_MT6813_PWM
			if((cmd->arg1)&&(Check_is_MT6813PWM_Mode()))
			{
				extern void Ecode_CS_for_MT6813();
				Ecode_CS_for_MT6813();
			}
			#endif
			break;
		case CMD_ENCODER_CHECK:
			CMD_CHECK_ISSUPPORT();
				Message_Send_4halfword(CMD_RETURN,Encoder_Probe(0xFF),0,cmd->arg3);
			break;
		case CMD_STEP_ENCODER_CHECK:
			CMD_CHECK_ISSUPPORT();
			{
				int coder,state;  //实际arg1代表编码器号
				short ret;
				ret = Encoder_getCoder(cmd->arg1, &coder, &state);
				Message_Send_4halfword(CMD_RETURN, coder, state, ret);
			}
			break;	

		case CMD_ENCODER_ZERO_MASK:
			CMD_CHECK_ISSUPPORT();
			if (cmd->arg2 == 0x0000)
			{
				Encoder_maskZero(cmd->arg1);      //实际传输的是编码器号
			}
			else if (cmd->arg2 == 0x0001)
			{
				Encoder_autoZero(cmd->arg1, cmd->arg3,0);
			}
			else if(cmd->arg2 == 0x0002)  /*单边模式慈星*/
			{
				Encoder_autoZero(cmd->arg1, cmd->arg3,1);
			}
			break;

		case CMD_ENCODER_ZERO_SETVAL:
			CMD_CHECK_ISSUPPORT();
			Encoder_setZero(cmd->arg1, cmd->arg2);
			break;
		case CMD_ENCODER_ZERO_READ:
			CMD_CHECK_ISSUPPORT();
			Message_Send_4halfword(CMD_RETURN, Encoder_getZero(cmd->arg1), 0, cmd->arg3);
			break;
			
		case CMD_ENCODER_DIR_SETUP:
			CMD_CHECK_ISSUPPORT();
			Encoder_setDir(cmd->arg1, cmd->arg2,1);
			break;

		case CMD_ENCODER_POS_TODO:
			CMD_CHECK_ISSUPPORT();
			if (cmd->arg2 == 0x0000)
			{
				Encoder_rPos(cmd->arg1, 1);
			}
			else if (cmd->arg2 == 0x0001)
			{
				if (cmd->arg1 == 0xFF)
					Encoder_adjPosAll();
				else
					Encoder_adjPos(cmd->arg1);
			}
			else if (cmd->arg2 == 0x0002)
			{
				Encoder_setThreshold(cmd->arg1,cmd->arg3);
			}
			break;
		case CMD_ENCODER_GET_STATE:
			CMD_CHECK_ISSUPPORT();
			{
				Encoder_Get_state(CMD_RETURN,cmd->arg1_l);

			}
			break;
	#endif	
		
		case CMD_STEP_WORK_CURRENT:
			CMD_CHECK_ISSUPPORT();
			{
				//myprintf("SetCurrent:[%d][%d]\r\n", cmd->arg1, cmd->arg2);
				#ifdef STEP_CURR_DEF
				break;
				#endif
				if (cmd->arg1 == 0x0000)	//0-3度目电流设置
				{
					stepmotor_current[MOTOR_TYPE_DENSITY-1] = cmd->arg2;
					DAC_SetVoltage_channel1(cmd->arg2);
				}
				else if (cmd->arg1 == 0x0001)	// 4,5
				{
					stepmotor_current[MOTOR_TYPE_ACTION-1] = cmd->arg2;
					DAC_SetVoltage_channel2(cmd->arg2);
				}
				else if (cmd->arg1 == 0x0002)	// 6,7
				{
					stepmotor_current[MOTOR_TYPE_LIFT-1] = cmd->arg2;
					#ifdef E490_V10_BOARD
					if (TZ_use_Step)
						LIFT_step_PWMDA_Set_val(cmd->arg2);
					else
						if(Head_Mode_ == HEAD_MODE_SKMODE2)
						{
							SKER_step_PWMDA_Set_val(cmd->arg2);
						}
					#else
					alert_set_Step_PWM_A(cmd->arg2);
					#endif
				}
				DA_is_out =1;
			}
			break;
		case CMD_SET_SYS_ENABLE:
			CMD_CHECK_ISSUPPORT();
			{
				LocalSys_is_enable = cmd->arg1 ? 1 : 0 ; //
			}
			break;
		case CMD_GET_SIG_PH://not ok  
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int zst=Get_SIG_PH_Status();
				Message_Send_4halfword(CMD_RETURN,zst & 0xffff,(zst>>16)&0xffff,cmd->arg3);	
			}
			break;
		case CMD_GET_AD_CUR:
			CMD_CHECK_ISSUPPORT();
			{
				extern volatile unsigned int DC24_P_CURR_A;
				extern volatile unsigned int DC24_N_CURR_A;
				extern volatile unsigned int DC24_P_CURR_V;
				extern volatile unsigned int DC24_N_CURR_V ;
				extern volatile unsigned int DC12_P_CURR_V ;
				extern volatile unsigned int Vbat_CURR_V ;
				extern volatile unsigned int Vrf_CURR_V ;
				extern volatile unsigned int TempS_CURR_V ;
				//extern volatile unsigned int TempS_CURR_v_o;
				unsigned short d[4];
				 unsigned short dt;
				d[0]=CMD_RETURN;
				d[3]=cmd->arg1_l;
				switch(cmd->arg1_l)
				{
				case 0:
					{    
						dt = DC24_P_CURR_A;
						
						if (dt>=1650)
						{
							d[1] = (dt*10-16500)/55;
						}
						else
							d[1] =0;
						dt = DC24_N_CURR_A;
						
						if (dt>=1650)
						{
							d[2] = (dt*10-16500)/55;
						}
						else
							d[2] =0;						
						
					}
					break;
				case 1:
					{
						d[1]=DC24_P_CURR_V *21/100;
						d[2]=DC24_N_CURR_V *21/100;
					}
					break;
				case 2:
					{
						d[1]=DC12_P_CURR_V *215/1500;
						d[2]=Vbat_CURR_V * 2;
					}
					break;
				case 3:	
					{	
						extern  volatile unsigned char fan_st_sys;
						d[1]=Vrf_CURR_V;
						dt = TempS_CURR_V;						
						d[2] = 250L -((long)(760-dt) * 100L)/25L ;	
						d[3] |=(fan_st_sys<<8); 
						//unsigned int tmp_ = 250L -((long)(760-TempS_CURR_V) * 100L)/25L ;			
						
					}
					break;	
					
				default:
						break;
						
				}
				Message_Send_4halfword(d[0],d[1],d[2],d[3]);	
			}
			break;
		case CMD_CHECK_RES_DATA:
			CMD_CHECK_ISSUPPORT();
			{
				need_check_rs =	cmd->arg1_l & 0x0F;
				Get_base_DC24_zero();
			}
			break;
		case CMD_GET_RES_DATA:	
			CMD_CHECK_ISSUPPORT();
			{
				int i;
				extern volatile unsigned int DC24_PN_data_Arry[];
				for (i=0;i<(MAX_DC24_DATA_COUNT>>1);i++)
				{
					Message_Send_4halfword(CMD_RETURN,DC24_PN_data_Arry[i*2+0],DC24_PN_data_Arry[i*2+1],((i*2+1)<<8)|(i*2+0));
				}
				break;
			}
		case CMD_SET_TEST_MODE_PWM:
			CMD_CHECK_ISSUPPORT();
			{
				SYS_Test_Mode_ISPWM =cmd->arg1_l ? 1 : 0 ;
				break;
			}
		case CMD_SET_PWM_DA_OR_DA:
			CMD_CHECK_ISSUPPORT();
			{	
				unsigned char whichmotortype;

				#ifdef STEP_CURR_DEF
				break;
				#endif

				switch (cmd->arg1)
				{
					case (0x0000):					//度目
						whichmotortype = MOTOR_TYPE_DENSITY;
						break;
					case (0x0001):	//生克	
						whichmotortype = MOTOR_TYPE_SKINER;
						break;
					case (0x0101):	//三角
						whichmotortype = MOTOR_TYPE_ACTION;
						break;
					case (0x0201):	//压脚
						whichmotortype = MOTOR_TYPE_FEET;
						break;
					case (0x0301):	//纱嘴					
						whichmotortype = MOTOR_TYPE_YARN;
						break;
					case (0x0401):	//抬针				
						whichmotortype = MOTOR_TYPE_LIFT;
						break;
					
					default:
						whichmotortype = MOTOR_TYPE_UNDEF;
							break;
				}
				
				//extern void alert_set_Step_PWM_A(int Step_A);	
				Exec_Set_Motor_Curr(whichmotortype,cmd->arg2);
			}			
			break;

			case CMD_POWER_CTRL_EX:
			CMD_CHECK_ISSUPPORT();
			{
				//extern void arch_Power_ctrl_ex(unsigned char whichpower,unsigned char onoff,unsigned char wh);
				arch_Power_ctrl_ex(cmd->arg1_l,cmd->arg1_h,43);
			}
			break;
			case CMD_SET_STEP_CHECK_TYPE:
				CMD_CHECK_ISSUPPORT();
			{
				Exec_Set_Step_check_Type(0xff,0xff,cmd->arg1_h & 0x03);  //实际没起作用
			}
			break;
			case CMD_STEP_PARAMETER_SET_WITH_MOTORTYPE:
				CMD_CHECK_ISSUPPORT();
			{	//
				Exec_Set_Step_Parameter_with_motortype(cmd->arg1_l,cmd->arg1_h,cmd->arg2,cmd->arg3);
			}
			break;	

			#ifdef TRY_ZERO_MIDD
			case CMD_STEP_SET_TRY_STEPS_TO_ZERO:
			CMD_CHECK_ISSUPPORT();
			{
				Set_try_steps_to_zero(cmd->arg1);
			}
			break;

			#endif

			case CMD_STEP_SET_ACC_STEPS:
			CMD_CHECK_ISSUPPORT();
			{
				Set_Motor_steps_for_ACC_DEC(cmd->arg1);
			}
			break;

			case CMD_DATA_STORE_SYS_KEY:
			CMD_CHECK_ISSUPPORT();
			{
				Data_store_key_sys = cmd->arg1;
			}
			break;

			case CMD_SYS_POWER_LOST:
			CMD_CHECK_ISSUPPORT();
			{
				Sys_power_lost_bit =cmd->arg1;
			}
			break;
			case CMD_ENCODER_CS:
			CMD_CHECK_ISSUPPORT();
			{
				Ecode_CS_set(cmd->arg1_l,cmd->arg1_h);
			}
			break;
			
		default:
			break;
	}	
	return ;
}

//extern void Get_Step_data();

void Command_exec_sysset(NEWCMDMASSAGE *cmd)
{
	if (cmd==NULL) return;
	if (!LocalSys_is_enable) return;
	switch(cmd->cmd_data)
	{
		case CMD_SYSSET_MODE_ISR:
			CMD_CHECK_ISSUPPORT();
			work_mode = cmd->arg1;
			break;
		case CMD_SYSSET_MOTOR_CONFIG_IDTYPE:
			CMD_CHECK_ISSUPPORT();
			if (cmd->arg1_h<MOTOR_OTHER_MAX)
				Stepmotor_Set_ID_Type_with_NO(cmd->arg1_l,cmd->arg2_l,motor_type_remap[cmd->arg1_h]);
			break;
		case CMD_SYSSET_MOTOR_CONFIG_INPUTID:
			CMD_CHECK_ISSUPPORT();
			Stepmotor_Set_inputID_with_NO(cmd->arg1_l,cmd->arg2_l,cmd->arg2_h,cmd->arg1_h);
			break;
		case CMD_SYSSET_SET_ZEROTYPE_WITH_MAINID:
			CMD_CHECK_ISSUPPORT();
			Stepmotor_Set_zerotype_with_NO(cmd->arg1_l,cmd->arg1_h);
			break;
		case CMD_SYSSET_GET_ZERO_ST:
			CMD_CHECK_ISSUPPORT();
			{
				short zerost = StepMotor_Get_Zero_ST(cmd->arg1_l);
				Message_Send_4halfword(CMD_RETURN, zerost, cmd->arg1, cmd->arg3);
			}
			break;
		case CMD_SYSSET_HEAD_MODE_SET:
			CMD_CHECK_ISSUPPORT();
			if (cmd->arg1_l==0)
			{
				StepMotor_Set_with_Head_Mode(cmd->arg1_h);
				//Set_check_mainbinding_timedelay(10);  /*设置机头模式之后，10秒检查是否绑定*/
			}
			if (cmd->arg1_l==1)
				Message_Send_4halfword(CMD_RETURN,Stepmotor_get_Head_Mode(),0,0);
			if (cmd->arg1_l==2)
			{
				Get_Step_data();
			}
			break;

		case CMD_SYSSET_SYSTYPE:
			#ifdef NEW_ALARM_STYLE
			{
				CMD_CHECK_ISSUPPORT();
				if ((cmd->arg1_l>0)&&(cmd->arg1_l<16))
				{
					sys_type[0] = cmd->arg1_l;	
				}
				if ((cmd->arg1_h>0)&&(cmd->arg1_h<16))
				{
					sys_type[1] = cmd->arg1_h;	
				}
				Set_stepmotor_alarm_id( arch_Get_ID(),sys_type);

			}
			#endif
			break;
		#ifdef NEW_ALARM_STYLE	
		case CMD_SYSSET_SYSTYPE+1:
			{
				CMD_CHECK_ISSUPPORT();
				Set_Test_new_alsrm_style();
			}
			break;
		#endif
		#ifdef CHECK_LIFT_STEP_IS_STOP
		case CMD_SYSSET_LIFTSTEP_AFTER_JQD_ARG:
			{
				CMD_CHECK_ISSUPPORT();
				lift_step_after_jqd_arg=cmd->arg1;
				
			}
			break;
		#endif	

		case CMD_SYSSET_STEP_RESET:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Reset(cmd->arg1_l,(unsigned short)(cmd->arg3_l>>4)&0x01);
			}
			break;
		case CMD_SYSSET_STEP_OPT:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_isr_exec(cmd->arg1_l, cmd->arg2, 1,cmd->arg3);
			}
			break;	
		case CMD_SYSSET_STEP_GETPOS:
			CMD_CHECK_ISSUPPORT();
			{
				short pos = StepMotor_Get_Position(cmd->arg1_l);
				Message_Send_4halfword(CMD_RETURN, pos, 0, cmd->arg3);
			}
			break;
		case CMD_SYSSET_STEP_SETPOS:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_Position(cmd->arg1_l, cmd->arg2);
			}
			break;	
		
		default:
			break;
		
	}
	return ;
}


void Command_exec_motor_density(NEWCMDMASSAGE *cmd)
{
	static unsigned short bind_cmd_idx=0;
if (cmd==NULL) return;
if (!LocalSys_is_enable) return;
#if 0
#define CMD_DENSITY_MOTOR_RST		(0x01)	// 所有度目电机归零arg1=(0--all,1--left,2--right,3--sig) arg2 =(0-15)
#define CMD_DENSITY_GET_ZERO_WORK_BUSY		(0x02)	// 取机头度目状态arg1l(0-zero,1-work,2-busy) arg1h(0-all,1-sig),arg2(0-15)

#define CMD_DENSITY_GET_MOT_POS	(0x08)	// 取机头指定度目位置arg1 表示度目号(0-15)
#define CMD_DENSITY_STEP_ISR		(0x09)	//执行度目动作	
											/*		度目动作					
												*		arg1	-对应度目电机号
												*		arg2 -电机步数
												*		arg3 - 是否检查传感器2的状态( 0--不检查，!0--要检查 )
											*/
#define CMD_DENSITY_SET_MOT_POS	(0x0A)	// 设置机头指定度目位置arg1 表示度目号(0-15) arg2 表示POS值
#define CMD_DENSITY_SET_SPEED		(0x0B)	//设置速度arg1(0-复位速度,1-最大速度,2,other) ,arg2 速度值
#define CMD_DENSITY_SET_ATTRIBUTE 	(0x0C)	// 设置度目属性arg1(0--enable,1--dir,2--fastmode,3--verystep,4--检测精度,5--细分) .arg2(attr_mask)
#define CMD_DENSITY_SET_DELAYTIME	(0x0D)	// 设置度目复位的延时时间arg1 电机号(0-15)，arg2(时间值ms)


#endif

	switch(cmd->cmd_data)
	{
		case CMD_DENSITY_STEP_ISR_BIND:
			CMD_CHECK_ISSUPPORT();
			{
				Motor_bind_cmd_set(bind_cmd_idx++,cmd->arg1_l, cmd->arg1_h, (short)cmd->arg2, (short)cmd->arg3);
			}
			break;
		case CMD_DENSITY_MOTOR_RST:
			CMD_CHECK_ISSUPPORT();
			{
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
					break;
				}
				switch (cmd->arg1)
				{
					case 0:	
						StepMotor_All_Reset((unsigned short)cmd->arg3_l);
						break;
					case 1:
						StepMotor_Reset_LeftMotor((unsigned short)cmd->arg3_l);
						break;
					case 2:
						StepMotor_Reset_RightMotor((unsigned short)cmd->arg3_l);
						break;
					case 3:
						{
							unsigned int stepno;					
							if(Get_StepMotor_isveryenable_IDall(cmd->arg2))
								break;
							stepno= StepMotor_get_no_with_IDall(cmd->arg2);
							StepMotor_Reset(stepno,(unsigned short)(cmd->arg3_l>>4)&0x01);
						}
						break;

					case 0xFF://add by zhuqiwei 160603
						StepMotor_Reset(cmd->arg2,(unsigned short)(cmd->arg3_l>>4)&0x01);
						break;
					default:
						break;
				}
			}
			break;
		case CMD_DENSITY_GET_ZERO_WORK_BUSY:
			CMD_CHECK_ISSUPPORT();
			if (Get_StepMotor_isveryenable_IDall(cmd->arg2))
				break;
			switch (cmd->arg1_l)
			{
			case 0:				
			case 1:				
			case 2:
				{
					if (cmd->arg1_h)
					{
						Message_Send_4halfword(CMD_RETURN,Get_Status_StepType_sig(cmd->arg1_l,MOTOR_TYPE_DENSITY,cmd->arg2),0,cmd->arg3);
					}
					else
					{
						Message_Send_4halfword(CMD_RETURN,Get_Status_StepType(cmd->arg1_l,MOTOR_TYPE_DENSITY),0,cmd->arg3);
					}
				}
				break;			
			case 3:
				{
					short pos = StepMotor_Get_Position(cmd->arg2);
					Message_Send_4halfword(CMD_RETURN, pos, 0, cmd->arg3);
				}break;
			case 4:
				{
					if (cmd->arg1_h)
					{
						Message_Send_4halfword(CMD_RETURN,Get_Status_StepType_sig(cmd->arg1_l,MOTOR_TYPE_DENSITY,cmd->arg2),0,cmd->arg3);							
					}
					else
					{
						break;
					}
				}
				break;

			default:
				break;
			}
			break;

		case CMD_DENSITY_STEP_ISR:
			CMD_CHECK_ISSUPPORT();
			{
				if(Get_StepMotor_isveryenable_IDall( cmd->arg1_l))
					break;
				
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
					break;
				}
				
				if ( cmd->arg1_h ){
						step_work_sign_alarmenable |= (0x01 << cmd->arg1_l);
					}
					else						
						step_work_sign_alarmenable &= ~(0x01 << cmd->arg1_l);

					if(Motor_is_doing_bind_cmd(StepMotor_get_no_with_IDall(cmd->arg1_l)))
						return;
					StepMotor_isr_exec(cmd->arg1_l, cmd->arg2, 1,cmd->arg3);
					//StepMotor_isr_exec(cmd->arg1_l, (short)cmd->arg2, 1);
			}
			break;
		case CMD_DENSITY_SET_MOT_POS:
			CMD_CHECK_ISSUPPORT();
			{
				if(Get_StepMotor_isveryenable_IDall(cmd->arg1_h))
					break;
				{
					unsigned int stno;
					stno = StepMotor_get_no_with_IDall(cmd->arg1_h);

					if (cmd->arg1_l )
					{
						StepMotor_Set_Position(stno, cmd->arg2);
					}
					else
					{
						StepMotor_Set_Input_error(stno, cmd->arg2);
					}
				}
			}
			break;

		case CMD_DENSITY_SET_SPEED:
			CMD_CHECK_ISSUPPORT();
			{
				if (cmd->arg1 ==0)
				{
					StepMotor_Set_Reset_Speed(cmd->arg2,MOTOR_TYPE_DENSITY);
				}
				else
				{
					StepMotor_Set_Speed(cmd->arg2,MOTOR_TYPE_DENSITY);
				}									
			}
			break;

		case CMD_DENSITY_SET_ATTRIBUTE:
			CMD_CHECK_ISSUPPORT();
			{
				switch (cmd->arg1)
				{
				case 0:
					StepMotor_Setup_Active(cmd->arg2,0);
					break;					
				case 0x0100:
					StepMotor_Setup_Active(cmd->arg2,1);
					break;					
				case 1:
					StepMotor_Setup_Direction(cmd->arg2);
					break;
				case 0x0101:
					StepMotor_Setup_Direction_log(cmd->arg2);
					break;	
				case 2:
					StepMotor_Set_FastMode(cmd->arg2,0);
					break;
				case 0x0102:
					StepMotor_Set_FastMode(cmd->arg2,1);
					break;
						
				case 3:					
					StepMotor_Set_VeryMode(cmd->arg2,0);
					break;
				case 0x0103:					
					StepMotor_Set_VeryMode(cmd->arg2,1);
					break;	
				case 4:
					//StepMotor_Set_Check(cmd->arg2);
					Step_Motor_Set_Check_all(MOTOR_TYPE_DENSITY,cmd->arg2);
					break;
				case 5:
					StepMotor_Setup_Resolution(MOTOR_TYPE_DENSITY,cmd->arg2);
					break;
				case 6:
					StepMotor_Set_Zero_go_Steps(MOTOR_TYPE_DENSITY,cmd->arg2);
					break;
				case 7:
					//#ifdef 	LX_DM_SPECIAL_
					//Exec_Set_LX_DM_special(cmd->arg2?1:0);
					StepMotor_Set_zero_type_(MOTOR_TYPE_DENSITY,0,cmd->arg2?1:0);
					//#endif	
					break;	

				case 8:
					StepMotor_Set_zero_detect(MOTOR_TYPE_DENSITY,cmd->arg2);
					break;

				#ifdef DMSTEP_MOVERTOZERO_DEC_ADV
				case 0x0009:
					StepMotor_Set_movetozero_adv_steps(cmd->arg2,MOTOR_TYPE_DENSITY);
					break;
				#endif	
					
				default:
					break;
										
				}
				//break;
			}
			break;

		case CMD_DENSITY_SET_DELAYTIME:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_Reset_Delay_Time(cmd->arg1, cmd->arg2);
			}
			break;

		case CMD_DENSITY_STEP_ISR_DD:
			CMD_CHECK_ISSUPPORT();
			{
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
					break;
				}
				//unsigned int stepno;
				///stepno = StepMotor_get_no_with_IDall(stepid);
				StepMotor_isr_exec_new_dd(cmd->arg1_l, cmd->arg2, cmd->arg3, 1 | ((cmd->arg1_h & 0x80) ? 0x200 : 0), cmd->arg1_h & 0x7F,0);
				//StepMotor_isr_exec_new_dd(cmd->arg1_l, cmd->arg2, cmd->arg3, 1 | ((cmd->arg1_h & 0x80) ? 0x200 : 0), cmd->arg1_h & 0x7F);
			}
			break;	

		
		case CMD_DENSITY_STEP_ISR_DD_CFG:
		case CMD_DENSITY_STEP_ISR_DD_CFG_POWEROFF:	
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_DD_config(cmd->arg1_l & 0x1F,cmd->arg2,cmd->arg1_h,(cmd->arg1_l>>5)& 0x07,(cmd->cmd_data ==CMD_DENSITY_STEP_ISR_DD_CFG_POWEROFF)?1:0);
			}
			break;
		case CMD_DENSITY_STEP_ISR_DD_RUN:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_isr_exec_new_dd(cmd->arg1_l, cmd->arg2, cmd->arg3, 1 | ((cmd->arg1_h & 0x80) ? 0x200 : 0), cmd->arg1_h & 0x7F,1);
			}
			break;
#if 0
		case CMD_DENSITY_STEP_ISR_DD:
			{
				//unsigned int stepno;
				//stepno = StepMotor_get_no_with_IDself(cmd->arg1_l,MOTOR_TYPE_DENSITY);//StepMotor_get_no_with_IDall(stepid);
				StepMotor_isr_exec_new_dd(cmd->arg1_l, cmd->arg2, cmd->arg3, 1 | ((cmd->arg1_h & 0x80) ? 0x200 : 0), cmd->arg1_h & 0x7F,0);
				//StepMotor_isr_exec_new_dd(cmd->arg1_l, cmd->arg2, cmd->arg3, 1 | ((cmd->arg1_h & 0x80) ? 0x200 : 0), cmd->arg1_h & 0x7F);
			}
			break;	
			#endif
		case CMD_DENSITY_MAXSPEED_HZ_FOR_DD:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_MAXSpeed_DD(cmd->arg1);
			}
			break;
		
		case CMD_DENSITY_SET_SPEED_MAX_DD:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_set_maxspeed_DD_HZ(cmd->arg1);
			}
		break;		

		case CMD_DENSITY_STEP_ZEROCHECK_STEPS:
			//CMD_CHECK_ISSUPPORT();
			{
				//if ()
			}			
			break;
			
		case CMD_DENSITY_SET_SPEED_EX:
			CMD_CHECK_ISSUPPORT();
			{
				if (!cmd->arg1_l)
				{
					StepMotor_Set_Speed_EX(MOTOR_TYPE_DENSITY,cmd->arg1_h,cmd->arg2);	
					
				}
			}
			break;
		case CMD_DENSITY_SET_STEP_CHECK_TYPE:
			CMD_CHECK_ISSUPPORT();
			{
				//
				Exec_Set_Step_check_Type(MOTOR_TYPE_DENSITY,(cmd->arg2_h!=0)?cmd->arg2_l:0xff,cmd->arg1_h & 0x03);
			}
			break;
		case CMD_DENSITY_SET_DDM_CHECK:
			CMD_CHECK_ISSUPPORT();
			{
				Exec_Set_DDM_need_Check(cmd->arg1);
			}
			break;
		case CMD_DENSITY_SET_MAX_CW_STEPS:
			CMD_CHECK_ISSUPPORT();
			{
				Motor_Set_maxsteps_cw(MOTOR_TYPE_DENSITY,cmd->arg2);
			}
			break;
		case CMD_DENSITY_SET_DDM_MIN_ECODE_RANG:
			CMD_CHECK_ISSUPPORT();
			{
				Motor_Set_minsteps_ecode_rang(MOTOR_TYPE_DENSITY,cmd->arg2);
			}
			break;
		case CMD_DENSITY_SET_DM_TYPE:
			CMD_CHECK_ISSUPPORT();
			{
				Motor_Set_DM_Type(MOTOR_TYPE_DENSITY,cmd->arg1_l);
			}
			break;
		case CMD_DENSITY_SET_CHECK_AREA:
			CMD_CHECK_ISSUPPORT();
			{
				Motor_Set_DM_check_area(cmd->arg1,cmd->arg2);
			}
			break;
		case CMD_DENSITY_GET_ALL_POSTION:
		case  CMD_DENSITY_GET_ALL_2POS:
			CMD_CHECK_ISSUPPORT();
			{	
				unsigned short allpos[3];				
				Motor_Get_All_postion(cmd->cmd_data==CMD_DENSITY_GET_ALL_2POS?1:0,allpos);
				Message_Send_4halfword(CMD_RETURN,(unsigned int)allpos[0],(unsigned int)allpos[1],(unsigned int)allpos[2]);
				
			}
			break;	
		case	CMD_DENSITY_SET_ZERO_CHECK:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_Density_check_zero_data(cmd->arg1);
			}			
			break;
		case CMD_DENSITY_SET_ZERODATA:
			CMD_CHECK_ISSUPPORT();	
			{
				StepMotor_other_Set_Zerowide_inputerror(MOTOR_TYPE_DENSITY,cmd->arg1_l,cmd->arg2,cmd->arg3);
			}
			break;
		case CMD_DENSITY_GET_ZERODATA:
			CMD_CHECK_ISSUPPORT();
			{
				short zw;
				short iner;
				{
					StepMotor_other_Get_Zerowide_inputerror(MOTOR_TYPE_DENSITY,cmd->arg1_l,&zw,&iner);
					Message_Send_4halfword(CMD_RETURN,cmd->arg1,zw,iner);
				}
			}
			break;
			
		default:
			break;		
	}	
return ;
}

int getact_pos_0_sum =0;

void Command_exec_motor_other(NEWCMDMASSAGE *cmd)
{
	unsigned char step_type;
	
	if (cmd==NULL) return;
	if (!LocalSys_is_enable) return;

	//myprintf("0x4 cmd data[0x%x] arg1[0x%04x] arg2[0x%04x]\r\n", cmd->cmd_data, cmd->arg1, cmd->arg2);

	
	if ((cmd->arg1_l<MOTOR_OTHER_MAX) &&(cmd->arg1_l!=MOTOR_OTHER_DM)) 
	{
		step_type = motor_type_remap[cmd->arg1_l];
	}
	
		

	switch (cmd->cmd_data)
	{
		case CMD_OTHER_ISR:			
		case CMD_OTHER_ISR_EX_WITH_SPEED:	
		case CMD_OTHER_ISR_LX_ACT_RUN_NORMAL:	
		CMD_CHECK_ISSUPPORT();
		{
#if 1	
				if ((step_type == MOTOR_TYPE_ACTION)
					&&((short)cmd->arg2 == 0))
					{
						getact_pos_0_sum++;
						Message_Send_4halfword(0x9999,cmd->arg1_h,getact_pos_0_sum,getact_pos_0_sum>>16);
					}
#endif
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
					break;
				}
			if (Get_StepMotor_isveryenable_IDself(cmd->arg1_h, step_type))
				break;
			//#ifdef LX_ACTION_STEP_SPECIAL
			StepMotor_Type_exec(cmd->arg1_h, (short)cmd->arg2, step_type,cmd->arg3_l,cmd->arg3_h,0,(cmd->cmd_data==CMD_OTHER_ISR_LX_ACT_RUN_NORMAL)?1:0);
			//#else
			//StepMotor_Type_exec(cmd->arg1_h, (short)cmd->arg2, step_type);
		//#endif
		}break;
		case CMD_OTHER_CHECK_OTHER_STEP_ALERT:
		CMD_CHECK_ISSUPPORT();
		{
			StepMotor_Type_exec(cmd->arg1_h, (short)cmd->arg2, step_type,cmd->arg3_l,cmd->arg3_h,1,0);
			Message_Send_4halfword(0xAAAA,cmd->arg1_h,getact_pos_0_sum,getact_pos_0_sum>>16);
		}
		break;		
		/*----------------------------------------------------------------------------*/
		case CMD_OTHER_RST:
		CMD_CHECK_ISSUPPORT();
		{
			
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
					break;
				}
				if (cmd->arg2)
				{
					if (Get_StepMotor_isveryenable_IDself(cmd->arg1_h, step_type))
						break;
					StepMotor_Type_Reset(cmd->arg1_h, step_type,(unsigned short)cmd->arg3_l);
				}
				else
				{
					StepMotor_Type_ResetALL(step_type,(unsigned short)cmd->arg3_l);
				}
			}break;
		/*----------------------------------------------------------------------------*/
		
		case CMD_OTHER_GET_ZERO_WORK_BUSY: 
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int ZWBP_sts =0;
				switch (cmd->arg1_h)
				{
					case 0:
					case 1:
					case 2:{
							if (cmd->arg2_h)
							{
								ZWBP_sts = Get_Status_StepType_sig(cmd->arg1_h, step_type, cmd->arg2_l);
							}
							else
							{
								ZWBP_sts = Get_Status_StepType(cmd->arg1_h, step_type);
							}
						}break;
					case 3:
					case 4:{
							if (cmd->arg2_h)
							{
								ZWBP_sts = Get_Status_StepType_sig(cmd->arg1_h, step_type, cmd->arg2_l);
							}
							else
							{
								break;
							}
						}
						break;
					default:
						break;
				}
				
				Message_Send_4halfword(CMD_RETURN, ZWBP_sts, 0, cmd->arg3);
			}break;
		
		/*----------------------------------------------------------------------------*/
		
		case CMD_OTHER_GET_ZEROWORK:
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int zw_st=0xff;
				zw_st = Get_ZeroWorkStatus_StepType_sig(step_type, cmd->arg1_h);
				Message_Send_4halfword(CMD_RETURN,zw_st,0,cmd->arg3);
			}break;
		/*----------------------------------------------------------------------------*/

		case CMD_OTHER_SET_POS: 
			CMD_CHECK_ISSUPPORT();
			{
				if(Get_StepMotor_isveryenable_IDself(cmd->arg1_h, step_type))
					break;
				StepMotor_Type_Set_Position(cmd->arg1_h, cmd->arg2, step_type);
			}break;
		/*----------------------------------------------------------------------------*/

		case CMD_OTHER_SET_SPEED:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Other_SetSpeed(cmd->arg1_l+MOTOR_TYPE_SKINER,cmd->arg1_h,cmd->arg2);
			}break;
		/*----------------------------------------------------------------------------*/
		
		case CMD_OTHER_SET_STEPS:
			CMD_CHECK_ISSUPPORT();
			{
				
				StepMotor_Set_work_steps(cmd->arg1_l+MOTOR_TYPE_SKINER,cmd->arg2,cmd->arg1_h);
			}
			
			break;
		/*----------------------------------------------------------------------------*/
		
		case CMD_OTHER_SET_OTHER:
			CMD_CHECK_ISSUPPORT();
			{
				if (cmd->arg1_l == 0x00)
				{
					Step_Motor_Set_Check_all(cmd->arg1_h+MOTOR_TYPE_SKINER,cmd->arg2);
				}
				if (cmd->arg1_l == 0x01)
				{
					StepMotor_Setup_Resolution(cmd->arg1_h+MOTOR_TYPE_SKINER,cmd->arg2);
				}
				if (cmd->arg1_l == 0x02)
				{
					StepMotor_Other_Set_isfast(cmd->arg1_h+MOTOR_TYPE_SKINER,cmd->arg2);
				}
				#ifdef E480_BOARD_V10
				if (cmd->arg1_l == 0x03)
				{
					StepMotor_Other_Set_Work_Enable(cmd->arg1_h+MOTOR_TYPE_SKINER,cmd->arg2);
				}
				#endif
				
			}break;
		/*----------------------------------------------------------------------------*/
		
#ifdef E480_BOARD_V10
		case CMD_OTHER_SET_ACTION_STEP:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Setup_ACTION_Para(cmd->arg1,cmd->arg2);
			}break;
			
		case CMD_OTHER_YARN_STEP:
			CMD_CHECK_ISSUPPORT();
			{
				Exec_Yarn_step_cmd(cmd->arg1,cmd->arg2,cmd->arg3);
			}break;
#endif			
		case CMD_OTHER_SET_ACTION_HP_STEP:
			CMD_CHECK_ISSUPPORT();
			{
				switch (cmd->arg1_l)
				{
					case 1:
						{
							if (!arch_check_power_isok(POWER_CTR_P_24))	
							{
								alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
								break;
							}
							Exec_Triangle_step(cmd->arg1_h, cmd->arg2,cmd->arg3 );

						}break;
					case 2:
						if (cmd->arg2>0)			//参数2>0 表示获取编码值。否则获取传感器状态。
						{
							Message_Send_4halfword(CMD_RETURN,StepMotor_Triangle_Get_Position(cmd->arg1_h),0,cmd->arg3);
							
						}
						else
						{
							Message_Send_4halfword(CMD_RETURN,StepMotor_Triangle_Get_Sign(cmd->arg1_h),0,cmd->arg3);
							
						}
						break;
					case 3:
						if (cmd->arg3>0)			//参数3>0 表示设置位置值。否则设置传感器状态。
						{
							(StepMotor_Action_Set_Position(cmd->arg1_h,cmd->arg2));
						}
						else
						{
							(StepMotor_Triangle_Set_Sign(cmd->arg1_h,cmd->arg2 & 0x3));
						}
						break;
					case 4:
						
						break;
					default:
						break;
					
				}
			}
			break;


		case CMD_OTHER_SET_SPEED_EX:
			CMD_CHECK_ISSUPPORT();
			{
				if (cmd->arg1_l<(MOTOR_TYPE_COUNT-1))
				{
					StepMotor_Set_Speed_EX(cmd->arg1_l+MOTOR_TYPE_SKINER,cmd->arg1_h,cmd->arg2);	
				}
			}break;
		
		case CMD_OTHER_SET_STEP_CHECK_TYPE:
			CMD_CHECK_ISSUPPORT();
			{
				Exec_Set_Step_check_Type(cmd->arg1_l+MOTOR_TYPE_SKINER,(cmd->arg2_h!=0)?cmd->arg2_l:0xff,cmd->arg1_h & 0x03);
			}break;

		case CMD_OTHER_SET_FEET_VAL:
			CMD_CHECK_ISSUPPORT();
			{
				if (step_type == MOTOR_TYPE_FEET)	
				StepMotor_Feet_Setup(cmd->arg1_h,cmd->arg2);
			}
			break;

		case CMD_OTHER_SET_STEP_SET_CHECK_POS:
			CMD_CHECK_ISSUPPORT();
			{
				if (step_type == MOTOR_TYPE_ACTION)
				{
					Step_Set_work_check_pos(cmd->arg1_h,cmd->arg2,step_type);	
				}
			}
			break;

		case CMD_OTHER_SET_STEP_ZERO_WIDE:
			CMD_CHECK_ISSUPPORT();
			{
				//if (step_type == MOTOR_TYPE_ACTION)
				//StepMotor_Triangle_Set_Zerowide_inputerror(cmd->arg1_h,cmd->arg2,cmd->arg3);
				StepMotor_other_Set_Zerowide_inputerror(step_type,cmd->arg1_h,cmd->arg2,cmd->arg3);
			}
			break;
		case CMD_OTHER_GET_STEP_ZERO_WIDE:
			CMD_CHECK_ISSUPPORT();
			{
				short zw;
				short iner;
				//if (step_type == MOTOR_TYPE_ACTION)
				{
					//zw = StepMotor_Triangle_Get_Zerowide(cmd->arg1_h);
					//iner = StepMotor_Triangle_Get_Inputerror(cmd->arg1_h);
					StepMotor_other_Get_Zerowide_inputerror(step_type,cmd->arg1_h,&zw,&iner);
					Message_Send_4halfword(CMD_RETURN,cmd->arg1,zw,iner);
				}
			}
			break;	
		case CMD_OTHER_SET_MOTOR_MAX_STEPS_CW:
			CMD_CHECK_ISSUPPORT();
			Motor_Set_maxsteps_cw(step_type,cmd->arg2);
			break;

		case CMD_OTHER_SET_MOTOR_MIN_RANG:
			CMD_CHECK_ISSUPPORT();
			Motor_Set_minsteps_ecode_rang(step_type,cmd->arg2);
			break;

		case CMD_OTHER_SET_MOTOR_ZERO_ADJ:
			CMD_CHECK_ISSUPPORT();
			StepMotor_Set_Zero_go_Steps(step_type,cmd->arg2);
			break;

		case CMD_OTHER_SET_MOTOR_ZERO_TYPE:
			CMD_CHECK_ISSUPPORT();
			StepMotor_Set_zero_type_(step_type,cmd->arg1_h,cmd->arg2);
			break;
		case CMD_OTHER_SET_MOTOR_ZERO_ADJ_EX:
			CMD_CHECK_ISSUPPORT();
			StepMotor_Set_Zero_go_Steps_ex(step_type,cmd->arg2);
			
			break;

		case CMD_OTHER_SET_ZERO_DETECT:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_zero_detect(step_type,cmd->arg2);
			}
			break;
		case CMD_OTHER_SET_ZERO_IS_POSITIVE_DIR:
			CMD_CHECK_ISSUPPORT();
			{
				extern void StepMotor_Set_zero_dir(unsigned char whichtype,unsigned short stepno,unsigned short dirdata);
				StepMotor_Set_zero_dir(step_type,cmd->arg1_h,cmd->arg2);
			}
			break;	
		case  CMD_OTHER_SET_MOTOR_POSTION_EX:			/*设置推针电机的位置*/
			CMD_CHECK_ISSUPPORT();
			{
				Set_Step_Motor_postion_ex(step_type,cmd->arg1_h,cmd->arg2_l,cmd->arg3);
			}
			break;
		case CMD_SK_MOTOR_TO_WHICH_POSTION:			/*推针电机当前应该在的位置*/
			step_type = MOTOR_TYPE_SKINER;
			goto Set_check_pos_other_step;
		//case  CMD_SK_MOTOR_TO_WHICH_POSTION	(0x22)	/*生克电机当前应该在的位置*/
		case CMD_ACT_MOTOR_TO_WHICH_POSTION:/*三角电机当前应该在的位置*/
			step_type = MOTOR_TYPE_ACTION;
			goto Set_check_pos_other_step;
		case CMD_FEET_MOTOR_TO_WHICH_POSTION:/*压脚电机当前应该在的位置*/
			step_type = MOTOR_TYPE_FEET;
			goto Set_check_pos_other_step;
		case CMD_YARN_L_MOTOR_TO_WHICH_POSTION:/*沙嘴1.2.3系统电机当前应该在的位置*/
		case CMD_YARN_R_MOTOR_TO_WHICH_POSTION:/*沙嘴5,6,7系统电机当前应该在的位置*/
			step_type = MOTOR_TYPE_YARN;
			goto Set_check_pos_other_step;
		case CMD_LIFT_MOTOR_TO_WHICH_POSTION:/*推针电机当前应该在的位置*/
			step_type = MOTOR_TYPE_LIFT;
			//goto Set_check_pos_other_step;
			Set_check_pos_other_step:
			
			CMD_CHECK_ISSUPPORT();
			{
				//step_type = motor_type_remap[cmd->cmd_data-CMD_SK_MOTOR_TO_WHICH_POSTION];
				unsigned short which_index;
				unsigned char id=arch_Get_ID();
				switch (id)
				{
				case 	0:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_L_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg1;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg1_l;							
						}
						break;
				case	1:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_L_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg2;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg1_h;	
						}
						break;
				case	2:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_L_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg3;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg2_l;	
						}
						break;
				case	4:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_R_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg1;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg2_h;	
						}
						break;
				case	5:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_R_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg2;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg3_l;	
						}
						break;
				case	6:
						{
							if (step_type == MOTOR_TYPE_YARN)	
							{
								if (cmd->cmd_data == CMD_YARN_R_MOTOR_TO_WHICH_POSTION)	
								{
									which_index =cmd->arg3;
								}
								else
									goto thiscmd_isend;
							}
							else
							which_index =cmd->arg3_h;	
						}
						break;
				default:
						break;
				}
				Set_Step_Motor_TO_Which_Postion(step_type,which_index);
			}
			thiscmd_isend:
			break;
		case CMD_OTHER_SET_MOTOR_ZEROPOS_WORKST:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Set_ZeroPos_WorkST(step_type,cmd->arg1_h);
			}
			break;
		case CMD_OTHER_SET_PAR_EX:
			CMD_CHECK_ISSUPPORT();
			{
				switch(cmd->arg1_h)
				{
				case 0:
					Set_LX_ex_adjData(step_type,cmd->arg2);
					break;
				default:

				break;
				}
				
			}

			break;		
		case CMD_OTHER_GET_SPEED_DATA:
			CMD_CHECK_ISSUPPORT();
			{
				StepMotor_Speed_report(CMD_RETURN,cmd->arg1_l,step_type);
			}
			break;

		case CMD_OTHER_SET_ACT_ADJ_ENABLE:
			CMD_CHECK_ISSUPPORT();			
			{
				extern void StepMotor_set_LX_ACT_AUTOadj_enable(unsigned short adjenable);
				StepMotor_set_LX_ACT_AUTOadj_enable(cmd->arg2);
			}
			break;
		case CMD_OTHER_SET_INPUT_NC_OR_NO:
			CMD_CHECK_ISSUPPORT();			
			{
			//	extern void StepMotor_set_LX_ACT_AUTOadj_enable(unsigned short adjenable);
				StepMotor_input_NCorNO_set(step_type,cmd->arg1_h,cmd->arg2_l);
			}
			break;
		case CMD_OTHER_SET_ALERT_DELAY_CNT:
			CMD_CHECK_ISSUPPORT();	
			{
				StepMotor_alert_delay_cnt_set(step_type,cmd->arg1_h,cmd->arg2_l);
			}
			break;
		case CMD_OTHER_SET_ACT_WORK_ZEROPOS_DISABLE:
			CMD_CHECK_ISSUPPORT();	
			{
				StepMotor_Set_Work_disable_with_ZeroPos(step_type,cmd->arg1_h);
			}

			break;

		case CMD_OTHER_SET_MOTOR_SIGNAL_EDG:
			CMD_CHECK_ISSUPPORT();	
			{
				StepMotor_set_motor_signal_edg_return(step_type,cmd->arg1_h,cmd->arg2_l);	
			//	StepMotor_Set_Work_disable_with_ZeroPos(step_type,cmd->arg1_h);
			}
			break;
			
			
		default:
			break;
	}
}


void Command_exec_dct(NEWCMDMASSAGE *cmd)
{
if (cmd==NULL) return;
if (!LocalSys_is_enable) return;
	switch(cmd->cmd_data)
	{	
		case CMD_DCT_ISR:
				CMD_CHECK_ISSUPPORT();
				{
					if (!arch_check_power_isok(POWER_CTR_P_24))	
					{
						alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
						break;
					}
					if (!arch_check_power_isok(POWER_CTR_N_24))	
					{
						alert_push(HEAD_POWER_ISOFF,(POWER_CTR_N_24<<8)|which_part_close_power);
						break;
					}		
			
				Exec_EMF_isr(cmd->arg1_l,cmd->arg1_h,cmd->arg2,cmd->arg3);
				}
			break;

		case CMD_DCT_GET_STS:
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int sts1=0;
				unsigned int sts2=0;

				if (cmd->arg1_l &0x01)
					sts1 |= (emf_status[2] & 0xff);
				if (cmd->arg1_l &0x02)
					sts1 |=((emf_status[0] & 0xff)<<8);
				if (cmd->arg1_l &0x04)
					sts2 |= (emf_status[1] & 0xff);				
				
				Message_Send_4halfword(CMD_RETURN,sts1,sts2,cmd->arg3);
			}
			break;

		case CMD_DCT_SET_HV:
			CMD_CHECK_ISSUPPORT();
			{
				if (cmd->arg1_l)		//纱嘴电磁铁
				{
					switch (cmd->arg1_h)
					{
						case 0:
						emfYARN_on_keep_time_setup = cmd->arg2;
						//emf_off_keep_time_setup = cmd->arg2;
						break;
						case 1:
						emfYARN_off_keep_time_setup = cmd->arg2;
						
						break;
						default:
						break;

					}
				}
				else			//动作电磁铁
				{
					switch (cmd->arg1_h)
					{
						case 0:
							emf_on_keep_time_setup = cmd->arg2;
							//emf_off_keep_time_setup = cmd->arg2;
							break;
						case 1:
							emf_off_keep_time_setup = cmd->arg2;
						
							break;
						default:
							break;

					}
				}
			}
			break;

		case CMD_DCT_WORKMODE:
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int dctid =0 ;	
				switch (cmd->arg1_l)
				{
					case 0:
						dctid = cmd->arg1_h +12;
						break;
					case 1:
						dctid = cmd->arg1_h +0;
						break;
					case 2:
						dctid = cmd->arg1_h +6;
						break;
					default:
						break;
				}
				
				if(cmd->arg2) {
					emf_work_mode |= 0x1 << dctid;
				}
				else {
					emf_work_mode &= ~(0x1 << dctid);
				}
				//break;

			}
			break;

		case CMD_DCT_TEST_RESISTANCE:
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int sts=0;
				#ifdef E480_BOARD_V10
				if ((Yarn_use_Step)&&(cmd->arg1_l==0))  //表示是纱嘴
				{
					break;
				}
				
				#endif
				
				if (cmd->arg1_l>2)
					break;
				Get_base_DC24_zero();
				test_delay_2ms = (cmd->arg3_h>>1)?cmd->arg3_h:0; 
				sts=emf_status[cmd->arg1_l?(cmd->arg1_l-1):2]&(0x01<<(cmd->arg1_h&0x07))?0:1;
				Exec_EMF_isr_EX(cmd->arg1_l,cmd->arg1_h,sts);
				need_check_rs_each=2;
				nowtestemfindex = cmd->arg1_l * 8+cmd->arg1_h+32;
			}
			break;
		case CMD_DCT_GET_RESISTANCE:
			CMD_CHECK_ISSUPPORT();
			{
				//if (cmd->arg1_l)
				//Exec_EMF_GET_EX(cmd->arg1_l,cmd->arg1_h,cmd->arg2);
				//arch_test_send_all_data_toshow(0);
				Message_Send_4halfword(CMD_RETURN,cmd->arg1_l|(cmd->arg1_h<<8),Exec_EMF_GET_EX(cmd->arg1_l,cmd->arg1_h,cmd->arg2),cmd->arg2?1:0);
			}
			break;

		case CMD_DCT_TEST_RESISTANCE7:
			CMD_CHECK_ISSUPPORT();
			{
				unsigned int sts=0;
				#ifdef E480_BOARD_V10
				if ((Yarn_use_Step)&&(cmd->arg1_l==0))  //表示是纱嘴
				{
					break;
				}
				
				#endif
				
				if (cmd->arg1_l>2)
					break;
				Get_base_DC24_zero();
				test_delay_2ms = (cmd->arg3_h>>1)?cmd->arg3_h:0; 
				sts=cmd->arg2?1:0;//emf_status[cmd->arg1_l?(cmd->arg1_l-1):2]&(0x01<<(cmd->arg1_h&0x07))?0:1;
				Exec_EMF_isr_EX(cmd->arg1_l,cmd->arg1_h,sts);
				need_check_rs_each=1;
				nowtestemfindex = cmd->arg1_l * 8+cmd->arg1_h+32;
			}
			break;	
		#ifdef YARN_ZERO_CHECK	
		case CMD_DCT_GET_YARN_ZERO_ST:
			CMD_CHECK_ISSUPPORT();
			{
				Get_yarn_zero_st(CMD_RETURN);	
			}
			break;
		#endif	
		default:
			break;
	}
return ;
}

unsigned int jqdisrcnt=0;
unsigned int jqdisrcnt_max=0;


#ifdef JQD_ISR_TIMER_CNT_50US
unsigned char JQD_50us_CNT_ARR[]={2,4,6,8,10,12,14,16,18,20};

#endif

extern void set_jqd_data_arr(unsigned int arg1,unsigned int arg2,unsigned int arg3);


/*
jqddata :bit0-7:选针器号，只在第一个选针器设刀头的时候检查
		bit8-15:选针器检查mask值。
*/
void Exec_new_check_JQD_status(unsigned short jqddata)
{
	int i;
	unsigned short jqdmask = (jqddata  >> 8) ;

	if((jqddata & 0x07) !=0) return;  /*1号选针器才生效*/
	
	for(i = 0; i < MAX_JACQUARD; i ++)
	{
		if ((jqdmask>>i) & 0x01) /*与534机头箱的条件相反*/
		{
			if((jqd_status[i]&0xff) != 0xFF)
				alert_push(JQD_UNClEAR1 + i, jqd_status[i]);
		}
		else
			continue;
	}
	
}


		
void Command_exec_xzq(NEWCMDMASSAGE *cmd)
{
if (cmd==NULL) return;
if (!LocalSys_is_enable) return;
switch (cmd->cmd_data)
{
case CMD_XZQ_INIT:
	CMD_CHECK_ISSUPPORT();

	Jacquard_Init();
	break;

case CMD_XZQ_ZHEN_ISR: //特殊处理
case CMD_XZQ_ZHEN_ISR_EX:
	CMD_CHECK_ISSUPPORT();
	{	
		unsigned char id=arch_Get_ID();
		unsigned int needledata;
		unsigned int jqdmask;

		jqd_dir_back = (cmd->cmd_data & 0x80) ?1:0;/*最高位是否为1，为1表示反推，选针顺序需要反向*/

		#ifdef JQD_ISR_CNT_ENABLE
		if (jqdisrcnt>=jqdisrcnt_max)
		{
			jqdisrcnt_max = 	jqdisrcnt;
			//alert_push(0xEF,jqdisrcnt);
		}
		if (jqdisrcnt)
			jqdisrcnt--;
		
		#endif

		if (!arch_check_power_isok(POWER_CTR_P_24))	
		{
			alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
			
			break;
		}
		if (!arch_check_power_isok(POWER_CTR_N_24))	
		{
			alert_push(HEAD_POWER_ISOFF,(POWER_CTR_N_24<<8)|which_part_close_power);
		
			break;
		}
		
		if (JQD_halfwork_mode)//8选针模式
		{
			switch (id)
			{
				case 0:
				case	3:
						{
							needledata =cmd->arg1_l ;
							jqdmask =cmd->arg1_h ;
						}
						break;
				case 	4:		
				case	1:
						{
							needledata =cmd->arg2_l ;
							jqdmask = cmd->arg2_h;
						}
						break;
				case 	5:		
				case	2:
						{
							needledata =cmd->arg3_l;
							jqdmask =cmd->arg3_h;
						}
						break;
				
					default:
						break;
									
			}
			
		}
		else				// 4选针 模式
		{
			#ifdef CHECK_LIFT_STEP_IS_STOP
		//	check_first_getinto_Knit_area(cmd);
			
			#endif
				
			switch (id)
			{
				case 	0:
						{
							needledata =cmd->arg1_l & 0x0F;
							jqdmask =( cmd->arg1_l >>4 )& 0x0F;
						}
						break;
				case	1:
						{
							needledata =cmd->arg1_h & 0x0F;
							jqdmask =( cmd->arg1_h>>4 )& 0x0F;
						}
						break;
				case	2:
						{
							needledata =cmd->arg2_l & 0x0F;
							jqdmask =( cmd->arg2_l>>4 )& 0x0F;
						}
						break;
				case	4:
						{
							needledata =cmd->arg2_h & 0x0F;
							jqdmask =( cmd->arg2_h>>4 )& 0x0F;
						}
						break;
				case	5:
						{
							needledata =cmd->arg3_l & 0x0F;
							jqdmask =( cmd->arg3_l>>4 )& 0x0F;
						}
						break;
				case	6:
						{
							needledata =cmd->arg3_h & 0x0F;
							jqdmask =( cmd->arg3_h>>4 )& 0x0F;
						}
						break;
				default:
						break;
			}
		}
	
		#ifdef CHECK_XZQ_ISR_INIT
		xzq_cnt++;
		#endif

		#ifdef CHECK_LIFT_STEP_IS_STOP
			check_first_getinto_Knit_area(needledata,jqdmask);	
		#endif
		
		Needle_isr(needledata, jqdmask);
		
	}
	break;

case CMD_XZQ_TEST_ISR:
	CMD_CHECK_ISSUPPORT();
	{
		if (!arch_check_power_isok(POWER_CTR_P_24))	
		{
			alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
			break;
		}
		if (!arch_check_power_isok(POWER_CTR_N_24))	
		{
			alert_push(HEAD_POWER_ISOFF,(POWER_CTR_N_24<<8)|which_part_close_power);
			break;
		}
	}
	Exec_Jacquard(cmd->arg1, (unsigned int )cmd->arg2|((unsigned int)cmd->arg3<<16));
	break;

case CMD_XZQ_GET_STS:
	CMD_CHECK_ISSUPPORT();
	Message_Send_4halfword(CMD_RETURN,jqd_status[cmd->arg1 & 0x7],0,cmd->arg3);
	break;

case CMD_XZQ_GET_DAO_ISR:
	CMD_CHECK_ISSUPPORT();
	Message_Send_4halfword(CMD_RETURN,Jacquard_Get_Blade(),0,cmd->arg3);
	break;

case CMD_XZQ_GET_READY_BLADE:
	CMD_CHECK_ISSUPPORT();
	Message_Send_4halfword(CMD_RETURN,Jacquard_Get_jqdBlade(cmd->arg1),0,cmd->arg3);
	break;

case CMD_XZQ_SET_START:
	CMD_CHECK_ISSUPPORT();
	{
#if  0
		if (!(cmd->arg1 & 0x07))
		{
			int i;
			for(i = 0; i < MAX_JACQUARD; i ++)
			{
				if((jqd_status[i]&0xff) != 0xFF)
					alert_push(JQD_UNClEAR1 + i, jqd_status[i]);
			}				
		}
#endif
		Exec_new_check_JQD_status(cmd->arg1);
		jqd_start_blade[cmd->arg1 & 0x07] = cmd->arg2;

#ifdef TEST_JQD_DATA_DW

		if(arch_Get_ID()==0)
		{
			if ((cmd->arg1 & 0x07)==0x03)
			{
				last_start_blad_set = cmd->arg2;
				last_needle_count =0;
			}
		}
		else
			if(arch_Get_ID()==1)
			{
				if ((cmd->arg1 & 0x07)==0x02)
				{
					last_start_blad_set = cmd->arg2;
					last_needle_count =0;
				}
			}
#endif			
			
		Jacquard_Init_blade(cmd->arg1 & 0x07, cmd->arg3);
	}
	
	break;
	#if 0
case CMD_XZQ_SET_START_EX:
	CMD_CHECK_ISSUPPORT();
	{
		unsigned char id=arch_Get_ID();
		//unsigned char jqd_start_arg[4];
		switch (id)
		{
			case 0:
			case 4:
				jqd_start_blade[0]=cmd->arg1_l & 0x0f;
				jqd_start_blade[1]=(cmd->arg1_l>>4) & 0x0f;
				jqd_start_blade[2]=cmd->arg1_h & 0x0f;
				jqd_start_blade[3]=(cmd->arg1_h>>4) & 0x0f;				
				break;
				
			case 1:
			case 5:
				jqd_start_blade[0]=cmd->arg2_l & 0x0f;
				jqd_start_blade[1]=(cmd->arg2_l>>4) & 0x0f;
				jqd_start_blade[2]=cmd->arg2_h & 0x0f;
				jqd_start_blade[3]=(cmd->arg2_h>>4) & 0x0f;	
				break;

			case 2:
			case 6:
				jqd_start_blade[0]=cmd->arg3_l & 0x0f;
				jqd_start_blade[1]=(cmd->arg3_l>>4) & 0x0f;
				jqd_start_blade[2]=cmd->arg3_h & 0x0f;
				jqd_start_blade[3]=(cmd->arg3_h>>4) & 0x0f;	
				break;
			
			default:
				jqd_start_blade[0]=cmd->arg1_l & 0x0f;
				jqd_start_blade[1]=(cmd->arg1_l>>4) & 0x0f;
				jqd_start_blade[2]=cmd->arg1_h & 0x0f;
				jqd_start_blade[3]=(cmd->arg1_h>>4) & 0x0f;				
			break;
				
		}
		Jacquard_Init();
		
	}
	break;
	#endif
case CMD_XZQ_SET_START_ALL123:
case CMD_XZQ_SET_START_ALL123_EX:
case CMD_XZQ_SET_START_ALL123_55AA:	
	CMD_CHECK_ISSUPPORT();
	{	
		unsigned int sysid=arch_Get_ID();
		unsigned short start_bno=0;
		int cmdi;
		if (sysid<4)
		{
			switch (sysid)
			{
				case 0:
						{
							start_bno =cmd->arg1;							
						}
						break;
				case	1:
						{
							start_bno =cmd->arg2;
						}
						break;
				case	2:
						{
							start_bno =cmd->arg3;
						}
						break;
				default:
					goto donothing_cmd_xzq;
					//break;
			}
			for (cmdi=0;cmdi<4;cmdi++)
			{
				jqd_start_blade[cmdi & 0x07] = (start_bno>>(cmdi<<2))&0x0f;
				Jacquard_Init_blade(cmdi & 0x07, (cmd->cmd_data == CMD_XZQ_SET_START_ALL123)?0:((cmd->cmd_data == CMD_XZQ_SET_START_ALL123_55AA)?0x55AA:1));
			}
		}			
	}
	donothing_cmd_xzq:
	break;
case CMD_XZQ_SET_START_ALL456:
case CMD_XZQ_SET_START_ALL456_EX:
case CMD_XZQ_SET_START_ALL456_55AA:		
	CMD_CHECK_ISSUPPORT();
	{
		unsigned int sysid=arch_Get_ID();
		unsigned short start_bno=0;
		int cmdi;
		if ((sysid<8)&&(sysid>=4))
		{
			switch (sysid)
			{
				case 	4:
						{
							start_bno =cmd->arg1;							
						}
						break;
				case	5:
						{
							start_bno =cmd->arg2;
						}
						break;
				case	6:
						{
							start_bno =cmd->arg3;
						}
						break;
				default:
					goto donothing_cmd_xzq_2;
					//break;
			}
			for (cmdi=0;cmdi<4;cmdi++)
			{
				jqd_start_blade[cmdi & 0x07] = (start_bno>>(cmdi<<2))&0x0f;
				Jacquard_Init_blade(cmdi & 0x07, (cmd->cmd_data == CMD_XZQ_SET_START_ALL456)?0:((cmd->cmd_data == CMD_XZQ_SET_START_ALL123_55AA)?0x55AA:1));
			}
		}		
	}
	donothing_cmd_xzq_2:

	break;
#ifdef 	JQD_IS_FAST_MODE_2_STEPS
	case CMD_XZQ_SET_FAST_HLV:
		CMD_CHECK_ISSUPPORT();
		set_jqd_data_arr(cmd->arg1,cmd->arg2,cmd->arg3);
		break;
#endif
case CMD_XZQ_SET_HLV:
	CMD_CHECK_ISSUPPORT();
		{
			if (!cmd->arg1)
			{
				#ifdef JQD_IS_FAST_MODE_2_STEPS
				extern void Set_Fast_JQD_Time(unsigned char time_index);
				Set_Fast_JQD_Time(cmd->arg2_l-2);
				
				#else

				
				
				if (cmd->arg3)
				{
					if (cmd->arg3<=JQD_MAX_TIMER_US)
					{
						#ifdef JQD_ISR_TIMER_CNT_50US

						jqd_keep_time_setup_us = (cmd->arg3<=50)?1: ((cmd->arg3+25)/50);
						#else
						jqd_keep_time_setup_us = cmd->arg3;

						
						#endif
					}	
						//jqd_keep_time_setup = 	(((cmd->arg2_h<<2) + 6)/10);
					//jqd_keep_time_setup = jqd_keep_time_setup<=0?1:jqd_keep_time_setup;
					
				}
				else
				{
					#if 0
					
					jqd_keep_time_setup = cmd->arg2_l* arch_time_500_pulse()/*2*/;

					
					#ifdef JQD_ISR_TIMER_ONE
					jqd_keep_time_setup_us = 300+(cmd->arg2_l-2)*20;
					#endif
					#endif
					//jqd_keep_time_setup_us = 1000+(cmd->arg2_l-2)*500;
					#ifdef JQD_ISR_TIMER_CNT_50US
					jqd_keep_time_setup_us = cmd->arg2_l* 10;
					//jqd_keep_time_setup_us = JQD_50us_CNT_ARR[cmd->arg2_l-2];
					#else					
					jqd_keep_time_setup_us = 1000+(cmd->arg2_l-2)*500;
					#endif						
				}
				#endif
				
				//jqd_keep_time_setup_2 = jqd_keep_time_setup;
			}
			else
			{
				;//jqd_pwm_time_setup = cmd->arg2;
			}
		}
	break;

case CMD_XZQ_SET_DAO_ISR:
	CMD_CHECK_ISSUPPORT();
	Jacquard_Set_Blade(cmd->arg1);
	break;

case CMD_XZQ_SET_DAO_MAX:
	CMD_CHECK_ISSUPPORT();
	jqd_max_blade = cmd->arg1 + 1;
	if (JQD_halfwork_mode)
	{	
		if(jqd_max_blade > (MAX_BLADE>>1)) jqd_max_blade =  (MAX_BLADE>>1);
	}
	else
	{
		if(jqd_max_blade > MAX_BLADE) jqd_max_blade = MAX_BLADE;
	}
	break;


	
case CMD_XZQ_SET_WORKMODE:
	CMD_CHECK_ISSUPPORT();
	{
		JQD_halfwork_mode = !(!(cmd->arg1));   
		if(JQD_halfwork_mode)
		{
			sys_max_jqd_cnt = MAX_JACQUARD;
		}
		else
		{
			sys_max_jqd_cnt = MAX_JACQUARD>>1;
		}
	}
	break;

case CMD_XZQ_SET_NEEDLE_STEP:
	CMD_CHECK_ISSUPPORT();
	{
		if ((cmd->arg1 ==1) ||(cmd->arg1==2))
		{
			needle_work_step=cmd->arg1;
		}
	}

	break;

case CMD_XZQ_TEST_RESISTANCE:
	CMD_CHECK_ISSUPPORT();
	{
		unsigned int sts=0;
		Get_base_DC24_zero();
		sts = jqd_status[cmd->arg1_l & 0x03]&(0x01<<(cmd->arg1_h&0x0F))?1:0;
		//sts =!sts;
		test_delay_2ms = (cmd->arg3_h>>1)?cmd->arg3_h:0; 
		if(test_delay_2ms)   /*需要动作*/
			sts =!sts;
		Exec_Jacquard_ex(cmd->arg1_l, cmd->arg1_h,sts,cmd->arg3_l);
		need_check_rs_each=2;

		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		if(cmd->arg1_h>=MAX_BLADE_HARDWARE_8)
		{
			nowtestemfindex = (cmd->arg1_l & 0x03)*2+72+cmd->arg1_h-MAX_BLADE_HARDWARE_8;
		}
		else
		#endif
		{
			nowtestemfindex = ((cmd->arg1_l & 0x03)<<3)+(cmd->arg1_h&0x07);
			//Message_Send_4halfword(0xFFFF,nowtestemfindex,cmd->arg1,((cmd->arg1_l & 0x03)<<3)+(cmd->arg1_h&0x07));
		}
		
		
	}	
	break;
case CMD_XZQ_GET_RESISTANCE:
	CMD_CHECK_ISSUPPORT();
	{
		//arch_test_send_all_data_toshow();
		Message_Send_4halfword(CMD_RETURN,cmd->arg1_l|(cmd->arg1_h<<8),Exec_JQD_GET_EX(cmd->arg1_l,cmd->arg1_h),cmd->arg2?1:0);
	}	
	break;
case CMD_XZQ_SET_REMAP:	
	CMD_CHECK_ISSUPPORT();
	{
		Jacquard_blade_remap(cmd->arg1_l,cmd->arg1_h,cmd->arg2_l);
	}
	break;
case CMD_XZQ_SET_REDO_CNT:
	CMD_CHECK_ISSUPPORT();
	{
		#ifdef JQD_XZQ_NEEDLE_REDO_MODE
			if (cmd->arg1 < MAX_BLADE)
			jqd_blade_redo_cnt_set =cmd->arg1;

			jqd_blade_redo_time_set = cmd->arg2 * arch_time_500_pulse();
		#endif
	}
	break;
case CMD_XZQ_SET_JQD_MODE:
	CMD_CHECK_ISSUPPORT();
	{
		extern volatile unsigned char JQD_mode_is_UN;
		JQD_mode_is_UN = cmd->arg1_l;
	}
	break;
case CMD_XZQ_SET_JQD_LX:
	CMD_CHECK_ISSUPPORT();
	{
		extern volatile unsigned short JQD_do_delay[];
		JQD_do_delay[0] = cmd->arg1;
		JQD_do_delay[1] = cmd->arg2;
		
	}
	break;
	
#ifdef 0
case 0xff:
	{
		//jqd_doitmust = cmd->arg1?1:0;
		//set_jqd_data_arr(cmd->arg1,cmd->arg2,cmd->arg3);
		#ifdef JQD_ISR_CNT_ENABLE
		Message_Send_4halfword(CMD_RETURN,jqdisrcnt_max,0,0);

#if 0
		arch_putout_MAX_ccr_cnt(0);
		arch_putout_MAX_ccr_cnt(1);
		arch_putout_MAX_ccr_cnt(2);
		arch_putout_MAX_ccr_cnt(3);
#endif
		#endif
	}
break;
#endif
	
default:
	break;
}

return ;
}


void Command_exec_alarm(NEWCMDMASSAGE *cmd)
{
extern volatile unsigned char SYS_is_PowerOn;
extern volatile unsigned int poweron_delay;
if (cmd==NULL) return;
if (!LocalSys_is_enable) return;
	switch (cmd->cmd_data)
	{	
		case CMD_ALARM_CLR:
			CMD_CHECK_ISSUPPORT();
			if (SYS_is_PowerOn)

			{
				if (!arch_check_power_isok(POWER_CTR_P_24))	
				{
					poweron_delay =3;	/*3*250us*/
					arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);				
				}
				if (!arch_check_power_isok(POWER_CTR_N_24))	
				{
					poweron_delay =3;	/*3*250us*/
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);				
				}
			}
			if(JQD_overload_alerted)
			{
				arch_8844_Reset();
				JQD_overload_alerted =0;
			}
			#ifdef TEST_JQD_DATA_DW

			Message_Send_4halfword(0x00FF | (arch_Get_ID()<<8),last_blad_id,last_start_blad_set,last_needle_count);
			
			
			#endif
			Alert_Clear();
			break;
		case CMD_ALARM_GET:
			CMD_CHECK_ISSUPPORT();
			{
				switch (cmd->arg1)
				{
					case 0:
						Message_Send_4halfword(CMD_RETURN,Get_ErrorStatus(),0,cmd->arg3);
						break;
					case 1:
						Check_DirStatus();
						break;
					#ifdef ENCODER_SUPPORT
					case 2:
						{
							extern unsigned int alertcode;
							if(alertcode) 
							{
								Message_Send(alertcode);
								alertcode = 0;
							}
							else 
							{
								Encoder_sendError();
							}
						}
						break;
					#endif	
					default:
						break;
				}
			}
			break;
		case CMD_ALARM_SET_ENABLEMASK:
			CMD_CHECK_ISSUPPORT();
			{
				error_active = cmd->arg1;
			}
			break;

		case CMD_ALARM_SET_OVERLOADENABLE:
			CMD_CHECK_ISSUPPORT();
			{
					
				OverLoad_Setup(cmd->arg1);
				
			}
			break;
		case CMD_ALARM_SET_TANZHEN:
			CMD_CHECK_ISSUPPORT();
			{	
				tanzhen_delay_set(cmd->arg1_l,(cmd->arg1_h >= 1 ? 1 : 0), cmd->arg1_h);
			}
			break;
		case CMD_ALARM_GET_TANZHEN:
			CMD_CHECK_ISSUPPORT();
			{
				tanzhen_zt_get(CMD_RETURN);
			}
			break;
		case CMD_ALARM_CHECK:
			CMD_CHECK_ISSUPPORT();
			{
				if (!cmd->arg1_l)			//start
				{
					Exec_alarm_check_start(cmd->arg1_h);
				}
				else					//end
				{
	
					Exec_alarm_check_end(cmd->arg1_h,cmd->arg2);
					
				}
			}
		//#define CMD_ALARM_SHOCK	(0x07)  //撞针的相关设置arg1_l(0--复位，1--是否使能，2--灵敏度设置)，arg1_h(0-不使能,!0-使能),(0-100表示灵敏度) 撞针灵敏度0-类似于关闭.

		case CMD_ALARM_SHOCK:
			CMD_CHECK_ISSUPPORT();
			{
				Exec_alarm_shock(cmd->arg1_l,cmd->arg1_h,1);
				
			}
			break;
		case CMD_ALARM_GETALERTSTR_CH:
			CMD_CHECK_ISSUPPORT();
			{
				//alert_Get_Content_withindex(cmd->arg1,cmd->arg2,0);
				Send_alert_content_loop(cmd->arg1_l);
			}
			break;

		case CMD_ALARM_GETALERTSTR_EN:
			//CMD_CHECK_ISSUPPORT();
			{
				//alert_Get_Content_withindex(cmd->arg1,cmd->arg2,1);
			}
			break;
		case CMD_ALARM_SET_OVERLOADDATA:
			CMD_CHECK_ISSUPPORT();
			{
				alert_Set_overloaddata(cmd->arg1,cmd->arg2);
			}
			break;
		case CMD_ALARM_SHOCK_NEW:
			CMD_CHECK_ISSUPPORT();
			{
				Exec_alarm_shock(cmd->arg1_l,cmd->arg1_h,cmd->arg2_l);
			}
			break;
			
		#ifdef DC_ERR_FN_ALERT
		case CMD_ALARM_NEW_ALARM_FN:
			CMD_CHECK_ISSUPPORT();
			{								
				Message_Send_4halfword(CMD_RETURN,1,0,cmd->arg3);
			}
			break;		
		#endif
		case CMD_ALARM_SET_TEMP_DATA:
			CMD_CHECK_ISSUPPORT();
			{
				extern volatile unsigned int Alarm_TEMP_Data_;	
				extern volatile unsigned int FAN_Start_TEMP_Data_;
 				extern volatile unsigned int FAN_Close_TEMP_Data_;
				
				if (cmd->arg1>100)
					Alarm_TEMP_Data_ = cmd->arg1;
				if (cmd->arg2>100)
					FAN_Start_TEMP_Data_ = cmd->arg2;
				if (cmd->arg3>50)
				{
					if (cmd->arg3 >= FAN_Start_TEMP_Data_)
					{
						FAN_Close_TEMP_Data_ =	FAN_Start_TEMP_Data_-50;
					}
					else 
						FAN_Close_TEMP_Data_ = cmd->arg3;
					
				}
				else
				{
					if (FAN_Close_TEMP_Data_>=FAN_Start_TEMP_Data_)
					{
						FAN_Close_TEMP_Data_ =	FAN_Start_TEMP_Data_-50;
					}
				}
			}
			break;
		case CMD_ALARM_ENABLE_LOG:
			CMD_CHECK_ISSUPPORT();
			{
				extern volatile unsigned char enable_log_tosend;
				if (!cmd->arg1_l)
				{
					enable_log_tosend=0;
				}else if(cmd->arg1_l ==1)
				{
					enable_log_tosend=1;
				}
				
			}
			break;
		#ifdef LOG_DEBUG_FOR_LX_AT_CH
		case CMD_ALARM_ENABLE_LOG_LX:
			CMD_CHECK_ISSUPPORT();
			{
				extern volatile unsigned char enable_log_tosend_LX;
				extern volatile unsigned char enable_log_zero_change;

				switch(cmd->arg1_h)
				{
					case 0:
						{
							if (!cmd->arg1_l)
							{
								enable_log_tosend_LX=0;
							}else if(cmd->arg1_l ==1)
							{
								enable_log_tosend_LX=1;
							}
						}
						break;
					case 1:
						{
							if (!cmd->arg1_l)
							{
								enable_log_zero_change=0;
							}else if(cmd->arg1_l ==1)
							{
								enable_log_zero_change=1;
							}

						}
						break;
					default:

						break;
				}

			}
			break;
		#endif
		case CMD_ALARM_ENABLE_RUNTIME_CHECK:
			CMD_CHECK_ISSUPPORT();
			{
				alert_runtime_check = cmd->arg1_l?1:0;
			}
			break;
		default:
			break;

	}

return ;
}


void Command_exec_otherset(NEWCMDMASSAGE *cmd)
{
if (cmd==NULL) return;
if (!LocalSys_is_enable) return;
switch (cmd->cmd_data)
{
case CMD_OTHERSET_GET_VER:
	CMD_CHECK_ISSUPPORT();
	{
		extern unsigned int Get_Special_attr(void);	
		unsigned int sr=Get_Special_attr();
		Message_Send_4halfword(CMD_RETURN,Get_Version(),sr&0xFFFF,sr>>16);
	}
	break;
case CMD_OTHERSET_GET_TYPE:
	CMD_CHECK_ISSUPPORT();
	{
		unsigned int boardtype_;		
		extern unsigned int Get_Special_attr(void);	
		unsigned int sr=Get_Special_attr();
		if(cmd->arg2 == arch_Get_Board_Type_boot())
		{
			boardtype_ = cmd->arg2;
		}
		else
		{
			boardtype_ =arch_Get_Board_Type() ;
		}
		Message_Send_4halfword(CMD_RETURN,boardtype_,sr&0xFFFF,sr>>16);
	}	
	break;
case CMD_OTHERSET_GET_FUNC:
	CMD_CHECK_ISSUPPORT();
	Message_Send_4halfword(CMD_RETURN,Get_Func_Type(),0,cmd->arg3);
	break;
case CMD_OTHERSET_GET_BTIME:
	CMD_CHECK_ISSUPPORT();
	Message_Send_4halfword(CMD_RETURN,build_time,build_time>>16,cmd->arg3);	
	break;
case CMD_OTHERSET_DEBUG:
	CMD_CHECK_ISSUPPORT();
	StepMotor_debug();
	break;
case CMD_OTHERSET_REQUEST:
	CMD_CHECK_ISSUPPORT();
	{
		#if 1				
		{
			if(cmd->arg1 >= 0x10)
				break;
		}
		#endif	

		//#if (VER>=0x0090)

		if (cmd->arg2)
		{
			if(cmd->arg2 == arch_Get_Board_Type_boot())
			{
				arch_StepMotor_Disable();
				arch_Power_Off();
				Upgrade_Boot();
				break;
			}
			else
			if ((unsigned short)arch_Get_Board_Type()!=cmd->arg2)
			{
				break;
			}
		}
		
		//#endif
		arch_StepMotor_Disable();
		arch_Power_Off();
		Upgrade_Start();
	}
	break;

case CMD_OTHERSET_GETCPLD:

	CMD_CHECK_ISSUPPORT();

	if (cmd->arg1_l == 0x01)
	{
		extern unsigned short cpld_name;
		
		Message_Send_4halfword(CMD_RETURN,cmd->arg1,cpld_name,cmd->arg3);
	}
	else
		if (cmd->arg1_l == 0x02) 
		{
			extern unsigned short cpld_ver;
			Message_Send_4halfword(CMD_RETURN,cmd->arg1,cpld_ver,cmd->arg3);
		}
	
	break;

case CMD_OTHERSET_GETCANERR:
	CMD_CHECK_ISSUPPORT();
	{
		Get_can_error_msg(cmd->arg1_l);
	}	
	break;	

case CMD_OTHERSET_GETSTEPDEBUGMSG:
	CMD_CHECK_ISSUPPORT();
	{
		Get_Step_debug_msg(cmd->arg1_l);
	}
	break;

case CMD_OTHERSET_GET_HEAD_CONFIG:
	CMD_CHECK_ISSUPPORT();
	{
		Get_Head_config_Msg(CMD_RETURN,cmd->arg1_l);
	}
	break;

case CMD_OTHERSET_GET_HEAD_SUP_REPORT:
	CMD_CHECK_ISSUPPORT();
	{					
		#ifndef TEST_STEPMOTOR_AUTOREPORT

		//#else
		Message_Send_4halfword(CMD_RETURN,1,0,cmd->arg3);
		#endif
	}
	break;	

case CMD_OTHERSET_MAIN_BOARD_ID_WRITE_1:
case CMD_OTHERSET_MAIN_BOARD_ID_WRITE_2:	
	CMD_CHECK_ISSUPPORT();
	{
		Exec_write_main_id_to_flash(&cmd->arg1,(cmd->cmd_data == CMD_OTHERSET_MAIN_BOARD_ID_WRITE_1)?0:1);
	}
	break;

case CMD_OTHERSET_MAIN_BOARD_ID_SET_1:
case CMD_OTHERSET_MAIN_BOARD_ID_SET_2:
	CMD_CHECK_ISSUPPORT();
	{
		Exec_Set_main_id_(&cmd->arg1,(cmd->cmd_data == CMD_OTHERSET_MAIN_BOARD_ID_SET_1)?0:1);
		Exec_Send_main_flash_id_(CMD_RETURN,(cmd->cmd_data == CMD_OTHERSET_MAIN_BOARD_ID_SET_1)?0:1);

		MainID_Get|=1<<((cmd->cmd_data == CMD_OTHERSET_MAIN_BOARD_ID_SET_1)?0:1);
		
		if ((MainID_Get & 0x03 )== 0x03)  /*查询一次*/
		{
			Exec_Set_main_id_timer();
			//Exec_check_is_binding_ok(0);
			MainID_Get &=~((unsigned char)0x03);
		}
	}
	break;
case CMD_OTHERSET_HEAD_BOARD_ID_GET_1:
case CMD_OTHERSET_HEAD_BOARD_ID_GET_2:
	CMD_CHECK_ISSUPPORT();
	{
		Exec_Get_head_id_(CMD_RETURN,cmd->cmd_data == CMD_OTHERSET_HEAD_BOARD_ID_GET_1?0:1);
	}
	break;
case CMD_OTHERSET_MAIN_BOARD_HEAD_BINDING:
	CMD_CHECK_ISSUPPORT();
	{
		short ispass=0;	

		if(BootVer_int<BOOTVER_SUPPORT_BINDING)
		{
			alert_push(HEAD_BOOT_VER_ERR,0);
			break;
		}
		
		if(Exec_check_Data_valid(cmd->arg1,cmd->arg2,cmd->arg3)==0)		
		{
			//unsigned short midp;
			if (cmd->arg3 <=1)
			{
				Exec_write_main_id_to_flash(&cmd->arg3,2);
			}			
			Exec_check_is_binding_ok(0,(cmd->arg3==2)?1:0);
			ispass =1;
		}
		else
			ispass =0;
		if(cmd->arg3==2)
			Message_Send_4halfword(CMD_RETURN,cmd->arg1,cmd->arg2,cmd->arg3|(ispass<<8));
	}
	break;	
case CMD_OTHERSET_GET_MAIN_BOARD_HEAD_BINDING:
	CMD_CHECK_ISSUPPORT();
	{
		Exec_Get_Binding_st_(CMD_RETURN);
	}
	break;
case CMD_OTHERSET_SET_TRYOUT_TIME:
	CMD_CHECK_ISSUPPORT();
	{
		/*不响应, 测试的时候可以使用*/
		Exec_Set_Tryout_time_flash(&cmd->arg1);
		Set_check_mainbinding_timedelay(head_tryout_time_par);
	}
	break;
case CMD_OTHERSET_SET_CHECKIN_TIME:
	CMD_CHECK_ISSUPPORT();
	{
		Set_checkin_time((unsigned int)cmd->arg2|((unsigned int)cmd->arg3<<16));
	}
	break;

default:
	break;
}
return ;
}




void send_check_online_f()
{
	if (Send_check_online)
	{
		if (Send_delay_online==0)
		{
			Send_check_online = 0;
			Message_Send_4halfword(CMD_RETURN_CHECKONLINE_D,0,0,0);
		}
	}
	return;
}


#define EEREAD_BASE_ADDRESS_DATA_STORE 26  



void Exec_Read_data_with_sys_key(unsigned short *data_buff,unsigned short syskey)
{
	unsigned short d[3];
	//unsigned short d_o;
	//int i;
	int ret=0;

	/*ret 说明:0--正常，1-解密密钥失败，2--解密序号失败，3--解密数据失败
	4--序号超出范围，5--读取数据失败,6--表示加密失败,7--写数据失败*/
	
	memcpy((unsigned char *)d,(unsigned char *)data_buff,6);
	ret = Decrypt_can_data(d,syskey);	

	if (ret!=0)
		goto Decrypt_error;


	if (d[1]>63)
	{
		ret =4;						/*序号超出范围*/
		goto Decrypt_error;
	}

	

	 ret = EE_Read(EEREAD_BASE_ADDRESS_DATA_STORE+d[1],&d[0],1);
	if (ret==0)
	{
		if (d[2]==0xffff)
		{
			d[2]=arch_get_ticktime() &0xffff;
			if(( d[2]==0)||(d[2]==0xffff))
			{
			d[2]=0x5CAE;	
			}
		}
		ret =Encryption_can_data(d,syskey);	
		if (ret!=0)
			ret =6;/*表示加密失败*/
	}
	else
	{
		ret = (ret==-1)?5:4;					/*5--读取数据失败,4--id号越界*/
		goto Decrypt_error;
	}

	

Decrypt_error:
	if (ret==0)
	{
		Message_Send_4halfword(CMD_RETURN_DATA_STORE_READ,d[0],d[1],d[2]);
	}
	else
	{
		Message_Send_4halfword(0x8000|CMD_RETURN_DATA_STORE_READ,ret,0,0);
	}
	#ifdef DEBUG_DATA_STORE
	 Debug_data_st = (ret==0)?DEBUG_ST_WRITE:DEBUG_ST_READ_ERR;
	#endif
	return;

}



void Exec_Write_data_sys_Key(unsigned short *data_buff,unsigned short syskey)
{
	unsigned short d[3];
	//unsigned short d_o;
	//int i;
	int ret=0;
	/*ret 说明:0--正常，1-解密密钥失败，2--解密序号失败，3--解密数据失败
	4--序号超出范围，5--读取数据失败,6--表示加密失败,7--写数据失败*/
	
	memcpy((unsigned char *)d,(unsigned char *)data_buff,6);
	ret = Decrypt_can_data(d,syskey);

	if (ret!=0)
		goto Decrypt_error;

	if (d[1]>63)
	{
		ret =4;						/*序号超出范围*/
		goto Decrypt_error;
	}
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+6;
	}
	#endif
	ret = EE_Write(EEREAD_BASE_ADDRESS_DATA_STORE+d[1],&d[0],1);
	if (ret!=0)
	{
		ret = (ret==-1)?7:4;					/*写数据失败*/
		goto Decrypt_error;
	}


Decrypt_error:
	Message_Send_4halfword(((ret==0)?0:0x8000)|CMD_RETURN_DATA_STORE_WRITE,ret,0,0);

#ifdef DEBUG_DATA_STORE
	 Debug_data_st = (ret==0)?DEBUG_ST_READ:DEBUG_ST_WRITE_ERR;
#endif	
	return ;

}

void Command_exec_data_store(NEWCMDMASSAGE *cmd)
{

	if (cmd==NULL) return;
	if (!LocalSys_is_enable) return;
	switch (cmd->cmd_data)
	{
		case CMD_DATA_STORE_READ_DATA:
		CMD_CHECK_ISSUPPORT();
		{
			Exec_Read_data_with_sys_key(&cmd->arg1,Data_store_key_sys);
		}
		break;
		case CMD_DATA_STORE_WRITE_DATA:
		CMD_CHECK_ISSUPPORT();
		{
			Exec_Write_data_sys_Key(&cmd->arg1,Data_store_key_sys);
		}
		break;
		
		default:
			break;
	}		

}

#ifdef DEBUG_DATA_STORE


#define DATA_KEY_DEBUG		0x2345

void Do_candata_for_data_store_tese(unsigned char isread)
{
	static unsigned short D_i=0;
	static unsigned char whichidx=0;
	unsigned short pData[3];

	if (isread)
	{
		pData[0]=0;
		pData[1]=whichidx;
		pData[2]=0xffff;//DATA_KEY_DEBUG;
		D_i++;
		whichidx++;
		if (whichidx>=64)
			whichidx =0;
	}
	else
	{
		pData[0]=D_i;
		pData[1]=whichidx;
		pData[2]=DATA_KEY_DEBUG;
		//D_i++;
		//whichidx++;
	}

	Encryption_can_data(pData,Data_store_key_sys);
	

	Message_Push(0x0A,isread?0x01:0x02, pData[0], pData[1],pData[2]);	
}

void Debug_data_store_self()
{
	//static unsigned int D_i;
	switch (Debug_data_st)
	{
		case DEBUG_ST_WRITE:
			Debug_data_st=DEBUG_ST_FREE;
 			Do_candata_for_data_store_tese(0);
			//Debug_data_st=DEBUG_ST_FREE;
			break;
		case DEBUG_ST_READ:
			Debug_data_st=DEBUG_ST_FREE;
			Do_candata_for_data_store_tese(1);
			
			break;
		case DEBUG_ST_READ_ERR:			/*读失败*/
			Message_Send_4halfword(0xFF0A,Debug_data_st,0,0);
			Debug_data_st=DEBUG_ST_FREE;
			break;
		case DEBUG_ST_WRITE_ERR:			/*写失败*/
			Message_Send_4halfword(0xFF0A,Debug_data_st,0,0);
			Debug_data_st=DEBUG_ST_FREE;
			break;			
		default:

			break;		
	}
	
}


#endif


void Command_exec_checkonline(NEWCMDMASSAGE *cmd)
{
	unsigned int ID;
	if (cmd==NULL) return;//无论是否enable 都要回复这条命令
	if (Check_CMD_is_Support(cmd)) 
	{	
		return;
	}
	ID = arch_Get_ID();
	if (ID)
	{
		Send_check_online=1;
		Send_delay_online=ID<<2;
	}
	else
	Message_Send_4halfword(CMD_RETURN_CHECKONLINE,0,0,cmd->arg3);
	

}

void Command_exec_Testcode(NEWCMDMASSAGE *cmd)
{
	if (cmd==NULL) return;

	switch (cmd->cmd_data)
	{
		case CMD_TESTCODE_START:
			CMD_CHECK_ISSUPPORT();
			/*发过来的值='H','Q','T','E','S','T'*/
			
			if ((cmd->arg1_l==0x48)&&
				(cmd->arg1_h==0x51)&&
				(cmd->arg2_l==0x54)&&
				(cmd->arg2_h==0x45)&&
				(cmd->arg3_l==0x53)&&
				(cmd->arg3_h==0x54))
				{
				Test_Step_enable = 1;
				}
		break;
		case CMD_TESTCODE_END:
			CMD_CHECK_ISSUPPORT();
			Test_Step_enable = 0;
		break;	
		case CMD_TESTCODE_STEPMOTO:
			CMD_CHECK_ISSUPPORT();
			if (Test_Step_enable)
			{
				StepMotor_justrun(cmd->arg1_l,cmd->arg1_h?1:0,cmd->arg2);
			}
		break;
		#ifdef QL_DEBUG_STEP_MOTOR_ECORD
		case CMD_TESTCODE_DEBUG:
			
			CMD_CHECK_ISSUPPORT();
			{	
				int i;
				extern unsigned char ecord_index;
				extern unsigned int ecord_read_temp[];
				Message_Send_4halfword(CMD_RETURN,0,0,ecord_index);
				for (i=0;i<ecord_index;i++)
				{
					Message_Send_4halfword(CMD_RETURN,i,ecord_read_temp[i],ecord_read_temp[i++]);
				}
			}
			//#endif
			break;
		case CMD_TESTCODE_DEBUG_STEP:
			CMD_CHECK_ISSUPPORT();
			{
				Step_debug_set_par(cmd->arg1,cmd->arg2);
			}
			break;
		#endif	

		case CMD_TESTCODE_DEBUG_CAN_CNT:
			CMD_CHECK_ISSUPPORT();
			{
				#ifdef CHECK_CAN_CNT
				extern volatile int can_send_to_main_cnt;
				extern volatile int can_receive_count;
				extern volatile int can_receive_jqd_count;
				switch (cmd->arg1)
				{
					case 0:
						Message_Send_4halfword(CMD_RETURN,cmd->arg1,can_receive_count,can_receive_count>>16);
						break;
					case 1:
						Message_Send_4halfword(CMD_RETURN,cmd->arg1,can_send_to_main_cnt,can_send_to_main_cnt>>16);
						break;
					case 2:
						Message_Send_4halfword(CMD_RETURN,cmd->arg1,can_receive_jqd_count,can_receive_jqd_count>>16);
						break;
					case 3:
						{
							Message_Send_4halfword(CMD_RETURN,0,can_receive_count,can_receive_count>>16);
							Message_Send_4halfword(CMD_RETURN,1,can_send_to_main_cnt,can_send_to_main_cnt>>16);
							Message_Send_4halfword(CMD_RETURN,2,can_receive_jqd_count,can_receive_jqd_count>>16);
						}
						break;
					case 4:
						{
							can_receive_count =0;
							break;
						}
					case 5:
						{
							can_send_to_main_cnt=0;
							break;
						}
					case 6:
						{
							can_receive_jqd_count =0;
							break;
						}
					case 7:
						{
							can_receive_count =0;
							can_send_to_main_cnt=0;
							can_receive_jqd_count =0;
							break;
						}
					case 8:
						{
							{
								int x=can_receive_jqd_count-can_jqdcmd_repeat_count;	
								if (x !=cmd->arg2)
								{
									//alert_push(0xED,(cmd->arg2-can_receive_jqd_count));
									Message_Send_4halfword(0xfff2,x,cmd->arg2,can_jqdcmd_repeat_count);
								}
							}
							can_receive_jqd_count=0;	
							can_jqdcmd_repeat_count =0;
						}
						break;
					
					default :
						break;
						
				}

				#endif
				
			//	Message_Send_4halfword(CMD_RETURN,can_send_to_main_cnt,can_send_to_main_cnt>>16,0);
			//	Message_Send_4halfword(CMD_RETURN,can_send_to_main_cnt,can_send_to_main_cnt>>16,0);
			}
			break;

		case CMD_TESTCODE_DEBUG_GET_ADCCNT:
			//CMD_CHECK_ISSUPPORT();
			{
				//Send_debug_adc_cnt();
				#ifdef JQD_XZQ_NEEDLE_REDO_MODE
					Send_debug_jqd_redo_cnt();
				#endif
			}
			break;	
			
		case CMD_TESTCODE_DEBUG_GET_MAINLOOP:
			{
				//extern void Send_debug_Get_main_loop(void);
				//Send_debug_Get_main_loop();
			}
			break;
		case  0x09:
			{
				//send_check_mainloop_and_01ms(cmd->arg1_l,cmd->arg1_h);
			}
		break;
		case 0x0A:
			{
				extern void Send_postion_ecode(void);
				Send_postion_ecode();
			}
		break;

		case 0x0B:
			{
				extern void Send_coder_info();
				Send_coder_info();
			}
			break;
		#ifdef YARN_ZERO_CHECK	
		case 0x0C:
			{
				Get_yarn_zero_debug(cmd->arg1);
			}	
			break;
		#endif

		#ifdef DEBUG_STEP_OPERATE_GONO_ERROR
		case 0x0D:
			{
				debug_motor_moveto_0_and_goto_N_dir(cmd->arg1_l);
			}
			break;
		#endif
		#ifdef DEBUG_STEP_DDM_CNT
		case 0x0E:
			{
				Get_pos_debug_cnt();
			}
		break;
		#endif

		case 0x0f:
			{
				extern  int Get_read_coder_cnt_();
				unsigned short xs=0xFFAA;
				unsigned int xi = (unsigned int )xs;
				Message_Send_4halfword(CMD_RETURN,xi& 0xffff,xi>>16,0);
				//extern  int get_coder_read_fifo_cnt();
				//int grccnt =timer6_isr_cnt;//Get_read_coder_cnt_(); 
				//int grccnt2 = get_coder_read_fifo_cnt();
				//arch_get_jqd_time_cnt(cmd->arg1_l &0x03);
				//Message_Send_4halfword(CMD_RETURN,grccnt & 0xffff,grccnt>>16,0);
			}
		break;

		case 0x10:
			{
				extern void Setlogid(unsigned char id);
				Setlogid(cmd->arg1_l);
			}
			break;

#ifdef READ_FLASH_DATA_FOR_BANDING
		case 0x3F:
		{
			set_start_to_read_flashdata(cmd->arg1);
		}
		break;
#endif		
		default:
		break;
	}
return ;

}




void Exec_Set_Motor_Curr_data(unsigned char steptype,unsigned short cruu)
{
	if ((steptype>MOTOR_TYPE_UNDEF)&&(steptype<MOTOR_TYPE_MAX))
	{
		stepmotor_current[steptype-1] = cruu;
	}
	else
		return;
}


void Exec_Set_Motor_Curr_phi_with_stepno(unsigned char stepno,unsigned short cruu)
{
	switch (stepno)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			DAC_SetVoltage_channel1(cruu);
			break;

		case 4:
		case 5:
			DAC_SetVoltage_channel2(cruu);
			break;
		case 6:
		case 7:
			SKER_step_PWMDA_Set_val(cruu);
			break;
		case 8:
		case 9:
		case 10:
		case 11:
			if (Yarn_use_Step)
				Yarn_step_PWMDA_Set_val(cruu);
			break;
		case 12:
		case 13:
			if (TZ_use_Step)
				LIFT_step_PWMDA_Set_val(cruu);
			break;
		#ifdef E692_STEPMOTOR_EX
		case 14:
		case 15:	
			if(Step_use_exBoard)
				EX_step_PWMDA_Set_val(stepno-14,cruu);	
			break;		
		#endif	
		default:
			break;			
	}
}

void Exec_Set_Motor_Curr_phi(unsigned char steptype,unsigned short cruu)
{
	
	
	
	switch (Head_Mode_)
	{
		case HEAD_MODE_DEFAULT:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:				
					DAC_SetVoltage_channel1(cruu);
					break;
				case MOTOR_TYPE_SKINER:				
					#ifdef E490_V10_BOARD						
						SKER_step_PWMDA_Set_val(cruu);
					#else
						#ifdef E499_BOARD_SUPPORT_							
							arch_set_Step_SK_A(cruu);
						#else	
						
							alert_set_Step_PWM_A(cruu);
						#endif
					#endif
					break;
				case MOTOR_TYPE_ACTION:					
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;
                    
                		case MOTOR_TYPE_LIFT:
					return;		
                    		//break;

			}
			
			break;
		case HEAD_MODE_SIMPLE:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
				case MOTOR_TYPE_SKINER:
					//alert_set_Step_PWM_A(cruu);
					return;
				//	break;
				case MOTOR_TYPE_ACTION:
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

			}
			
			break;
		case HEAD_MODE_SKMODE2:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
				case MOTOR_TYPE_SKINER:
					DAC_SetVoltage_channel2(cruu);
					break;
				case MOTOR_TYPE_ACTION:
					return;
					//break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;
				case MOTOR_TYPE_LIFT:					
					if (TZ_use_Step)
						LIFT_step_PWMDA_Set_val(cruu);
					else
						SKER_step_PWMDA_Set_val(cruu);	
					break;	

			}
			
			break;
		case HEAD_MODE_SKMODE4:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:					
					DAC_SetVoltage_channel1(cruu);
					break;
				case MOTOR_TYPE_SKINER:
					alert_set_Step_PWM_A(cruu);
					DAC_SetVoltage_channel2(cruu);
					break;
				case MOTOR_TYPE_ACTION:
					return;
					//break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

			}
			
			break;
		case HEAD_MODE_LIFT_HP:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
					
				case MOTOR_TYPE_SKINER:
					if (TZ_use_Step)
						SKER_step_PWMDA_Set_val(cruu);
											
					break;
					
				case MOTOR_TYPE_ACTION:
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_LIFT:					
					if (TZ_use_Step)
						LIFT_step_PWMDA_Set_val(cruu);
					else
						SKER_step_PWMDA_Set_val(cruu);
					break;

			}
			break;
		case HEAD_MODE_LIFT2:
		case HEAD_MODE_LIFT_EX:
		
		case HEAD_MODE_LX_ACT:	
		case HEAD_MODE_FH_ACT:	
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
					
				case MOTOR_TYPE_SKINER:
					#ifdef E490_V10_BOARD
						SKER_step_PWMDA_Set_val(cruu);
					#else					
						#ifdef E499_BOARD_SUPPORT_
							arch_set_Step_SK_A(cruu);
						#else					
							alert_set_Step_PWM_A(cruu);
						#endif
					#endif
					break;
					
				case MOTOR_TYPE_ACTION:
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_FEET:
					return;
					//break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_LIFT:					
					if (TZ_use_Step)
					LIFT_step_PWMDA_Set_val(cruu);
					break;

			}
			break;
#ifdef E490_V10_BOARD
		case HEAD_MODE_FEET:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
					
				case MOTOR_TYPE_SKINER:
									
					SKER_step_PWMDA_Set_val(cruu);
					
					break;
					
				case MOTOR_TYPE_ACTION:  //挂在推针的位置上
					if (TZ_use_Step)
					LIFT_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_FEET:
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_LIFT:
					//if (TZ_use_Step)
					//LIFT_step_PWMDA_Set_val(cruu);
					break;

			}
			break;
		case HEAD_MODE_FEET_CX:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:
					DAC_SetVoltage_channel1(cruu);
					break;
					
				case MOTOR_TYPE_SKINER:
									
					SKER_step_PWMDA_Set_val(cruu);
					
					break;
					
				case MOTOR_TYPE_ACTION:  //挂在推针的位置上
					//if (TZ_use_Step)
					//LIFT_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_FEET:
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;

				case MOTOR_TYPE_LIFT:
					if (TZ_use_Step)
					LIFT_step_PWMDA_Set_val(cruu);
					break;

			}
			break;
#ifdef E692_STEPMOTOR_EX
		case HEAD_MODE_ALL_STEP_CX:
			switch (steptype)
			{
				case MOTOR_TYPE_UNDEF:
					return;
					//break;
				case MOTOR_TYPE_DENSITY:				
					DAC_SetVoltage_channel1(cruu);
					break;
				case MOTOR_TYPE_SKINER:	
					if (Step_use_exBoard)
					{
						EX_step_PWMDA_Set_val(0,cruu);
						EX_step_PWMDA_Set_val(1,cruu);					
					}
					break;
				case MOTOR_TYPE_ACTION:					
					DAC_SetVoltage_channel2(cruu);
					break;

				case MOTOR_TYPE_FEET:
					SKER_step_PWMDA_Set_val(cruu);					
					break;

				case MOTOR_TYPE_YARN:
					if (Yarn_use_Step)
					Yarn_step_PWMDA_Set_val(cruu);
					break;
                    
                		case MOTOR_TYPE_LIFT:
					if (TZ_use_Step)
					LIFT_step_PWMDA_Set_val(cruu);		
					break;

			}
			
			break;

#endif
			


#endif			

		default:
		break;
	}
}

void Exec_Set_Motor_Curr(unsigned char steptype,unsigned short cruu)
{
	Exec_Set_Motor_Curr_data(steptype,cruu);
	Exec_Set_Motor_Curr_phi(steptype,cruu);

}

void Exec_Set_Motor_Curr_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short cruu)
{
	Exec_Set_Motor_Curr_data(steptype,cruu);
	Exec_Set_Motor_Curr_phi_with_stepno(stepno,cruu);

}


void Exec_Set_Motor_RestSpeed_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short speed)
{
	StepMotor_Set_Speed_EX(steptype,0,speed);
}

void Exec_Set_Motor_RunSpeed_with_stepno(unsigned char steptype,unsigned char stepno,unsigned short speed)
{
	StepMotor_Set_Speed_EX(steptype,1,speed);
}



#ifdef E480_BOARD_V10


#define YARN_STEP_CMD_RETURN_HEAD	((CMD_OTHER_YARN_STEP<<8)|(CMDTYPE_MOTOR_OTHER))
extern void Exec_SET_DCT_sts(int arg2,int arg3);
extern void Exec_GET_Moto_Zero_Width(int arg2);
extern void Exec_SET_Moto_Zero_Width(int arg2,int arg3);
extern void Exec_Reset_step_moto_all(int arg2,unsigned short otherarg);
extern int arch_get_yarnstep_sign_(void);

extern volatile unsigned int arch_board_id;
extern volatile unsigned int POS_Yarn_DO;

void Exec_Yarn_step_cmd(unsigned short arg1,unsigned short arg2,unsigned short arg3)
{
	unsigned int tx_data[4]={0,0,0,0};
	if ((!Yarn_use_Step)&&(arg1!=0x10) &&(arg1!=0x0A))
	{
		//Message_Send_4halfword(YARN_STEP_CMD_RETURN_HEAD,Yarn_use_Step?1:0,0,0);

		#ifdef NEW_ALARM_STYLE
		alert_push(CMD_ERR_CODE_ARG(1,arg1));
		#else
		//alert_push(YARN_CMD_ERROR, arg1);
		#endif
		return;
	}

				switch(arg1)
				{
				case 0:
						{
							
 						if((arg2 & 0x00ff)==(arch_board_id+0x10))
	    					{
              					
               					Message_Send_4halfword(YARN_STEP_CMD_RETURN_HEAD,emf_status[2],(arch_board_id+0x10)<<8,0);
	   					 }
	    						break;
						}
						
				case 1:
						{
						 	Exec_SET_DCT_sts(arg2,arg3);
	    						break;
						}
					
				case 2:
						{ 
							Exec_GET_Moto_Zero_Width(arg2);
	    						break;
						}
						
				case 3:
						{
							Exec_SET_Moto_Zero_Width(arg2,arg3);
	    						break;						

						}
						
				case 4:
						{
							if (!arch_check_power_isok(POWER_CTR_P_24))	
							{
								alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
								break;
							}
							Exec_Reset_step_moto_one(arg2,0);
							#ifdef TEST_DEBUG_CODE_ENABLE
							if (EMF_Operate_Reback)
							{	
								Message_Send_EMF_REBACK(cmd->cmd,arg1,arg2,0);
							}
							#endif
	    						break;	

						}
				case 5:
						{
							if (!arch_check_power_isok(POWER_CTR_P_24))	
							{
								alert_push(HEAD_POWER_ISOFF,(POWER_CTR_P_24<<8)|which_part_close_power);
								break;
							}       					
	   					 	Exec_Reset_step_moto_all(arg2,0);
							#ifdef TEST_DEBUG_CODE_ENABLE
							if (EMF_Operate_Reback)
							{	
								Message_Send_EMF_REBACK(cmd->cmd,arg1,arg2,0);
							}
							#endif
	    						break;	

						}
				case 6:
						{	
							//Exec_GET_version_EX(arg2);
							break;
						}
				case 7:{
							unsigned int step_mask = 0xFFFFFFFF;
							unsigned int yarnstep_active; 
							int step_no;
							step_no = (arg2 +1)>>1;
							if (step_no >4)
							{
								step_no = 4;
							}	
							yarnstep_active = ~(step_mask <<step_no);							
							
							StepMotor_Setup_Yarnstep_Active(yarnstep_active);
							#ifdef TEST_DEBUG_CODE_ENABLE
							if (EMF_Operate_Reback)
							{	
								Message_Send_EMF_REBACK(cmd->cmd,arg1,arg2,0);
							}
							#endif
					}
					break;
				case 8:
					{
							//StepMotor_Setup_Resolution(arg2);
					}
				
				break;
				case 9:
					{
						if (((arg2>0) &&(arg2<400))&&(Yarn_use_Step)) 
						{
							POS_Yarn_DO = arg2;
						}
					}
					break;
				case 0x0A:
					{
						if(Yarn_use_Step)	
						{
							if (((arg2 & 0x00ff)==(arch_board_id+0x10))) 
							{
								tx_data[0]=arch_get_yarnstep_sign_();
								tx_data[1]=(arch_board_id+0x10)<<8;
	              					//arch_SendMessage(1,&tx_data[0],1); 
								Message_Send_4halfword(YARN_STEP_CMD_RETURN_HEAD,tx_data[0],tx_data[1],0);	
	         					}
						}
						else   /*给个假数据*/
						{
							//Message_Send_4halfword(YARN_STEP_CMD_RETURN_HEAD,0,(arch_board_id+0x10)<<8,0);	
	         					
						}
						
					}
					break;

				case 0x0B:
					{
							Yarn_step_PWMDA_Set_val(arg2);
					}
					break;

				case 0x0C:
					{
							StepMotor_Set_Reset_Speed(arg2,MOTOR_TYPE_YARN);
					}
					break;
				case 0x0D:
					{
							StepMotor_Set_Speed(arg2,MOTOR_TYPE_YARN);
					}
					break;
				case 0x0E:
					{
							//Steps_lost_alarm = arg2 ;  //如果值为0的话，表示不报警，自动修正到正中间.
							Step_Motor_Set_Check_all(MOTOR_TYPE_YARN,arg2);
					}
					break;
				case 0x0F:
					{
							
							//Steps_Done_report_sts = arg2;
					}
					break;
				case 0x10:
					{
							Message_Send_4halfword(YARN_STEP_CMD_RETURN_HEAD,Yarn_use_Step?1:0,0,0);
					}
					break;
				/*2020-02-27 by hlc 慈星纱嘴电机用于纱嘴上下动作*/
				case 0x11:
					{
						Yarn_Step_Updown = arg2?1:0;
					}
					break;	
				/***************/	
				case 0x12:  /*LX纱嘴电机报警检查*/
					{
						Motor_Check_Zeroinput_and_alert((unsigned char)arg2,MOTOR_TYPE_YARN);
					}
					break;
				default :break;						
					
				}
	  		
}

#endif


int Exec_EMF_isr_EX(unsigned int which,unsigned int dctno,unsigned int sts)
{
	unsigned int emfid;
	
	switch (which)
	{
		case 0:
			emfid = dctno+12;		//纱嘴		
			arch_check_JQD_YARN_ACT_Bit |=(0x01<<1);
			break;
		case 1:
			emfid = dctno+0;		// 后床三角
			arch_check_JQD_YARN_ACT_Bit |=(0x01<<2);
			break;
		case 2:
			emfid = dctno+6;		// 前床三角
			arch_check_JQD_YARN_ACT_Bit |=(0x01<<2);
			break;
		default:
			return -1;
			//break;
	}

	Exec_EMF(emfid, sts,1,0);

	if(emf_work_mode & (0x1 << emfid)) 
	{
		/*IO模式，不用自检*/
		return 0;		
	}
	else
	{		
		return 1;
	}
}


unsigned int Exec_EMF_GET_EX(unsigned int which,unsigned int dctno,unsigned int sts)
{
	unsigned int emfid;
	
	switch (which)
	{
		case 0:
			emfid = (sts?1:0)*MAX_YARN+dctno+32;		//纱嘴		
			//arch_check_JQD_YARN_ACT_Bit |=(0x01<<1);
			break;
		case 1:
			// 后床三角
			//arch_check_JQD_YARN_ACT_Bit |=(0x01<<2);
			emfid = (sts?1:0)*MAX_ACTEMF+dctno+32+16;	
			break;
		case 2:
			// 前床三角
			emfid = (sts?1:0)*MAX_ACTEMF+dctno+6+32+16;	
			//arch_check_JQD_YARN_ACT_Bit |=(0x01<<2);
			break;
		default:
			emfid =0xffffffff;
			break;
	}
	//Exec_EMF(emfid, sts);
	if (emfid<MAX_DC24_DATA_COUNT)
	{
		return DC24_PN_data_Arry[emfid];
	}else
return 0xffff;
	
	
}


void Exec_EMF_isr(unsigned int which,unsigned int dctno,unsigned int sts,unsigned short times_ms)
{
	unsigned int emfid;
	
	switch (which)
	{
		case 0:
			emfid = dctno+12;		//纱嘴		
			break;
		case 1:
			emfid = dctno+0;		// 后床三角
			break;
		case 2:
			emfid = dctno+6;		// 前床三角
			break;
		default:
			return;
			//break;
	}
	Exec_EMF(emfid, sts,0,times_ms);
}



void Get_base_DC24_zero()
{
	DC24_N_CURR_Zero = arch_get_ad_base_cur_zero(0);
	DC24_P_CURR_Zero = arch_get_ad_base_cur_zero(1);
}





void Exec_alarm_shock(unsigned int whichdata,unsigned int CNdata,unsigned int whichshock)
{
	if (!arch_is_shock_board())
		return;

	switch (whichdata)
	{
		case 0:
			if (CNdata)
			{
				arch_shock_reset(whichshock);	
			}			
			break;
		case 1:
			{
				arch_shock_activate(CNdata,whichshock);
			}
			break;
		case 2:
			{
				if (CNdata<=100)		//(CNdata>=0) &&
				{
					extern void alert_set_shock_PWM(int shock_sensitivity,int whichshock);
					/*	0-100(灵明度用高到低)对应的PWM占空比250-750.对应的电压值2.5V-1.0V.
					*基准电压为2.5V，所以越接近2.5V就越容易报警。差值越大越不报警
					*
					*/
					alert_set_shock_PWM(300+5*(100-CNdata),whichshock);//150+3
				}
			}
			break;
		default:
			break;
	}

}

void Exec_alarm_check_start(unsigned int ischeckdir)
{
		if(ischeckdir & 0x01 )
		{
			if(feet_enable)
				return;
			if((arch_Get_DirStatus() & 0x3) != 0) 
			{
				#ifdef NEW_ALARM_STYLE
				alert_push(DIR_CHECK_ERR_CODE_ARG( arch_Get_DirStatus()&0x00ff));
				#else
				alert_push(DIR_ERR, arch_Get_DirStatus());
				#endif
			}
			else 
			{
				check_dir_1count = 0;
				check_dir_2count = 0;
				check_dir_still_zero = 1;
			}
		}
}


void Exec_alarm_check_end(unsigned int maskdata,unsigned int data_a)
{
		if(maskdata & 0x01)
			{
					if(!feet_enable)
					{	//break;
						if((arch_Get_DirStatus() & 0x3) != 0)
						{
							#ifdef NEW_ALARM_STYLE
							alert_push(DIR_CHECK_ERR_CODE_ARG( (arch_Get_DirStatus()&0xff)|(0x01<<8)));
							#else
							alert_push(DIR_ERR, arch_Get_DirStatus() << 4 );
							#endif
						}
						else
						{
							if(check_dir_still_zero) 
							{
								int ret = 0;
								check_dir_still_zero = 0;
							
								if (check_dir_1count < data_a)
									ret = 1;
								if (check_dir_2count < data_a)
									ret = 2;

								if(ret)
									#ifdef NEW_ALARM_STYLE
									alert_push(DIR_CHECK_ERR_CODE_ARG(( arch_Get_DirStatus()&0xff)|(0x02<<8)));
									#else
									alert_push(DIR_ERR, (ret&0xff) << 8);
									#endif
							}
						}
					}
				}
//#endif				
				if(maskdata & 0x02) 
				{
					check_sinker_done();
				}

				if(maskdata& 0x04)
				{
					check_sti_done(0);
				}
				if(maskdata & 0x08)
				{
					check_sti_done(1);
				}

				if(maskdata& 0x10)
				{
					unsigned int yarn_st_maskdata=0xffff;
					if (data_a)
					{
						yarn_st_maskdata = 0x0000;

					}					
					{
						if((emf_status[EMF_INDEX_YARN] & 0xff) != (yarn_st_maskdata & 0xff))
						{
							#ifdef NEW_ALARM_STYLE
							alert_push(YARN_NOT_CLEAR_CODE_ARG( emf_status[EMF_INDEX_YARN]));
							#else
							//alert_push(YARN_STATE_NOT_CLEAR, emf_status[EMF_INDEX_YARN]);
							
							exec_EMF_YARN_((yarn_st_maskdata & 0xff));

							
							#endif
						}
					}
				}

				#if 1
				if (maskdata & 0x20)  //check ACTemf
				{
					if ((emf_status_isdone & 0xFFF) !=0)
					{
						#ifdef NEW_ALARM_STYLE
						alert_push(ACT_NOT_CLEAR_CODE_ARG( (emf_status_isdone & 0xFFF));
						#else
						//alert_push(ACT_STATE_NOT_CLEAR,  (emf_status_isdone & 0xFFF));
						Message_send_log_ecode(ACT_STATE_NOT_CLEAR,(emf_status_isdone & 0xFFF),0);//alert_push
						#endif
					}					
						
				}

				#endif
				
}

extern volatile unsigned char DC24_Start_check;

void Exec_Check_Resistance_Each()
{
	unsigned int sts=0;
	if (!need_check_rs_each) return;
	if (!DC24_Start_check) //说明采集完成了
	{
		test_delay_2ms?wait_ms(test_delay_2ms): wait_us(50);
//		wait_ms(test_delay_2ms);
		need_check_rs_each--;
		if (need_check_rs_each==0)
		{
			arch_send_Operate_data();
			need_check_rs_each=0;
		}
		else
		{	
			unsigned int ischeckjqd=0;
			unsigned int jqdno_loc,blade_loc;

			//Message_Send_4halfword(0xFFFF,nowtestemfindex,0,0xEEEE);

			if(nowtestemfindex<32)
			{
				ischeckjqd =1;	
				jqdno_loc = nowtestemfindex>>3;
				blade_loc= nowtestemfindex&0x07;
			}
			#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
			if(nowtestemfindex>=72)	
			{
				ischeckjqd =1;				
				jqdno_loc = (nowtestemfindex-72)>>1;
				blade_loc= MAX_BLADE_HARDWARE_8+((nowtestemfindex-72) & 0x01);
			}			
			#endif
			if (ischeckjqd)
			{
				sts = jqd_status[jqdno_loc]&(0x01<<(blade_loc))?1:0;
				if(test_delay_2ms)
					sts =!sts;
				Exec_Jacquard_ex(jqdno_loc, blade_loc,sts,0);
			}
			else
				if (nowtestemfindex<56)
				{	
					unsigned int emfid=0;
					unsigned int emfidx=0;
					
					emfid = (nowtestemfindex-32)>>3;
					emfidx=(nowtestemfindex-32)&0x07;
					sts=emf_status[emfid?(emfid-1):2]&(0x01<<(emfidx&0x07))?0:1;
					Exec_EMF_isr_EX(emfid,emfidx,sts);	

				}
				
		}
		

		//arch_test_send_all_data_toshow();
	}
}


void Exec_Check_Resistance_branch()
{
	static unsigned int whichid=0;
	static unsigned int casestep=0;
	static unsigned char doitagain=0;
	unsigned int sts=0;

	
	if (!need_check_rs) return;

	if (need_check_rs&0x01)		// 选针正
	{
		sts =0;		
		switch (casestep)
		{
		case 0:
			{
				unsigned int j_,b_;
			#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
			if(whichid>=32)
			{
				j_=(whichid-32)>>1;
				b_=(whichid-32)&0x01 + MAX_BLADE_HARDWARE_8;
			}
			else
			#endif
			{
				j_=whichid>>3;
				b_=whichid&0x07;
			}
			sts = jqd_status[j_]&(0x01<<b_)?1:0;
			Exec_Jacquard_ex(j_,b_,sts,0);
			casestep++;
			}
			break;
		case 1:
			if ((DC24_Start_check)||(whichOverload_in_progress_mask&0x03))   //说明采集中，还没完成,或者过流中，还没过流检测完毕
			{
				break;
			}	
			else
			{
				whichid++;
				casestep++;
				wait_us(50);
			}	
			break;
		case 2:
			{
				unsigned char maxblade_loc =32;
				#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
				maxblade_loc = sys_max_blade_phi <<2 ;// 4 个选针器
				#endif
				
				if (whichid>=maxblade_loc)
				{
					if (!doitagain)
					{
						doitagain =1;
						whichid =0;
						casestep =0;
					}
					else
					{
						need_check_rs &=~((unsigned char)0x01);
						whichid =0;
						casestep =0;
						doitagain =0;
					}
				}
				else
					casestep=0;
			}
			break;
		default:
			break;

		}
		goto Check_is_end;
			
	}
	if (need_check_rs & 0x02)   	//纱嘴负两厢
	{	
		if (Yarn_use_Step)
		{
			need_check_rs &=~((unsigned char)0x02);
			goto Check_is_end;
		}
		
		switch (casestep)
		{
		case 0:
			sts = (emf_status[EMF_INDEX_YARN]>>(whichid&0x07))&0x01;
			if (Exec_EMF_isr_EX(0,(whichid&0x07),sts?0:1)==0)
			{
				whichid++;
			}
			else
			{
				casestep++;
			}	
			break;
		case 1:
			if ((DC24_Start_check)||(whichOverload_in_progress_mask&0x03))   //说明采集中，还没完成,或者过流中，还没过流检测完毕
			{
				break;
			}	
			else
			{
				whichid++;
				casestep++;
				wait_us(50);
				//wait_us(50);
				//wait_us(50);
				//wait_us(50);
				//wait_us(50);
			}	
			break;
		case 2:
			if (whichid>=16)
			{
				need_check_rs &=~((unsigned char)0x02);
				whichid =0;
				casestep =0;
			}
			else
				casestep=0;
			break;
		default:
			break;

		}
		goto Check_is_end;
			
	}

	if (need_check_rs & 0x04)   	//三角正两厢
	{	
		switch (casestep)
		{
		case 0:
			sts = (emf_status[((whichid / 6)&0x01)]>>(whichid % 6))&0x01;
			if (Exec_EMF_isr_EX(((whichid / 6)&0x01) +1,(whichid % 6),sts?0:1)==0)
			{
				whichid++;	
			}
			else
			{
				casestep++;
			}
			break;
		case 1:
			if ((DC24_Start_check)||(whichOverload_in_progress_mask&0x03))   //说明采集中，还没完成,或者过流中，还没过流检测完毕
			{
				break;
			}	
			else
			{
				whichid++;
				casestep++;
				wait_us(50);
			}	
			break;
		case 2:
			if (whichid>=24)
			{
				need_check_rs &=~((unsigned char)0x04);
				whichid =0;
				casestep =0;
			}
			else
				casestep=0;
			break;
		default:
			break;

		}
		goto Check_is_end;
			
	}


//
	if (need_check_rs&0x08)		// 选针负
	{
		need_check_rs &=~((unsigned char)0x08);
		goto Check_is_end;
	}
#if 0
	if (need_check_rs&0x08)		// 选针负
	{
		sts =1;
		switch (casestep)
		{
		case 0:
			Exec_Jacquard_ex((whichid>>3),(whichid&0x07),sts,0);
			casestep++;
			break;
		case 1:
			if (DC24_Start_check)   //说明采集中，还没完成
			{
				break;
			}	
			else
			{
				whichid++;
				casestep++;
				wait_us(50);
			}	
			break;
		case 2:
			if (whichid>=32)
			{

				if (!doitagain)
				{
					doitagain =1;
					whichid =0;
					casestep =0;
				}
				else
				{
				need_check_rs &=~((unsigned char)0x08);
				whichid =0;
				casestep =0;
				doitagain =0;
				}
			}
			else
				casestep=0;
			break;
		default:
			break;

		}
		goto Check_is_end;
			
		}
#endif	
Check_is_end:	
	if (!need_check_rs)  //全部检查完毕
	{
		Message_Send_4halfword(0x0f01,0,0,0);
		whichid =0;
		casestep =0;
		doitagain =0;
	}
}

#ifndef ECODE_USE_MT6813_PWM

unsigned short send_check_mainloop_and_01ms(unsigned char jqdno,unsigned char blandno)
{
	//unsigned int getsys=Get_systick();
	extern u16 Get_systick(void);
	extern unsigned int Get_systick_32(void);
	unsigned int main_loop_abs=0;
	unsigned int ms01_abs=0;
	
	unsigned short ret=0;
	Message_Send_4halfword(0xf2|(0xF0<<8), jqdno|(blandno<<8),jqd_blade_sts_check_mainloop[jqdno][blandno]&0xffff,jqd_blade_sts_check_mainloop[jqdno][blandno]>>16);
	Message_Send_4halfword(0xf2|(0xF1<<8), jqdno|(blandno<<8),main_loop_cnt & 0xffff,main_loop_cnt>>16);
	Message_Send_4halfword(0xf2|(0xF2<<8), jqdno|(blandno<<8),jqd_blade_sts_check_01ms[jqdno][blandno]&0xffff,jqd_blade_sts_check_01ms[jqdno][blandno]>>16);
	Message_Send_4halfword(0xf2|(0xF3<<8), jqdno|(blandno<<8),Get_systick(),ms01_loop_cnt);
	//Massage_Send_Alert_log(0xf2|(0xF2<<8), jqdno,blandno,0);

	main_loop_abs = abs(main_loop_cnt-jqd_blade_sts_check_mainloop[jqdno][blandno]);
	if (main_loop_abs >0x7FFFFFFF)
		main_loop_abs = 0xFFFFFFFF - main_loop_abs;

	ms01_abs = abs(Get_systick_32()-jqd_blade_sts_check_01ms[jqdno][blandno]);
	if ((ms01_abs >0x7FFFFFFF))
		ms01_abs = 0xFFFFFFFF - ms01_abs;
	
	
	if (main_loop_abs<=3)
	{
		ret= 1;
	}
	if (ms01_abs<=50)
	{
		ret |=0x02;	
	}
	return ret;
	
	
}

void Set_mainloop_01ms(unsigned char jqdno,unsigned char blandno)
{
	extern unsigned int Get_systick_32(void);
	jqd_blade_sts_check_01ms[jqdno][blandno]=Get_systick_32();
	jqd_blade_sts_check_mainloop[jqdno][blandno]=main_loop_cnt;
}

unsigned int Get_main_loop_cnt()
{
return main_loop_cnt;
}


#endif

void Command_loop()
{
	MASSAGE_TYPE *cmd;
	NEWCMDMASSAGE newcmd;
	//extern uint16_t Get_ADC_data_channel(unsigned int);
	main_loop_cnt++;
	while(1) {
		cmd = Message_Pop();
		if(cmd == NULL)
			break;

		newcmd.cmd_type = cmd->cmd;
		newcmd.cmd_data = cmd->arg1;
		newcmd.arg1= cmd->arg2;
		newcmd.arg2 = cmd->arg3;
		newcmd.arg3 = cmd->arg4;
		newcmd.arg1_l = newcmd.arg1 & 0xff;
		newcmd.arg1_h = (newcmd.arg1 >>8) & 0xff;
		newcmd.arg2_l = newcmd.arg2 & 0xff;
		newcmd.arg2_h = (newcmd.arg2 >>8) & 0xff;
		newcmd.arg3_l = newcmd.arg3 & 0xff;
		newcmd.arg3_h = (newcmd.arg3 >>8) & 0xff;
		
		if(work_mode) {
			cmd->cmd -= 0x80;
		}

		

		//myprintf("Get_CANdta:cmd[%0x %0x];arg1[%0x %0x];arg2[%0x %0x];arg3[%0x %0x];[%d];[%d]; \n\r",
			//newcmd.cmd_type,newcmd.cmd_data,newcmd.arg1_l,newcmd.arg1_h,newcmd.arg2_l,newcmd.arg2_h,newcmd.arg3_l,newcmd.arg3_h,Get_ADC_data_channel(0),Get_ADC_data_channel(1));
		//Message_Send_4halfword(newcmd.cmd_data|(newcmd.cmd_type<<8),newcmd.arg1,newcmd.arg2,newcmd.arg3);
		switch (newcmd.cmd_type & 0x7F)
		{
			case CMDTYPE_UNDEFINE:
				Command_exec_undefine(&newcmd);
				break;
			case CMDTYPE_PERIPHERAL:
				Command_exec_peripheral(&newcmd);
				break;
			case CMDTYPE_SYSSET:
				Command_exec_sysset(&newcmd);
				break;
			case CMDTYPE_MOTOR_DENSITY:
				Command_exec_motor_density(&newcmd);
				break;
			case CMDTYPE_MOTOR_OTHER:
				Command_exec_motor_other(&newcmd);
				break;
			case CMDTYPE_DCT:
				Command_exec_dct(&newcmd);
				break;
			case CMDTYPE_XZQ_EX:	
				{
					unsigned char idx_now=(newcmd.cmd_data & 0x70)>>4;
					unsigned char needbreak=0;

					if(!(newcmd.cmd_type & 0x80))
					{
						//needbreak					
						if(!JQD_Return_reset)
						{
							JQD_Return_reset=1;

							//JQD_cmd_index = idx_now;//(newcmd.cmd_data & 0x70)>>4;
							
						}
						else
						{
							if(JQD_cmd_index ==idx_now)
							{
								//
								can_jqdcmd_repeat_count++;
								alert_push(JQD_CMD_REPEATED_ERROR,JQD_cmd_index|(idx_now<<8));
								needbreak=1;
							}
							else if(JQD_cmd_index_next!=idx_now){
								alert_push(JQD_CMD_REPEATED_ERROR,JQD_cmd_index_next|(idx_now<<8));
								//needbreak=1;
								//JQD_cmd_index = idx_now;
								
							}						
						}
						JQD_cmd_index = idx_now;
						JQD_cmd_index_next = JQD_cmd_index+1;
						if(JQD_cmd_index_next>0x07)
						{
							JQD_cmd_index_next=0;
						}

						if (needbreak)
						{
							break;
						}
						
					}	
					newcmd.cmd_data &= 0x8F;/*clear bit4.5.6*/
				}
			case CMDTYPE_XZQ:
				Command_exec_xzq(&newcmd);
				break;
			case CMDTYPE_ALARM:
				Command_exec_alarm(&newcmd);
				break;
			case CMDTYPE_OTHERSET:
				Command_exec_otherset(&newcmd);
				break;
			case CMDTYPE_CHECKONLINE:
				Command_exec_checkonline(&newcmd);
				break;
			case CMDTYPE_DATASTORE:
				Command_exec_data_store(&newcmd);
				break;
			case CMDTYPE_TEST_CODE:
				Command_exec_Testcode(&newcmd);
				break;
			default :
				break;
		}
		#ifndef LX_ACT_SPECIAL
		#ifdef FOR_BW_DM_SLOWLYRUN
		Alert_Clear();   //Debug_hlc
		#endif
		#else
		Alert_Clear();   //Debug_hlc
		#endif
	}
}

void reg_init()
{
	//unsigned int i;

	Alarm_Init();
	
	JQD_Data_Init();

	Jacquard_Init();
	
	EMF_Init();

	alert_init();
	
	Message_Init();
	
	StepMotor_Init();

	Overload_Init();

	//StepMotor_Poll();

}

extern void EXTI_CHECK_iscomming(void);


//#define QL_2A_
//#define ACT_1200MA_

int main()
{
 	extern unsigned long test_count_whichdo[];
	extern void hook_can_tx_2ms_isr(void);
 	int i;
	BootVer_int =0;
	for (i=0;i<4;i++)
	{
		if (BootVer[i]!=i)
		break;	
	}
	if (i==4)
	{
		BootVer_int =BootVer[4];	
	}
	
	
	DA_is_out =0;
	//SCB->SHCSR |= 0x00007000;
	arch_init();

	wait_us(10);
	wait_ms(1000);  //wait fro 1s fro cpld init
#if 1
	 build_time = Get_Build_Time(NULL);
#endif

#ifdef ECODE_USE_MT6813_PWM
	Set_is_MT6813PWM_Mode();
	if(!Check_is_MT6813PWM_Mode())
	{
		Gpio_PB4_config_def();	
	}
#endif
	////while(1) Keydog_Read();
//#ifndef MLOCK
	arch_LED_Setup(99/*99*/);

#ifdef TEST_SELF
	Test_prog();
#endif

#ifdef ENCODER_SUPPORT
	if (arch_is_Ecode_board())
	{
	#if ENCODER_DMA_SUPPORT
		spi1_init();
	#endif /* ENCODER_DMA_SUPPORT */
		Encoder_Init(StepMotor_Count_MAX);
	}
#endif
	
#if 0
	Shell_Run();
#endif

//myprintf("\r\n ADC_start_main()...\n\r");

ADC_start_main();

//myprintf("\n DAC_SetVoltage_channel1\n\r");
#if 0

#define DEFAULT_DA_CURRENT_DEN_MA		(800)	/*度目默认电流值*/
#define DEFAULT_DA_CURRENT_SK_MA			(800)
#define DEFAULT_DA_CURRENT_ACT_MA		(800)
#define DEFAULT_DA_CURRENT_YARN_MA		(1500)
#define DEFAULT_DA_CURRENT_LIFT_MA		(800)
#define DEFAULT_DA_CURRENT_FEET_MA		(800)


#endif

	
	DAC_SetVoltage_channel1(stepmotor_current[MOTOR_TYPE_DENSITY-1]);
	DAC_SetVoltage_channel2(stepmotor_current[MOTOR_TYPE_ACTION-1]);

	#ifdef QL_2A_
	stepmotor_current[MOTOR_TYPE_DENSITY-1] = 2000;
	DAC_SetVoltage_channel1(2000);
	#endif
	#ifdef ACT_1200MA_
	stepmotor_current[MOTOR_TYPE_ACTION-1] = 2000;
	DAC_SetVoltage_channel2(1200);
	#endif
	DA_is_out =1;

	#ifdef E480_BOARD_V10
	{
		//alert_set_Step_PWM_A(800);
		if (Yarn_use_Step)
		{
			Yarn_step_PWMDA_Set_val(stepmotor_current[MOTOR_TYPE_YARN-1] );
		}
		#ifdef E490_V10_BOARD
		if (TZ_use_Step)
		{
			LIFT_step_PWMDA_Set_val(stepmotor_current[MOTOR_TYPE_LIFT-1]);
		}

		if (SK_use_Step)
		{
			SKER_step_PWMDA_Set_val(stepmotor_current[MOTOR_TYPE_SKINER-1]);
		}

		#endif
		
		#ifdef E499_BOARD_SUPPORT_
		if (arch_is_EMF_2_SK_board())
		{
			arch_set_Step_SK_A(stepmotor_current[MOTOR_TYPE_SKINER-1]);
		}
		#endif

		#ifdef E692_STEPMOTOR_EX
		if(Step_use_exBoard)
		{
			EX_step_PWMDA_Set_val(0,stepmotor_current[MOTOR_TYPE_SKINER-1]);
			EX_step_PWMDA_Set_val(1,stepmotor_current[MOTOR_TYPE_SKINER-1]);
		}
		#endif
	}
	#endif
#ifdef NEW_ALARM_STYLE
	Set_stepmotor_alarm_id( arch_Get_ID(),sys_type);
#endif

	//Exec_alarm_shock(1,1,0);
	main_loop_cnt =0;

#if 0
	{
		unsigned int firstint=0;
		firstint = *(unsigned int *)0x08000000;
		Message_Send_4halfword(0xAAAA,firstint & 0xffff,(firstint>>16)&0xffff,arch_Get_ID());
	}
	//#endif

	{
		unsigned short firstint=0;
		unsigned short firstint_1=0;
		
		firstint = *(unsigned short *)0x08004000;
//		Message_Send_4halfword(0xAAAA,firstint & 0xffff,(firstint>>16)&0xffff,arch_Get_ID());
		*(unsigned short *)0x08004000 =0xAABB;
		firstint_1 = *(unsigned short *)0x08004000;

		Message_Send_4halfword(0xAAAA,firstint ,0,firstint_1);

		while(1)
		{
			int i;
			unsigned int firstaddress=0x08004000;
			for (i=0;i<1024;i++)
			{
				*(unsigned short *)firstaddress =0xAABB;
				firstaddress +=2;
			}
		}
		//firstint_1 = *(unsigned short *)0x08004000;
		//Message_Send_4halfword(0xAAAA,firstint ,0,firstint_1);
		

		
	}
	#endif
	#ifdef ECODE_USE_MT6813_PWM
	Message_Send_4halfword(Check_is_MT6813PWM_Mode()?0x0200:0x0100,Yarn_use_Step,Step_use_exBoard,arch_Get_ID());
	
	#else
	Message_Send_4halfword(0x0100,Yarn_use_Step,Step_use_exBoard,arch_Get_ID());
	#endif

	#ifdef DEBUG_MT6813_PWM

	arch_LED_Off();
	#endif
	
	while (1)
	{
		//unsigned int first_tc_;

		//first_tc_ = Get_now_time_counter();
		
		Command_loop();  
		//Get_main_loop_time_us(0,first_tc_);

		Alert_Poll();	
		//Get_main_loop_time_us(1,first_tc_);
		
		//dbgu_poll();
		//Shell_Poll();	
		
		Overload_Poll();
		//Get_main_loop_time_us(2,first_tc_);
		// by xhl 2011/04/27
		
		if (arch_is_Ecode_board())
			StepMotor_Poll();
		//Get_main_loop_time_us(3,first_tc_);

		#ifdef ECODE_USE_MT6813_PWM
		tim3_pwm_updata_loop();

		tim3_pwm_timeout_loop();

		#endif
		alert_push_again();
		//Get_main_loop_time_us(4,first_tc_);

		

		#ifdef JQD_NEXT_DO_FIFO
		arch_do_next_jqd();	
		
		#endif

		EXTI_CHECK_iscomming();
		//Get_main_loop_time_us(5,first_tc_);

		Exec_Check_Resistance_branch();
		//Get_main_loop_time_us(6,first_tc_);
		Exec_Check_Resistance_Each();
		//Get_main_loop_time_us(7,first_tc_);
//		check_can_send_error();
		//Get_main_loop_time_us(8,first_tc_);
		//check_step_goon();
		check_step_motor_loop_();
		//Get_main_loop_time_us(9,first_tc_);
		send_check_online_f();
		//Get_main_loop_time_us(10,first_tc_);

#ifdef DEBUG_DATA_STORE
		Debug_data_store_self();
#endif

	//	Get_main_loop_time_us();
	#ifdef CAN_SEND_ISR_ENABLE
		hook_can_tx_2ms_isr();
	#endif
		check_fan_ctr();
		Checkin_need_send_data(CMD_RETURN_CHECKIN_SET);
		//Step_debug_goto_();
		Check_Can_ERR_loop();
		arch_get_char_loop();
		Motor_bind_exec_loop();

#ifdef READ_FLASH_DATA_FOR_BANDING
		read_flash_data();
#endif
		arch_SendMessage_with_main();

		#ifdef DEBUG_MT6813_PWM
		if(Check_Debug_Led_next_Off())
		{
			arch_LED_Off();
		}
		#endif
		
	}
}

