/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_flash.h"
#include "misc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_exti.h"
#include "platform_config.h"
#include "config.h"
#include "alert.h"
#include "step.h"
#include "arch.h"
#include "Eeprom.h"
#include "massage.h"
#include "command.h"

#define dprintf myprintf 

//volatile unsigned short writeflash_index=0;
//#define _ZERO_FILTER_SUPPORT

#ifdef _ZERO_FILTER_SUPPORT
#define _ZERO_INPUT_COUNT	20
#define _ZERO_INPUT_FILTER	4
static char _step_zero_filter_ticks[_ZERO_INPUT_COUNT];
static char _step_zero_shadow[_ZERO_INPUT_COUNT];
static char _step_zero_state[_ZERO_INPUT_COUNT];
#endif

static int DC24_Check_Timer_Start = 0;


typedef unsigned long  DWORD;
typedef unsigned char  BYTE;
//typedef unsigned long  uint32_t;
typedef void (*pFunction)(void);




struct OVERLOAD_ST_BITS {
   unsigned short rover		:1;		// 0: 标记当前是否处于过流状态
   unsigned short rchk		:1;		// 1: 标记当前是否处于复查状态
   unsigned short ralert		:1;		// 2: 标记是否需要报警1-需要
   unsigned short rpower	:1;		// 3: 标记是否需要上电
   unsigned short rsvd		:4;		// reserved
   unsigned short index		:8;		// 序号
};

union OVERLOAD_ST_REG {
   uint16_t	all;
   struct OVERLOAD_ST_BITS bit;
};


typedef struct {	
	union OVERLOAD_ST_REG st;
	unsigned int    first_overload_time;			/* 第一次过流时间,用于计算什么时候开始复查*/
	unsigned int 	rcheck_time_out;			/* 第一次过流时间,到复查之间的时间参数(200ms) */
	//unsigned int 	first_stop_time;				/*第一次获取到电机停下的时间(250us为单位)*/
	unsigned int    overload_cnt;				/*短时间内总的过流次数*/
	unsigned int    Max_overload_cnt;			/*短时间内允许的过流次数*/
	unsigned int    Last_power_time;			/*上一次的上电时间,和本次的过流时间比较，时间很短需要计次*/
	//unsigned int    max_overload_time;			/* */	
	//unsigned int 	check_timeout;   			/*当电机停下之后这么长时间，编码器不检查*/
}Overload_t;

volatile unsigned char JQD_mode_is_UN=0;/*20190403 选针器线圈反过来*/



volatile unsigned short Main_Board_id_[6];
//volatile unsigned short Main_Board_id_timer[6];/*定时器里面用到的*/
volatile unsigned short Main_Board_id_flash[6];
volatile unsigned short Head_Board_id_[6];

volatile unsigned short Main_Head_Binding_flag;
volatile unsigned short Main_Head_Binding_is_fail=0;		/*表示当前状态为绑定失败的,默认成功的*/
volatile unsigned short Main_Head_is_lock=0;				/*表示当前机头锁定了*/
volatile unsigned char ack_ok=0;

volatile unsigned int wait_time_ms_for_check_binding=0;

volatile unsigned int checkin_time_delay2ms_set=0;/*20190301 by hlc 主动上报时间间隔*/	
volatile unsigned int checkin_time_delay2ms_delay=0;/*20190301 by hlc 主动上报时间间隔*/	
volatile unsigned int checkin_need_send=0;
volatile unsigned int checkin_index=0;			/*20190301 by hlc 主动上报ID号*/

volatile unsigned short head_tryout_time_par;
volatile unsigned short head_tryout_time_setflag;

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
volatile unsigned char sys_max_blade_phi=MAX_BLADE_HARDWARE;  /*初始化8或10，实际在软件初始化的时候根据具体的选针板类型来确定是8还是10*/
#endif



volatile DWORD step_speed[STEP_NUM];
volatile BYTE led_status = 0;
volatile BYTE led_disable = 0;
volatile DWORD led_delay = 0;
volatile DWORD led_delay_setup = 0;

volatile unsigned char which_part_close_power=0;

volatile unsigned int wait_time_ms = 0;
volatile unsigned int wait_time_us = 0;

volatile unsigned int wait_time_ms_test = 0;
volatile unsigned int basetime_tick = 0;



	volatile unsigned int DC24N_max=0;
	volatile unsigned int DC24P_max=0;



/*2019 01 28 更新纱嘴门限值12A，瞬间(10次)*/
volatile unsigned int DC24N_ALARM_Yarn_set=660;	//1320 mv=1320/55=24A  //1045/55=19A /*660/55=19A*/
volatile unsigned int DC24N_ALARM_set=1045;	//1320 mv=1320/55=24A  //1045/55=19A
volatile unsigned int DC24P_ALARM_set=1045;	//1320 mv=1320/55=24A   //1045/55=19A



volatile unsigned int arch_mode = 0;
volatile unsigned int arch_board_id = 0;

volatile unsigned char overload_check_on = 1;
volatile unsigned char overload_support = 1;
volatile unsigned char overload_status = 0;
volatile unsigned int overload_delay = 0;

volatile unsigned char system_power_24_isoverload	=0;

volatile unsigned char arch_check_JQD_YARN_ACT_Bit = 0;
volatile unsigned int DC24_CURR_A_overload_cnt[3] ={0,0,0};



extern volatile int feet_enable;
extern unsigned int error_active;
extern int can_receive_count;
extern volatile unsigned int JQD_halfwork_mode;		
extern volatile unsigned char Yarn_use_Step;

static int jqdbld_map[MAX_BLADE] = {0, 1, 2, 3, 4, 5, 6, 7,8,9};

static int overload_count = 0;

volatile unsigned char SYS_is_PowerOn=0;

volatile unsigned char sys_max_jqd_cnt = MAX_JACQUARD>>1; /*默认4个选针器*/

volatile unsigned int poweron_delay;

volatile unsigned char jqd_pwmsts[MAX_JACQUARD][MAX_BLADE];
volatile unsigned char jqd_clear_sts[MAX_JACQUARD][MAX_BLADE];

volatile unsigned char whichEMFisTest_sts[MAX_DC24_DATA_COUNT];


volatile unsigned int Tanzhenstatus;

volatile unsigned int shock_enable =0;
volatile unsigned int shock_is_created;
volatile unsigned int shock_reset_timer;

volatile unsigned int DCT_do_shock_disable_time=0;
volatile unsigned int shock_dct_is_enable=1;


unsigned short cpld_name=0;
unsigned short cpld_ver=0;
unsigned short cpld_expID=0;
unsigned int sys_type[2]={2,0};   // 系统类型(1-单系统，2-双系统，3-三系统,4--四系统)


volatile unsigned short board_hard_type=0;
volatile short N24_on_delay_1s=0; 
volatile short P24_on_delay_1s=0;


#ifdef ALARM_SHOCK_DOUBLE
volatile unsigned int shock_2_enable =0;
volatile unsigned int shock_2_is_created;
volatile unsigned int shock_2_reset_timer;

volatile unsigned int shock_double_enable=0;

#endif



extern volatile unsigned char TZ_use_Step;
extern volatile unsigned char SK_use_Step;

extern volatile unsigned int DC24_PN_CURR_Arry[];
extern volatile unsigned int DC24_PN_Count;
extern volatile unsigned int DC24_PN_Count_test;
extern volatile unsigned char DC24_PN_EnableBIT;
extern volatile unsigned char DC24_Start_check;

extern volatile unsigned char DC24_PN_whichNO ;
extern volatile unsigned char SYS_Test_Mode_ISPWM;

extern volatile unsigned int DC24_PN_CURR_Zero ;

extern volatile unsigned int DC24_P_CURR_Zero;
extern volatile unsigned int DC24_N_CURR_Zero;

extern volatile unsigned int DC24_PN_data_Arry[];

extern volatile unsigned int DC24_P_CURR_A ;
extern volatile unsigned int DC24_N_CURR_A ;
extern volatile unsigned int Vbat_CURR_V;
extern volatile unsigned int Vrf_CURR_V ;
extern volatile unsigned int TempS_CURR_V;

extern volatile unsigned int DC24_BASE_CURR_Arry[2][MAX_DC24_A_BASE_DATA_COUNT];

extern volatile unsigned char StepMotor_Count_MAX;

extern volatile unsigned char Step_use_exBoard;

#define CHECK_DC24_JQD	(0)
#define CHECK_DC24_YARN	(1)
#define CHECK_DC24_ACT	(2)
#define CHECK_DC24_JQD_P	(3)

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
#define CHECK_DC24_JQD_EX	(4)
#define CHECK_DC24_JQD_EX_P (5)

#endif

//取最后10个平均数
#define CHECK_DATA_COUNT_VAR	(5)   


#define CHECK_DATA_COUNT_VAR_EMF	(50)   

#ifdef LX_JQD_CHECK_ENABLE
volatile unsigned char jqd_blade_sts_check[MAX_JACQUARD][MAX_BLADE];
#endif

#ifdef YARN_OPT_CHECK_ENABLE
volatile unsigned char yarn_opt_sts_check[MAX_YARN];

#endif

#ifdef ACT_OPT_CHECK_ENABLE
volatile unsigned char act_opt_sts_check[MAX_ACTEMF];

#endif

volatile unsigned long dc_status =0;	

volatile unsigned long test_howlongtobreak=0;


 volatile unsigned long Overload_status_JQD_ACT_YARN = 0;
 
volatile unsigned char power_st = 0; /*bit0--负，bit1--正    (0-关，1-开)*/
volatile unsigned char power_istest = 0; /*bit0--负，bit1--正    (0-正常，1-测试)*/

 volatile unsigned int whichOverload_in_progress_mask=0;	/*bit 0--负24v,bit1--正24V*/

//#define MAX_OVERLOAD_CNT	9
enum {
	OVERLOAD_N_24 = 0,			/*系统负24伏过流*/
	OVERLOAD_P_24,				/*系统正24伏过流*/
	OVERLOAD_N_YARN_24,		/*纱嘴过流*/
	OVERLOAD_ACT1,			/*动作电磁铁1过流*/
	OVERLOAD_ACT2,			/*动作电磁铁2过流*/	
	OVERLOAD_XZQ1,			/*选针器1过流*/ 
	OVERLOAD_XZQ2,			/*选针器2过流*/
	OVERLOAD_XZQ3,			/*选针器3过流*/ 
	OVERLOAD_XZQ4,			/*选针器4过流*/
	OVERLOAD_YARN_F,			/*纱嘴AD采样芯片报警*/
	OVERLOAD_N_F,				/*系统负24伏采样芯片报警*/
	OVERLOAD_P_F,				/*系统正24伏采样芯片报警*/
	OVERLOAD_MAX,
};

Overload_t SYS_overload_c[OVERLOAD_MAX];
 

void GPIO_Configuration(void);
void USART_Configuration(void);
void CAN_Configuration(void);
void TIM_Init(void);
void FSMC_Init(void);
void FLASH_Unlock(void);
u16 EE_Init(void);

void EXTI_Config(void);
void EXTI_Enable(u32 EXTI_Line);
void EXTI_Disable(u32 EXTI_Line);
void reg_init(void);
void time_2ms_prog(void);
void time_250us_prog(void);
void wait_ms(unsigned int ms);
void wait_us(unsigned int us);
void Shell_Init(void);
void myprintf(const char *format, ...);
void IWDG_Init(void);
void IWDG_Enable(void);
void shell_hook_time(void);
void arch_delay_us(int us);
unsigned char arch_need_close711_alert();

unsigned int EMF_Get_ACT_Status(int grp);
unsigned int EMF_Get_YARN_Status(void);
unsigned int Jacquard_Get_Status(int jqdno);
unsigned long Get_alarm_status_withMASK(unsigned long maskdata,char *od);

void arch_Exti_enable(void);
void arch_Exti_disable(void);

void Clear_alarm_status_withMASK(unsigned long maskdata,char * od);

unsigned int arch_get_ad_cur_zero(unsigned int whichc);
void arch_test_send_all_data_toshow(int isfirst);
unsigned char arch_is_shock_board(void);
unsigned char arch_is_E490_board(void);
unsigned int arch_is_EMF_2_SK_board(void);
extern void Time_PWM_init(unsigned char isshockboard);
unsigned int arch_get_ticktime(void);
void Set_overloadalarm_withMASK(char *ot);
unsigned int arch_Get_Board_Type(void);

unsigned int arch_is_E690_board(void);

void Exec_main_head_board_id_init(void);

void Exec_Read_main_id_from_flash(void);

#ifdef READ_FLASH_DATA_FOR_BANDING
void arch_read_data_timer250us();
#endif

void GetMcu_IDCode(void)  ;

#ifdef YARN_ZERO_CHECK
void Yarn_zero_check_input_st(void);
#endif

#ifdef JQD_NEXT_DO_FIFO

#define JQD_FIFO_DEEP_MAX	12
JQD_BLADE_TYPE jqd_next_to_do_fifo[MAX_JACQUARD][JQD_FIFO_DEEP_MAX];
unsigned char jqd_fifo_rpt[MAX_JACQUARD];
unsigned char jqd_fifo_wpt[MAX_JACQUARD];
unsigned char jqd_timer_isbusy[MAX_JACQUARD];



int jqd_operate_fifo_cnt(unsigned char whichjqd)
{

	int count = jqd_fifo_wpt[whichjqd];
	count += JQD_FIFO_DEEP_MAX;
	count -= jqd_fifo_rpt[whichjqd];
	if(count >= JQD_FIFO_DEEP_MAX) {
		count -= JQD_FIFO_DEEP_MAX;
	}

	return count;
	
}
void jqd_operate_fifo_init()
{
int i,j;
for (i=0;i<MAX_JACQUARD;i++)
{
	for(j=0;j<JQD_FIFO_DEEP_MAX;j++)
	{
		jqd_next_to_do_fifo[i][j].jqd_no=0;
		jqd_next_to_do_fifo[i][j].bladeno=0;
		jqd_next_to_do_fifo[i][j].do_st =0 ;			
	}
	jqd_fifo_rpt[i]=jqd_fifo_wpt[i]=0;
		
}

}
int jqd_operate_fifo_push(unsigned char whichjqd,unsigned char bno,unsigned char bst)
{
	unsigned char wpt;
	if(jqd_operate_fifo_cnt(whichjqd) >= JQD_FIFO_DEEP_MAX - 1) {		
		return -1;
	}
	wpt = jqd_fifo_wpt[whichjqd];
	jqd_next_to_do_fifo[whichjqd][wpt].jqd_no= whichjqd;
	jqd_next_to_do_fifo[whichjqd][wpt].bladeno= bno;
	jqd_next_to_do_fifo[whichjqd][wpt].do_st = bst;	

	wpt ++;
	if(wpt >= JQD_FIFO_DEEP_MAX) {
		wpt -= JQD_FIFO_DEEP_MAX;
	}
	jqd_fifo_wpt[whichjqd] =wpt;
	return 0;	


}

#ifdef  JQD_NEXT_DO_FIFO
JQD_BLADE_TYPE *jqd_operate_fifo_pop(unsigned char whichjqd)
{
	if(jqd_operate_fifo_cnt(whichjqd) > 0) {
		unsigned char rptr = jqd_fifo_rpt[whichjqd];
		jqd_fifo_rpt[whichjqd] ++;
		if(jqd_fifo_rpt[whichjqd] >= JQD_FIFO_DEEP_MAX) {
			jqd_fifo_rpt[whichjqd] -= JQD_FIFO_DEEP_MAX;
		}
		return &jqd_next_to_do_fifo[whichjqd][rptr];
	}
	return NULL;
}

#endif


#endif


void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(ENABLE);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_3);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		/* PCLK2 = HCLK */
	
		RCC_PCLK2Config(RCC_HCLK_Div2); 
		RCC_PCLK1Config(RCC_HCLK_Div4); 
		

		/* PCLK1 = HCLK/2 */
		//RCC_PCLK1Config(RCC_HCLK_Div4);    //当这个值不为1的时候，作为时钟频率的话就是APB1频率的2倍

		/* ADCCLK = PCLK2/4 */
		//RCC_ADCCLKConfig(RCC_PCLK2_Div4);

		/* PLLCLK = 8MHz/8 * 240 /2 = 120 MHz */
	
		RCC_PLLConfig(RCC_PLLSource_HSE, 8,240,2,5);

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while((RCC_GetSYSCLKSource() >> 2) != RCC_SYSCLKSource_PLLCLK)
		{
		}
	}

	
}

void NVIC_Configuration(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
#ifdef  VECT_TAB_RAM  
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */ 
	#ifdef APPRUNNING_AFTERBOOT
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x20000);
	#else
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	
	#endif
#endif
	
#if 0
	//中断NVIC设置：允许中断，设置优先级 
	/* CAN */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);                             //写入设置 

	/* TIM */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);                             //写入设置   

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_Init(&NVIC_InitStructure);                             //写入设置   

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_Init(&NVIC_InitStructure);                             //写入设置   

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQChannel;
	NVIC_Init(&NVIC_InitStructure);                             //写入设置   

	// EXTI
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQChannel;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);  
#endif
}

u8 myStrCmp(char *s0, char *s1)
{
	for (; *s0 == *s1; ++s0, ++s1)
		if (*s0 == '\0')
			return 0;

	return (*(unsigned char *)s0 < *(unsigned char *)s1 ? -1 : +1);
}

void delay(volatile unsigned int us)
{
	while(--us);
}

void delayus()
{
	volatile int us = 1;
	while(--us);
}


unsigned int arch_Check_isYarn_step_withstepindex(unsigned int stepidex)
{
	#ifdef E480_BOARD_V10
	if ((stepidex>=8)&&(stepidex<12))
	{
		if (Yarn_use_Step)
		{
			return stepidex;
		}
		else
			return 0xFFFF;
	}
	#ifdef E490_V10_BOARD
	else if(stepidex>=12)
	{
		if ((TZ_use_Step) ||(Step_use_exBoard))
		{
			return stepidex;
		}
		else
			return 0xFFFF;
	}
	#endif
	else
	{
		return stepidex;
	}
	
	#endif
	//return stepidex;

}

#if 0
void arch_StepMotor_Enable_single(unsigned int whichstep)
{
	#ifdef E480_BOARD_V10
	return;
	#else
	void arch_StepMotor_Init_single();
	arch_StepMotor_Init_single();
	switch (whichstep)
	{
		case 0:
			MOTO1_ENABLE_OUTPUT(0);
			break;
		case 1:
			MOTO2_ENABLE_OUTPUT(0);
			break;
		case 2:
			MOTO3_ENABLE_OUTPUT(0);
			break;
		case 3:
			MOTO4_ENABLE_OUTPUT(0);
			break;
		case 4:
			MOTO5_ENABLE_OUTPUT(0);
			break;
		case 5:
			MOTO6_ENABLE_OUTPUT(0);
			break;
		default:
			break;
			
	}
	#endif

}

#endif

#define STEPMODE_FULL	0
#define STEPMODE_HALF	1
#define STEPMODE_QUARTER	2
#define STEPMODE_EIGHTH	3

void arch_StepMotor_Set_UMS(unsigned int mode,unsigned int mototype)  //这里需要根据mode 设置???
{

	if (arch_is_E490_board())
	{
		return;
	}
	
	switch (mototype)
	{
		case MOTOR_TYPE_DENSITY:
			{
				switch (mode)
				{
				 case STEPMODE_FULL:
				 	MOTO_DM_USM0(0);
					MOTO_DM_USM1(1);			 	

					break;
				 	
				 case STEPMODE_HALF:
				 	MOTO_DM_USM0(1);
					MOTO_DM_USM1(1);			 	

					break;
				 case STEPMODE_QUARTER:
				 	MOTO_DM_USM0(0);
					MOTO_DM_USM1(0);			 	

					break;
				 case STEPMODE_EIGHTH:
				 	MOTO_DM_USM0(1);
					MOTO_DM_USM1(0);			 	

					break;
				 	default:
						MOTO_DM_USM0(0);
						MOTO_DM_USM1(0);
						break;

				}
			}

			break;
		case MOTOR_TYPE_SKINER:
			{
				switch (mode)
				{
				 case STEPMODE_FULL:
				 	MOTO_SK_USM0(0);
					MOTO_SK_USM1(1);			 	

					break;
				 	
				 case STEPMODE_HALF:
				 	MOTO_SK_USM0(1);
					MOTO_SK_USM1(1);			 	

					break;
				 case STEPMODE_QUARTER:
				 	MOTO_SK_USM0(0);
					MOTO_SK_USM1(0);			 	

					break;
				 case STEPMODE_EIGHTH:
				 	MOTO_SK_USM0(1);
					MOTO_SK_USM1(0);			 	

					break;
				 	default:
						MOTO_SK_USM0(0);
						MOTO_SK_USM1(0);
						break;

				}
			}

			break;

		#ifdef E480_BOARD_V10

		case MOTOR_TYPE_ACTION:
			{
				switch (mode)
				{
				 case STEPMODE_FULL:
				 	MOTO_SJ_USM0(0);
					MOTO_SJ_USM1(1);			 	

					break;
				 	
				 case STEPMODE_HALF:
				 	MOTO_SJ_USM0(1);
					MOTO_SJ_USM1(1);			 	

					break;
				 case STEPMODE_QUARTER:
				 	MOTO_SJ_USM0(0);
					MOTO_SJ_USM1(0);			 	

					break;
				 case STEPMODE_EIGHTH:
				 	MOTO_SJ_USM0(1);
					MOTO_SJ_USM1(0);			 	

					break;
				 	default:
						MOTO_SJ_USM0(0);
						MOTO_SJ_USM1(0);
						break;

				}
			}
			break;
		#endif
		default:
			break;
	}
	
}

#define YARNSTEP_ENABLE		0
#define YARNSTEP_DISABLE	1


#ifdef E490_V10_BOARD
void arch_StepMotor_Enable(void)
{
	// by xhl 2012/08/07
	void arch_StepMotor_Init();
	arch_StepMotor_Init();
	MOTO1_ENABLE_OUTPUT(0);
	MOTO2_ENABLE_OUTPUT(0);
	MOTO3_ENABLE_OUTPUT(0);
	MOTO4_ENABLE_OUTPUT(0);
	MOTO5_ENABLE_OUTPUT(0);
	MOTO6_ENABLE_OUTPUT(0);
	MOTO7_ENABLE_OUTPUT(0);
	MOTO8_ENABLE_OUTPUT(0);
	if (Yarn_use_Step)
	{
		MOTO9_ENABLE_OUTPUT(YARNSTEP_ENABLE);
		MOTO10_ENABLE_OUTPUT(YARNSTEP_ENABLE);
		MOTO11_ENABLE_OUTPUT(YARNSTEP_ENABLE);
		MOTO12_ENABLE_OUTPUT(YARNSTEP_ENABLE);
	}
	if (TZ_use_Step)
	{
		MOTO13_ENABLE_OUTPUT(0);
		MOTO14_ENABLE_OUTPUT(0);
	}
	#ifdef E692_STEPMOTOR_EX
	if(Step_use_exBoard)
	{
		MOTO_EX1_ENABLE_OUTPUT(0);
		MOTO_EX2_ENABLE_OUTPUT(0);		
	}
	#endif
}

void arch_StepMotor_Disable(void)
{
	
	
	MOTO1_ENABLE_OUTPUT(1);
	MOTO2_ENABLE_OUTPUT(1);
	MOTO3_ENABLE_OUTPUT(1);
	MOTO4_ENABLE_OUTPUT(1);
	MOTO5_ENABLE_OUTPUT(1);
	MOTO6_ENABLE_OUTPUT(1);
	MOTO7_ENABLE_OUTPUT(1);
	MOTO8_ENABLE_OUTPUT(1);
	if (Yarn_use_Step)
	{
		MOTO9_ENABLE_OUTPUT(YARNSTEP_DISABLE);
		MOTO10_ENABLE_OUTPUT(YARNSTEP_DISABLE);
		MOTO11_ENABLE_OUTPUT(YARNSTEP_DISABLE);
		MOTO12_ENABLE_OUTPUT(YARNSTEP_DISABLE);
	}
	if (TZ_use_Step)
	{
		MOTO13_ENABLE_OUTPUT(1);
		MOTO14_ENABLE_OUTPUT(1);
		//MOTO15_ENABLE_OUTPUT(1);
		//MOTO16_ENABLE_OUTPUT(1);
	}
	#ifdef E692_STEPMOTOR_EX
	if(Step_use_exBoard)
	{
		MOTO_EX1_ENABLE_OUTPUT(1);
		MOTO_EX2_ENABLE_OUTPUT(1);		
	}
	#endif
	
}


#else
void arch_StepMotor_Enable(void)
{
	// by xhl 2012/08/07
	void arch_StepMotor_Init();
	arch_StepMotor_Init();
	//MOTOALL_RESET_OUTPUT(1);
	#ifdef E480_BOARD_V10
	MOTO1_8_ENABLE_OUTPUT(0);
	if (Yarn_use_Step)
	{
		if (arch_is_E490_board())
		{
			MOTO9_12_ENABLE_OUTPUT_E490(0);
		}
		else
		MOTO9_12_ENABLE_OUTPUT(0);
	}
	#ifdef E499_BOARD_SUPPORT_
	if (arch_is_EMF_2_SK_board())
	{
		MOTO13_ENABLE_OUTPUT(0);
		MOTO14_ENABLE_OUTPUT(0);
		myprintf("\r\n MOTO13_ENABLE_OUTPUT(0)...\n\r");		
	}
	#endif
	#else
	MOTO1_ENABLE_OUTPUT(0);
	MOTO2_ENABLE_OUTPUT(0);
	MOTO3_ENABLE_OUTPUT(0);
	MOTO4_ENABLE_OUTPUT(0);
	MOTO5_ENABLE_OUTPUT(0);
	MOTO6_ENABLE_OUTPUT(0);
	#endif
}

void arch_StepMotor_Disable(void)
{
	#ifdef E480_BOARD_V10
	MOTO1_8_ENABLE_OUTPUT(1);
	if (Yarn_use_Step)
	{
		if (arch_is_E490_board())
		{
			MOTO9_12_ENABLE_OUTPUT_E490(1);
		}
		else
		MOTO9_12_ENABLE_OUTPUT(1);
	}
	#ifdef E499_BOARD_SUPPORT_
	if (arch_is_EMF_2_SK_board())
	{
		MOTO13_ENABLE_OUTPUT(1);
		MOTO14_ENABLE_OUTPUT(1);
			
	}
	#endif
	#else
	MOTO1_ENABLE_OUTPUT(1);
	MOTO2_ENABLE_OUTPUT(1);
	MOTO3_ENABLE_OUTPUT(1);
	MOTO4_ENABLE_OUTPUT(1);
	MOTO5_ENABLE_OUTPUT(1);
	MOTO6_ENABLE_OUTPUT(1);
	#endif
	//MOTOALL_RESET_OUTPUT(0);
}

#endif

void arch_StepMotor_Enable_onestepmoto(unsigned int whichstep)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(whichstep);
	switch (stepindex)
	{
		case 0:		
		MOTO1_ENABLE_OUTPUT(0);
		break;
		case 1:
		MOTO2_ENABLE_OUTPUT(0);
		break;
		case 2:
		MOTO3_ENABLE_OUTPUT(0);
		break;
		case 3:
		MOTO4_ENABLE_OUTPUT(0);
		break;
		case 4:
		MOTO5_ENABLE_OUTPUT(0);
		break;
		case 5:
		MOTO6_ENABLE_OUTPUT(0);
		break;
		case 6:		
		MOTO7_ENABLE_OUTPUT(0);
		break;
		case 7:
		MOTO8_ENABLE_OUTPUT(0);
		break;
		case 8:
		MOTO9_ENABLE_OUTPUT(0);
		break;
		case 9:
		MOTO10_ENABLE_OUTPUT(0);
		break;
		case 10:
		MOTO11_ENABLE_OUTPUT(0);
		break;
		case 11:
		MOTO12_ENABLE_OUTPUT(0);
		break;
		case 12:		
		MOTO13_ENABLE_OUTPUT(0);
		break;
		case 13:
		MOTO14_ENABLE_OUTPUT(0);
		break;
		#ifdef E692_STEPMOTOR_EX			
			case 14:
				MOTO_EX1_ENABLE_OUTPUT(0);
				break;
			case 15:
				MOTO_EX2_ENABLE_OUTPUT(0);
				break;		
		#else
			case 14:
			MOTO15_ENABLE_OUTPUT(0);
			break;
			case 15:
			MOTO16_ENABLE_OUTPUT(0);
			break;
		#endif
		default:
			break;
		}

}

void arch_StepMotor_Disable_onestepmoto(unsigned int whichstep)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(whichstep);
	switch (stepindex)
	{
		case 0:		
		MOTO1_ENABLE_OUTPUT(1);
		break;
		case 1:
		MOTO2_ENABLE_OUTPUT(1);
		break;
		case 2:
		MOTO3_ENABLE_OUTPUT(1);
		break;
		case 3:
		MOTO4_ENABLE_OUTPUT(1);
		break;
		case 4:
		MOTO5_ENABLE_OUTPUT(1);
		break;
		case 5:
		MOTO6_ENABLE_OUTPUT(1);
		break;
		case 6:		
		MOTO7_ENABLE_OUTPUT(1);
		break;
		case 7:
		MOTO8_ENABLE_OUTPUT(1);
		break;
		case 8:
		MOTO9_ENABLE_OUTPUT(1);
		break;
		case 9:
		MOTO10_ENABLE_OUTPUT(1);
		break;
		case 10:
		MOTO11_ENABLE_OUTPUT(1);
		break;
		case 11:
		MOTO12_ENABLE_OUTPUT(1);
		break;
		case 12:		
		MOTO13_ENABLE_OUTPUT(1);
		break;
		case 13:
		MOTO14_ENABLE_OUTPUT(1);
		break;		
		#ifdef E692_STEPMOTOR_EX			
			case 14:
				MOTO_EX1_ENABLE_OUTPUT(1);
				break;
			case 15:
				MOTO_EX2_ENABLE_OUTPUT(1);
				break;		
		#else
			case 14:
			MOTO15_ENABLE_OUTPUT(1);
			break;
			case 15:
			MOTO16_ENABLE_OUTPUT(1);
			break;
		#endif

		default:
			break;
		}
}




void arch_StepMotor_Half(int stepno, int onoff)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);
#if 0
	if (onoff)
	{
		if (stepindex<4)
		{
			return;
		}
	}
#endif
	switch (stepindex)
	{
		case 0:
			MOTO1_HALF_OUTPUT(onoff);
			break;
		case 1:
			MOTO2_HALF_OUTPUT(onoff);
			break;
		case 2:
			MOTO3_HALF_OUTPUT(onoff);
			break;
		case 3:
			MOTO4_HALF_OUTPUT(onoff);
			break;
		case 4:
			MOTO5_HALF_OUTPUT(onoff);
			break;
		case 5:
			MOTO6_HALF_OUTPUT(onoff);
			break;
		#ifdef E480_BOARD_V10
		case 6:
			MOTO7_HALF_OUTPUT(onoff);
			break;
		case 7:

			if (arch_is_E490_board())
			{
				MOTO8_HALF_OUTPUT_E490(onoff);
			}else
			
			MOTO8_HALF_OUTPUT(onoff);
			break;	
			
		case 8:
			MOTO9_HALF_OUTPUT(onoff);
			break;	
		case 9:
			MOTO10_HALF_OUTPUT(onoff);
			break;
		case 10:
			MOTO11_HALF_OUTPUT(onoff);
			break;
		case 11:
			MOTO12_HALF_OUTPUT(onoff);
			break;	
		#ifdef E490_V10_BOARD
			#ifdef TZ_NOT_HALF_
			case 12:
				MOTO13_HALF_OUTPUT(0);
				break;
			case 13:
				MOTO14_HALF_OUTPUT(0);
				break;
			#else
			
			case 12:
				MOTO13_HALF_OUTPUT(onoff);
				break;
			case 13:
				MOTO14_HALF_OUTPUT(onoff);
				break;
			#endif

			#ifdef E692_STEPMOTOR_EX
			case 14:
				MOTO_EX1_HALF_OUTPUT(onoff);
				break;
			case 15:
				MOTO_EX2_HALF_OUTPUT(onoff);
				break;		
				
			#endif
			
		#else
			#ifdef E499_BOARD_SUPPORT_
			case 12:
				MOTO13_HALF_OUTPUT(onoff);
				break;
			case 13:
				MOTO14_HALF_OUTPUT(onoff);
				break;		
			#endif
		#endif
		#endif
		default :
			break;
	}
}

void arch_StepMotor_Dir(int stepno, int onoff)
{
	//static unsigned short idx=0;
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

	switch (stepindex)	
	{
		case 0:
			MOTO1_DIR_OUTPUT(onoff);
			break;
		case 1:
			MOTO2_DIR_OUTPUT(onoff);
			break;
		case 2:
			MOTO3_DIR_OUTPUT(onoff);
			break;
		case 3:
			MOTO4_DIR_OUTPUT(onoff);
			break;
		case 4:
			MOTO5_DIR_OUTPUT(onoff);
			break;
		case 5:
			MOTO6_DIR_OUTPUT(onoff);
			break;
		#ifdef E480_BOARD_V10
		case 6:
			MOTO7_DIR_OUTPUT(onoff);
			break;
		case 7:
			MOTO8_DIR_OUTPUT(onoff);
			break;
		case 8:
			MOTO9_DIR_OUTPUT(onoff);
			break;
		case 9:
			MOTO10_DIR_OUTPUT(onoff);
			break;
		case 10:
			MOTO11_DIR_OUTPUT(onoff);
			break;
		case 11:
			MOTO12_DIR_OUTPUT(onoff);
			break;
		#ifdef E490_V10_BOARD
		case 12:
			MOTO13_DIR_OUTPUT(onoff);
			break;
		case 13:
			MOTO14_DIR_OUTPUT(onoff);
			break;
		#ifdef E692_STEPMOTOR_EX			
			case 14:
				MOTO_EX1_DIR_OUTPUT(onoff);
				break;
			case 15:
				MOTO_EX2_DIR_OUTPUT(onoff);
				break;		
		#endif
			
		#else
			#ifdef E499_BOARD_SUPPORT_
			case 12:
				MOTO13_DIR_OUTPUT(onoff);
				break;
			case 13:
				MOTO14_DIR_OUTPUT(onoff);
				break;		
			#endif
		#endif	
			
		#endif
		default :
			break;
	}

	//Message_Send_4halfword(0x99|(stepno<<8),onoff,0,idx++);

}

void arch_StepMotor_Pulse(int stepno, int onoff)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

	switch (stepindex)	
	{
		case 0:
			MOTO1_PULSE_OUTPUT(onoff);
			break;
		case 1:
			MOTO2_PULSE_OUTPUT(onoff);
			break;
		case 2:
			MOTO3_PULSE_OUTPUT(onoff);
			break;
		case 3:
			MOTO4_PULSE_OUTPUT(onoff);
			break;
		case 4:
			MOTO5_PULSE_OUTPUT(onoff);
			break;
		case 5:
			MOTO6_PULSE_OUTPUT(onoff);
			break;
		#ifdef E480_BOARD_V10
		case 6:
			MOTO7_PULSE_OUTPUT(onoff);
			break;
		case 7:
			MOTO8_PULSE_OUTPUT(onoff);
			break;
		case 8:
			MOTO9_PULSE_OUTPUT(onoff);
			break;
		case 9:
			MOTO10_PULSE_OUTPUT(onoff);
			break;
		case 10:
			MOTO11_PULSE_OUTPUT(onoff);
			break;
		case 11:
			MOTO12_PULSE_OUTPUT(onoff);
			break;
		#ifdef E490_V10_BOARD
		case 12:
			MOTO13_PULSE_OUTPUT(onoff);
			break;
		case 13:
			MOTO14_PULSE_OUTPUT(onoff);
			break;
		#ifdef E692_STEPMOTOR_EX
		case 14:
			MOTO_EX1_PULSE_OUTPUT(onoff);
			break;
		case 15:
			MOTO_EX2_PULSE_OUTPUT(onoff);			
			break;
		
		#endif
			
		#else
			#ifdef E499_BOARD_SUPPORT_
			case 12:
				MOTO13_PULSE_OUTPUT(onoff);
				break;
			case 13:
				MOTO14_PULSE_OUTPUT(onoff);
				break;		
			#endif	
		#endif	
		#endif
		default :
			break;
	}
}



void arch_StepMotor_Set_Speed(unsigned int stepno, unsigned int speed)
{

	step_speed[stepno] = speed;
}

void arch_StepMotor_Start(unsigned int stepno)
{
	u16 capture = 100;
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

	CLI();
	switch(stepindex) {
	case 1:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare1(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
		break;
	case 0:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare2(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
		break;
	case 2:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare3(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
		break;
	case 3:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare4(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
		break;
	#ifndef ECODE_USE_MT6813_PWM	
	case 4:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare1(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare2(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
		break;
	#else
	case 4:
		capture += TIM_GetCounter(TIM9);
		TIM_SetCompare1(TIM9, capture&0xffff);
		TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		capture += TIM_GetCounter(TIM9);
		TIM_SetCompare2(TIM9, capture&0xffff);
		TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
		break;	
		
	#endif	
	#ifdef E480_BOARD_V10
	#ifndef ECODE_USE_MT6813_PWM
	case 6:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare3(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
		break;
	case 7:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare4(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		break;
	#else
	case 6:
		capture += TIM_GetCounter(TIM12);
		TIM_SetCompare1(TIM12, capture&0xffff);
		TIM_ITConfig(TIM12, TIM_IT_CC1, ENABLE);
		break;
	case 7:
		capture += TIM_GetCounter(TIM12);
		TIM_SetCompare2(TIM12, capture&0xffff);
		TIM_ITConfig(TIM12, TIM_IT_CC2, ENABLE);
		break;
	#endif	
	case 8:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare1(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
		break;
	case 9:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare2(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
		break;
	case 10:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare3(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
		break;
	case 11:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare4(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
		break;
#ifdef E499_BOARD_SUPPORT_
	case 12:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare1(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
		break;
	case 13:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare2(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
		break;
#endif	
#ifdef E692_STEPMOTOR_EX
	case 14:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare3(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
		break;
	case 15:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare4(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
		break;

#endif

		
	#endif
	
	default:
		break;
	}
	SEI();
}
#ifdef JAQ_WORKMODE_8_16	
extern volatile unsigned int jaq_work_mode_8_16;
#endif
extern volatile unsigned int step_resolution[];


unsigned int arch_StepMotor_remainTime_us(unsigned int stepno)
{
	unsigned int remaintimeus;
	unsigned int remaintimecnt=0;
	unsigned int remaintimeccr=0;	
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);
	switch(stepindex) 
	{
	case 1:		
		remaintimecnt = TIM_GetCounter(TIM2);
		remaintimeccr = TIM_GetCapture1(TIM2);
		break;
	case 0:		
		remaintimecnt = TIM_GetCounter(TIM2);
		remaintimeccr = TIM_GetCapture2(TIM2);	
		break;
	case 2:
		remaintimecnt = TIM_GetCounter(TIM2);
		remaintimeccr = TIM_GetCapture3(TIM2);
		break;
	case 3:
		remaintimecnt = TIM_GetCounter(TIM2);
		remaintimeccr = TIM_GetCapture4(TIM2);
		break;
	#ifndef ECODE_USE_MT6813_PWM		
	case 4:
		remaintimecnt = TIM_GetCounter(TIM3);
		remaintimeccr = TIM_GetCapture1(TIM3);
		break;
	case 5:
		remaintimecnt = TIM_GetCounter(TIM3);
		remaintimeccr = TIM_GetCapture2(TIM3);
		break;
	#else
	case 4:
		remaintimecnt = TIM_GetCounter(TIM9);
		remaintimeccr = TIM_GetCapture1(TIM9);
		break;
	case 5:
		remaintimecnt = TIM_GetCounter(TIM9);
		remaintimeccr = TIM_GetCapture2(TIM9);
		break;
	#endif	
	#ifdef E480_BOARD_V10
	#ifndef ECODE_USE_MT6813_PWM
	case 6:
		remaintimecnt = TIM_GetCounter(TIM3);
		remaintimeccr = TIM_GetCapture3(TIM3);
		break;
	case 7:
		remaintimecnt = TIM_GetCounter(TIM3);
		remaintimeccr = TIM_GetCapture4(TIM3);
		break;
	#else
	case 6:
		remaintimecnt = TIM_GetCounter(TIM12);
		remaintimeccr = TIM_GetCapture1(TIM12);
		break;
	case 7:
		remaintimecnt = TIM_GetCounter(TIM12);
		remaintimeccr = TIM_GetCapture2(TIM12);
		break;
	#endif	
	case 8:
		remaintimecnt = TIM_GetCounter(TIM8);
		remaintimeccr = TIM_GetCapture1(TIM8);
		break;
	case 9:
		remaintimecnt = TIM_GetCounter(TIM8);
		remaintimeccr = TIM_GetCapture2(TIM8);
		break;
	case 10:
		remaintimecnt = TIM_GetCounter(TIM8);
		remaintimeccr = TIM_GetCapture3(TIM8);
		break;
	case 11:
		remaintimecnt = TIM_GetCounter(TIM8);
		remaintimeccr = TIM_GetCapture4(TIM8);
		break;
	#ifdef E499_BOARD_SUPPORT_
	case 12:
		remaintimecnt = TIM_GetCounter(TIM1);
		remaintimeccr = TIM_GetCapture1(TIM1);
		break;
	case 13:
		remaintimecnt = TIM_GetCounter(TIM1);
		remaintimeccr = TIM_GetCapture2(TIM1);
		break;

	#endif
	#ifdef E692_STEPMOTOR_EX
	case 14:
		remaintimecnt = TIM_GetCounter(TIM1);
		remaintimeccr = TIM_GetCapture3(TIM1);
		break;
	case 15:
		remaintimecnt = TIM_GetCounter(TIM1);
		remaintimeccr = TIM_GetCapture4(TIM1);
		break;

	#endif
	
	#endif
	default:
		break;
	}	

	remaintimeus = remaintimeccr+65536-remaintimecnt;
	if(remaintimeus>=65536)
		remaintimeus -=65536;	

	return remaintimeus;
	
}

void arch_StepMotor_Active(unsigned int stepno,unsigned int  add_speed)
{
	DWORD capture = step_speed[stepno];
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);


	#ifdef STEP_MOTOR_DDDM_SUPPORT
	
	capture = 1000L*1000L/capture;	

	#else

	if (add_speed)
		capture += add_speed;
	else
		capture += 100;	 


	#endif

	CLI();

	switch(stepindex) {
	case 1:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare1(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
		break;
	case 0:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare2(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
		break;
	case 2:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare3(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
		break;
	case 3:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare4(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
		break;
	#ifndef ECODE_USE_MT6813_PWM		
	case 4:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare1(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare2(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
		break;
	#else
	case 4:
		capture += TIM_GetCounter(TIM9);
		TIM_SetCompare1(TIM9, capture&0xffff);
		TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		capture += TIM_GetCounter(TIM9);
		TIM_SetCompare2(TIM9, capture&0xffff);
		TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
		break;
	#endif	
	#ifdef E480_BOARD_V10
	#ifndef ECODE_USE_MT6813_PWM
	case 6:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare3(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
		break;
	case 7:
		capture += TIM_GetCounter(TIM3);
		TIM_SetCompare4(TIM3, capture&0xffff);
		TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		break;
	#else
	case 6:
		capture += TIM_GetCounter(TIM12);
		TIM_SetCompare1(TIM12, capture&0xffff);
		TIM_ITConfig(TIM12, TIM_IT_CC1, ENABLE);
		break;
	case 7:
		capture += TIM_GetCounter(TIM12);
		TIM_SetCompare2(TIM12, capture&0xffff);
		TIM_ITConfig(TIM12, TIM_IT_CC2, ENABLE);
		break;
	#endif	
	case 8:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare1(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
		break;
	case 9:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare2(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
		break;
	case 10:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare3(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
		break;
	case 11:
		capture += TIM_GetCounter(TIM8);
		TIM_SetCompare4(TIM8, capture&0xffff);
		TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
		break;
	#ifdef E499_BOARD_SUPPORT_
	case 12:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare1(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
		break;
	case 13:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare2(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
		break;

	#endif
	#ifdef E692_STEPMOTOR_EX
	case 14:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare3(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
		break;
	case 15:
		capture += TIM_GetCounter(TIM1);
		TIM_SetCompare4(TIM1, capture&0xffff);
		TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
		break;

	#endif
	
	#endif
	default:
		break;
	}
	SEI();
}



void arch_StepMotor_EnableIT(unsigned int stepno)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

	switch(stepindex) {
	case 1:
		TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
		break;
	case 0:
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
		break;
	case 2:

		TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
		break;
	case 3:
		TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
		break;
	#ifndef ECODE_USE_MT6813_PWM		
	case 4:
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
		break;
	#else
	case 4:
		TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
		break;
	case 5:
		TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
		break;
	#endif	
	#ifdef E480_BOARD_V10
	#ifndef ECODE_USE_MT6813_PWM
	case 6:
		
		TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
		break;
	case 7:
		TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		break;
	#else
	case 6:

		TIM_ITConfig(TIM12, TIM_IT_CC1, ENABLE);
		break;
	case 7:

		TIM_ITConfig(TIM12, TIM_IT_CC2, ENABLE);
		break;
	#endif	
	case 8:
	
		TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
		break;
	case 9:
	
		TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
		break;
	case 10:
		
		TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
		break;
	case 11:
		
		TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
		break;
	#ifdef E499_BOARD_SUPPORT_
	case 12:
	
		TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
		break;
	case 13:
		
		TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
		break;

	#endif
	#ifdef E692_STEPMOTOR_EX
	case 14:
		
		TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
		break;
	case 15:
	
		TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
		break;

	#endif
	
	#endif
	default:
		break;
	}
}



void arch_StepMotor_Stop(unsigned int stepno)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

	#ifdef LOG_DEBUG_FOR_LX_AT_CH
	//		if (((errsteps_&0xFFF)>=ADJ_NEED_LOG_STEPS)&&(enable_log_tosend_LX))	
	{
		unsigned short d[3];
		//d[0] 	= //Step->moto_remap_config.moto_attr.bit.moto_type_config;
		//d[0] |= Step->moto_remap_config.moto_remap_id_self<<4;
		d[0] = LX_LOG_TYPE_MOVETOZERO_ADD400_2<<8;
		d[0] |= 2<<12;
		d[1] =stepno;
		d[2] =0;								
		Message_send_log_LX(d[0],d[1],d[2]);
	}
	#endif



	switch(stepindex) {
	case 1:
		TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
		break;
	case 0:
		TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
		break;
	case 2:
		TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);
		break;
	case 3:
		TIM_ITConfig(TIM2, TIM_IT_CC4, DISABLE);
		break;
	#ifndef ECODE_USE_MT6813_PWM	
	case 4:
		TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
		break;
	case 5:
		TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
		break;
	#else
	case 4:
		TIM_ITConfig(TIM9, TIM_IT_CC1, DISABLE);
		break;
	case 5:
		TIM_ITConfig(TIM9, TIM_IT_CC2, DISABLE);
		break;
	
	#endif	
	#ifdef E480_BOARD_V10
	#ifndef ECODE_USE_MT6813_PWM
	case 6:
		TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
		break;
	case 7:
		TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
		break;
	#else
	case 6:
		TIM_ITConfig(TIM12, TIM_IT_CC1, DISABLE);
		break;
	case 7:
		TIM_ITConfig(TIM12, TIM_IT_CC2, DISABLE);
		break;
	#endif	
	case 8:
		TIM_ITConfig(TIM8, TIM_IT_CC1, DISABLE);
		break;
	case 9:
		TIM_ITConfig(TIM8, TIM_IT_CC2, DISABLE);
		break;
	case 10:
		TIM_ITConfig(TIM8, TIM_IT_CC3, DISABLE);
		break;
	case 11:
		TIM_ITConfig(TIM8, TIM_IT_CC4, DISABLE);
		break;	
	#ifdef E499_BOARD_SUPPORT_
	case 12:
		TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
		break;
	case 13:
		TIM_ITConfig(TIM1, TIM_IT_CC2, DISABLE);
		break;
	#endif
	#ifdef E692_STEPMOTOR_EX
	case 14:
		TIM_ITConfig(TIM1, TIM_IT_CC3, DISABLE);
		break;
	case 15:
		TIM_ITConfig(TIM1, TIM_IT_CC4, DISABLE);
		break;
	#endif
		
	#endif
	
	default:
		break;
	}
}

void arch_StepMotor_Init_single(unsigned int whichstep)
{
	int i;
 	i=whichstep;
	if(i < StepMotor_Count_MAX) 
	{
		arch_StepMotor_Half(i, MOTOR_HALF_OUTPUT);
		arch_StepMotor_Dir(i, 0);
		arch_StepMotor_Pulse(i, 0);
	}
}


void arch_StepMotor_Init()
{
	int i;

	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		arch_StepMotor_Half(i, MOTOR_HALF_OUTPUT);
		arch_StepMotor_Dir(i, 0);
		arch_StepMotor_Pulse(i, 0);
	}
	arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_DENSITY-1],MOTOR_TYPE_DENSITY);
	#ifdef E480_BOARD_V10
		arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_ACTION-1],MOTOR_TYPE_ACTION);
	#else	
		arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_SKINER-1],MOTOR_TYPE_SKINER);
	#endif

	#ifdef DEBUG_STEP_DDM_CNT
		debug_cnt_init();
	#endif
	
}


#ifdef E490_V10_BOARD

unsigned int arch_StepMotor_Zero(unsigned int stepno)
{
	switch(stepno) {
	case 0:
		return MOTO1_ZERO_INPUT();
	case 1:
		return MOTO2_ZERO_INPUT();
	case 2:
		return MOTO3_ZERO_INPUT();
	case 3:
		return MOTO4_ZERO_INPUT();
	case 4:
		return MOTO5_ZERO_INPUT();
	case 5:
		return MOTO6_ZERO_INPUT();
	case 6:
		return MOTO7_ZERO_INPUT();
	case 7:
		return MOTO8_ZERO_INPUT();
	case 8:
		return MOTO9_ZERO_INPUT();
	case 9:
		return MOTO10_ZERO_INPUT();
	case 10:
		return MOTO11_ZERO_INPUT();
	case 11:
		return MOTO12_ZERO_INPUT();
	case 12:
		return MOTO13_ZERO_INPUT();
	case 13:		
		return MOTO14_ZERO_INPUT();
	case 14:
		return MOTO15_ZERO_INPUT();
	case 15:		
		return MOTO16_ZERO_INPUT();	
	case 16:		
		return 	SW1_INPUT();
	case 17:
		return  	SW2_INPUT();
	case 18:
		return ERROR1_INPUT();
	case 19:
		return ERROR2_INPUT();
	#ifdef E692_STEPMOTOR_EX
	case 20:
		return Step_use_exBoard?TANZHEN_EX_IN():0;
	#endif
	default:
		break;
	}
	return 0;

}

#else


unsigned int arch_StepMotor_Zero(unsigned int stepno)

{
#ifdef E480_BOARD_V10

	switch(stepno) {
	case 0:
		return MOTO1_ZERO_INPUT();
	case 1:
		return MOTO2_ZERO_INPUT();
	case 2:
		return MOTO3_ZERO_INPUT();
	case 3:
		return MOTO4_ZERO_INPUT();
	case 4:
		return MOTO5_ZERO_INPUT();
	case 5:
		return MOTO6_ZERO_INPUT();
	case 6:
		return SW1_INPUT();
	case 7:
		return SW2_INPUT();
	case 8:
		return MOTO9_ZERO_INPUT();
	case 9:
		return MOTO10_ZERO_INPUT();
	case 10:
		return MOTO11_ZERO_INPUT();
	case 11:
		return MOTO12_ZERO_INPUT();
	case 12:
		return MOTO7_ZERO_INPUT();
	case 13:		
		return MOTO8_ZERO_INPUT();
	case 14:
		return ERROR1_INPUT();
	case 15:		
		return ERROR2_INPUT();	
	default:
		break;
	}
	return 0;
#else

	switch(stepno) {
	case 0:
		return MOTO1_ZERO_INPUT();
	case 1:
		return MOTO2_ZERO_INPUT();
	case 2:
		return MOTO3_ZERO_INPUT();
	case 3:
		return MOTO4_ZERO_INPUT();
	case 4:
		return MOTO5_ZERO_INPUT();
	case 5:
		return MOTO6_ZERO_INPUT();
	case 6:
		return MOTO7_ZERO_INPUT();
	case 7:
		return MOTO8_ZERO_INPUT();
	case 8:
		return SW1_INPUT();
	case 9:
		return SW2_INPUT();
	case 10:
		return ERROR1_INPUT();
	case 11:
		return ERROR2_INPUT();
	
	default:
		break;
	}
	return 0;
#endif

}
#endif


//unsigned int Triangle_input_is_ok(unsigned int zerono,unsigned int workno)


unsigned int  Triangle_is_left_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2)
{
	 return (!arch_StepMotor_Zero_mid_layer(zerono,isnc1) && arch_StepMotor_Zero_mid_layer(workno,isnc2))?1:0;
 	
}
unsigned int  Triangle_is_right_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2)
{
	return (arch_StepMotor_Zero_mid_layer(zerono,isnc1) && !arch_StepMotor_Zero_mid_layer(workno,isnc2))?1:0;
}

unsigned int  Triangle_is_zero_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2)
{
	return (arch_StepMotor_Zero_mid_layer(zerono,isnc1) && arch_StepMotor_Zero_mid_layer(workno,isnc2))?1:0;
}

unsigned int arch_StepMotor_work(unsigned int stepno)
{
	return arch_StepMotor_Zero(stepno);
}

unsigned int arch_get_ad_base_cur_zero(unsigned int NorP)
{
	unsigned int i=NorP;
	
	if (i<2)
	{
		unsigned int ds=0;
		unsigned int k;
		
		for (k=0;k<MAX_DC24_A_BASE_DATA_COUNT;k++)
			ds+=DC24_BASE_CURR_Arry[i][k];

		return ds/MAX_DC24_A_BASE_DATA_COUNT;

	}
	else
		return 1650;
}


unsigned int arch_get_ad_cur_zero(unsigned int whichc)
{
	
	
	//return 1650;

	switch (whichc)
	{
		case CHECK_DC24_JQD:			
			return DC24_N_CURR_Zero;
			//break;
		case CHECK_DC24_YARN:
			return DC24_N_CURR_Zero;
			//break;
		case CHECK_DC24_ACT:
			return DC24_P_CURR_Zero;
			
			//break;
		case CHECK_DC24_JQD_P:
			return DC24_P_CURR_Zero;
			//break;
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		case CHECK_DC24_JQD_EX:

			return DC24_N_CURR_Zero;
		case CHECK_DC24_JQD_EX_P:

			return DC24_P_CURR_Zero;
				
		#endif
		default:
			return 1650;
			//break;			
	}

	
}


/* 特殊处理????*/
unsigned char arch_Get_Test_act_id_(unsigned int whichc,unsigned int no1,unsigned int no2)
{
	unsigned char ret=0;	
	unsigned int baseadd;
	unsigned int addstep;
	//unsigned int enablebit;	
	//unsigned char sts=0;

	switch (whichc)
	{
		case CHECK_DC24_JQD:
		
			addstep=MAX_BLADE_HARDWARE_8;
			baseadd =0;
			//enablebit =0;
			//sts=0;
			break;
		case CHECK_DC24_YARN:
				addstep=MAX_YARN;
				baseadd =32;	
				//enablebit =0;
				//sts=no1;
			break;
		case CHECK_DC24_ACT:
				addstep=MAX_ACTEMF;
				baseadd =32+16;	
				//enablebit =1;
				//sts=no1;
			break;
		case CHECK_DC24_JQD_P:
				addstep=MAX_BLADE_HARDWARE_8;
				baseadd =0;
				//enablebit =1;
				//sts=1;
			break;	
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		case CHECK_DC24_JQD_EX:
				addstep=MAX_BLADE_HARDWARE_10-MAX_BLADE_HARDWARE_8;
				baseadd =72;
			break;
		case CHECK_DC24_JQD_EX_P:
				addstep=MAX_BLADE_HARDWARE_10-MAX_BLADE_HARDWARE_8;
				baseadd =72;
			break;	
		#endif	
		default:
			break;			
	}
	ret = baseadd + no1*addstep+no2;//[0-31:选针，]
	return ret;
}

void arch_Set_check_JQD_YARN_ACT(unsigned int whichc,unsigned int no1,unsigned int no2)
{

	unsigned int baseadd;
	unsigned int addstep;
	unsigned int enablebit;	
	//unsigned char sts=0;

	switch (whichc)
	{
		case CHECK_DC24_JQD:
		
			addstep=MAX_BLADE_HARDWARE_8;
			baseadd =0;
			enablebit =0;
			//sts=0;
			break;
		case CHECK_DC24_YARN:
				addstep=MAX_YARN;
				baseadd =32;	
				enablebit =0;
				//sts=no1;
			break;
		case CHECK_DC24_ACT:
				addstep=MAX_ACTEMF;
				baseadd =32+16;	
				enablebit =1;
				//sts=no1;
			break;
		case CHECK_DC24_JQD_P:
				addstep=MAX_BLADE_HARDWARE_8;
				baseadd =0;
				enablebit =1;
				//sts=1;
			break;
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		case CHECK_DC24_JQD_EX:
				addstep=MAX_BLADE_HARDWARE_10-MAX_BLADE_HARDWARE_8;
				baseadd =72;
				enablebit =0;
			break;
		case CHECK_DC24_JQD_EX_P:
				addstep=MAX_BLADE_HARDWARE_10-MAX_BLADE_HARDWARE_8;
				baseadd =72;
				enablebit =1;
			break;	
		#endif
				
		default:
			break;			
	}

	if ( arch_check_JQD_YARN_ACT_Bit & (0x01<<whichc))
	{
		DC24_PN_whichNO =baseadd + no1*addstep+no2;//[0-31:选针，]
		//DC24_PN_data_Arry[DC24_PN_whichNO] = DC24_PN_whichNO;
		DC24_PN_Count = 0;
		DC24_PN_Count_test=0;
		DC24_PN_EnableBIT |= (0x01<<enablebit); //后续查???
		DC24_Start_check =1;
		whichEMFisTest_sts[DC24_PN_whichNO] = 1;
		if (SYS_Test_Mode_ISPWM)
		{
			extern void Time_start_time7(void);
			if ((whichc==CHECK_DC24_YARN)||(whichc==CHECK_DC24_ACT)) {
				//Time_start_time7();
				DC24_Check_Timer_Start = 1;
			}
		}		
	}

}

extern volatile unsigned int DC24_P_CURR_V;
extern  volatile unsigned int DC24_N_CURR_V ;

unsigned int get_verydata_with_Arry(unsigned char isPostiv,unsigned char whichtype)
{
	unsigned int DSUM=0;
	unsigned int i;
	unsigned int vc=CHECK_DATA_COUNT_VAR;
	unsigned int ret;
	unsigned int vc2;
	unsigned int pp=1;

	unsigned int v240=240;

	if (SYS_Test_Mode_ISPWM)
	{
		if ((whichtype==CHECK_DC24_YARN)||(whichtype==CHECK_DC24_ACT))
		{
			vc = CHECK_DATA_COUNT_VAR_EMF;
			pp =0x01<<whichtype;
		}
	}

	
	if (DC24_PN_Count>=vc)
	{
		for (i=DC24_PN_Count-vc;i<DC24_PN_Count;i++)
		{
			DSUM +=DC24_PN_CURR_Arry[i];
		}		
	}
	else		
	{
			for (i=0;i<DC24_PN_Count;i++)
			{
			DSUM +=DC24_PN_CURR_Arry[i];
			}
			vc2 = i;
			if (DC24_PN_Count_test>DC24_PN_Count)   //说明队列溢出了
			{
				for (i=MAX_DC24_COUNT-1;i>MAX_DC24_COUNT-1-(vc-vc2);i--)
				{
					DSUM +=DC24_PN_CURR_Arry[i];
				}
				vc2 = vc;
			}
			vc=vc2;
	}
	
	
	// if (vc>0)
	 {
		 int data_temp;
		
	 	data_temp = DSUM -DC24_PN_CURR_Zero*vc;

		if (data_temp<=0)
		{
			ret = 9999;
		}
		else
		{
			if (isPostiv)
			{
				v240=DC24_P_CURR_V *21L /(100L*pp);
			}else
				v240 = DC24_N_CURR_V *21L /(100L*pp);	
			
			ret =(v240 * 55L * vc) /data_temp;
			if (ret >1000)
			{
				ret = 9999;
			}
			//return (24*55*vc*10) /data_temp; // 24V 55是指每55mv等于一个安培，10，放大10倍
		}
		
		return ret;
	 }
	// else
	 //	return 0;
}


void arch_send_Operate_data()
{
	//unsigned int DSUM=0;
	unsigned int i;
	unsigned short d[4];
	unsigned char isjqd=0;
	unsigned char jqdno_,blade_no_;
	
	extern void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
//	for (i=0;i<((DC24_PN_Count+1)>>1);i++)
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if((DC24_PN_whichNO>=72)||(DC24_PN_whichNO<32))
	{
		isjqd =1;
		if(DC24_PN_whichNO<32)
		{
			jqdno_ = DC24_PN_whichNO>>3;
			blade_no_ =DC24_PN_whichNO&0x07;
		}
		else
		{
			jqdno_ = (DC24_PN_whichNO-72)>>1;
			blade_no_ =(DC24_PN_whichNO-72)&0x01+MAX_BLADE_HARDWARE_8;
		}
	}
	
	#else
	if (DC24_PN_whichNO<32)
	{
		isjqd =1;
		jqdno_ = DC24_PN_whichNO>>3;
		blade_no_ =DC24_PN_whichNO&0x07;
	}
	#endif

	if (isjqd)
	{
		d[0]=0x0D06;
		d[1]=(jqdno_)|((blade_no_)<<8);
		d[2]=DC24_PN_data_Arry[DC24_PN_whichNO];
		d[3]=(DC24_PN_whichNO<<8)|(0);
	}
	else
		if(DC24_PN_whichNO<48)
		{
			i= DC24_PN_whichNO-32;
			d[0]=0x0505;
			d[1]=(0)|((i&0x07)<<8);
			d[2]=DC24_PN_data_Arry[32+(i&0x07)];
			d[3]=DC24_PN_data_Arry[32+(i&0x07)+8];
		}
		else
			if(DC24_PN_whichNO<MAX_DC24_DATA_COUNT)
			{
				i= DC24_PN_whichNO-32-16;
				d[0]=0x0505;
				d[1]=(((i /6)&0x01) +1)|((i % 6)<<8);
				d[2]=DC24_PN_data_Arry[32+16+(i%12)];
				d[3]=DC24_PN_data_Arry[32+16+(i%12)+12];
			}
			else
				{
					return;
				}
		Message_Send_4halfword(d[0],d[1],d[2],d[3]);	


//arch_test_send_all_data_toshow(1);
		
	 return ;

}


void arch_test_send_all_data_toshow(int isfirst)
{
#if 0
	unsigned int DSUM=0;
	unsigned int i;
	unsigned short d[4];
#endif
	return;
#if 0
	extern void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
//	for (i=0;i<((DC24_PN_Count+1)>>1);i++)
	for (i=0;i<(MAX_DC24_COUNT>>1);i++)
	{
		d[0] =i;
		d[1] =DC24_PN_CURR_Arry[i*2+0];
		d[2] =DC24_PN_CURR_Arry[i*2+1];
		if (i==0)
			{
		d[3] =DC24_PN_Count;
			}
		else
			{d[3] =DC24_PN_Count_test;}

		Message_Send_4halfword(d[0],d[1],d[2],d[3]);
		
		//DSUM +=DC24_PN_CURR_Arry[i];
		
	}
Message_Send_4halfword(0xff11,DC24_PN_whichNO,0xAABB,0xAABB);
Message_Send_4halfword(0xff22,DC24_N_CURR_A,DC24_P_CURR_A,DC24_PN_CURR_Zero);
Message_Send_4halfword(0xff33,TempS_CURR_V,Vbat_CURR_V,Vrf_CURR_V);
#endif

#if 0
	myprintf("\r\n===isfirst(%d),i=%d--%d===\r\n",isfirst,isfirst?0:(MAX_DC24_COUNT>>1),(MAX_DC24_COUNT>>(isfirst?1:0))-1);

	for (i=isfirst?0:(MAX_DC24_COUNT>>1);i<MAX_DC24_COUNT>>(isfirst?1:0);i++)
	{
		myprintf("%4d\r\n",DC24_PN_CURR_Arry[i]);
	}
	myprintf("\r\n==DC24_PN_Count= %d, DC24_PN_Count_test=%d==\r\n",DC24_PN_Count,DC24_PN_Count_test);

		
	 return ;
 #endif

}

void arch_Clear_check_JQD_YARN_ACT(unsigned int whichc)
{
	//unsigned int bits;
	//unsigned int baseadd;
	//unsigned int addstep;
	extern void Time_stop_time7(void);
	unsigned char isn=0;

	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if (whichc>CHECK_DC24_JQD_EX_P) return;
	if((whichc<=CHECK_DC24_YARN) ||(whichc == CHECK_DC24_JQD_EX))
	{
		isn = 0; 
	}
	else
	{
		isn =1;
	}
	#else	
	if (whichc>CHECK_DC24_JQD_P) return;
	isn = (whichc<=CHECK_DC24_YARN)?0:1;
	#endif
	if ( arch_check_JQD_YARN_ACT_Bit & (0x01<<whichc))
	{
		//Time_stop_time7();
		DC24_Check_Timer_Start = 0;
		if (DC24_PN_whichNO<MAX_DC24_DATA_COUNT)
			DC24_PN_data_Arry[DC24_PN_whichNO]=get_verydata_with_Arry(isn,whichc);
		DC24_PN_EnableBIT &= ~ ((unsigned char)0x01<<(isn)); 	
		arch_check_JQD_YARN_ACT_Bit &=~((unsigned char)0x01<<whichc);
		DC24_Start_check =0;
		
	}
	else
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		if ((whichc==CHECK_DC24_JQD)||(whichc==CHECK_DC24_JQD_EX))
		{
			unsigned int whichc_loc=(whichc==CHECK_DC24_JQD?CHECK_DC24_JQD_P:CHECK_DC24_JQD_EX_P);
			isn = 1;
			if ( arch_check_JQD_YARN_ACT_Bit & (0x01<<whichc_loc))
			{
				//Time_stop_time7();
				DC24_Check_Timer_Start = 0;
				if (DC24_PN_whichNO<MAX_DC24_DATA_COUNT)
				DC24_PN_data_Arry[DC24_PN_whichNO]=get_verydata_with_Arry(isn,whichc);
				DC24_PN_EnableBIT &= ~ ((unsigned char)0x01<<isn); 	
				arch_check_JQD_YARN_ACT_Bit &=~((unsigned char)0x01<<whichc_loc);
				DC24_Start_check =0;
			}
		}
		#else
		if (whichc==CHECK_DC24_JQD)
		{
			if ( arch_check_JQD_YARN_ACT_Bit & (0x01<<(whichc+CHECK_DC24_JQD_P)))
			{
				//Time_stop_time7();
				DC24_Check_Timer_Start = 0;
				if (DC24_PN_whichNO<MAX_DC24_DATA_COUNT)
				DC24_PN_data_Arry[DC24_PN_whichNO]=get_verydata_with_Arry(((whichc+CHECK_DC24_JQD_P)<=0x01)?0:1,whichc);
				DC24_PN_EnableBIT &= ~ ((unsigned char)0x01<<(((whichc+CHECK_DC24_JQD_P)<=0x01)?0:1)); 	
				arch_check_JQD_YARN_ACT_Bit &=~((unsigned char)0x01<<(whichc+CHECK_DC24_JQD_P));
				DC24_Start_check =0;
			}
		}
		#endif
	if (whichOverload_in_progress_mask&0x03)	//说明过流了
	{
		DC24_PN_data_Arry[DC24_PN_whichNO] = 0;
	}
}


void arch_yarnemf_close_one(int yno)
{
	switch (yno)
	{
	case 0:
		SZ1_Q0_OUTPUT(1);
		SZ1_Q1_OUTPUT(1);
		break;
	case 1:
		SZ2_Q0_OUTPUT(1);
		SZ2_Q1_OUTPUT(1);
		break;
	case 2:
		SZ3_Q0_OUTPUT(1);
		SZ3_Q1_OUTPUT(1);
		break;
	case 3:
		SZ4_Q0_OUTPUT(1);
		SZ4_Q1_OUTPUT(1);
		break;
	case 4:
		SZ5_Q0_OUTPUT(1);
		SZ5_Q1_OUTPUT(1);
		break;
	case 5:
		SZ6_Q0_OUTPUT(1);
		SZ6_Q1_OUTPUT(1);
		break;
	case 6:
		SZ7_Q0_OUTPUT(1);
		SZ7_Q1_OUTPUT(1);
		break;
	case 7:
		SZ8_Q0_OUTPUT(1);
		SZ8_Q1_OUTPUT(1);
		break;
	default :
		break;
	}
}

void arch_yarnemf_pwm_setup(int yno, int onoff)
{
	unsigned char yarn1;
	unsigned char yarn2;
	
	if((yno &0x07) >= MAX_YARN) {
		return;
	}
	
	if(onoff) {
		yarn1 = 1;
		yarn2 = 1;
	}
	else {
		if (yno<8)
		{
			yarn1 = 0;
			yarn2 = 1;
		}
		else
		{
			yarn1 = 1;
			yarn2 = 0;
		}		
	}

	arch_yarnemf_close_one(yno &0x07);
	
	switch ((yno &0x07) )
	{
	case 0:
		SZ1_Q0_OUTPUT(yarn1);		
		SZ1_Q1_OUTPUT(yarn2);
		break;
	case 1:
		SZ2_Q0_OUTPUT(yarn1);
		SZ2_Q1_OUTPUT(yarn2);
		break;
	case 2:
		SZ3_Q0_OUTPUT(yarn1);
		SZ3_Q1_OUTPUT(yarn2);
		break;
	case 3:
		SZ4_Q0_OUTPUT(yarn1);
		SZ4_Q1_OUTPUT(yarn2);
		break;
	case 4:
		SZ5_Q0_OUTPUT(yarn1);
		SZ5_Q1_OUTPUT(yarn2);
		break;
	case 5:
		SZ6_Q0_OUTPUT(yarn1);
		SZ6_Q1_OUTPUT(yarn2);
		break;
	case 6:
		SZ7_Q0_OUTPUT(yarn1);
		SZ7_Q1_OUTPUT(yarn2);
		break;
	case 7:
		SZ8_Q0_OUTPUT(yarn1);
		SZ8_Q1_OUTPUT(yarn2);
		break;
	default :
		break;
	}

}


void arch_YARNEMF_Setup(int yno, int onoff,int check)
{	
	unsigned char yarn1;
	unsigned char yarn2;
	
	if(yno >= MAX_YARN) {
		return;
	}

#ifdef E480_BOARD_V10
	if (Yarn_use_Step)
	{
		arch_YARN_Step_Setup(yno,onoff);
		return;
	}
#endif

#ifdef YARN_OPT_CHECK_ENABLE
if (check)
{
	if (yarn_opt_sts_check[yno]!=0xff)
	{
		unsigned short errno=0;
		unsigned short errno_1=0;
		
		if (onoff)
		{
			if (yarn_opt_sts_check[yno]==1)
			{
				//errno = 1;
			}
			else
			{
				errno = 1;
			}
			
		}
		else
		{
			if (yarn_opt_sts_check[yno]==0)
			{
				//errno = 3;
			}
			else
			{
				errno = 2;
			}
				
		}
		if (errno)
		{
			//unsigned int nowtick;
			//extern unsigned int Get_blade_group_keep_time(unsigned char d,unsigned char x);
			errno = (errno<<12)|(0<<8)|(0xF<<4)|(yno);
			//Message_send_log_ecode(unsigned int arg1, unsigned int arg2, unsigned int arg3)
			errno_1 = 0;//send_check_mainloop_and_01ms(jqdnotemp,i);
		
			errno |= (errno_1<<14);
			//alert_push
			Message_send_log_ecode(JQD_OPERATE_ERROR,errno,0);
			
		}
		
	}

}
#endif
	
	if(onoff) {
		yarn1 = 1;
		yarn2 = 0;
	}
	else {
		yarn1 = 0;
		yarn2 = 1;
	}

	DC24_PN_CURR_Zero = arch_get_ad_cur_zero(CHECK_DC24_YARN);

	//Message_Send_4halfword(0x0101,CHECK_DC24_YARN,DC24_PN_CURR_Zero,0x0202);



	#ifdef YARN_OPT_CHECK_ENABLE
	yarn_opt_sts_check[yno] = onoff?1:0;

	#endif
	arch_yarnemf_close_one(yno &0x07);
	switch (yno)
	{
	case 0:
		SZ1_Q0_OUTPUT(yarn1);		
		SZ1_Q1_OUTPUT(yarn2);
		break;
	case 1:
		SZ2_Q0_OUTPUT(yarn1);
		SZ2_Q1_OUTPUT(yarn2);
		break;
	case 2:
		SZ3_Q0_OUTPUT(yarn1);
		SZ3_Q1_OUTPUT(yarn2);
		break;
	case 3:
		SZ4_Q0_OUTPUT(yarn1);
		SZ4_Q1_OUTPUT(yarn2);
		break;
	case 4:
		SZ5_Q0_OUTPUT(yarn1);
		SZ5_Q1_OUTPUT(yarn2);
		break;
	case 5:
		SZ6_Q0_OUTPUT(yarn1);
		SZ6_Q1_OUTPUT(yarn2);
		break;
	case 6:
		SZ7_Q0_OUTPUT(yarn1);
		SZ7_Q1_OUTPUT(yarn2);
		break;
	case 7:
		SZ8_Q0_OUTPUT(yarn1);
		SZ8_Q1_OUTPUT(yarn2);
		break;
	default :
		break;
	}

arch_Set_check_JQD_YARN_ACT(CHECK_DC24_YARN,onoff?1:0,yno);

	


}

void arch_YARNEMF_Clear(int yno)
{
	//int addr;

	if(yno >= MAX_YARN) {
		return;
	}

#ifdef E480_BOARD_V10
	if (Yarn_use_Step)
	{
		return;
	}
#endif
	
	arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_YARN);

	arch_yarnemf_close_one(yno);
#if 0	
	switch (yno)
	{
	case 0:
		SZ1_Q0_OUTPUT(1);
		SZ1_Q1_OUTPUT(1);
		break;
	case 1:
		SZ2_Q0_OUTPUT(1);
		SZ2_Q1_OUTPUT(1);
		break;
	case 2:
		SZ3_Q0_OUTPUT(1);
		SZ3_Q1_OUTPUT(1);
		break;
	case 3:
		SZ4_Q0_OUTPUT(1);
		SZ4_Q1_OUTPUT(1);
		break;
	case 4:
		SZ5_Q0_OUTPUT(1);
		SZ5_Q1_OUTPUT(1);
		break;
	case 5:
		SZ6_Q0_OUTPUT(1);
		SZ6_Q1_OUTPUT(1);
		break;
	case 6:
		SZ7_Q0_OUTPUT(1);
		SZ7_Q1_OUTPUT(1);
		break;
	case 7:
		SZ8_Q0_OUTPUT(1);
		SZ8_Q1_OUTPUT(1);
		break;
	default :
		break;
	}
#endif
	#ifdef YARN_OPT_CHECK_ENABLE
	yarn_opt_sts_check[yno]=0xff;
	#endif

}

void arch_YARNEMF_AllClear()
{
	
#ifdef E480_BOARD_V10
	if (Yarn_use_Step)
	{
		return;
	}
#endif

	SZ1_Q0_OUTPUT(1);
	SZ1_Q1_OUTPUT(1);
	SZ2_Q0_OUTPUT(1);
	SZ2_Q1_OUTPUT(1);
	SZ3_Q0_OUTPUT(1);
	SZ3_Q1_OUTPUT(1);
	SZ4_Q0_OUTPUT(1);
	SZ4_Q1_OUTPUT(1);
	SZ5_Q0_OUTPUT(1);
	SZ5_Q1_OUTPUT(1);
	SZ6_Q0_OUTPUT(1);
	SZ6_Q1_OUTPUT(1);
	SZ7_Q0_OUTPUT(1);
	SZ7_Q1_OUTPUT(1);
	SZ8_Q0_OUTPUT(1);
	SZ8_Q1_OUTPUT(1);

	#ifdef YARN_OPT_CHECK_ENABLE
	{
		int i;
		for (i=0;i<MAX_YARN;i++)
			yarn_opt_sts_check[i]=0xff;
	}
	#endif
	

}



void arch_ACTEMF_PWM_Setup(int actno, int onoff)
{
	unsigned long addr;

	#ifdef E499_BOARD_SUPPORT_
if (arch_is_EMF_2_SK_board())
	return;
#endif

	if(actno >= MAX_ACTEMF*2) {
		return;
	}

	#ifdef E490_V10_BOARD
	
	if ((actno<6)||((actno>11)&&(actno<18)))
	{
		addr = ADDR_ACTBOARD1;
	}else
	addr = ADDR_ACTBOARD2;
	addr += (actno%6);

	#else

	
	
	addr = ADDR_ACTBOARD1;
	addr += actno;
	if (actno>=MAX_ACTEMF)
		addr -= MAX_ACTEMF;
	#endif

	if(onoff) {
		OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
		OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
	}
	else {
#ifdef ACT_USE_8803
		
		if (actno<MAX_ACTEMF)
			{
				OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?1:0);
				OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
			}
		else
			{
				OUTPUT(addr, ACT_OUT_IS_NOT?1:0);
				OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
			}

#else
		if (actno<MAX_ACTEMF)
			{
				OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
				OUTPUT(addr, ACT_OUT_IS_NOT?1:0);
			}
		else
			{
				OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
				OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?1:0);
			}

#endif
	}
//arch_Set_check_JQD_YARN_ACT(CHECK_DC24_ACT,onoff?1:0,actno);
	
}


void arch_ACTEMF_Setup(int actno, int onoff,int check)
{
	unsigned long addr;
	int oldactno=actno;

	#ifdef E499_BOARD_SUPPORT_
if (arch_is_EMF_2_SK_board())
	return;
#endif

	if(actno >= MAX_ACTEMF) {
		return;
	}

#ifdef ACT_OPT_CHECK_ENABLE
if (check)
{
	if (act_opt_sts_check[actno]!=0xff)
	{
		unsigned short errno=0;
		unsigned short errno_1=0;
		
		if (onoff)
		{
			if (act_opt_sts_check[actno]==1)
			{
				//errno = 1;
			}
			else
			{
				errno = 1;
			}
			
		}
		else
		{
			if (act_opt_sts_check[actno]==0)
			{
				//errno = 3;
			}
			else
			{
				errno = 2;
			}
				
		}
		if (errno)
		{
			//unsigned int nowtick;
			//extern unsigned int Get_blade_group_keep_time(unsigned char d,unsigned char x);
			errno = (errno<<12)|(0<<8)|(0xE<<4)|(actno);
			//Message_send_log_ecode(unsigned int arg1, unsigned int arg2, unsigned int arg3)
			errno_1 = 0;//send_check_mainloop_and_01ms(jqdnotemp,i);
		
			errno |= (errno_1<<14);
			Message_send_log_ecode(JQD_OPERATE_ERROR,errno,0);//alert_push
		}
		
	}

}

#endif

	

#ifdef E490_V10_BOARD
	if(actno < 6) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 6;
		addr = ADDR_ACTBOARD2;
	}

#else
	if(actno < 12) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 12;
		addr = ADDR_ACTBOARD2;
	}
#endif

	//addr += actno * 2;
	addr += actno;

	DC24_PN_CURR_Zero = arch_get_ad_cur_zero(CHECK_DC24_ACT);


	#ifdef ACT_OPT_CHECK_ENABLE
	act_opt_sts_check[oldactno]=onoff?1:0;
	
	#endif

	#ifdef ACT_USE_8803
	if(onoff) {
		OUTPUT(addr, ACT_OUT_IS_NOT?1:0);
		delayus();
		OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);

	}
	else {

		OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?1:0);
		delayus();
		OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
	}
	#else

	if(onoff) {
		OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
		delayus();
		OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?1:0);

	}
	else {

		OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
		delayus();
		OUTPUT(addr, ACT_OUT_IS_NOT?1:0);
	}
	#endif
arch_Set_check_JQD_YARN_ACT(CHECK_DC24_ACT,onoff?1:0,oldactno);
	
}

void arch_ACTEMF_Clear(int actno)
{
#ifndef ACTAUTOCLEAR
	unsigned long addr;
	unsigned int oldactno=actno;

#ifdef E499_BOARD_SUPPORT_
if (arch_is_EMF_2_SK_board())
	return;
#endif

	if(actno >= MAX_ACTEMF) {
		return;
	}

#ifdef E490_V10_BOARD

	if(actno < 6) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 6;
		addr = ADDR_ACTBOARD2;
	}

#else
	
	if(actno < 12) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 12;
		addr = ADDR_ACTBOARD2;
	}
#endif
	//addr += actno * 2;
	addr += actno;

	arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_ACT);

	OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
	delayus();
	OUTPUT(addr + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
#ifdef ACT_OPT_CHECK_ENABLE
	act_opt_sts_check[oldactno]=0xff;
#endif
#endif
}

#ifdef E490_V10_BOARD
void arch_ACTEMF_AllClear()
{
#ifndef ACTAUTOCLEAR
	int i;
#ifdef E499_BOARD_SUPPORT_
if (arch_is_EMF_2_SK_board())
	return;
#endif
	
	for(i = 0; i < 6; i ++) {
		OUTPUT(ADDR_ACTBOARD1 + i, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD1 + i + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD2 + i, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD2 + i + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
		#ifdef ACT_OPT_CHECK_ENABLE
		act_opt_sts_check[i]=0xff;
		act_opt_sts_check[6+i]=0xff;
		
		#endif
	}
#endif
}
#else

void arch_ACTEMF_AllClear()
{
#ifndef ACTAUTOCLEAR
	int i;
#ifdef E499_BOARD_SUPPORT_
if (arch_is_EMF_2_SK_board())
	return;
#endif
	
	for(i = 0; i < 6 * 2; i ++) {
		OUTPUT(ADDR_ACTBOARD1 + i, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD1 + i + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
		//OUTPUT(ADDR_ACTBOARD2 + i, 1);
		//OUTPUT(ADDR_ACTBOARD2 + i + 0x40, 1);
		#ifdef ACT_OPT_CHECK_ENABLE
		act_opt_sts_check[i]=0xff;
		//act_opt_sts_check[6+i]=0xff;
		
		#endif
	}
#endif
}
#endif

int arch_setup_Jacquard_blade_map(int blade, int remap)
{
	if (blade>=MAX_BLADE) return 0;     //hlc 20141204 支持10段选针
	jqdbld_map[blade] = remap;

	return 0;
}


#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR

static unsigned long Get_addr_with_jqdno_blade(int jqdno, int blade,int isen)
{
	unsigned long addr;
	
	
	if(blade>=MAX_BLADE_HARDWARE_8) 
	{
		if (jqdno<2)//说明是后床
		{
			addr = ADDR_JQDBOARD_EX1;
		}
		else
		{
			addr = ADDR_JQDBOARD_EX2;
		}
		jqdno &=0x01;
		addr +=jqdno * (MAX_BLADE_HARDWARE_10-MAX_BLADE_HARDWARE_8) + blade-MAX_BLADE_HARDWARE_8;
		/*??*/
		if(isen)
			addr +=	ADDR_DIS_JQD_IN_EN_EX;
		
	}	
	else
	{
		addr = ADDR_JQDBOARD1;

		if (JQD_halfwork_mode)
		{
			jqdno &= 0x7;
			if (jqdno<4)	//后床
			{
				addr = ADDR_JQDBOARD1;
			}
			else
			{
				addr = ADDR_JQDBOARD2;
			}
			jqdno &= 0x3;
			
			addr += jqdno * (MAX_BLADE_HARDWARE_8>>1) + blade;
		}
		else
		{	
			jqdno &= 0x3;
			if (jqdno<2)//说明是后床
			{
				addr = ADDR_JQDBOARD1;
			}
			else
			{
				addr = ADDR_JQDBOARD2;
			}
			jqdno &=0x01;
			
			addr += jqdno * (MAX_BLADE_HARDWARE_8) + blade;
		}
		if(isen)
			addr +=	ADDR_DIS_JQD_IN_EN;
	}

return addr;

}

#else

static unsigned long Get_addr_with_jqdno_blade(int jqdno, int blade)
{
	unsigned long addr;
	
	
	addr = ADDR_JQDBOARD1;

	if (JQD_halfwork_mode)
	{
		jqdno &= 0x7;
		if (jqdno<4)	//后床
		{
			addr = ADDR_JQDBOARD1;
		}
		else
		{
			addr = ADDR_JQDBOARD2;
		}
		jqdno &= 0x3;
		
		addr += jqdno * (MAX_BLADE_HARDWARE>>1) + blade;
	}
	else
	{	
		jqdno &= 0x3;
		if (jqdno<2)//说明是后床
		{
			addr = ADDR_JQDBOARD1;
		}
		else
		{
			addr = ADDR_JQDBOARD2;
		}
		jqdno &=0x01;
		
		addr += jqdno * (MAX_BLADE_HARDWARE) + blade;
	}

return addr;
}

#endif



int arch_Jacquard_Setup(int jqdno, int blade, int onoff,int check,char idx)
{
	unsigned long addr,addr_en;
	int ret;
	int loc_jqdno = jqdno;
	int loc_blade = blade;
	int loc_st = onoff;
	int maxblades = MAX_BLADE_HARDWARE;
	
#ifdef LX_JQD_CHECK_ENABLE
	int jqdnotemp=jqdno;
	int i=blade;
#endif
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	maxblades = sys_max_blade_phi;
	#endif
	if (blade>=MAX_BLADE) return -1;     //hlc 20141204 支持10段选针

	ret = 0;
	blade = jqdbld_map[blade];
	if (blade>=maxblades) return -1;     //hlc 20141204 支持10段选针

#ifdef LX_JQD_CHECK_ENABLE
if (check)
{
	if (jqd_blade_sts_check[jqdnotemp][i]!=0xff)
	{
		unsigned short errno=0;
		unsigned short errno_1=0;
		
		if (onoff)
		{
			if (jqd_blade_sts_check[jqdnotemp][i]==1)
			{
				//errno = 1;
			}
			else
			{
				errno = 1;
			}
			
		}
		else
		{
			if (jqd_blade_sts_check[jqdnotemp][i]==0)
			{
				//errno = 3;
			}
			else
			{
				errno = 2;
			}
				
		}
		if (errno)
		{
			//void arch_putout_Tim_CNT_CCR(unsigned int whichjqd);
			//extern unsigned int Get_blade_group_keep_time(unsigned char d,unsigned char x);
			errno = (errno<<12)|(idx<<8)|(jqdnotemp<<4)|(i);
			//Message_send_log_ecode(unsigned int arg1, unsigned int arg2, unsigned int arg3)
			errno_1 = 0;//send_check_mainloop_and_01ms(jqdnotemp,i);
		
			errno |= (errno_1<<14);
			alert_push(JQD_OPERATE_ERROR,errno);
			//Message_Send_4halfword(0x11ff,loc_jqdno, loc_blade,loc_st);
			//	arch_putout_Tim_CNT_CCR(jqdno);
			//Message_send_log_ecode(0|(0xEE<<8),errno,0xEEEE);

#if 0
			Message_Send_4halfword(0xAABB,errno,Get_blade_group_keep_time(i,jqdnotemp),0);
			nowtick = Get_blade_group_last_setup_systick(i,jqdnotemp);
			Message_Send_4halfword(0xAAB1,nowtick,nowtick>>16,errno);
			nowtick = arch_get_ticktime();
			Message_Send_4halfword(0xAAB2,nowtick,nowtick>>16,errno);
#endif			
			//return 1;
		}
		
	}
}

#endif	

#if 0
{
	extern void Set_mainloop_01ms(unsigned char jqdno,unsigned char blandno);
	Set_mainloop_01ms(jqdnotemp,i);
}
#endif
	//addr = ADDR_JQDBOARD1;

	//addr =Get_addr_with_jqdno_blade(jqdno,blade,0);
#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	addr =Get_addr_with_jqdno_blade(jqdno,blade,0);
	addr_en =Get_addr_with_jqdno_blade(jqdno,blade,1);	
#else
	addr =Get_addr_with_jqdno_blade(jqdno,blade);
	addr_en = addr+ADDR_DIS_JQD_IN_EN;
#endif

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if(blade>=MAX_BLADE_HARDWARE_8)
	{
		DC24_PN_CURR_Zero = arch_get_ad_cur_zero(onoff?CHECK_DC24_JQD_EX:CHECK_DC24_JQD_EX_P);
	}
	else
#endif		
		DC24_PN_CURR_Zero = arch_get_ad_cur_zero(onoff?CHECK_DC24_JQD:CHECK_DC24_JQD_P);

	//Message_Send_4halfword(0x0101,onoff?CHECK_DC24_JQD:CHECK_DC24_JQD_P,DC24_PN_CURR_Zero,0x0202);	

	#ifdef LX_JQD_CHECK_ENABLE
	//Set_blade_group_last_setup_systick(i,jqdnotemp);
	//blade_group_last_setup_systick[i][jqdnotemp]=arch_get_ticktime();
	jqd_blade_sts_check[jqdnotemp][i]=onoff?1:0;
	#endif

	OUTPUT(addr, JQD_mode_is_UN?(onoff?1:0):(onoff?0:1));			//in	
	delayus();
	OUTPUT(addr_en, 1);				//en

	jqd_clear_sts[jqdno][blade]=JQD_mode_is_UN?(onoff?0:1):(onoff?1:0);

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if(blade>=MAX_BLADE_HARDWARE_8)
	{
		arch_Set_check_JQD_YARN_ACT(onoff?CHECK_DC24_JQD_EX:CHECK_DC24_JQD_EX_P,jqdno,blade-MAX_BLADE_HARDWARE_8);
	}
	else
#endif
	arch_Set_check_JQD_YARN_ACT(onoff?CHECK_DC24_JQD:CHECK_DC24_JQD_P,jqdno,blade);

	#ifdef LX_JQD_CHECK_ENABLE
	return ret;
	#endif

	
}

void arch_Jacquard_PWMBlade(int jqdno, int blade)
{
	//unsigned long addr;
	//unsigned char pwmsts;

	return;

	#if 0
	
	blade = jqdbld_map[blade];
	if (blade>=MAX_BLADE_HARDWARE) return;    
	
	addr =Get_addr_with_jqdno_blade(jqdno,blade);

	pwmsts = jqd_pwmsts[jqdno][blade];
	
	OUTPUT(addr + ADDR_DIS_JQD_IN_EN, pwmsts);
	jqd_pwmsts[jqdno][blade] = pwmsts ? 0 : 1 ;
	#endif
}

unsigned char whichjqblad_is_clear[MAX_JACQUARD][32];
unsigned char whichjqblad_is_clear_par[MAX_JACQUARD];


void arch_Jacquard_ClearBlade(int jqdno, int blade)
{

	unsigned long addr,addr_en;
	unsigned char jqdsts;

	unsigned int max_blade_loc=MAX_BLADE_HARDWARE;
	#ifdef LX_JQD_CHECK_ENABLE
	int jqdnotemp=jqdno;
	int i=blade;

#endif	
#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	max_blade_loc=sys_max_blade_phi;
#endif
	
	blade = jqdbld_map[blade];
	if (blade>=max_blade_loc) return;     //hlc 20141204 支持10段选针
	

	
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if(blade>=MAX_BLADE_HARDWARE_8)
	{
		arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_JQD_EX);
	}
	else
	{
		arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_JQD);
	}
	addr =Get_addr_with_jqdno_blade(jqdno,blade,0);
	addr_en =Get_addr_with_jqdno_blade(jqdno,blade,1);	

	#else
	arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_JQD);
	addr =Get_addr_with_jqdno_blade(jqdno,blade);
	addr_en = addr+ADDR_DIS_JQD_IN_EN;
	#endif
	{		
		//unsigned char jqdsts;
		jqdsts=jqd_clear_sts[jqdno][blade];

		OUTPUT(addr, jqdsts);
		delayus();
	}

	OUTPUT(addr_en, 0);

	#ifdef LX_JQD_CHECK_ENABLE
 	jqd_blade_sts_check[jqdnotemp][i]=0xff;

	#if 0
	whichjqblad_is_clear[jqdnotemp][whichjqblad_is_clear_par[jqdnotemp]]=i;
	whichjqblad_is_clear_par[jqdnotemp]++;
	if (whichjqblad_is_clear_par[jqdnotemp]>=32)
	{
		whichjqblad_is_clear_par[jqdnotemp]=0;
	}
#endif
	#endif
	
//	jqd_clear_sts[jqdno][blade] = jqdsts?0:1;
	

}


void arch_Jacquard_Clear_Step(int jqdno,int stepid)//////////? 这里有疑问
{

	unsigned long addr,addr_en;
	int i;
	unsigned int max_jqd_blades_loc=MAX_BLADE_HARDWARE;
#ifdef LX_JQD_CHECK_ENABLE
	int jqdnotemp=jqdno;
#endif
	
	//if (jqdno>=8) return;     //hlc 20141204 支持10段选针//前半段，后半段
	if ((stepid>1) ||(stepid<0)) return;


	#ifndef E693_TEN_BLADE_NEEDLE_SELECTOR	
	addr =Get_addr_with_jqdno_blade(jqdno,0);
	addr_en = addr+ADDR_DIS_JQD_IN_EN;
	max_jqd_blades_loc = sys_max_blade_phi;
	#endif
	
	//addr =Get_addr_with_jqdno_blade(jqdno,0);



	arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_JQD);/*目前不使用*/

	if (JQD_halfwork_mode)		
	{
		for(i = stepid*(MAX_BLADE_HARDWARE_8>>2); i < ((stepid+1)*(MAX_BLADE_HARDWARE_8>>2)); i ++)
		{
			{
				unsigned char jqdsts;
				jqdsts=jqd_clear_sts[jqdno][i];
				OUTPUT(addr+i, jqdsts);
				delayus();
			}
			
			OUTPUT(addr + i + ADDR_DIS_JQD_IN_EN, 0);
			delayus();
		}
	
	}
	else
	{
	
		for(i = stepid*(max_jqd_blades_loc>>1); i < ((stepid+1)*(max_jqd_blades_loc>>1)); i ++)
		{

			#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
			addr = Get_addr_with_jqdno_blade(jqdno,i,0);
			addr_en =Get_addr_with_jqdno_blade(jqdno,i,1);	
			#else
			addr = Get_addr_with_jqdno_blade(jqdno,i);
			addr_en = addr+ADDR_DIS_JQD_IN_EN;
			#endif
			{					
				unsigned char jqdsts;
				jqdsts=jqd_clear_sts[jqdno][i];
				OUTPUT(addr, jqdsts);			
				delayus();
			}
			OUTPUT(addr_en, 0);
			delayus();
			
		}

		#ifdef LX_JQD_CHECK_ENABLE
		 jqd_blade_sts_check[jqdnotemp][i]=0xff;
		#endif
	}

}


void arch_Jacquard_Clear(int jqdno)//////////? 这里有疑问
{
//#ifndef JQDAUTOCLEAR
	unsigned long addr,addr_en;
	int i;
	unsigned int max_jqd_blades_loc=MAX_BLADE_HARDWARE;
	#ifdef LX_JQD_CHECK_ENABLE
	int jqdnotemp=jqdno;
	#endif
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	max_jqd_blades_loc = sys_max_blade_phi;
	#endif
	//if (jqdno>=8) return;     //hlc 20141204 支持10段选针

//	addr =Get_addr_with_jqdno_blade(jqdno,0);
	arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_JQD);/*?????*/
	//for(i = 0; i < MAX_BLADE * 2; i ++)
	for(i = 0; i < (JQD_halfwork_mode ? (max_jqd_blades_loc>>1):max_jqd_blades_loc); i ++)
	{

		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		addr = Get_addr_with_jqdno_blade(jqdno,i,0);
		addr_en =Get_addr_with_jqdno_blade(jqdno,i,1);	
		#else
		addr = Get_addr_with_jqdno_blade(jqdno,i);
		addr_en = addr+ADDR_DIS_JQD_IN_EN;
		#endif
		{					
			unsigned char jqdsts;
			jqdsts=jqd_clear_sts[jqdno][i];
			OUTPUT(addr, jqdsts);			
			delayus();
		}
		OUTPUT(addr_en, 0);
		delayus();

		#ifdef LX_JQD_CHECK_ENABLE
		 jqd_blade_sts_check[jqdnotemp][i]=0xff;
		#endif
		
	}
//#endif
}

#ifdef E490_V10_BOARD

void arch_Jacquard_AllClear()    //这里也有问题
{

	int i;
	unsigned char jqdsts;
	unsigned int addr_in,addr_en;

	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	int j;
	for(j=0;j<MAX_JACQUARD;j++)
	{
		for(i = 0; i < sys_max_blade_phi; i ++) 
		{					
			jqdsts=jqd_clear_sts[j][i];
			addr_in =Get_addr_with_jqdno_blade(j,i,0);
			addr_en =Get_addr_with_jqdno_blade(j,i,1);			
			OUTPUT(addr_in, jqdsts);
			delayus();
			OUTPUT(addr_en, 0);
			delayus();							
		}
	}

	#else
	
	for(i = 0; i < MAX_BLADE_HARDWARE* 2; i ++) 
	{					
		jqdsts=jqd_clear_sts[i>>3][i&0x07];
		OUTPUT(ADDR_JQDBOARD1 +i, jqdsts);
		delayus();
		OUTPUT(ADDR_JQDBOARD1 + i + ADDR_DIS_JQD_IN_EN, 0);
		delayus();
		jqdsts=jqd_clear_sts[(i>>3)+2][i&0x07];
		OUTPUT(ADDR_JQDBOARD2 +i, jqdsts);
		delayus();
		OUTPUT(ADDR_JQDBOARD2 + i + ADDR_DIS_JQD_IN_EN, 0);
		delayus();	
						
	}
	#endif

	#ifdef LX_JQD_CHECK_ENABLE


	for (i=0;i<MAX_JACQUARD;i++)
	{
		int j;
		for (j=0;j<MAX_BLADE;j++)
		{
			jqd_blade_sts_check[i][j] = 0xff;
		}
	}

#endif	

}
#else

void arch_Jacquard_AllClear()    //这里也有问题
{

	int i;
	for(i = 0; i < MAX_BLADE_HARDWARE* 4; i ++) 
	{

		{			
			unsigned char jqdsts;
			jqdsts=jqd_clear_sts[i>>3][i&0x07];
			OUTPUT(ADDR_JQDBOARD1 +i, jqdsts);
			delayus();
		}		
		OUTPUT(ADDR_JQDBOARD1 + i + ADDR_DIS_JQD_IN_EN, 0);
		delayus();				
	}

}
#endif

void arch_CPLD_Reset(void)
{

	#ifdef E490_V10_BOARD
	;
	#else
	OUTPUT(ADDR_JQDBOARD1 + 0x7f, 1);	
	#endif
}


#ifdef E490_V10_BOARD

void	arch_8803_Reset()
{	
	unsigned int i;
	unsigned int addr;

	for (i=0;i<2;i++)
	{
		if (i==0)
		{
			addr = ADDR_ACTBOARD1_RESET;
		}
		else
		{
			addr = ADDR_ACTBOARD2_RESET;
		}
			
		OUTPUT(addr, 0);
		delay(100);
		OUTPUT(addr, 1);
		delay(100);
		OUTPUT(addr, 0);
		
	}
	

	
}


void	arch_8844_Reset()
{
	unsigned int i;
	unsigned int addr;

	for (i=0;i<2;i++)
	{
	
	if (i<1)
	{
		addr = ADDR_JQDBOARD1_RESET;
	}
	else
	{
		addr = ADDR_JQDBOARD2_RESET;
	}
		
	OUTPUT(addr, 0);
	delay(100);
	OUTPUT(addr, 1);
	delay(100);
	OUTPUT(addr, 0);

	}
	
}

void arch_8844_Reset_one(unsigned char whichone)
{
	unsigned int i;
	unsigned int addr;
	i = (whichone & 0x03)>>1; 
	if (i<1)
	{
		addr = ADDR_JQDBOARD1_RESET;
	}
	else
	{
		addr = ADDR_JQDBOARD2_RESET;
	}
		
	OUTPUT(addr, 0);
	delay(100);
	OUTPUT(addr, 1);
	delay(100);
	OUTPUT(addr, 0);
}

#else

void	arch_8803_Reset()
{
#ifdef E499_BOARD_SUPPORT_
	if (arch_is_EMF_2_SK_board())
		return;
#endif


	SJ_RESET_OUTPUT(0);
	delay(100);
	SJ_RESET_OUTPUT(1);
	delay(100);	
	SJ_RESET_OUTPUT(0);

	
}


void	arch_8844_Reset()
{

	XZ_RESET_OUTPUT(0);	
	delay(100);
	XZ_RESET_OUTPUT(1);
	delay(100);
	XZ_RESET_OUTPUT(0);
	
}

#endif



unsigned long  Get_Overload_Status(void)
{
	
	return Overload_status_JQD_ACT_YARN & OVERLOAD_MASK;

	
}




// 使能主板的CPLD输出管脚
void MainboardUnlock( void)
{
	//OUTPUT(0xFA, 0x00);

#ifdef E490_V10_BOARD
	return;
#else
	
	OUTPUT(ADDR_MAINBOARD + 0x91, 0xFF);
	wait_ms(5);
	OUTPUT(ADDR_MAINBOARD + 0x91, 0x00);
#endif	
}

void OverLoad_Setup(int onoff)
{
	
	overload_check_on = onoff ? 1 : 0 ;
	if (overload_check_on)
	{
		arch_Exti_enable();
	}
	else
	{
		arch_Exti_disable();
	}

}

// 禁止主板的CPLD输出管脚
void MainboardLock(void)
{
	//OUTPUT(0xFA, 0xFF);
}

// 选针板复位 
void EXT_NRST_Reset(void)
{
	//OUTPUT(0xF9, 0x00);
	//wait_ms(5);
	//OUTPUT(0xF9, 0xFF);
}
extern volatile unsigned char Step_use_exBoard;
void arch_Exti_enable()
{
	#ifdef E490_V10_BOARD
	EXTI_Enable(EXTI_Line0);
	EXTI_Enable(EXTI_Line1);
	EXTI_Enable(EXTI_Line2);
	EXTI_Enable(EXTI_Line3);
	#ifdef E692_STEPMOTOR_EX
	if(!arch_need_close711_alert())
	{
		EXTI_Enable(EXTI_Line7);
		#ifndef TEST_EMC_711_PN
		EXTI_Enable(EXTI_Line8);
		EXTI_Enable(EXTI_Line12);
		#endif
	}
	#endif
	EXTI_Enable(EXTI_Line11);
	EXTI_Enable(EXTI_Line15);

	#else

	EXTI_Enable(EXTI_Line2);
	EXTI_Enable(EXTI_Line3);
	EXTI_Enable(EXTI_Line6);
	//EXTI_Enable(EXTI_Line8);
	EXTI_Enable(EXTI_Line11);
	//EXTI_Enable(EXTI_Line12);
	EXTI_Enable(EXTI_Line13);
#ifdef E499_BOARD_SUPPORT_
	if (!arch_is_EMF_2_SK_board())
#endif		
	{
		EXTI_Enable(EXTI_Line14);	
		EXTI_Enable(EXTI_Line15);
	}
	
	#endif
}


void arch_Exti_disable()
{
	#ifdef E490_V10_BOARD
	EXTI_Disable(EXTI_Line0);
	EXTI_Disable(EXTI_Line1);
	EXTI_Disable(EXTI_Line2);
	EXTI_Disable(EXTI_Line3);
	#ifdef E692_STEPMOTOR_EX
	if(!arch_need_close711_alert())
	{
		EXTI_Disable(EXTI_Line7);
		#ifndef TEST_EMC_711_PN
		EXTI_Disable(EXTI_Line8);
		EXTI_Disable(EXTI_Line12);
		#endif
	}
	#endif
	
	EXTI_Disable(EXTI_Line11);
	EXTI_Disable(EXTI_Line15);

	#else


	EXTI_Disable(EXTI_Line2);
	EXTI_Disable(EXTI_Line3);
	EXTI_Disable(EXTI_Line6);
	EXTI_Disable(EXTI_Line8);
	EXTI_Disable(EXTI_Line11);
	EXTI_Disable(EXTI_Line12);
	EXTI_Disable(EXTI_Line13);
#ifdef E499_BOARD_SUPPORT_
	if (!arch_is_EMF_2_SK_board())
#endif		
	{	
		EXTI_Disable(EXTI_Line14);	
		EXTI_Disable(EXTI_Line15);
	}
#endif
}

void arch_shock_enable(char x)
{
	if(x)
	{
		E495_OUT4(1);
		E495_OUT3(1);		
	}
	else
	{
		E495_OUT4(1);
		E495_OUT3(0);	
	}
}

void arch_shock_init()
{
	extern void alert_set_shock_PWM(int shock_sensitivity,int whichshock);
	
	// 初试化...
	

	shock_reset_timer = 0;

	shock_enable = 0;

	//SHOCK_ENABLE(0);
	arch_shock_enable(0);
	
	alert_set_shock_PWM(500,1);

	#ifdef ALARM_SHOCK_DOUBLE

	if (shock_double_enable)
	{
		shock_2_reset_timer = 0;

		shock_2_enable = 0;

		//SHOCK_2_ENABLE(0);
	
		alert_set_shock_PWM(500,2);
	}
	#endif
	
			
}



void arch_shock_reset(int whichshock)
{

	switch (whichshock)
	{
		case 0:
			//SHOCK_ENABLE(0);
			arch_shock_enable(0);
			shock_is_created = 0;
			shock_reset_timer = 10;	// defer to raise SHOCK_ENABLE...
			#ifdef ALARM_SHOCK_DOUBLE			
			//SHOCK_2_ENABLE(0);
			shock_2_is_created = 0;
			shock_2_reset_timer = 10;	// defer to raise SHOCK_ENABLE...
			#endif
			break;
		case 1:
			//SHOCK_ENABLE(0);
			arch_shock_enable(0);
			shock_is_created = 0;
			shock_reset_timer = 10;	// defer to raise SHOCK_ENABLE...
			break;
		#ifdef ALARM_SHOCK_DOUBLE			
		case 2:
			//SHOCK_2_ENABLE(0);
			arch_shock_enable(0);
			shock_2_is_created = 0;
			shock_2_reset_timer = 10;	// defer to raise SHOCK_ENABLE...
			break;
		#endif	
		default:
			break;
	}

}


void arch_shock_ctr(int isenable)
{

	//extern volatile unsigned int DCT_do_shock_disable_time;
	//extern volatile unsigned int shock_dct_is_enable;
	if (!arch_is_shock_board())
	return;
	{
		if(shock_enable)
		{
			arch_shock_enable(isenable?1:0);	
			shock_dct_is_enable = isenable?1:0;
			DCT_do_shock_disable_time = isenable?0:50;
		}		
		//#ifdef ALARM_SHOCK_DOUBLE			
		//shock_2_enable = onoff ?1:0;
	}
	return ;
		

}


void arch_shock_activate(int onoff,int whichshock)
{

	switch (whichshock)
	{
		case 0:
			shock_enable = onoff ?1:0;
			//SHOCK_ENABLE(onoff ?1:0);
			arch_shock_enable(onoff ?1:0);
			#ifdef ALARM_SHOCK_DOUBLE			
			shock_2_enable = onoff ?1:0;
			//SHOCK_2_ENABLE(onoff ?1:0);
			#endif
			break;
		case 1:
			shock_enable = onoff ?1:0;
			//SHOCK_ENABLE(onoff ?1:0);
			arch_shock_enable(onoff ?1:0);
			break;
		#ifdef ALARM_SHOCK_DOUBLE			
		case 2:
			shock_2_enable = onoff ?1:0;
			arch_shock_enable(onoff ?1:0);
			//SHOCK_2_ENABLE(onoff ?1:0);
			break;
		#endif	
		default:
			break;
	}

	
	
}


void arch_shock_timer_infun_1()
{
	static int shock_stable_timer = 1000;	// 2s..	
	
	if (shock_is_created)
	{
		if(shock_stable_timer > 0) {
			shock_stable_timer --;
			if(shock_stable_timer <= 0) {
				arch_shock_reset(1);	
				//myprintf("arch_shock_reset:shock_is_created \r\n");
				shock_stable_timer = 1000;
			}
		}
	}
	
	if(shock_reset_timer > 0) {
		shock_reset_timer --;
		if((shock_reset_timer <= 0) && (shock_enable == 1)&&(shock_dct_is_enable)) {
			arch_shock_enable(1);
			//SHOCK_ENABLE(1);
			//myprintf("SHOCK_ENABLE,shock_reset_timer: \r\n");
		}
	}

}

#ifdef ALARM_SHOCK_DOUBLE	
void arch_shock_timer_infun_2()
{
	static int shock_2_stable_timer = 1000;	// 2s..	
	
	if (shock_2_is_created)
	{
		if(shock_2_stable_timer > 0) {
			shock_2_stable_timer --;
			if(shock_2_stable_timer <= 0) {
				arch_shock_reset(2);	
				//myprintf("arch_shock_reset:shock_is_created \r\n");
				shock_2_stable_timer = 1000;
			}
		}
	}
	
	if(shock_2_reset_timer > 0) {
		shock_2_reset_timer --;
		if((shock_2_reset_timer <= 0) && (shock_2_enable == 1)&&(shock_dct_is_enable)) {
			arch_shock_enable(1);
			//SHOCK_2_ENABLE(1);
			//myprintf("SHOCK_ENABLE,shock_reset_timer: \r\n");
		}
	}
}

#endif

void arch_shock_timer(int whichshock)
{

	switch (whichshock)
	{
		case 0:
			arch_shock_timer_infun_1();
			#ifdef ALARM_SHOCK_DOUBLE			
			arch_shock_timer_infun_2();
			#endif
			break;
		case 1:
			arch_shock_timer_infun_1();
			break;
		#ifdef ALARM_SHOCK_DOUBLE			
		case 2:
			arch_shock_timer_infun_2();
			break;
		#endif	
		default:
			break;
	}

	if (DCT_do_shock_disable_time)
	{
		DCT_do_shock_disable_time--;
		if (DCT_do_shock_disable_time==0)
		arch_shock_ctr(1);
	}

}

unsigned char arch_check_power_isok(unsigned char whichpower)
{
	return (power_st>>whichpower)&0x01;
}




void arch_Power_ctrl_ex(unsigned char whichpower,unsigned char onoff,unsigned char which_cp)
{
	short *pnd;
	pnd = whichpower?(short *)&P24_on_delay_1s:(short *)&N24_on_delay_1s;
	if (onoff)
	{
		*pnd = 500;
	}

	if (whichpower)		//+24V
	{
		POWER_XZ_POSITIVE24V_OUTPUT(onoff?1:0);  
	}
	else					//-24V
	{		
		POWER_XZ_NEGATIVE24V_OUTPUT(onoff?0:1);
	}
	
	if (onoff)
		power_st |=(0x01<<whichpower); 
	else
	{
		power_st &=~(0x01<<whichpower); 
		which_part_close_power = which_cp ;		
	}	
}




void arch_Power_On(void)
{
	myprintf("Power On \r\n");
	// 外部上电

	//XZ_RESET_OUTPUT(1);	

	SYS_is_PowerOn =1;

	poweron_delay =3;	/*3*250us*/

	
	
	POWER_EXT_3V3_OUTPUT(0);

	arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);
	arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);
	
	//POWER_XZ_POSITIVE24V_OUTPUT(1);  

	//arch_delay_us(1000);
	
	//POWER_XZ_NEGATIVE24V_OUTPUT(0);
	//POWER_SJ_NEGATIVE24V_OUTPUT(0);
	//POWER_SZ_NEGATIVE24V_OUTPUT(0);
	//power_st = 3;

	arch_delay_us(100);
		
	if (overload_check_on)
	{
		arch_Exti_enable();
	}
	else
	{
		arch_Exti_disable();
	}


	overload_count = 0;
	//system_start_delay_1s=500;/*500*2ms=1s*/

	

	
}

void arch_Power_Off(void)
{
	myprintf("Power Off ____\r\n");

	SYS_is_PowerOn =0;
	arch_Exti_disable();

	arch_StepMotor_Disable();

	//system_start_delay_1s=-1;  /*关电之后，不需要报警保险丝失效*/

	POWER_EXT_3V3_OUTPUT(1);

	arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,14);
	arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,15);
	
	//POWER_XZ_POSITIVE24V_OUTPUT(0);  
	//POWER_XZ_NEGATIVE24V_OUTPUT(1);   //guan
	//power_st =0;
	//POWER_SJ_NEGATIVE24V_OUTPUT(1);
	//POWER_SZ_NEGATIVE24V_OUTPUT(1);
	
	

}

void  SetXzqHighVoltageTime(u16  timelong)
{
   	OUTPUT(ADDR_JQDBOARD1 + timelong, 0xFF);
	OUTPUT(ADDR_JQDBOARD2 + timelong, 0xFF);
}

void  SetDctHighVoltageTime(u16  timelong)
{
   	OUTPUT(ADDR_JQDBOARD1 + timelong, 0xFF);
	OUTPUT(ADDR_JQDBOARD2 + timelong, 0xFF);
}

int Board_Check(void)
{
	if(((INPUT(ADDR_MAINBOARD + 0xF0) & 0xFF) != 0x55) ||
	   ((INPUT(ADDR_MAINBOARD + 0xF1) & 0xFF) != 0xAA)) {
		return 0;
	}

	return 1;
}


void CheckOverloadBranch()
{
	//int status;
	if (!overload_check_on) return;
		
	if (Overload_status_JQD_ACT_YARN & OVERLOAD_MASK)
	{
		if(overload_status)
		{
			#ifdef NEW_ALARM_STYLE
			alert_push(OVERLOAD_ERR_CODE_ARG(0, Overload_status_JQD_ACT_YARN));
			#else
			alert_push(OVERLOAD_HEAD, Overload_status_JQD_ACT_YARN);
			#endif
			//Message_Send_Alert_DEBUG(OVERLOAD_HEAD, Overload_status_JQD_ACT_YARN);
		}
		overload_status = 0;
		//overload_delay = 100/*500*/;
	}	
		
}


void arch_set_overload_delay()  //ms
{
	poweron_delay =2;	
}

unsigned int arch_get_overload_delay()
{
	return poweron_delay;
}

void arch_overload_check_poweroff_(char *od_t)
{
	Overload_t *overload_temp;
	overload_temp = (Overload_t *)od_t;
	if (overload_temp->st.bit.rpower==0)
	{
		switch (overload_temp->st.bit.index)
		{
			case OVERLOAD_N_24:
				arch_Power_ctrl_ex(0,0,16);
				break;
			case OVERLOAD_P_24:
				arch_Power_ctrl_ex(1,0,17);
				break;
			case OVERLOAD_N_YARN_24:
				arch_Power_ctrl_ex(0,0,18);
				break;
			case OVERLOAD_ACT1:				
			case OVERLOAD_ACT2:
				arch_Power_ctrl_ex(1,0,19);
				break;
			case OVERLOAD_XZQ1:				
			case OVERLOAD_XZQ2:				
			case OVERLOAD_XZQ3:				
			case OVERLOAD_XZQ4:
				arch_Power_ctrl_ex(0,0,20);
				arch_Power_ctrl_ex(1,0,21);
				break;
			case OVERLOAD_YARN_F:
				arch_Power_ctrl_ex(0,0,22);
				break;
			case OVERLOAD_N_F:
				arch_Power_ctrl_ex(0,0,23);
				break;
			case OVERLOAD_P_F:
				arch_Power_ctrl_ex(1,0,24);
				break;	
			default:
				break;
		}
		alert_push(OVERLOAD_POWEROFF, overload_temp->st.bit.index);
		//Message_Send_Alert_DEBUG(OVERLOAD_POWEROFF, overload_temp->st.bit.index);
			
	}
	else
	{
		switch (overload_temp->st.bit.index)
		{
			case OVERLOAD_N_24:
				arch_Power_ctrl_ex(0,1,0);
				break;
			case OVERLOAD_P_24:
				arch_Power_ctrl_ex(1,1,0);
				break;
			case OVERLOAD_N_YARN_24:
				arch_Power_ctrl_ex(0,1,0);
				break;
			case OVERLOAD_ACT1:				
			case OVERLOAD_ACT2:
				arch_Power_ctrl_ex(1,1,0);
				break;			
			case OVERLOAD_YARN_F:
				arch_Power_ctrl_ex(0,1,0);
				break;
			case OVERLOAD_N_F:
				arch_Power_ctrl_ex(0,1,0);
				break;
			case OVERLOAD_P_F:
				arch_Power_ctrl_ex(1,1,0);
				break;	
			default:
				break;
		}
	}
}



void arch_power_IC_fault_check(int board)
{
	//char i =0;
	unsigned long maskdata;
	char *od_t;
	int mode;

//yarn_poweron:
//POWER_XZ_NEGATIVE24V_OUTPUT(0);

//clear:

if (board==0)
{	
	maskdata = OVERLOAD_MASK_YARN_F;
	od_t = (char *)&SYS_overload_c[OVERLOAD_YARN_F];
	mode = 0;
}
else
	if(board==1)
	{
		maskdata = OVERLOAD_MASK_24V_C_N;
		od_t = (char *)&SYS_overload_c[OVERLOAD_N_F];
		mode =0;
	}
	else
	{
		maskdata = OVERLOAD_MASK_24V_C_P;
		od_t = (char *)&SYS_overload_c[OVERLOAD_P_F];
		mode=1;
	}

	Clear_alarm_status_withMASK(maskdata,od_t);
	//system_power_24_isoverload |=(0x01<<(mode?1:0));

	

if (mode)
{	
	if (power_st & 0x02)
	{
		//POWER_XZ_POSITIVE24V_OUTPUT(0); 
		arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,25);
		arch_delay_us(10);	
		arch_set_overload_delay();
		//POWER_XZ_POSITIVE24V_OUTPUT(1); 
		//arch_Power_ctrl_ex(1,1);
		arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);
	}
	else
		goto overload_return_2;
}else
{
	if (power_st & 0x01)
	{
		//POWER_XZ_NEGATIVE24V_OUTPUT(1);
		arch_Power_ctrl_ex(0,0,26);
		arch_delay_us(10);
		arch_set_overload_delay();
		//POWER_XZ_NEGATIVE24V_OUTPUT(0);
		arch_Power_ctrl_ex(0,1,0);
	}
	else
		goto overload_return_2;	
}

while (arch_get_overload_delay());		

overload_return_2:
	Overload_recheck_finish(od_t);
	
}


void arch_Jacquard_Overload_Check(int board, int mode)
{
	char i;
	unsigned int jqd_status;
	unsigned long maskdata;
	int offset;
	char checknow=0;
	char * od_t;
	unsigned int max_bladeno=MAX_BLADE_HARDWARE_8;
	unsigned int first_bladeno=0;
	unsigned steps_other_JQD=2;
	

	switch (board)
	{
		case 0:
			if (mode)
			{
				maskdata =0; //OVERLOAD_MASK_XZ1_P;
			}
			else
			{
				maskdata = OVERLOAD_MASK_XZ1;
				od_t=(char *)&SYS_overload_c[OVERLOAD_XZQ1];
			}
			offset = 0;
			break;
		case 1:
			if (mode)
			{
				maskdata = 0; //OVERLOAD_MASK_XZ2_P;
			}
			else
			{
				maskdata = OVERLOAD_MASK_XZ2;
				od_t=(char *)&SYS_overload_c[OVERLOAD_XZQ2];
			}
			offset = 1;
			break;
		case 2:
			if (mode)
			{
				maskdata = 0; //OVERLOAD_MASK_XZ3_P;
				checknow=1;
			}
			else
			{
				maskdata = OVERLOAD_MASK_XZ3;
				od_t=(char *)&SYS_overload_c[OVERLOAD_XZQ3];
			}
			offset = 2;
			break;
		case 3:
			if (mode)
			{
				maskdata = 0; //OVERLOAD_MASK_XZ4_P;
				checknow=1;
			}
			else
			{
				maskdata = OVERLOAD_MASK_XZ4;
				od_t=(char *)&SYS_overload_c[OVERLOAD_XZQ4];
			}
			offset = 3;
			break;
		default:
			break;
	}
#if 0	
	alert_push(OVERLOAD_JQD, ((offset+1) << 8) | (1));

return;
#endif

clear:

	#ifdef E490_V10_BOARD
	arch_8844_Reset_one(board);
	#else
	arch_8844_Reset();
	#endif
	Clear_alarm_status_withMASK(maskdata,od_t);	
	arch_delay_us(500);
	if(Get_alarm_status_withMASK(maskdata,od_t)) 
	{
		i++;
		if(i < 2) {
			wait_ms(5);
			goto clear;
		}
		
		#ifdef NEW_ALARM_STYLE
		alert_push(OVERLOAD_ERR_CODE_ARG(3, 0);
		#else
		alert_push(OVERLOAD_JQD, 0);
		#endif
		Set_overloadalarm_withMASK(od_t);
		goto overload_return;
	}
	//myprintf("lets check it again 0x%x \r\n", maskdata);

	jqd_status = 0xFFFF;
	max_bladeno = MAX_BLADE_HARDWARE_8;
	first_bladeno=0;
again:

	for(i = first_bladeno; i < max_bladeno; i ++) 
	{
			arch_Jacquard_Setup(offset, i, jqd_status & (0x1 << i),0,0);
			arch_delay_us(100);
			if (checknow)
			{
				extern unsigned char EXTI_CHECK_justcomming(void);
				EXTI_CHECK_justcomming();
			}
			
			arch_Jacquard_ClearBlade(offset,i);

			if(Get_alarm_status_withMASK(maskdata,od_t)) 
			{				
				//myprintf("[%d] JQD overload [%d][%d] mode = %d \r\n", !!jqd_status, offset+1, i+1, mode);
				#ifdef NEW_ALARM_STYLE
				alert_push(OVERLOAD_ERR_CODE_ARG(3, ((offset+1) << 8) | (i + 1));
				#else
				alert_push(OVERLOAD_JQD, ((offset+1) << 8) | (i + 1));
				#endif
				#ifdef E490_V10_BOARD
				arch_8844_Reset_one(board);
				#else
				arch_8844_Reset();
				#endif
				Set_overloadalarm_withMASK(od_t);
			}
			Clear_alarm_status_withMASK(maskdata,od_t);	
			arch_delay_us(10);			
	}

	//myprintf("check it ok 0x%x \r\n", jqd_status);
	
	if(jqd_status) {
		jqd_status = 0x0000;
		goto again;
	}
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	else
	if(steps_other_JQD)	
	{	
		
		 if(sys_max_blade_phi>=MAX_BLADE_HARDWARE_8)
		 {
			if(!(mode & 0x01))  /*如果是0,2两个选针器的话，有可能要多几个*/
			{
				jqd_status =0xFFFF;
				offset = (mode==0)?0:2;
				if(steps_other_JQD==1)
				{
					offset+=1;
				}
				first_bladeno = MAX_BLADE_HARDWARE_8;
				max_bladeno =sys_max_blade_phi;
				steps_other_JQD--;
				goto again;
			}
		 }
		 else
		 {
			steps_other_JQD=0;
		 }
	}
	#endif

overload_return:
	Overload_recheck_finish(od_t);
	
}



void Clear_alarm_status_withMASK(unsigned long maskdata,char *ot)
{
	Overload_t *overload_temp;
	Overload_status_JQD_ACT_YARN  &= ~(maskdata);	
	overload_temp = (Overload_t *)ot;	
	if (overload_temp->st.bit.rover)
	{		
		overload_temp->st.bit.rover=0;
	}
	
}

unsigned long Get_alarm_status_withMASK(unsigned long maskdata,char *ot)
{	
	Overload_t *overload_temp;	
	overload_temp = (Overload_t *)ot;
	return overload_temp->st.bit.rover;

}

void Set_overloadalarm_withMASK(char *ot)
{	
	Overload_t *overload_temp;	
	overload_temp = (Overload_t *)ot;
	overload_temp->st.bit.ralert=1;

}



void arch_ACT_Overload_Check(int board, int mode)
{
	unsigned int ACT_status;	
	int offset;
	unsigned long maskdata;
	char *od_t;
	char i =0;

	if (board==0)
	{
		maskdata = OVERLOAD_MASK_ACT1;
		offset =0;
		od_t = (char *)&SYS_overload_c[OVERLOAD_ACT1];
	}
	else
	{
		maskdata = OVERLOAD_MASK_ACT2;
		offset =1;
		od_t = (char *)&SYS_overload_c[OVERLOAD_ACT2];
	}
			

clear:

	Clear_alarm_status_withMASK(maskdata,od_t);	

	if ((power_st & 0x02)==0)
	{
		goto overload_return;
	}

	power_istest |=0x02;
	arch_set_overload_delay();
	//POWER_XZ_POSITIVE24V_OUTPUT(1);	 //poweron	
	arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,27);
	arch_delay_us(500);
	
	while (arch_get_overload_delay());
	if(Get_alarm_status_withMASK(maskdata,od_t)) 
	{
		i++;
		if(i < 2) 
		{
			wait_ms(5);
			goto clear;
		}
		#ifdef NEW_ALARM_STYLE
		alert_push(OVERLOAD_ERR_CODE_ARG(4, 0);
		#else
		alert_push(OVERLOAD_ACT, 0);
		#endif	
		Set_overloadalarm_withMASK(od_t);
		goto overload_return_3_1;
	}

	ACT_status = 0xFFFF;
again:
	for(i = 0; i < 6; i ++) 
	{
			arch_ACTEMF_Setup(offset * 6 + i, ACT_status & (0x1 << i),0);
			arch_delay_us(100);
			arch_ACTEMF_Clear(offset * 6 + i);
			if(Get_alarm_status_withMASK(maskdata,od_t)) 
			{				
				//myprintf("ACT1 overload [%d][%d]\r\n", offset+1, i+1);
				#ifdef NEW_ALARM_STYLE
				alert_push(OVERLOAD_ERR_CODE_ARG(4, ((offset+1) << 8) | (i+1));
				#else
				alert_push(OVERLOAD_ACT, ((offset+1) << 8) | (i+1));
				#endif
				Set_overloadalarm_withMASK(od_t);
				//POWER_XZ_POSITIVE24V_OUTPUT(0);	 //poweroff
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,28);
				arch_delay_us(10);	
				arch_set_overload_delay();
				//POWER_XZ_POSITIVE24V_OUTPUT(1);	 //poweron
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);
				while (arch_get_overload_delay());

			}
			Clear_alarm_status_withMASK(maskdata,od_t);	
			arch_delay_us(10);			
	}
	if(ACT_status) {
		ACT_status = 0x0000;
		goto again;
	}

overload_return_3_1:
power_istest &=~(0x01<<1);	
overload_return:
	Overload_recheck_finish(od_t);

}

void arch_AD24_Overload_Check(int board, int mode)
{
	//unsigned int yarn_status;
	char i =0;
	unsigned long maskdata;
	char *od_t;

//yarn_poweron:
//POWER_XZ_NEGATIVE24V_OUTPUT(0);



if (mode)
{	
	maskdata = OVERLOAD_MASK_AD_24_P;
	od_t = (char *)&SYS_overload_c[OVERLOAD_P_24];
}
else
{
	maskdata = OVERLOAD_MASK_AD_24_N;
	od_t = (char *)&SYS_overload_c[OVERLOAD_N_24];
}

clear:
	
	Clear_alarm_status_withMASK(maskdata,od_t);
if (mode)
{
	if (power_st & 0x02)  /*打开的*/		
	{
		power_istest |=0x02;
		arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);
		//POWER_XZ_POSITIVE24V_OUTPUT(1);
	}
	goto overload_return_2;	
}
else
{
	if (power_st & 0x01)  /*打开的*/		
	{
		power_istest |=0x01;
		arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);
		//POWER_XZ_NEGATIVE24V_OUTPUT(0);	
	}
	else
		goto overload_return_2;	
}
	arch_set_overload_delay();
	arch_delay_us(500);
	while (arch_get_overload_delay());

	if(Get_alarm_status_withMASK(maskdata,od_t)) 
	{
		i++;
		if(i < 2) 
		{
			wait_ms(10);
			goto clear;
		}
		#ifdef NEW_ALARM_STYLE
		alert_push(OVERLOAD_ERR_CODE_ARG(1, mode?24:0xE8)); 
		#else
		alert_push(OVERLOAD_24V, mode?1:0);
		#endif
		//Message_Send_Alert_DEBUG(OVERLOAD_24V, mode?1:0);
		Set_overloadalarm_withMASK(od_t);
		goto overload_return_2_1;
	}
	

if (mode)
{
	//POWER_XZ_POSITIVE24V_OUTPUT(0); 
	arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,29);
	arch_delay_us(10);	
	arch_set_overload_delay();
	//	POWER_XZ_POSITIVE24V_OUTPUT(1); 
	arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_ON_NP_24,0);
}
else
{
	//POWER_XZ_NEGATIVE24V_OUTPUT(1);
	arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,30);
	arch_delay_us(10);
	arch_set_overload_delay();
	//POWER_XZ_NEGATIVE24V_OUTPUT(0);
	arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);
}

while (arch_get_overload_delay());

overload_return_2_1:
power_istest &=~(0x01<<(mode?1:0));		

overload_return_2:
	Overload_recheck_finish(od_t);

}



void arch_YARN_Overload_Check(int board, int mode)
{
	unsigned int yarn_status;
	char i =0;
	char * od_t;

	od_t = (char *)&SYS_overload_c[OVERLOAD_N_YARN_24];
	
clear:

	Clear_alarm_status_withMASK(OVERLOAD_MASK_YARN,od_t);	


	//if ((system_power_24_isoverload & 0x01))
	//{
	//	goto overload_return_1;
	//}


	//20170418 纱嘴过流测试不到具体的电磁铁。
	//if ((power_st&0x01)==0)   /*关闭了*/
	//goto overload_return_1;	

	arch_set_overload_delay();
	power_istest |=0x01;
	//POWER_XZ_NEGATIVE24V_OUTPUT(0);	 //poweron
	arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);
	arch_delay_us(500);
	while (arch_get_overload_delay());
	
	if(Get_alarm_status_withMASK(OVERLOAD_MASK_YARN,od_t)) 
	{
		i++;
		if(i < 2) 
		{
			wait_ms(10);
			goto clear;
		}
		#ifdef NEW_ALARM_STYLE
		alert_push(OVERLOAD_ERR_CODE_ARG(2, 0)); 
		#else
		alert_push(OVERLOAD_YARN, 0);
		#endif
		Set_overloadalarm_withMASK(od_t);
		goto overload_return_1_1;
	}
	yarn_status = 0xFFFF;

	
	
again:

	for(i = 0; i < MAX_YARN; i ++) {
		arch_YARNEMF_Setup(i, yarn_status & (0x1 << i),0);
		arch_delay_us(100);
		arch_YARNEMF_Clear(i);
		if(Get_alarm_status_withMASK(OVERLOAD_MASK_YARN,od_t)) 
		{
			//myprintf("YARN overload %d \r\n", i+1);

			#ifdef NEW_ALARM_STYLE
			alert_push(OVERLOAD_ERR_CODE_ARG(2, i+1)); 
			#else
			alert_push(OVERLOAD_YARN, i + 1);		
			#endif
			Set_overloadalarm_withMASK(od_t);
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);	 //poweroff
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,31);
			arch_delay_us(10);	
			arch_set_overload_delay();
			//POWER_XZ_NEGATIVE24V_OUTPUT(0);	 //poweron
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_ON_NP_24,0);
			while (arch_get_overload_delay());
			
		}
		Clear_alarm_status_withMASK(OVERLOAD_MASK_YARN,od_t);
		arch_delay_us(10);
	}
	if(yarn_status) 
	{
		yarn_status = 0x0000;
		goto again;
	}

overload_return_1_1:

power_istest &=~(0x01<<0);
//overload_return_1:
	Overload_recheck_finish(od_t);
}

void Overload_Init()
{	char i=0;
	for(i=0;i<OVERLOAD_MAX;i++)
	{
		memset(&SYS_overload_c[i],0,sizeof(Overload_t));
		SYS_overload_c[i].st.bit.index=i;
		SYS_overload_c[i].Max_overload_cnt = (i>=OVERLOAD_YARN_F)?3:10;
		SYS_overload_c[i].rcheck_time_out = (i>=OVERLOAD_YARN_F)?50:200;	/*200ms*/
	}
}


int Overload_recheck_finish(char * Ot)
{
	//int ret;
	Overload_t *overload_temp;
	unsigned int time;
	
	overload_temp = (Overload_t *)Ot;
	overload_temp->st.bit.ralert=0;
	overload_temp->st.bit.rchk=0;
	overload_temp->st.bit.rover = 0;
	overload_temp->st.bit.rpower=1;
	time = arch_get_ticktime();
	time -= overload_temp->Last_power_time;
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	if ((time>>2)<1000)  /*在一秒钟内，又来了一次*/
	{
		if (overload_temp->overload_cnt++ > overload_temp->Max_overload_cnt)
		{
			overload_temp->st.bit.rpower=0;   /*表示不能再上电了*/
		}
	}
	else
		overload_temp->overload_cnt =0;  /*计数清零*/	
	
	overload_temp->Last_power_time=arch_get_ticktime();
	return  overload_temp->st.bit.rpower;
}

int Overload_istimeout(char * Ot)
{
	unsigned int time;
	Overload_t *overload_temp;
	overload_temp = (Overload_t *)Ot;
	
	time = arch_get_ticktime();
	time -= overload_temp->first_overload_time;
	time = abs(time);
	if (time > 0x7FFFFFFF)
		time = 0xFFFFFFFF - time;
	if ((time>>2) > overload_temp->rcheck_time_out)   	// 250us 转换成1ms 
	{		
		return 1;
	}
	else return 0;	

}

void Overload_is_reCheck(char * Ot)
{
	//unsigned int time;
	Overload_t *overload_temp;
	overload_temp = (Overload_t *)Ot;
	if (overload_temp->st.bit.rchk)
	{
		switch (overload_temp->st.bit.index)
		{
			case OVERLOAD_N_24:
			case OVERLOAD_P_24:
				arch_AD24_Overload_Check(0x0, overload_temp->st.bit.index-OVERLOAD_N_24);//正的话，那就直接退出了		
				whichOverload_in_progress_mask &= ~((unsigned int)0x01<<(overload_temp->st.bit.index-OVERLOAD_N_24));
				break;
			case OVERLOAD_N_YARN_24:
				if (!Yarn_use_Step)
				{
					arch_YARN_Overload_Check(0x0, 0x0);
					whichOverload_in_progress_mask &= ~((unsigned int)0x01<<0);
				}
				break;
			case OVERLOAD_ACT1:				
			case OVERLOAD_ACT2:
				arch_ACT_Overload_Check(overload_temp->st.bit.index-OVERLOAD_ACT1, 0x0);
				whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
				break;
			case OVERLOAD_XZQ1:				
			case OVERLOAD_XZQ2:				
			case OVERLOAD_XZQ3:				
			case OVERLOAD_XZQ4:
				arch_Jacquard_Overload_Check(overload_temp->st.bit.index-OVERLOAD_XZQ1,0);
				whichOverload_in_progress_mask &= ~((unsigned int)0x03<<0);
				break;
			case OVERLOAD_YARN_F:
				arch_power_IC_fault_check(overload_temp->st.bit.index-OVERLOAD_YARN_F);
				whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
				break;
			case OVERLOAD_N_F:
				arch_power_IC_fault_check(overload_temp->st.bit.index-OVERLOAD_YARN_F);
				whichOverload_in_progress_mask &= ~((unsigned int)0x01<<0);
				break;
			case OVERLOAD_P_F:
				arch_power_IC_fault_check(overload_temp->st.bit.index-OVERLOAD_YARN_F);
				whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
				break;	
			default:
				break;
		}
		
		arch_overload_check_poweroff_(Ot);	
				
	}
	else
		return;
	
		

}



int Overload_is_come(unsigned int overloadindex)
{
	char whichoverload=0;
	Overload_t *overload_temp;
	//char ret;
	switch(overloadindex)
	{
		case OVERLOAD_MASK_ACT1:
			whichoverload = OVERLOAD_ACT1;
		break;
		case OVERLOAD_MASK_ACT2:
			whichoverload = OVERLOAD_ACT2;
		break;
		case OVERLOAD_MASK_YARN:
			whichoverload = OVERLOAD_N_YARN_24;
		break;
		case OVERLOAD_MASK_AD_24_N:
			whichoverload = OVERLOAD_N_24;
		break;
		case OVERLOAD_MASK_AD_24_P:
			whichoverload = OVERLOAD_P_24;
		break;		
		case OVERLOAD_MASK_XZ1:
			whichoverload = OVERLOAD_XZQ1;
		break;
		case OVERLOAD_MASK_XZ2:
			whichoverload = OVERLOAD_XZQ2;
		break;
		case OVERLOAD_MASK_XZ3:
			whichoverload = OVERLOAD_XZQ3;
		break;
		case OVERLOAD_MASK_XZ4:
			whichoverload = OVERLOAD_XZQ4;
		break;	
		case OVERLOAD_MASK_YARN_F:
			whichoverload = OVERLOAD_YARN_F;
		break;	
		case OVERLOAD_MASK_24V_C_N:
			whichoverload = OVERLOAD_N_F;
		break;
		case OVERLOAD_MASK_24V_C_P:
			whichoverload = OVERLOAD_P_F;
		break;
		default:
			return 0;
	}
	overload_temp = &SYS_overload_c[whichoverload];	
	//ret =0;
	if ((overload_temp->st.bit.rchk==0)&&(overload_temp->st.bit.rover==0))
	{		
		overload_temp->first_overload_time=arch_get_ticktime();
		overload_temp->st.bit.ralert =1;
		overload_status |= (1<<whichoverload);
		//ret =1;
	}
	else
	{
		overload_temp->st.bit.ralert =0;
		overload_status &= ~(1<<whichoverload);
	}
	overload_temp->st.bit.rover=1;
	return overload_temp->st.bit.ralert;
	
	
}

extern volatile unsigned int alertcode;
void Overload_Poll()
{
	//unsigned long status;
	unsigned char i=0;
	Overload_t *overload_temp;
	if (!overload_check_on) return;	//不检测 那就退出
	for(i=0;i<OVERLOAD_MAX;i++)
	{
		overload_temp = &SYS_overload_c[i];
		if (overload_temp->st.bit.rover)/*首先已经报警了*/
		{	
			if (Overload_istimeout((char *)overload_temp))
			{
				overload_temp->st.bit.rchk=1;
				Overload_is_reCheck((char *)overload_temp);
				overload_temp->st.bit.ralert=0;				
				overload_temp->st.bit.rchk=0;
				//overload_temp->first_overload_time=0;
			}
			else
			{
				overload_temp->st.bit.rchk=0;	
			}
			continue;

		}
		else
			continue;

	}

#if 0	

	status = Get_Overload_Status();
	if (status & 0x03) return ;
	if(status) 
	{
		if(overload_count > 10  * 50) {
			if(alertcode == 0) {
				overload_count = 0;
			}
		       	return ;
		}
		if(overload_delay > 0) return ;
		overload_delay = 100;
		myprintf("overload check 0x%x \r\n", status);
		goto overload_check;
	} 
	else overload_count = 0;

	if((overload_status == 0) ||
	   (overload_delay > 0)) {
		return ;
	}

overload_check:

	myprintf("overload status 0x%x \r\n", status);

	if(status & OVERLOAD_MASK_AD_24_N) {
		arch_AD24_Overload_Check(0x0, 0x0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x01<<0);
		
	}

	
	if(status & OVERLOAD_MASK_AD_24_P) {
		arch_AD24_Overload_Check(0x0, 0x1);//正的话，那就直接退出了
		//alert_push(OVERLOAD_24V, 1);
		whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
		//goto exit_return;
		
	}
	
	if(status & OVERLOAD_MASK_YARN) {		

#ifdef E480_BOARD_V10
		if (!Yarn_use_Step)
		{
			arch_YARN_Overload_Check(0x0, 0x0);
			whichOverload_in_progress_mask &= ~((unsigned int)0x01<<0);
		}
	
#else
		arch_YARN_Overload_Check(0x0, 0x0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x01<<0);
#endif
		
	
	}


	if (status & OVERLOAD_MASK_ACT1)
	{
		arch_ACT_Overload_Check(0x0, 0x0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
	}
	if (status & OVERLOAD_MASK_ACT2)
	{
		arch_ACT_Overload_Check(0x1, 0x0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x01<<1);
	}
	

	if (status & OVERLOAD_MASK_XZ1)
	{
		//myprintf("arch_Jacquard_Overload_Check 0x%x \r\n", status);
		arch_Jacquard_Overload_Check(0,0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x03<<0);
	}

#if 0
	if (status & OVERLOAD_MASK_XZ1_P)
	{
		arch_Jacquard_Overload_Check(0,1);
	}

	#endif

	if (status & OVERLOAD_MASK_XZ2)
	{
		arch_Jacquard_Overload_Check(1,0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x03<<0);
	}
#if 0
	if (status & OVERLOAD_MASK_XZ2_P)
	{
		arch_Jacquard_Overload_Check(1,1);
	}
#endif	

	if (status & OVERLOAD_MASK_XZ3)
	{
		arch_Jacquard_Overload_Check(2,0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x03<<0);
	}
#if 0
	if (status & OVERLOAD_MASK_XZ3_P)
	{
		arch_Jacquard_Overload_Check(2,1);
	}
#endif

	if (status & OVERLOAD_MASK_XZ4)
	{
		arch_Jacquard_Overload_Check(3,0);
		whichOverload_in_progress_mask &= ~((unsigned int)0x03<<0);
	}
#if 0
	if (status & OVERLOAD_MASK_XZ4_P)
	{
		arch_Jacquard_Overload_Check(3,1);
	}
#endif
	overload_status = 0;
	return;

exit_return:
	whichOverload_in_progress_mask =0; 
	return ;
#endif

}

void WatchDog_Kick(void)
{
#ifdef WDG_ENABLE
	IWDG_ReloadCounter();
#endif
}

pFunction Jump_To;
volatile uint32_t JumpAddress;
void ReBoot(void)
{
#if 0
	JumpAddress = *(volatile uint32_t*) (BOOT_START_ADDRESS + 4);
	Jump_To = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__MSR_MSP(*(volatile uint32_t*) BOOT_START_ADDRESS);
	Jump_To();
#else
	//NVIC_SETFAULTMASK();
	//NVIC_GenerateSystemReset();
	NVIC_SystemReset();
	//NVIC_GenerateCoreReset();
#endif
}

void GoApp(void)
{
#if 0
	//myprintf("\r\nGoto Application\r\n");
	JumpAddress = *(volatile uint32_t*) (APP_START_ADDRESS + 4);
	Jump_To = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__MSR_MSP(*(volatile uint32_t*) APP_START_ADDRESS);
	Jump_To();
	#endif
}


#ifdef BOOT_UPGRADE_CHECK_CRC
extern unsigned char * arch_Uprade_get_Data_Start_add();
extern unsigned short arch_have_crc_flag(unsigned int *arg);
extern unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, int CRCinit);
extern unsigned short arch_crc_ok(unsigned char *buff,unsigned short CRCdata,unsigned long datelen,unsigned short *crcd);
#endif


#define BUFF_SIZE_BOOT 0x100
void Upgrade_Boot(void)
{
	myprintf("upgrade boot start \n\r");

	wait_us(10);
	//
	CLI();

	{
		int Count;
		unsigned long Data_Count;
		unsigned long addr;
		unsigned short *pData;
		unsigned int cmd_len;
		unsigned int cmd_Rcv;
		unsigned short Buff[BUFF_SIZE_BOOT];
		#ifdef BOOT_UPGRADE_CHECK_CRC
		unsigned short lastdata[3]={0xffff,0xffff,0xffff};
		#endif
		unsigned short delay_t=4000;
		
		fmc_init(fmc_start_addr(1));
		

		Message_Send_immediately(UPGRADE_REQUEST);

		//arch_Upgrade_Set_Receive();
		Count = 0;
		Data_Count = 0;

		//Buff = malloc(BUFF_SIZE_BOOT);

		arch_LED_On();
		while(1) {
			pData = (unsigned short*)CAN0_Receive(NULL);
			if(pData == NULL) {
				continue;
			}
			Message_Send_immediately(UPGRADE_DATA_RECEIVED);

			
			cmd_Rcv = *pData & 0xFF;
			cmd_len = 3;
			if(cmd_Rcv == UPGRADE_ENDDATA) {
				cmd_len = (*pData >> 8) & 0xFF;
			}

			do {
				pData ++;
				Buff[Count++] = *pData;

				Data_Count ++;

				if(Count == BUFF_SIZE_BOOT) {
					arch_LED_On();
					#ifdef BOOT_UPGRADE_CHECK_CRC
					{
						int i;
						for (i=0;i<3;i++)
						{
							lastdata[i] = Buff[Count-3+i];
						}
					}
					#endif
					fmc_program((BYTE*)&Buff, Count << 1);
					arch_LED_Off();
					Count = 0;
				}
			}while(--cmd_len);

			//SendMassage(UPGRADE_DATA_RECEIVED);

			if(cmd_Rcv == UPGRADE_ENDDATA) {
				if(Count) {
					arch_LED_On();
					#ifdef BOOT_UPGRADE_CHECK_CRC
					{
						int i;
						if(Count>=3)
						{
							for (i=0;i<3;i++)
							{
								lastdata[i] = Buff[Count-3+i];
							}
						}
						else
						{
							for (i=0;i<3;i++)
							{
								if (i<(3-Count))
								{
									lastdata[i] = lastdata[i+1];
								}	
								else
								{
									lastdata[i] = Buff[i-(3-Count)];
								}
							}							
						}
					}
					#endif
					fmc_program((BYTE*)&Buff, Count << 1);
					arch_LED_Off();
				}
				Data_Count <<= 1;
				//fmc_Verify(SECTOR_ADDR_0, SECTOR_ADDR_5, Data_Count);

				dprintf("Total boot Count %d\r\n", Data_Count);
				//arch_Upgrade_Set_Burn_Size(Data_Count);
				//arch_Upgrade_Set_Burn();

				//arch_Upgrade_APP_Flag_init();

				#ifdef BOOT_UPGRADE_CHECK_CRC
				//dprintf("Upgrade Get_CRC_Flag(CRC_Flag:[0x%X,0x%X,0x%X])\r\n",lastdata[0],lastdata[1],lastdata[2]);
						
				if(arch_have_crc_flag((unsigned int *)&lastdata[0]))
				{
					unsigned short CRC_data_com;
					if(!arch_crc_ok(arch_Uprade_get_Data_Start_add(),lastdata[2],Data_Count-6,&CRC_data_com))
					{

						dprintf("Upgrade UNsuccessful(CRC_ERROR:[0x%X--0x%X])\r\n",lastdata[2],CRC_data_com);
						Message_Send_immediately(UPGRADE_DATA_RECEIVEDAPPCRC_ERROR);
						goto boot_upg_end;
						
					}
					//dprintf("Upgrade Successful(CRC_PASS:[0x%X--0x%X])\r\n",lastdata[2],CRC_data_com);
						
					
				}
				
				#endif

				#ifdef BOOT_UPGRADE_CHECK_CRC
				//arch_Upgrade_Set_CRC_Data(lastdata[2]);
				arch_Upgrade_Set_Buf_Type(0);//0
				
				#endif
				

				fmc_init(fmc_start_addr(2));
				arch_LED_On();
				dprintf("fmc_EraseSector_boot\r\n");
				fmc_EraseSector_boot();
				addr = fmc_start_addr(1);
				fmc_program((BYTE *)addr, Data_Count);
				arch_LED_Off();
				break;
			}
		}

		Message_Send_immediately(UPGRADE_SUCCESS);

		//arch_Upgrade_Finish();
		arch_LED_Off();

		dprintf("Upgrade boot Successful\r\n");

		boot_upg_end:
		arch_LED_Setup(1000);
		delay_t=4000;
		while(1) {
			if(delay_t)
			{
				Message_Send_Alert(DSP_FATAL_ERR,0);
			}
			//Message_Send_Alert(DSP_FATAL_ERR,0);
			delay_t=4000;
			while(CAN0_Receive(NULL) == NULL)
			{
				delay_t--;
				if (delay_t==0)
					break;
			}
			//break;
			//goto yes_can_gotoapp;
		}		
	}	

	SEI();

}
void Upgrade_Start(void)
{
#define CMD_UPGRADE_START	0xA050AA00
#define CMD_UPGRADE_RCVDATA	0xA050AA55
#define CMD_UPGRADE_BURN	0xA0505A5A
#define CMD_UPGRADE_SUCCESS	0xA05055AA
	u32 upgrade_cmd = CMD_UPGRADE_START;
	//writeflash_index=WRTEFLASH_BASE+1;
	EE_Write(0, (u16*)&upgrade_cmd, 2);

	myprintf("upgrade start \n\r");

	wait_us(10);

	ReBoot();
}

void Jtag_Security(void)
{
#if 0
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	// 改变指定管脚的映射 GPIO_Remap_SWJ_Disable SWJ 完全禁用（JTAG+SW-DP）
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能
	GPIO_Configuration(); // 配置使用的 GPIO 口
#endif
#ifdef FLASH_READ_PROTECTED
	if(FLASH_OB_GetRDP() == RESET)
	{
		FLASH_Unlock();	
		FLASH_OB_Unlock();	
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGPERR | FLASH_FLAG_WRPERR);		
		FLASH_OB_RDPConfig(OB_RDP_Level_1); //读保护1     
		FLASH_OB_Launch();
	}
	#endif
}

int Jtag_Security_State(void)
{
	return (FLASH_OB_GetRDP() == RESET);
	//return (FLASH_GetReadOutProtectionStatus() == SET);
}

void arch_LED_On(void)
{
	//GPIO_ResetBits(GPIOD, GPIO_Pin_2);
	LED_OUTPUT(0);
	//SHOCK_PULS_OUTPUT(0);
}

void arch_LED_Off(void)
{
	//GPIO_SetBits(GPIOD, GPIO_Pin_2);
	LED_OUTPUT(1);
	//SHOCK_PULS_OUTPUT(1);
}

void arch_FAN_OFF()
{
	FAN_CTR_OPEN(0);
}


void arch_FAN_ON()
{
	FAN_CTR_OPEN(1);
}

unsigned char arch_FAN_JC()
{
	return FAN_JC_INPUT()?1:0;
}

void arch_LED_Setup(DWORD time)
{
	led_delay_setup = time;
}

void arch_LED_timer(void)
{
	//static unsigned short led_cnt=0;
	if(led_delay_setup == 0) return;
	if(led_delay) {
		led_delay --;
		return ;
	}

	led_delay = led_delay_setup;
	led_status = !led_status;

	#ifndef DEBUG_MT6813_PWM
	if(led_status) arch_LED_On();
	else arch_LED_Off();
	#endif

	//Message_Send_4halfword(0xAABB, basetime_tick, basetime_tick>>16, led_cnt++);
}

void arch_LED_Start(void)
{
	led_disable = 0;
}

void arch_LED_Stop(void)
{
	led_disable = 1;
}


extern volatile unsigned int tanzhen_delay[2];
extern volatile unsigned int tanzhen_active[2];

extern volatile unsigned int tanzhen_alarm_delay_num[2];
extern volatile unsigned int tanzhen_alarm_delay_count[2];
extern volatile unsigned int tanzhen_alarm[2];

int alert_push(int alert_code, int alert_arg);
#define ERR_DELAY	10
//extern unsigned long arch_adc_test();

 unsigned int err_cnt[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// unsigned long test_count_whichdo[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
#define MAX_CURR_OVERLOAD_CNT 	50 
#define MAX_CURR_OVERLOAD_CNT_YARN 	10 

											//(2)		20180831 由原来的2改成10
void arch_adc_isr()
{
	extern unsigned char DA_is_out;
	//extern unsigned int shock_is_created;	
	extern volatile unsigned int DC24_P_CURR_A ;
	extern volatile unsigned int DC24_N_CURR_A ;
	extern volatile unsigned int DC24_N_CURR_A_Yarn;
	extern unsigned long arch_adc_test(void);
	int ol=0;


	//test_count_whichdo[12]++;
	//dc_status |= arch_adc_test();
	dc_status = arch_adc_test();  /*2017 11 16  之前是触发了报警，就需要报出来。改成查询的时候有报警才报*/
	
	if (!DA_is_out) 
	{
		dc_status = 0;
	}

#ifndef  USE_ADC2_
	if (poweron_delay==0)
	{	

		if ((!Yarn_use_Step))   //纱嘴过流
		{
			if (DC24_N_CURR_A_Yarn>(DC24N_ALARM_Yarn_set+1650))
			{
				if ((DC24_CURR_A_overload_cnt[2]++ >MAX_CURR_OVERLOAD_CNT)||(power_istest&0x01))
				{					
					//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,32);
					ol =Overload_is_come(OVERLOAD_MASK_YARN);
						
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_YARN;			
					whichOverload_in_progress_mask |=0x01; 
					
					//CheckOverloadBranch();
				}	
			
			}
			else
				DC24_CURR_A_overload_cnt[2]=0;
		}
	
		if (DC24_N_CURR_A>(DC24N_ALARM_set+1650))
		{
			// 负24伏过流
			if ((DC24_CURR_A_overload_cnt[0]++ >MAX_CURR_OVERLOAD_CNT)||(power_istest&0x01))
			{				
				//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
				arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,33);
				whichOverload_in_progress_mask |=0x01; 

				#ifdef E480_BOARD_V10
				if (((Overload_status_JQD_ACT_YARN & OVERLOAD_MASK_NOYARN )==0)&&(!Yarn_use_Step))  // 说明没有别的过流
				{
					ol =Overload_is_come(OVERLOAD_MASK_YARN);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_YARN;			
				}
				else
				{
					ol =Overload_is_come(OVERLOAD_MASK_AD_24_N);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_AD_24_N;
				}
				#else

				if ((Overload_status_JQD_ACT_YARN & OVERLOAD_MASK_NOYARN )==0)  // 说明没有别的过流
				{
					ol =Overload_is_come(OVERLOAD_MASK_YARN);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_YARN;			
				}
				else
				{
					ol =Overload_is_come(OVERLOAD_MASK_AD_24_N);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_AD_24_N;
				}
				#endif
				//CheckOverloadBranch();
				//ol=1;
			}	
			
		}
		else
			DC24_CURR_A_overload_cnt[0]=0;

		if (DC24_P_CURR_A>(DC24P_ALARM_set+1650) ) 
		{
			if ((DC24_CURR_A_overload_cnt[1]++>MAX_CURR_OVERLOAD_CNT)||(power_istest&0x02))
			{
			// 正24伏过流
				//POWER_XZ_POSITIVE24V_OUTPUT(0); 
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,34);
				whichOverload_in_progress_mask |=0x02; 
				ol =Overload_is_come(OVERLOAD_MASK_AD_24_P);

				Overload_status_JQD_ACT_YARN |=OVERLOAD_MASK_AD_24_P;
			//	CheckOverloadBranch();
				
			}			
		}
		else
			DC24_CURR_A_overload_cnt[1] =0;
	
	}

	
#endif
if (ol)
	CheckOverloadBranch();	
}

#ifdef  USE_ADC2_


void arch_adc2_isr()
{
	char ol=0;
	extern volatile unsigned int DC24_N_CURR_A_Yarn;
	extern void arch_adc_test_DMA2_string2_ch1(void);
	arch_adc_test_DMA2_string2_ch1();
	if (poweron_delay==0)
	{

		if ((!Yarn_use_Step))   //纱嘴过流
		{
			if (DC24_N_CURR_A_Yarn>(DC24N_ALARM_Yarn_set+1650))
			{
				if ((DC24_CURR_A_overload_cnt[2]++ >MAX_CURR_OVERLOAD_CNT_YARN)||(power_istest&0x01))
				{					
					//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,35);
					ol=Overload_is_come(OVERLOAD_MASK_YARN);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_YARN;			
					whichOverload_in_progress_mask |=0x01; 
					
					//CheckOverloadBranch();
				}				
			}
			else
				DC24_CURR_A_overload_cnt[2]=0;
		}

		if (DC24_N_CURR_A>(DC24N_ALARM_set+1650))
		{
			// 负24伏过流
			if ((DC24_CURR_A_overload_cnt[0]++ >MAX_CURR_OVERLOAD_CNT)||(power_istest&0x01))
			{	
			
				//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
				arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,36);
				whichOverload_in_progress_mask |=0x01; 
				#ifdef E480_BOARD_V10
				if (((Overload_status_JQD_ACT_YARN & OVERLOAD_MASK_YARN ))&&(!Yarn_use_Step))  // 说明没有别的过流
				{
					ol=Overload_is_come(OVERLOAD_MASK_YARN);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_YARN;			
				}
				else
				{
					ol=Overload_is_come(OVERLOAD_MASK_AD_24_N);
					Overload_status_JQD_ACT_YARN|=OVERLOAD_MASK_AD_24_N;	
				}	
				
				#endif
				
			//CheckOverloadBranch();
			}
			
		}
		else
			DC24_CURR_A_overload_cnt[0]=0;

		if (DC24_P_CURR_A>(DC24P_ALARM_set+1650) ) 
		{
			if ((DC24_CURR_A_overload_cnt[1]++>MAX_CURR_OVERLOAD_CNT)||(power_istest&0x02))
			{
				// 正24伏过流
				//POWER_XZ_POSITIVE24V_OUTPUT(0); 
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,37);
				whichOverload_in_progress_mask |=0x02;
				ol=Overload_is_come(OVERLOAD_MASK_AD_24_P);

				Overload_status_JQD_ACT_YARN |=OVERLOAD_MASK_AD_24_P;
			//	CheckOverloadBranch();
				
			}			
		}
		else
			DC24_CURR_A_overload_cnt[1]=0;	
	}

	if (ol)
		CheckOverloadBranch();
}

#else
void arch_adc2_isr()
{
	return;
}
#endif


 volatile  int Alarm_TEMP_Data_=1200;
 volatile unsigned char fan_st_sys=0;

 volatile unsigned int FAN_Start_TEMP_Data_=400;
 volatile unsigned int FAN_Close_TEMP_Data_=350;
 #define FAN_CHECK_DELAY_TIME_2MS		500
 volatile unsigned int FAN_Start_delay_2ms = 0;/*风扇控制打开之后间隔多久开始检查风扇是否转堵转*/
 


void arch_ctr_fan_with_temp_(unsigned int tempdata)
{
	static unsigned char fan_st=0;
	//static unsigned char fan_err_cnt=0;

	if (tempdata>=FAN_Start_TEMP_Data_)
	{
		if (!fan_st)  /*说明风扇没打开*/
		{
			fan_st =1;
			FAN_Start_delay_2ms=FAN_CHECK_DELAY_TIME_2MS;
			arch_FAN_ON();

			//myprintf("\r\n==arch_FAN_ON()==\r\n");
			
		}	
	}
	else 
		if (tempdata<=FAN_Close_TEMP_Data_)
		{
			fan_st =0;
			FAN_Start_delay_2ms =0;
			arch_FAN_OFF();	
			//myprintf("\r\n==arch_FAN_OFF()==\r\n");
		}
		
	fan_st_sys = FAN_JC_INPUT();
	#if 0
	if (fan_st)			/*开启风扇了*/
	{
		if (!FAN_Start_delay_2ms)/*延时到了*/
		{
			if (FAN_JC_INPUT())/*正常*/
			{
				fan_err_cnt =0;
			}else		/*没接或堵转了*/
			{
				if (fan_err_cnt++ >10)
				{
					if(alert_find(FAN_STALL_ERROR, 0) == 0) 	
					{
						alert_push(FAN_STALL_ERROR,0);
					}
					FAN_Start_delay_2ms=FAN_CHECK_DELAY_TIME_2MS;
					fan_err_cnt =0;
					//myprintf("\r\n==arch_FAN_alertpush==\r\n");
				}
			}
		}
		else
			fan_err_cnt =0;
	}
	else
	{
		if (fan_err_cnt )
			fan_err_cnt =0;	
	}

	#endif	
	
	
}

void check_fan_ctr()
{
	unsigned int tmp_ = 250L -((long)(760-TempS_CURR_V) * 100L)/25L ;

	if (arch_is_E690_board())
	{
		arch_ctr_fan_with_temp_(tmp_);		
	}
}

unsigned int Get_TZ_error_st(unsigned char whichtz)
{
	extern volatile unsigned char Head_Mode_;
	#ifdef E692_STEPMOTOR_EX
	if(Head_Mode_==HEAD_MODE_ALL_STEP_CX)
	{
		if((arch_board_id)&&(whichtz))
			return TANZHEN_EX_IN();
		if((arch_board_id==0)&&(whichtz==0))
			return TANZHEN_EX_IN();
		return 0;
	}
	else	
	#endif
	{
		if(whichtz==0)
		{
			return ERROR1_INPUT();
		}
		else
		{
			return ERROR2_INPUT();			
		}
	}
}


unsigned int Scan_error(void)
{
	extern volatile unsigned char alert_runtime_check;
	unsigned long dc_status_loc=dc_status;

	if(!alert_runtime_check)
		return 0;

if (arch_is_shock_board())
{
	
	if ((error_active &  ERROR_ACTIVE_MASK_SHOCK ))  //撞针接线，0前床，1后场，报警内容0-后床，1--前床
	{
		if ((shock_is_created==0)&&(shock_enable))
		{
			if(SHOCK_INPUT()) {
				err_cnt[0] ++;
				if(err_cnt[0] > ERR_DELAY) {
					#ifdef NEW_ALARM_STYLE
					if(alert_find(OTHER_ERR_CODE_ARG(0,0)) == 0) 
					{		
						alert_push(OTHER_ERR_CODE_ARG(0,0));
					}
					#else
					if(alert_find(SHOCK_ALARM, 1) == 0) 					
					{						
						alert_push(SHOCK_ALARM, 1);
						
					}
					#endif
					shock_is_created =1;
				}
			}	
			else {
				err_cnt[0] = 0;
			}
		}
		#ifdef ALARM_SHOCK_DOUBLE
		if ((shock_2_is_created==0)&&(shock_2_enable))
		{
			if(SHOCK_2_INPUT()) {
				err_cnt[1] ++;
				if(err_cnt[1] > ERR_DELAY) {
					#ifdef NEW_ALARM_STYLE
					if(alert_find(OTHER_ERR_CODE_ARG(0,1)) == 0) {						
						alert_push(OTHER_ERR_CODE_ARG(0,1));					
					}
					#else
					if(alert_find(SHOCK_ALARM, 0) == 0) {						
						alert_push(SHOCK_ALARM, 0);						
					}
					#endif
					shock_2_is_created =1;
				}
			}	
			else {
				err_cnt[1] = 0;
			}
		}
		#endif
		
	}
}

/*
	if ( DC24N_max<DC24_N_CURR_A)
	{
		DC24N_max = DC24_N_CURR_A;
		

		//if ((DC24N>1800)||(DC24P>1800))
			{
		myprintf("DC24_N = %d MV,%d \n\r",DC24N_max,DC24N_max-1650);
			}
	}
	if ( (DC24P_max<DC24_P_CURR_A))
	{
		DC24P_max = DC24_P_CURR_A;
		myprintf("DC24_P =%d MV,%d \n\r",DC24P_max,DC24P_max-1650);
	}
*/

if(error_active & ERROR_ACTIVE_MASK_DC24_N) {
		if(((dc_status_loc & ALARM_N_24_LOW_MASK)&&(!N24_on_delay_1s))
			||(dc_status_loc & ALARM_N_24_HIGH_MASK )
			||((dc_status_loc & ALARM_N_24_LOW_LOW_MASK )&&(!N24_on_delay_1s))) 
		{
			err_cnt[5] ++;
			if(err_cnt[5] > ERR_DELAY) {
				#ifdef NEW_ALARM_STYLE
				if(alert_find(POWER_ERR_CODE_ARG(0,0xE8|0xF1)) == 0) {					
					alert_push(POWER_ERR_CODE_ARG(0,0xE8|0xF1));				
				}
				#else
					#ifdef DC_ERR_FN_ALERT
					{
						unsigned short arg=0;
						unsigned short argt=0xF1;
						#ifdef CHECK_24_ALERT_
						if (dc_status_loc & ALARM_N_24_LOW_MASK)
						{
							argt = 0xE2;
						}
						if (dc_status_loc & ALARM_N_24_LOW_LOW_MASK)
						{
							argt = 0xD2;
						}
						#endif

						
						arg = 0xE8 | (((arch_board_id & 0x03)*3 + argt)<<8) ;
						if (alert_find(DC24_N_PERR, arg) == 0) {					
							alert_push(DC24_N_PERR, arg);					
						}
					}
					#else
					
					if(alert_find(DC24_N_PERR, 0) == 0) {					
						alert_push(DC24_N_PERR, 0);					
					}
					#endif
				#endif

				dc_status_loc &=~(ALARM_N_24_LOW_MASK);
				dc_status_loc &=~(ALARM_N_24_HIGH_MASK);
				dc_status_loc &=~(ALARM_N_24_LOW_LOW_MASK);
				
				
			}
		}
		else {
			err_cnt[5] = 0;
		}
		//test_count_whichdo[4]++;
	}

#ifndef E490_V10_BOARD
if (!arch_is_E490_board())////////////////////////E490v10 is exist
#endif
{
	
		if(error_active & ERROR_ACTIVE_MASK_DC24_P) 
		{	
			int alert_code=0;
		#ifdef DC24_P_PERR_BC_L		

			if (dc_status_loc & ALARM_DC24_LOW_MASK)
			{
				alert_code = DC24_P_PERR_BC_L;
			}
			if (dc_status_loc & ALARM_DC24_HIGH_MASK)
			{
				alert_code = DC24_P_PERR_BC_H;
			}
			if (dc_status_loc & ALARM_DC24_LOW_LOW_MASK)
			{
				alert_code = DC24_P_PERR_BC_LL;
			}

		if (alert_code)	
		#else
		 alert_code=DC24_P_PERR ;
		if(((dc_status_loc & ALARM_DC24_LOW_MASK)&&(!P24_on_delay_1s)) 
			||(dc_status_loc & ALARM_DC24_HIGH_MASK )
			||((dc_status_loc & ALARM_DC24_LOW_LOW_MASK)&&(!P24_on_delay_1s))) 
		#endif
		{			
			err_cnt[4] ++;
			if(err_cnt[4] > ERR_DELAY) 
			{
				#ifdef NEW_ALARM_STYLE
				if(alert_find(POWER_ERR_CODE_ARG(0,24|0xF2)) == 0) {
					
					alert_push(POWER_ERR_CODE_ARG(0,24|0xF2));
					
				}
				#else
					#ifdef DC_ERR_FN_ALERT
					{
						unsigned short arg=0;
						unsigned short argt=0xF2;
						#ifdef CHECK_24_ALERT_
						if (dc_status_loc & ALARM_DC24_LOW_MASK)
						{
							argt = 0xE2;
						}
						if (dc_status_loc & ALARM_DC24_LOW_LOW_MASK)
						{
							argt = 0xD2;
						}
						#endif
						arg = 0x18 | (((arch_board_id & 0x03)*3 + argt)<<8) ;
						if (alert_find(alert_code, arg) == 0) {					
							alert_push(alert_code, arg);					
						}
					}
					#else
					
					if(alert_find(alert_code, 0) == 0) {					
						alert_push(alert_code, 0);					
					}
					#endif
				#endif
				dc_status_loc &=~(ALARM_DC24_LOW_MASK);
				dc_status_loc &=~(ALARM_DC24_HIGH_MASK);
				dc_status_loc &=~(ALARM_DC24_LOW_LOW_MASK);
				
			}
		}
		else {
			err_cnt[4] = 0;
		}
		//test_count_whichdo[4]++;
	}
}



	if(error_active & ERROR_ACTIVE_MASK_DC12_P) {
		if((dc_status_loc & ALARM_DC12_LOW_MASK) ||(dc_status_loc & ALARM_DC12_HIGH_MASK )) {
			err_cnt[9] ++;
			if(err_cnt[9] > ERR_DELAY) {
				#ifdef NEW_ALARM_STYLE
				if(alert_find(POWER_ERR_CODE_ARG(0,12|0xF3)) == 0) {
					
					alert_push(POWER_ERR_CODE_ARG(0,12|0xF3));
					
				}
				#else
					#ifdef DC_ERR_FN_ALERT
						{
							unsigned short arg=0;
							arg = 0x0C;
							if(alert_find(DC12_P_PERR, arg) == 0) 
							{
								alert_push(DC12_P_PERR, arg);
							}							
						}

					#else		
					if(alert_find(DC12_P_PERR, 0) == 0) {
						
						alert_push(DC12_P_PERR, 0);
						
					}
					#endif
				#endif
				dc_status_loc &=~(ALARM_DC12_LOW_MASK);
				dc_status_loc &=~(ALARM_DC12_HIGH_MASK);
				//dc_status_loc &=~(ALARM_DC24_LOW_LOW_MASK);
				
			}
		}
		else {
			err_cnt[9] = 0;
		}
		//test_count_whichdo[9]++;
	}

	

{
	if (tanzhen_active[0])
	{
		if(error_active & ERROR_ACTIVE_MASK_SW1) {
		if(Get_TZ_error_st(0) ) {
			err_cnt[10] ++;
			if(err_cnt[10] > ERR_DELAY) {
				#ifdef NEW_ALARM_STYLE
				if(alert_find(OTHER_ERR_CODE_ARG(3,0)) == 0) 
				#else
				if(alert_find(TZ_L_ERR, 0) == 0) 
				#endif
					{
#if 1
					if(tanzhen_alarm_delay_count[0] >= tanzhen_delay[0]) {
						tanzhen_alarm[0] = 0;
						tanzhen_alarm_delay_count[0] = 0;
						#ifdef NEW_ALARM_STYLE
						alert_push(OTHER_ERR_CODE_ARG(3,0));
						#else
						alert_push(TZ_L_ERR, 0);
						#endif
					}
					else
						tanzhen_alarm[0] = 1;
#else
					alert_push(LINE_ERR, 1);
#endif
				}
			}
		}
		else {
			err_cnt[10] = 0;
			if(tanzhen_alarm[0]) {
				tanzhen_alarm[0] = 0;
				tanzhen_alarm_delay_count[0] = 0;
			}
		}
	}
	//test_count_whichdo[10]++;
		}
	

if (tanzhen_active[1])
{
if(error_active & ERROR_ACTIVE_MASK_SW2) {
		if(Get_TZ_error_st(1)) {
			err_cnt[11] ++;
			if(err_cnt[11] > ERR_DELAY) {
				#ifdef NEW_ALARM_STYLE
				if(alert_find(OTHER_ERR_CODE_ARG(3,1)) == 0)
				#else
				if(alert_find(TZ_L_ERR, 1) == 0)
				#endif
				{
#if 1
					if(tanzhen_alarm_delay_count[1] >= tanzhen_delay[1]) {
						tanzhen_alarm[1] = 0;
						tanzhen_alarm_delay_count[1] = 0;
						#ifdef NEW_ALARM_STYLE
						alert_push(OTHER_ERR_CODE_ARG(3,1));
						#else
						alert_push(TZ_L_ERR, 1);
						#endif
					}
					else
						tanzhen_alarm[1] = 1;
#else
					alert_push(LINE_ERR, 1);
#endif
				}
			}
		}
		else {
			err_cnt[11] = 0;
			if(tanzhen_alarm[1]) {
				tanzhen_alarm[1] = 0;
				tanzhen_alarm_delay_count[1] = 0;
			}
		}
	}
//test_count_whichdo[11]++;
}
}

if (board_hard_type !=arch_Get_Board_Type())
{
	static unsigned short apperrorcnt=0;

	if (apperrorcnt==0)
	{
		if(alert_find(BOARD_APP_ERROR, 0) == 0) 
		{
			alert_push(BOARD_APP_ERROR, 0);
		}
	}
	apperrorcnt++;
}

#if 0
if ((error_active & ERROR_ACTIVE_MASK_TEMP)&&(0))
{
	//dt = TempS_CURR_V;						
	int tmp_ = 250L -((long)(760-TempS_CURR_V) * 100L)/25L ;

	//if (arch_is_E690_board())
	//{
	//	arch_ctr_fan_with_temp_(tmp_);		
	//}

	if(tmp_ >=Alarm_TEMP_Data_) {
			err_cnt[12] ++;
			if(err_cnt[12] > ERR_DELAY) {
				#ifdef NEW_ALARM_STYLE
				if(alert_find(OTHER_ERR_CODE_ARG(4,700)) == 0) {
					alert_push(OTHER_ERR_CODE_ARG(4,700));
				}
				#else
				if(alert_find(TEMP_OVER_ERROR, 0) == 0) {
					alert_push(TEMP_OVER_ERROR, 0);
				}
				#endif
			}
		}
		else {
			err_cnt[12] = 0;
		}
}
#endif

	dc_status=dc_status_loc;
	return 0;
}




unsigned int Get_ErrorStatus(void)	//
{
	unsigned int status = 0;

	if(STATUS_LINE1_INPUT()) {
		status |= 0x1;
	}
	if(V24_PERR_INPUT()) {
		status |= 0x2;
	}
	if(JQD1_ERR1_INPUT()) {
		status |= 0x4;
	}
	if(JQD1_ERR2_INPUT()) {
		status |= 0x8;
	}
#ifdef SUPPORT_2S
	if(JQD2_ERR1_INPUT()) {
		status |= 0x10;
	}
	if(JQD2_ERR2_INPUT()) {
		status |= 0x20;
	}
#endif
	if(STEP_ERR_INPUT()) {
		status |= 0x40;
	}
	if(STATUS_LINE2_INPUT()) {
		status |= 0x80;
	}
	/*
	if(V12_PERR_INPUT()) {
		status |= 0x200;
	}
	*/

	return status;
}

unsigned int arch_Get_DirStatus(void)
{
	// by xhl 2010/09/18
	int sts = 0xff;
	{
		if(SW1_INPUT())
			sts &= ~(0x1);
		if(SW2_INPUT())
			sts &= ~(0x2);
	}
	return sts;
}
unsigned int arch_get_dir1(void)
{
	return !SW1_INPUT();
}
unsigned int arch_get_dir2(void)
{
	return !SW2_INPUT();
}




unsigned int arch_GetKey(void)
{
	//return STATUS_KEY_INPUT();
	return 0;
}

unsigned int arch_GetBoardID(void)
{
	unsigned int ret = 0;
	ret= BOARD_ID_INPUT_0();
	ret |=(BOARD_ID_INPUT_1()<<1);
	ret |=(BOARD_ID_INPUT_2()<<2);
	ret |=(BOARD_ID_INPUT_3()<<3);	
	return ret;
}


unsigned int arch_GetBoardID_E490(void)
{
	unsigned int ret = 0;
	ret= BOARD_ID_INPUT_0();
	ret |=(BOARD_ID_INPUT_1()<<1);		
	return ret;
}

unsigned int arch_GetBoardID_E480(void)
{
	unsigned int ret = 0;
	ret= BOARD_ID_INPUT_2();
	ret |=(BOARD_ID_INPUT_3()<<1);
	return ret;
}


unsigned int arch_Get_ID(void)
{
	return arch_board_id;
}

unsigned int arch_Set_CANID(unsigned int isbroadcastID)
{
	if (isbroadcastID)
	{
		return 0x361;
	}
	else
		return 0x362+arch_board_id+1;
}




unsigned int arch_Get_Mode(void)
{
	return 0; //arch_mode ;
}

unsigned int arch_System(void)
{
	return arch_Get_ID()+1;

}


unsigned int arch_is_EMF_2_SK_board()
{
	#ifdef E490_V10_BOARD
	
	return 0;	

	#else
	
	#ifdef E499_BOARD_SUPPORT_
	return  (cpld_ver ==0x0001)?1:0;	
	#else
	return 0;
	#endif
	#endif
}

unsigned int arch_is_E690_board()  /*690的板子是在490的基础上增加了风扇控制*/
{
	return  (cpld_ver ==0x0001)?1:0;	
}

unsigned int arch_need_current_add(unsigned int currentdata)
{
	return (cpld_ver ==0x0001)?currentdata:(currentdata*11/10);
	//return currentdata;
}



// by xhl 2010/06/08
void Timer_Check(void)
{
	void alert_cpu_fatal(int x);
	unsigned long delay;

	// 20ms
	wait_time_ms = 3;
	wait_time_us = 3;
	delay = 40L * 1000L * 50L; /* = 50 ms ???*/
	while(1) {
		if(delay != 0) {
			if((wait_time_ms == 0) &&
			   (wait_time_us == 0))
				break;
			delay --;
		       	continue ;
		}

		//SoftReset();
		alert_cpu_fatal(DSP_FATAL_ERR_ARG_TIMER);

		can_receive_count = 0;
		while(can_receive_count == 0);
	}
}

unsigned int Get_Version(void);
void StepMotor_Encoder_Load(void);
extern void ADC_HardInit(void);
extern void ADC2_HardInit(void);
//extern void DAC_HardInit(void);

#ifdef E490_V10_BOARD
unsigned short Get_CPLD_Ver_E490()
{
	unsigned short ret=0;
	ret= E490_BOARD_VER0();
	ret |=(E490_BOARD_VER1()<<1);
	ret |=(E490_BOARD_VER2()<<2);
	//ret |=(E490_BOARD_VER3()<<3);
	return ret;	
}

unsigned short Get_EXP_Board_E490()
{
	unsigned short ret=0;
	ret= E490_BOARD_EXP0();
	ret |=(E490_BOARD_EXP1()<<1);
	return ret;
}



void Get_Cpld_Name_Ver()
{
	//int i;
	//int k;
	unsigned short cpld_name_temp=BOARD_NAME_E490;
	unsigned short cpld_ver_temp=0;
	unsigned short cpld_expID_temp=0;

	//k=0;
	if (cpld_name_temp !=cpld_name)
	{
		cpld_name =cpld_name_temp;		
	}

	cpld_ver_temp = Get_CPLD_Ver_E490();	
	if (cpld_ver_temp !=cpld_ver)
	{
		cpld_ver =cpld_ver_temp;
	}


	
	cpld_expID_temp =Get_EXP_Board_E490();
	

	if (cpld_expID_temp !=cpld_expID)
	{
		cpld_expID =cpld_expID_temp;

	}
		
}

#else

void Get_Cpld_Name_Ver()
{
	int i;
	int k;
	unsigned short cpld_name_temp=0;
	unsigned short cpld_ver_temp=0;
	unsigned short cpld_expID_temp=0;

	k=0;
read_name:	
	for (i=0;i<16;i++)
	{
		cpld_name_temp |=((GET_CPLD_NAME_NEW(i)?1:0)<<i);
	}

	if (cpld_name_temp !=cpld_name)
	{
		cpld_name =cpld_name_temp;
		k++;
		if (k<10)			
		goto read_name;
	}

	k=0;
read_ver:	
	
	for (i=0;i<16;i++)
	{
		cpld_ver_temp |=((GET_CPLD_VER_NEW(i)?1:0)<<i);
	}
	if (cpld_ver_temp !=cpld_ver)
	{
		cpld_ver =cpld_ver_temp;
		k++;
		if (k<10)			
		goto read_ver;
	}

	k=0;
read_exp:
	for (i=0;i<3;i++)
	{
		cpld_expID_temp |=((GET_CPLD_EXP_NEW(i)?1:0)<<i);
	}

	if (cpld_expID_temp !=cpld_expID)
	{
		cpld_expID =cpld_expID_temp;
		k++;
		if (k<10)			
		goto read_exp;
	}
	

	
	
}
#endif

#ifdef E499_BOARD_SUPPORT_
void Arch_E499_step_pwm_enable()
{
	OUTPUT(ADDR_JQDBOARD1+0x30+15,1);
}

void arch_set_Step_SK_A(unsigned int cruu)
{
	int i;

	cruu=arch_need_current_add(cruu);
	
	if (cruu>2200)
	{
		cruu =2200;
	}
	else if (cruu<600)
	{
		cruu =600;
	}
	cruu /= 100;
	cruu -=6;


	for (i=0 ;i<15;i++)
	{
		OUTPUT(ADDR_JQDBOARD1+0x30+i,0);
	}

	if (cruu<=14) //(cruu>=0)&&(
	{
		OUTPUT(ADDR_JQDBOARD1+0x30+cruu,1);
		//OUTPUT(ADDR_JQDBOARD1+0x30+cruu-1,1);
	}
}

#endif

unsigned char arch_is_EMF_ACT()
{
	return ((cpld_expID>>2)&0x01)?1:0;
}


unsigned char arch_is_shock_board()
{
	return (cpld_name ==BOARD_NAME_E490) ? (((cpld_expID & 0x03)==0x03) ?1:0):1;
}

unsigned char arch_EXPID_branch()
{
	return (cpld_expID&0x03);
}


unsigned char arch_is_E490_board()
{
	return  (cpld_name ==BOARD_NAME_E490) ?1:0;
}

unsigned char arch_is_Ecode_board()
{
	return ((arch_EXPID_branch() & 0x01) ==0x01)?1:0;   
}

unsigned char arch_is_StepBoard()/*2018 11 29 判断是不是扩展的电机板*/
{
	return ((arch_EXPID_branch() & 0x03) ==0x02)?1:0;  
}

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
unsigned char arch_is_tenBladeSelector_Board()
{
	extern volatile unsigned char Needle_Selector_is_tenBlade;
	return Needle_Selector_is_tenBlade?1:0;
}
#endif
void Set_StepMotor_Count()
{
	switch (cpld_name)
	{
		case BOARD_NAME_E475:
			StepMotor_Count_MAX =6;
			break;
		case BOARD_NAME_E480:
			if (Yarn_use_Step)
				StepMotor_Count_MAX =12;
			else
				StepMotor_Count_MAX =8;	
			break;
		
		case BOARD_NAME_E490:
			if (Yarn_use_Step)
				StepMotor_Count_MAX =12;
			else
				StepMotor_Count_MAX =8;
			#ifdef E490_V10_BOARD
			if (TZ_use_Step)
			{
				StepMotor_Count_MAX =14;
			}
			
			#endif
			#ifdef E499_BOARD_SUPPORT_
			if (arch_is_EMF_2_SK_board())
			{
				StepMotor_Count_MAX =14;
			}
			#endif

			#ifdef E692_STEPMOTOR_EX
			if(Step_use_exBoard)
			{
				StepMotor_Count_MAX =16;
			}
			#endif
			
			break;

		default:
			StepMotor_Count_MAX =6;
			break;
	}
	
{ 
   MOTO1_RESET_OUTPUT(1); 
   MOTO2_RESET_OUTPUT(1); 
   MOTO3_RESET_OUTPUT(1); 
   MOTO4_RESET_OUTPUT(1); 
   MOTO5_RESET_OUTPUT(1); 
   MOTO6_RESET_OUTPUT(1); 
   MOTO7_RESET_OUTPUT(1); 
   MOTO8_RESET_OUTPUT(1); 
   MOTO9_RESET_OUTPUT(1); 
   MOTO10_RESET_OUTPUT(1); 
   MOTO11_RESET_OUTPUT(1); 
   MOTO12_RESET_OUTPUT(1); 
   MOTO13_RESET_OUTPUT(1); 
   MOTO14_RESET_OUTPUT(1); 
   MOTO15_RESET_OUTPUT(1); 
   MOTO16_RESET_OUTPUT(1); 
}


	
}

void arch_init(void)
{
	/* Configure the system clocks */
	extern void GPIO_Configuration_powerctr(void);
	extern void GPIO_Configuration_PB9ctr(void);
	extern void Gpio_cfg_yarn_(unsigned int isyarnusestep);
	extern void Gpio_cfg_EXP_Board_(unsigned char boardtype);
	extern void Gpio_cfg_E499Board_(void);
	extern void GPIO_EX_E690Board(void);
	extern void DAC_HardInit(void);
	
	RCC_Configuration();

#ifdef WDG_ENABLE
	IWDG_Init();
#endif
	
	/* NVIC Configuration */
	NVIC_Configuration();

	GPIO_Configuration_powerctr();

	arch_Power_Off();

	GPIO_Configuration_PB9ctr();

	if (GET_PB9_ST() || 1)
	{
		board_hard_type = 0x8009;
	}
	else
	{
		board_hard_type = 0x8008;
	}


	/* Configure the GPIO ports */
	GPIO_Configuration();



	#ifdef E480_BOARD_V10

	Yarn_use_Step =SWITCH_YARN_BOARD();
	Gpio_cfg_yarn_(Yarn_use_Step);
		
	#endif

	#ifdef E490_V10_BOARD
	TZ_use_Step = E490_TZSTEP_CS();
	#endif

	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	Needle_Selector_is_tenBlade = E693_TEN_NS_CS();
	#endif


	arch_LED_Off();

	FSMC_Init();

	Get_Cpld_Name_Ver();

	if (arch_is_E490_board())
	{
		
		Gpio_cfg_EXP_Board_(arch_EXPID_branch());
	

		if (arch_is_EMF_2_SK_board())
		{
			// do gpio cfg with TZACTstep
			Gpio_cfg_E499Board_();
			Arch_E499_step_pwm_enable();
		}
		else
			if(arch_is_E690_board())
			{
				GPIO_EX_E690Board();
				arch_FAN_OFF();
			}
			{
				
				Step_use_exBoard = arch_is_StepBoard();	
			}
	}

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	if(arch_is_tenBladeSelector_Board())
	{
		/*initialize */
	}
#endif	

	Set_StepMotor_Count();

	/* USART Init */
	USART_Configuration();

	#if 1
	{
	RCC_ClocksTypeDef XX;
	extern void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
	extern unsigned long SYSHCLK_AHB;
	RCC_GetClocksFreq(&XX);
	SYSHCLK_AHB = XX.HCLK_Frequency;
	//myprintf("系统时钟 is [ SYScl:%d, 中文时钟:%d,P1:%d,p2:%d ]\r\n",XX.SYSCLK_Frequency,XX.HCLK_Frequency,XX.PCLK1_Frequency,XX.PCLK2_Frequency);
	
	}
	#endif

	//arch_Power_Off();

	//Jtag_Security();
	
	arch_mode = 0;//arch_GetKey() & 0x1;

	//arch_board_id = arch_GetBoardID() & 0xF;
	switch (cpld_name)
	{
		case BOARD_NAME_E475:
			arch_board_id = arch_GetBoardID() & 0xF;
			break;
		case BOARD_NAME_E480:
			arch_board_id = arch_GetBoardID_E480() & 0x3;	
			break;
		case BOARD_NAME_E490:
			#ifdef E490_V10_BOARD
			arch_board_id = arch_GetBoardID() & 0xF;	
			#else
			arch_board_id = arch_GetBoardID() & 0xF;	
			#endif
			break;
			
		default:
			arch_board_id = arch_GetBoardID() & 0xF;
			break;
	}
	

	#ifdef ALARM_SHOCK_DOUBLE
	shock_double_enable=((arch_EXPID_branch()&0x03) ==0x03)?1: SHOCK_DOUBLE_INPUT();
	#endif
	
	reg_init();

	/* CAN Init */
	CAN_Configuration();

	
	

	TIM_Init();


	Time_PWM_init(arch_is_shock_board()||arch_is_StepBoard());

	#ifdef E490_V10_BOARD
	{
		unsigned char mask;
		extern void Step_PWMDA_Init_exp(unsigned char whichpwmmask);
		mask = (TZ_use_Step?1:0)|((Yarn_use_Step?1:0)<<1)|((SK_use_Step)<<2);
		Step_PWMDA_Init_exp(mask);
	}
	#else
	
#ifdef E480_BOARD_V10

	if (Yarn_use_Step)
	{
		Yarn_step_PWMDA_Init();
		//myprintf("\r\n==Yarn_step_PWMDA_Init==");
	}
	
#endif
#endif
	if (arch_is_shock_board())
	{
		arch_shock_init();
	}

	Timer_Check();

	ADC_HardInit();//???

	
	//myprintf("\r\n===========ADC2_HardInit()============");
	ADC2_HardInit();
	
	DAC_HardInit();
	

#ifdef WDG_ENABLE
	IWDG_Enable();
#endif


	myprintf("\r\n============================");
	myprintf("\r\n==                            ==");
	myprintf("\r\n==   HKnit Header Control     ==");
	myprintf("\r\n==   Version V%x            ==", Get_Version());
	myprintf("\r\n==  CPLD Version %X - V%04X --EXPID %d ==", cpld_name,cpld_ver,cpld_expID);
	myprintf("\r\n==                            ==");
	#if 0
	myprintf("\r\n========%d=%d============",shock_double_enable,Yarn_use_Step);

	#else
		myprintf("\r\n====TZ_USER = %d, Yarn_Step=%d===",TZ_use_Step,Yarn_use_Step);

	#endif

	myprintf("\r\n\r\n");
	
	//Shell_Init();

	EXTI_Config();





	arch_CPLD_Reset();
	arch_8803_Reset();
	arch_8844_Reset();
	EXT_NRST_Reset();



	arch_StepMotor_Disable();
	arch_StepMotor_Init();
	arch_Jacquard_AllClear();
	arch_ACTEMF_AllClear();
	arch_YARNEMF_AllClear();	
	MainboardUnlock();
	FLASH_Unlock();	
	EE_Init();
	
//#endif

	Exec_main_head_board_id_init();
	FLASH_Lock();
	arch_LED_On();
#ifdef _ZERO_FILTER_SUPPORT
	Time_start_time7();
#endif
}


void wait_ms(unsigned int ms)
{
	//wait_time_ms = ms << 1;
	wait_time_ms = ms >> 1;
	while(wait_time_ms);
}

void wait_us(unsigned int us)
{
	wait_time_us = ((us + 511) >> 9)+1;
	while(wait_time_us);
}


#ifdef JQD_ISR_TIMER_CNT_50US

extern volatile unsigned int blade_group_active_jqd[MAX_BLADE];
extern volatile unsigned int jqd_blade_next_do[MAX_JACQUARD][5];
extern volatile unsigned int blade_group_keep_time[MAX_BLADE/*8*/][MAX_JACQUARD];
extern volatile unsigned int jqd_keep_time_setup_us;
extern volatile unsigned int jqd_status[MAX_JACQUARD/*8*/];
extern volatile unsigned int blade_group_delay_time[MAX_BLADE/*8*/][MAX_JACQUARD];
extern volatile unsigned int jqd_status_last[MAX_JACQUARD/*8*/];

void arch_JQD_isr_timer_cnt_50us()
{
	int jqdno,bladno;
	for (jqdno=0;jqdno<sys_max_jqd_cnt;jqdno++)
	{
		for (bladno=0;bladno<MAX_BLADE;bladno++)
		{
			if(blade_group_active_jqd[bladno] & ((unsigned short)0x01<<jqdno))
			{
				if(blade_group_delay_time[bladno][jqdno]==0)
				{
					if (blade_group_keep_time[bladno][jqdno] ==jqd_keep_time_setup_us)
					{
						//open;
						/*20191108 确保状态更新*/
						if(jqd_status[jqdno] & ((unsigned short)0x01<<bladno))
							jqd_status_last[jqdno] |=(0x01<<bladno);
						else
							jqd_status_last[jqdno] &=~(0x01<<bladno);							
						
						arch_Jacquard_Setup(jqdno, bladno, jqd_status[jqdno] & ((unsigned short)0x01<<bladno),1,1);
						//Exec_Jqd_blade_next(jqdno,bladno,jqd_status[jqdno] & (0x01<<bladno),0);
					}
					else
					if(blade_group_keep_time[bladno][jqdno]==0)
					{
						//clear;
						arch_Jacquard_ClearBlade(jqdno,bladno);	
						blade_group_active_jqd[bladno] &=~((unsigned short)0x01<<jqdno);
						if(!jqd_blade_next_do[jqdno][3])  /*可以进中断取下一个*/
						{							
							Exec_check_and_do_next_blade(jqdno);
						}
						else
						{
							/*置标志*/
							jqd_blade_next_do[jqdno][4]=1;
						}
					}
					if(blade_group_keep_time[bladno][jqdno])
						blade_group_keep_time[bladno][jqdno]--;
				}
				else
				{
					blade_group_delay_time[bladno][jqdno]--;
				}					

			}
		}
		if(jqd_blade_next_do[jqdno][4])
		{
			if(!jqd_blade_next_do[jqdno][3])  /*可以进中断取下一个*/
			{							
				Exec_check_and_do_next_blade(jqdno);
				jqd_blade_next_do[jqdno][4]=0;
			}			
		}

	}
	
}





#endif




#ifdef JQD_ISR_TIMER_ONE



extern volatile unsigned char jqd_last_blade[MAX_JACQUARD];
extern volatile unsigned int jqd_last_blade_st[MAX_JACQUARD];
extern volatile unsigned char jqd_isfirst_do[MAX_JACQUARD][MAX_BLADE];

//extern volatile unsigned int jqd_max_blade;

//volatile unsigned short jqd_last_cnt[MAX_JACQUARD][4];

//volatile unsigned short jqd_timer_delay_max[MAX_JACQUARD][2];/*0--当前值，1--最大值*/

//volatile unsigned short jqd_isr_isdo=0;
//volatile unsigned short jqd_isr_isdo_EX=0;
//volatile unsigned short jqd_isr_isdo_EX_cct[MAX_JACQUARD];



extern volatile unsigned int jqd_keep_time_setup_us;
extern volatile unsigned int jqd_status[MAX_JACQUARD/*8*/];

volatile unsigned short jqd_time_arr[]={350,20,150,50};//,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50};
volatile unsigned short jqd_time_arr_first[]={200,20,100,50};//,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50,20,50};


#define JQD_TIME_ARR_ROWE_CNT 	10
volatile unsigned short fast_jqd_time_arr[JQD_TIME_ARR_ROWE_CNT][3]={
{200,20,300},
{250,20,250},
{270,20,230},
{290,20,210},
{300,20,200},
{310,20,190},
{330,20,170},
{350,20,150},
{380,20,120},
{400,20,100}
};


//volatile  short jqd_blad_timer_cnt[MAX_JACQUARD][8];
//volatile unsigned short jqd_blad_timer_cnt_st[MAX_JACQUARD][8];



void arch_start_jqd_timer(unsigned int whichjqd,unsigned int timercnt_us,unsigned char isfirst);


#if 0
void arch_get_jqd_time_cnt(unsigned int whi)
{
	Message_Send_4halfword(jqd_keep_time_setup_us,jqd_last_cnt[whi][0],jqd_last_cnt[whi][1],jqd_last_cnt[whi][2]);
}
#endif


void arch_test_Tim4_isr(unsigned char whichid)
{
	switch (whichid)
	{
		case 0:
			{
				TIM_ITConfig(TIM4, TIM_IT_CC1, DISABLE);
			}
			break;
		case 1:
			{				
				TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
			}				
			break;					
	}
	
}

#if 0
#define JQD_PWM_OFF_TIME	80
#define JQD_PWM_ON_TIME	80

void arch_jqd_isr_50_20_us()
{
	static char st=0;
	DWORD capture = st?JQD_PWM_OFF_TIME:JQD_PWM_ON_TIME;
	int jqdno,bladno;
	//return;
	for (jqdno=0;jqdno<4;jqdno++)
	{
		for (bladno=0;bladno<8;bladno++)
		{
			if (jqd_blad_timer_cnt[jqdno][bladno]>0)
			{
				jqd_blad_timer_cnt[jqdno][bladno]-=capture;//st?JQD_PWM_OFF_TIME:JQD_PWM_ON_TIME;

				if (jqd_blad_timer_cnt_st[jqdno][bladno]==0)
				{
					if(st==0)
					{
						if (jqd_blad_timer_cnt[jqdno][bladno]>0)
						{
							arch_Jacquard_Setup(jqdno, bladno, jqd_status[jqdno] & (0x01<<bladno),1,0);
						}
						// Exec_Jqd_blade_next(jqdno,bladno,jqd_status[jqdno] & (0x01<<bladno),1);
					}
					else
					{
						arch_Jacquard_ClearBlade(jqdno,bladno);	
					}	
				}
				if (jqd_blad_timer_cnt_st[jqdno][bladno]>0)
					jqd_blad_timer_cnt_st[jqdno][bladno]--;
				
			}
		}
	}


	capture += TIM_GetCounter(TIM10);
	TIM_SetCompare1(TIM10, capture&0xffff);
	TIM_ClearFlag(TIM10, TIM_FLAG_CC1);
	//jqd_last_cnt[whichjqd][3]=TIM_GetCapture1(TIM10);
	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);
	
	
	st =!st;
	
	
}
#endif

#ifdef JQD_NEXT_DO_FIFO

#define JQD_FIRST_TIMER_US		200

volatile unsigned short jqd_blad_index[MAX_JACQUARD][2];

void arch_JQD_Isr_call(unsigned int whichjqd,unsigned char clear)
{
	int bldno;
	//int i;
	unsigned short blad_mask;
	unsigned int allt= jqd_keep_time_setup_us;

	//if (clear)
	

	//jqd_last_cnt[whichjqd][0] ;
	//jqd_last_cnt[whichjqd][1]=TIM_GetCounter(TIM8);
	//jqd_last_cnt[whichjqd][2]++;
	switch (whichjqd)
	{
		case 0:
			{
				TIM_ITConfig(TIM8, TIM_IT_CC1, DISABLE);
			}
			break;
		case 1:
			{				
				TIM_ITConfig(TIM8, TIM_IT_CC2, DISABLE);
			}				
			break;
		case 2:
			{				
				TIM_ITConfig(TIM8, TIM_IT_CC3, DISABLE);
			}
			break;
		case 3:
			{			
				
				TIM_ITConfig(TIM8, TIM_IT_CC4, DISABLE);
			}
			break;			
	}
	
	if (jqd_last_blade[whichjqd] ==0xff)//说明中断来多了。
	{
		return;
	}
	
	
	if (clear)
	{
		//if (jqd_last_blade[whichjqd] ==0xff)
			//return;
		goto doclear;
	}
	#if 0
	if (jqd_last_cnt[whichjqd][1]>jqd_last_cnt[whichjqd][0])
	{
		jqd_timer_delay_max[whichjqd][0] = (jqd_last_cnt[whichjqd][1]-jqd_last_cnt[whichjqd][0]);
	}
	else
	{
		jqd_timer_delay_max[whichjqd][0] = ((unsigned short)0xffff -jqd_last_cnt[whichjqd][0]) + jqd_last_cnt[whichjqd][1];
	}
	
	if (jqd_timer_delay_max[whichjqd][0] >jqd_timer_delay_max[whichjqd][1] )
	{
		jqd_timer_delay_max[whichjqd][1] =jqd_timer_delay_max[whichjqd][0] ;
	}
	#endif

	//jqd_isr_isdo_EX &=~(0X01<<whichjqd);
	//jqd_isr_isdo_EX_cct[whichjqd]++;
	
	if(jqd_blad_index[whichjqd][1]<allt)
	{
		unsigned char is_first=0;
		bldno = jqd_last_blade[whichjqd];
		#ifdef FILTER_1ST_2ND_JQD_OVERLOAD
		{
			unsigned char blade_do_cnt;
		
			blade_do_cnt = jqd_isfirst_do[whichjqd][bldno];

			if(blade_do_cnt<=2)
			{
				is_first=1;
			}
		}
		#endif
		
		if (jqd_blad_index[whichjqd][0] & 0x01)
		{
			arch_Jacquard_ClearBlade(whichjqd, bldno);		
			
		}
		else
		{
			
			arch_Jacquard_Setup(whichjqd, bldno,jqd_last_blade_st[whichjqd],1,7);
			
			
		}
		arch_start_jqd_timer(whichjqd,jqd_keep_time_setup_us,is_first);

	}
	else
	{
	doclear:
	jqd_blad_index[whichjqd][1]=0;
	jqd_blad_index[whichjqd][0]=0;
	
	//#endif
	
	//jqd_isr_isdo &=~(0x01<<whichjqd);
	
	
	bldno = jqd_last_blade[whichjqd];
	//for(i=0;i<8;i++)
	{
		arch_Jacquard_ClearBlade(whichjqd, bldno);
		//jqd_blad_timer_cnt_st[whichjqd][bldno]= 1  ;
		//jqd_blad_timer_cnt[whichjqd][bldno] =200;//jqd_keep_time_setup_us-JQD_FIRST_TIMER_US;
		//if ()
		
	}
	if(!(jqd_isfirst_do[whichjqd][bldno]))
	{
			jqd_isfirst_do[whichjqd][bldno] =1;	
	}	

	jqd_last_blade[whichjqd] =0xff;

	//#ifdef JQD_NEXT_DO_FIFO
	jqd_timer_isbusy[whichjqd] = 0;

//#endif
	}

}


void arch_start_jqd_timer(unsigned int whichjqd,unsigned int timercnt_us,unsigned char isfirst)
{
	DWORD capture= timercnt_us;
	//if (jqd_blad_index[whichjqd][0]==0)
	#if 0
	{
		if (timercnt_us>JQD_FIRST_TIMER_US)
		{
			capture=JQD_FIRST_TIMER_US;
		}
		 //jqd_time_arr[jqd_blad_index[whichjqd][0]];
	}
	#endif
	#if 1
	if (jqd_blad_index[whichjqd][0]<3)
	{
		capture=isfirst?jqd_time_arr_first[jqd_blad_index[whichjqd][0]]:jqd_time_arr[jqd_blad_index[whichjqd][0]];	
	}
	else
	{
		capture =(jqd_blad_index[whichjqd][0] & 0x01)? 50:20;
	}
	if((jqd_blad_index[whichjqd][1]+capture)>timercnt_us)
	{
		capture -= (jqd_blad_index[whichjqd][1]+capture - timercnt_us);
	}
		
	jqd_blad_index[whichjqd][1]+=capture;
	jqd_blad_index[whichjqd][0]++;
	#endif

	
		
	//jqd_last_cnt[whichjqd][0]=jqd_blad_index[whichjqd][1];//TIM_GetCounter(TIM8);
	switch (whichjqd)
	{
		case 0:
			{
				capture += TIM_GetCounter(TIM8);
				TIM_SetCompare1(TIM8, capture&0xffff);
				TIM_ClearFlag(TIM8, TIM_FLAG_CC1);
				//jqd_last_cnt[whichjqd][3]=TIM_GetCapture1(TIM8);
				TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
			}
			break;
		case 1:
			{
				capture += TIM_GetCounter(TIM8);
				TIM_SetCompare2(TIM8, capture&0xffff);
				TIM_ClearFlag(TIM8, TIM_FLAG_CC2);
				//jqd_last_cnt[whichjqd][3]=TIM_GetCapture2(TIM8);
				TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
			}				
			break;
		case 2:
			{
				capture += TIM_GetCounter(TIM8);
				TIM_SetCompare3(TIM8, capture&0xffff);
				TIM_ClearFlag(TIM8, TIM_FLAG_CC3);
				//jqd_last_cnt[whichjqd][3]=TIM_GetCapture3(TIM8);
				TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
			}
			break;
		case 3:
			{
				capture += TIM_GetCounter(TIM8);
				TIM_SetCompare4(TIM8, capture&0xffff);
				TIM_ClearFlag(TIM8, TIM_FLAG_CC4);
				//jqd_last_cnt[whichjqd][3]=TIM_GetCapture4(TIM8);
				TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
			}
			break;			
	}
	//jqd_isr_isdo |=(0x01<<whichjqd);
	//jqd_isr_isdo_EX |=(0x01<<whichjqd);
	
}

#endif

#ifdef JQD_IS_FAST_MODE_2_STEPS
void set_jqd_data_arr(unsigned int arg1,unsigned int arg2,unsigned int arg3)
{
	jqd_time_arr[0]=arg1;
	jqd_time_arr[1]=arg2;
	jqd_time_arr[2]=arg3;
	
	jqd_keep_time_setup_us = jqd_time_arr[0]+jqd_time_arr[1]+jqd_time_arr[2];
}

void Set_Fast_JQD_Time(unsigned char time_index)
{
	if (time_index<JQD_TIME_ARR_ROWE_CNT)
	set_jqd_data_arr(fast_jqd_time_arr[time_index][0],fast_jqd_time_arr[time_index][1],fast_jqd_time_arr[time_index][2]);	
}

#endif
#if 0
void arch_putout_Tim_CNT_CCR(unsigned int whichjqd)
{	 int i;
	Message_Send_4halfword(0xFFFF,jqd_last_cnt[whichjqd][3],TIM_GetCounter(TIM8),(whichjqd<<8)|(jqd_isr_isdo_EX & 0XF)|((jqd_isr_isdo & 0X0F)<<4));
	Message_Send_4halfword(0xFFEE,jqd_timer_delay_max[whichjqd][1],jqd_isr_isdo_EX_cct[whichjqd],whichjqd);
	#if 0
	for (i=0;i<32;i++)
	{
		Message_Send_4halfword(0xFFBB,whichjqblad_is_clear[whichjqd][i],i,whichjqblad_is_clear_par[whichjqd]);
	}
	#endif
}



void arch_putout_MAX_ccr_cnt(unsigned int whichjqd)
{
	Message_Send_4halfword(0xFFAA,jqd_timer_delay_max[whichjqd][1],0xffff,whichjqd);
}


int arch_get_timer_isdoing(unsigned int whichjqd)
{
	
	//TIM_GetITenableStatus(TIM8,);
	switch (whichjqd)
	{
		case 0:
			{
			 	return TIM_GetITenableStatus(TIM8, TIM_IT_CC1);
			}
			break;
		case 1:
			{
				return TIM_GetITenableStatus(TIM8, TIM_IT_CC2);
			}				
			break;
		case 2:
			{
				return TIM_GetITenableStatus(TIM8, TIM_IT_CC3);
			}
			break;
		case 3:
			{
				return TIM_GetITenableStatus(TIM8, TIM_IT_CC4);
			}
			break;			
	}
}
#endif


#endif



#ifdef _ZERO_FILTER_SUPPORT
unsigned int arch_StepMotor_Zero(unsigned int stepno)
{
	return _step_zero_state[stepno];
}

static int arch_StepZero_filter_timer()
{
	int i;
	int sts;

	for(i < 0; i < _ZERO_INPUT_COUNT; i++) {
		sts =0; //arch_StepMotor_Zero_impl(i)?1:0;
		if(_step_zero_shadow[i] != sts) {
			_step_zero_shadow[i] = sts;
			_step_zero_filter_ticks[i] = 0;
		}
		else if(_step_zero_state[i] != sts) {
			_step_zero_filter_ticks[i] ++;
			if(_step_zero_filter_ticks[i] > _ZERO_INPUT_FILTER) {
				_step_zero_state[i] = sts;
			}
		}
	}
}
#endif

#if 0
void arch_StepZero_filter_timer_test()
{
	int i;
	int sts;
	//static int szs=0;
	//static int szs=0;
	
	
	
	for(i < 0; i < _ZERO_INPUT_COUNT; i++) {
		sts = arch_StepMotor_Zero(i)?1:0;
		if(_step_zero_shadow[i]!= sts) {
			_step_zero_shadow[i] = sts;
			_step_zero_filter_ticks[i] = 0;
		}
		else if(_step_zero_state[i] != sts) {
			_step_zero_filter_ticks[i] ++;
			if(_step_zero_filter_ticks[i] > _ZERO_INPUT_FILTER) {
				_step_zero_state[i] = sts;
			}
		}
	}
}

#endif
void hook_time_50us(void)
{
	unsigned int addr;
	unsigned int sts;

if(DC24_Check_Timer_Start) {
	if (DC24_Start_check==0)//说明停了
	{
		sts=1;
	}
	else
	{
		sts=whichEMFisTest_sts[DC24_PN_whichNO];
	}
	
	if (DC24_PN_whichNO<32) // 说明是选针
	{
		addr = ADDR_JQDBOARD1+DC24_PN_whichNO;		
		OUTPUT(addr + ADDR_DIS_JQD_IN_EN, sts?0:1);	
	}
	else
	{
		if (DC24_PN_whichNO<48)
		{
			arch_yarnemf_pwm_setup(DC24_PN_whichNO-32,sts);
		}
		else if (DC24_PN_whichNO<MAX_DC24_DATA_COUNT)
		{
			arch_ACTEMF_PWM_Setup(DC24_PN_whichNO-48,sts);
		}
	}
	whichEMFisTest_sts[DC24_PN_whichNO] = sts?0:1;
}

#ifdef _ZERO_FILTER_SUPPORT
	arch_StepZero_filter_timer();
#endif

	//arch_StepZero_filter_timer_test();

}

unsigned int power_opentime=2500;

int arch_check_powerdown()
{
	//20000/21 =
	if (power_opentime==0)
	{
		if((DC24_P_CURR_V *21/100)<120)
		{
			CLI();
			arch_Power_Off();
			PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
			while(1);
		}
	}
	else
		{power_opentime--;}

	
}
void hook_time_2ms(void)
{
	if(wait_time_ms > 0) wait_time_ms --;
	//if(overload_delay > 0) overload_delay --;

	if (wait_time_ms_test>0) wait_time_ms_test--;

	time_2ms_prog();
	arch_LED_timer();
	

	if(Get_Overload_Status()) {
		overload_count ++;
	}

	if (FAN_Start_delay_2ms>0)
		FAN_Start_delay_2ms--;
	if (N24_on_delay_1s>0)
		N24_on_delay_1s--;
	if (P24_on_delay_1s>0)
		P24_on_delay_1s--;
	arch_check_powerdown();

	#ifdef ECODE_USE_MT6813_PWM
	Encoder_timeout_check_2ms();
	#endif

	Alert_Code_Send_Delay_ISR();
	
}
  
int arch_time_500_pulse()
{
	return 2; // = 500 / 250
}

void hook_time_250us(void)
{
	extern volatile unsigned int Send_delay_online;
	extern volatile unsigned char enable_log_zero_change;
	basetime_tick++;
	if (wait_time_us > 0) wait_time_us --;
	if (poweron_delay > 0) poweron_delay--;
	if (Send_delay_online>0)Send_delay_online--;

	#ifdef ECODE_USE_MT6813_PWM
	time3_pwm_timeout_250us_cnt();
	#endif

#ifdef READ_FLASH_DATA_FOR_BANDING
	arch_read_data_timer250us();
#endif
	time_250us_prog();
	//arch_LED_timer();
	#ifdef YARN_ZERO_CHECK
	Yarn_zero_check_input_st();
	#endif
	#ifdef LOG_DEBUG_FOR_LX_AT_CH
	if(enable_log_zero_change)
		{
		extern volatile unsigned int jqd_direction;
		time_250us_zero_change(basetime_tick,jqd_direction);
		}
	#endif

	//shell_hook_time();
}

unsigned int arch_get_ticktime(void)
{
	return basetime_tick;
}

unsigned short arch_Get_Board_Type_boot()
{
	
#ifdef E475_480_BOARD

switch (cpld_name)
{
	case BOARD_NAME_E475:

	return 0x7007;
	//break;
	case BOARD_NAME_E480:	
	{
		return 0x7008;
	}
	//break;
	case BOARD_NAME_E490:	
	{
		return 0x7009;
	}
	//break;
	
	default:
	{
		return 0x0000;
		//break;		
	}
}

	//return 0x8006;
#endif
}

unsigned int arch_Get_Board_Type(void)
{


#ifdef E475_480_BOARD

switch (cpld_name)
{
	case BOARD_NAME_E475:

	return 0x8007;
	//break;
	case BOARD_NAME_E480:	
	{
		return 0x8008;
	}
	//break;
	case BOARD_NAME_E490:	
	{
		return 0x8009;
	}
	//break;
	
	default:
	{
		return 0x0000;
		//break;		
	}
}

	//return 0x8006;
#endif
}

int get_cpld_ver(char *data)
{
	assert_param(data);

	data[0] = GET_CPLD_VER1();
	data[1] = GET_CPLD_VER2();
	data[2] = GET_CPLD_VER3();
	data[3] = GET_CPLD_VER4();

	return 0;
}

void SSI_DeSelect_ecode(unsigned int s_id)
{
	char on_=0;
	char off_=1;
#ifdef ENCODER_IC_IS_AEAT8800
on_ =1;
off_=0;

#endif
	if (arch_is_shock_board())
	{
		E495_OUT4(1);
		E495_OUT3(1);
		return ;
	}

switch (s_id)
	{
	case 0:
		ECODE_SELECT_CODE_1(off_);
		break;
	case 1:
		ECODE_SELECT_CODE_2(off_);
		break;
	case 2:
		ECODE_SELECT_CODE_3(off_);
		break;
	case 3:
		ECODE_SELECT_CODE_4(off_);
		break;
	case 4:
		ECODE_SELECT_CODE_5(off_);
		break;
	case 5:
		ECODE_SELECT_CODE_6(off_);
		break;
	case 6:
		ECODE_SELECT_CODE_7(off_);
		break;
	case 7:		
		ECODE_SELECT_CODE_8(off_);
		break;
	default:
		break;
	}
}	





void SSI_Select_ecode(unsigned int s_id)
{
	char on_=0;
	char off_=1;
#ifdef ENCODER_IC_IS_AEAT8800
on_ =1;
off_=0;

#endif
	
	if (arch_is_shock_board())
	{
		//char outst=0;
		E495_OUT4(1);
		E495_OUT3(1);

	//	outst = s_id;
		
		E495_OUT4(0);
		E495_OUT3((s_id>>2)&0x01);
		E495_OUT2((s_id>>1)&0x01);
		E495_OUT1((s_id>>0)&0x01);
		
	}
	else
	{

	ECODE_SELECT_CODE_1(off_);
	ECODE_SELECT_CODE_2(off_);
	ECODE_SELECT_CODE_3(off_);
	ECODE_SELECT_CODE_4(off_);
	ECODE_SELECT_CODE_5(off_);
	ECODE_SELECT_CODE_6(off_);
	ECODE_SELECT_CODE_7(off_);
	ECODE_SELECT_CODE_8(off_);

	switch (s_id)
	{
	case 0:
		ECODE_SELECT_CODE_1(on_);
		break;
	case 1:
		ECODE_SELECT_CODE_2(on_);
		break;
	case 2:
		ECODE_SELECT_CODE_3(on_);
		break;
	case 3:
		ECODE_SELECT_CODE_4(on_);
		break;
	case 4:
		ECODE_SELECT_CODE_5(on_);
		break;
	case 5:
		ECODE_SELECT_CODE_6(on_);
		break;
	case 6:
		ECODE_SELECT_CODE_7(on_);
		break;
	case 7:		
		ECODE_SELECT_CODE_8(on_);
		break;
	default:
		break;
	}
	
		}
}


void SSI_Select_all_ecode()
{
	char on_=0;
	//char off_=1;
	
	ECODE_SELECT_CODE_1(on_);
	ECODE_SELECT_CODE_2(on_);
	ECODE_SELECT_CODE_3(on_);
	ECODE_SELECT_CODE_4(on_);
	ECODE_SELECT_CODE_5(on_);
	ECODE_SELECT_CODE_6(on_);
	ECODE_SELECT_CODE_7(on_);
	ECODE_SELECT_CODE_8(on_);

}

#if 0

void check_can_send_error()
{
	extern volatile int can_send_error;
	static volatile int last_can_send_error=0;
	static volatile int can_reinit_cnt=0;
	if  (last_can_send_error!=can_send_error)
	{
		last_can_send_error = can_send_error;
		if (can_send_error)
		{
			arch_LED_Setup(999);			
		}
		else
		{			
			arch_LED_Setup(99);
		}
			
	}
	if (can_send_error)
	{
		if (!can_reinit_cnt)
		{
			CAN_Configuration();
		}
		if (can_reinit_cnt++ ==1000)
		{
			can_reinit_cnt=0;
		}
	}
	else
	{
		if (can_reinit_cnt)
			can_reinit_cnt=0;
	}
	return;

	
}
#endif

void Get_Head_config_Msg(unsigned int rtcdata,unsigned char whichCFG)
{
	unsigned short cfgdata;
	if (whichCFG==0)
	{
		cfgdata = 0x0000;
		cfgdata |=(Yarn_use_Step?0x01:0x00)<<0;
		cfgdata |=(TZ_use_Step?0x01:0x00)<<1;
		cfgdata |=(cpld_expID&0x03)<<2;	
		Message_Send_4halfword(rtcdata,whichCFG,cfgdata,0);
	}
	return;
}


#ifdef YARN_ZERO_CHECK

#define YARN_ST_IDX_CURR	0	
#define YARN_ST_IDX_MAX	1
#define YARN_ST_IDX_MIN	2

//#define YARN_ST_MAX_IDX	YARN_ST_IDX_MIM+1


unsigned short yarn_zero_st=0;

/*记录纱嘴信号存在的最大值和最小值,以及当前最后一次的值,*/
unsigned short yarn_zero_input_cnt[8][3]=
{{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535}};

/*记录纱嘴信号从无到有的最大值和最小值,以及当前最后一次的值,*/
unsigned short yarn_zero_change_cnt[8][3]=
{{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535},{0,0,65535}};


unsigned char Get_Yarn_zero_st(unsigned char id_y)
{
	switch(id_y)
	{
		case 0:return YARN0_ZERO_INPUT();
		case 1:return YARN1_ZERO_INPUT();
		case 2:return YARN2_ZERO_INPUT();
		case 3:return YARN3_ZERO_INPUT();
		case 4:return YARN4_ZERO_INPUT();
		case 5:return YARN5_ZERO_INPUT();
		case 6:return YARN6_ZERO_INPUT();
		case 7:return YARN7_ZERO_INPUT();
		default:return 0;
	}
}

void Yarn_zero_check_input_st()
{
	extern void Send_yarn_zero_st_change(unsigned short yst);
	int i;
	unsigned short yzst_change=0;
	unsigned short yzst=0;
	
	static unsigned char yzs_c[8]={0,0,0,0,0,0,0,0};
	static unsigned short td[8][2]={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
	for (i=0;i<8;i++)
	{
		if(Get_Yarn_zero_st(i))
		{
			if (!yzs_c[i])/*说明是第一次检查到*/
			{
				yarn_zero_change_cnt[i][YARN_ST_IDX_CURR] = td[i][0];
				if(yarn_zero_change_cnt[i][YARN_ST_IDX_CURR]>yarn_zero_change_cnt[i][YARN_ST_IDX_MAX])/*比之前记录的最大值还要大*/	
				{
					yarn_zero_change_cnt[i][YARN_ST_IDX_MAX] = yarn_zero_change_cnt[i][YARN_ST_IDX_CURR];/**/
				}
				if(yarn_zero_change_cnt[i][YARN_ST_IDX_CURR]<yarn_zero_change_cnt[i][YARN_ST_IDX_MIN])/*比之前记录的最大值还要大*/	
				{
					yarn_zero_change_cnt[i][YARN_ST_IDX_MIN] = yarn_zero_change_cnt[i][YARN_ST_IDX_CURR];/**/
				}
				td[i][0]=0;
			}
			if (td[i][0]<0xffff)
				td[i][0]++;
			//yarn_zero_input_cnt[i][YARN_ST_IDX_CURR]++;/*当前检测到了*/
			if (yzs_c[i]<0xFF)
				yzs_c[i]++;
			if(yzs_c[i]==3)  /*连续采集3次.只有第三次*/
			{
				yzst_change|=(0x01<<i);
			}
			yzst|=(0x01<<i);
		}
		else
		{
			if (yzs_c[i])
			{
				yzs_c[i]=0;
				yarn_zero_input_cnt[i][YARN_ST_IDX_CURR] = td[i][0];
				
				if(yarn_zero_input_cnt[i][YARN_ST_IDX_CURR]>yarn_zero_input_cnt[i][YARN_ST_IDX_MAX])/*比之前记录的最大值还要大*/	
				{
					yarn_zero_input_cnt[i][YARN_ST_IDX_MAX] = yarn_zero_input_cnt[i][YARN_ST_IDX_CURR];/**/
				}
				if(yarn_zero_input_cnt[i][YARN_ST_IDX_CURR]<yarn_zero_input_cnt[i][YARN_ST_IDX_MIN])/*比之前记录的最大值还要大*/	
				{
					yarn_zero_input_cnt[i][YARN_ST_IDX_MIN] = yarn_zero_input_cnt[i][YARN_ST_IDX_CURR];/**/
				}
				td[i][0] =0;
			}
			if (td[i][0]<0xFFFF)
				td[i][0]++;
		}
	}

	if (yzst_change)
	{
		/*有新信号*/
		Send_yarn_zero_st_change(yzst_change);
	}

	yarn_zero_st = yzst;  //当前状态

	
}



void Get_yarn_zero_debug(unsigned char whichone)
{
	int i;
	if (whichone>=8)
	{
		for (i=0;i<8;i++)
		{
			Message_Send_4halfword(0xCC|(i<<8)|(arch_Get_ID()<<12), yarn_zero_input_cnt[i][0], yarn_zero_input_cnt[i][1],yarn_zero_input_cnt[i][2]);
			Message_Send_4halfword(0xDD|(i<<8)|(arch_Get_ID()<<12), yarn_zero_change_cnt[i][0], yarn_zero_change_cnt[i][1],yarn_zero_change_cnt[i][2]);
		}
	}
	else
	{
		i=whichone;
		Message_Send_4halfword(0xCC|(i<<8)|(arch_Get_ID()<<12), yarn_zero_input_cnt[i][0], yarn_zero_input_cnt[i][1],yarn_zero_input_cnt[i][2]);
		Message_Send_4halfword(0xDD|(i<<8)|(arch_Get_ID()<<12), yarn_zero_change_cnt[i][0], yarn_zero_change_cnt[i][1],yarn_zero_change_cnt[i][2]);
	}


}


#endif







void GetMcu_IDCode(void)  
{  
	int i;
	
	for (i=0;i<3;i++)
		{
//获取CPU唯一ID  
	*(vu32 *)&Head_Board_id_[(i<<1)]=*(vu32*)(0x1fff7A10+(i<<2));  
		}
	//*(vu32 *)&Head_Board_id_[2]=*(vu32*)(0x1fff7A14);  
	//*(vu32 *)&Head_Board_id_[4]=*(vu32*)(0x1fff7A18);  
//Lock_Code=(CpuID[0]>>1)+(CpuID[1]>>2)+(CpuID[2]>>3);  
}


#define MAIN_BOARD_ID_FLASH_ADDR		90
#define MAIN_BINDING_FLAG_ENABLE		0xBABA			/*绑定*/
#define MAIN_BINDING_FLAG_DISABLE		0xABAB			/*解除*/
#define MAIN_BINDING_FLAG_ACK			0xAABB			/*心跳包*/

#define HEAD_APP_FLAG					0x0D3A			/*写入E方的特定值，表示具备某一种功能*/


/*
*
*	whichid =0--前半部分，只记录，不存档
*	whichid =1--后半部分，记录，并存档
*	whichid =2--绑定标志，记录，并存档
*/
void Exec_write_main_id_to_flash(unsigned short *mainid,unsigned char whichid)
{
	int ret=0;
	unsigned short *w_p;
	unsigned char len;
	int addr_; 
	if (whichid<2)
	{		
		len =6;
		addr_ = MAIN_BOARD_ID_FLASH_ADDR;
		w_p =  (unsigned short *)Main_Board_id_flash;
		memcpy((unsigned char *)&Main_Board_id_flash[whichid?3:0],(unsigned char *)mainid,len);
	}
	else
	{
		//des_p =(unsigned char *)&Main_Head_Binding_flag;
		len =1;
		addr_ = MAIN_BOARD_ID_FLASH_ADDR+6;
		w_p = mainid;
		Main_Head_Binding_flag = (*mainid)?1:0;
		*mainid = (Main_Head_Binding_flag)?(MAIN_BINDING_FLAG_ENABLE):(MAIN_BINDING_FLAG_DISABLE);
	}	
		if (whichid)
		{
			//writeflash_index=WRTEFLASH_BASE+2;
			ret = EE_Write(addr_,w_p,len);
			if (ret)
			{
			/*write flash error*/
				Message_Send_4halfword(0xFFF8, addr_,len,ret);
		
			}
		
		}
	
	return;
}

unsigned short IDMASK(unsigned char *buf,unsigned long dlen, int poly, unsigned short init)
{      unsigned char ch;       
	unsigned short mask = init;     
	int i;     
	while (dlen--)       
	{              
		ch = *buf++;         
		mask ^= (((unsigned short) ch) << 8);              
		for (i = 0; i < 8; i++)             
		{                      
			if (mask & 0x8000)          
				mask = (mask << 1) ^ poly;           
			else                             
				mask <<= 1;                
		}       
	}  
	return mask ^ 0xFFFF;
}

int Exec_check_Data_valid(unsigned short checkdata,unsigned short poly,unsigned short  isbinding)
{
	static unsigned short  id_acd=0; 
	unsigned short Main_Board_id_TEMP[6];	
	unsigned short mask_data;
	unsigned short def_data;
	
	memcpy((unsigned char *)Main_Board_id_TEMP,(unsigned char *)Main_Board_id_flash,sizeof(Main_Board_id_TEMP));
	switch(isbinding)
	{
		case 0:
			def_data = MAIN_BINDING_FLAG_DISABLE;
			break;
		case 1:
			def_data = MAIN_BINDING_FLAG_ENABLE;
			break;
		case 2:
			def_data = MAIN_BINDING_FLAG_ACK;
			Main_Board_id_TEMP[1]+=id_acd;
			id_acd++;
			break;
		default :
			return -2;
			break;
	}

	
	
	mask_data = IDMASK((unsigned char *)Main_Board_id_TEMP,12,poly,def_data);
	//Message_Send_4halfword(0xFFF7,mask_data, poly,def_data);
	if (mask_data!=checkdata)
	{
		return -1;
	}
	else
		return 0;	
}

void Exec_Set_Tryout_time_flash(unsigned short *tryouttime)
{
	int ret=0;
	unsigned short *w_p;
	unsigned char len;
	int addr_; 	
			
	if((head_tryout_time_setflag==0)||(head_tryout_time_par!=*tryouttime))
	{
	#if 0
		len =1;
		addr_ = MAIN_BOARD_ID_FLASH_ADDR+8;
		w_p = tryouttime;	
		ret = EE_Write(addr_,w_p,len);
		if (ret)
		{
			/*write flash error*/
			Message_Send_4halfword(0xFFF8, addr_,len,ret);		
		}
	#endif	
		head_tryout_time_par = *tryouttime;
		head_tryout_time_setflag =1; /**/
	}		
	return;

}

/*机头默认试用时间30分钟*/
#define HEAD_TRYOUT_TIME_DEFAULT_M	30
unsigned short Exec_read_Tryout_time_flash()
{
	unsigned short bf=0;
	int ret =1;// EE_Read(MAIN_BOARD_ID_FLASH_ADDR+8,&bf,1);
	if (ret)
	{
		head_tryout_time_par= HEAD_TRYOUT_TIME_DEFAULT_M;
		head_tryout_time_setflag= 0;
	}
	else
	{
		head_tryout_time_par= bf;
		head_tryout_time_setflag= 1;
	}
}

void  Exec_Set_main_id_(unsigned short *mainid,unsigned char whichid)
{
	memcpy((unsigned char *)&Main_Board_id_[whichid?3:0],(unsigned char *)mainid,6);
	//memcpy((unsigned char *)&Main_Board_id_timer[whichid?3:0],(unsigned char *)mainid,6);
}

void  Exec_Set_main_id_timer()
{
//	memcpy((unsigned char *)&Main_Board_id_timer[0],(unsigned char *)&Main_Board_id_[0],12);
}



void  Exec_Get_head_id_(unsigned short cmdret,unsigned char whichid)
{
	unsigned short *p_hbid;
	p_hbid = (unsigned short *)&Head_Board_id_[whichid?3:0];
	Message_Send_4halfword(cmdret,*p_hbid,*(p_hbid+1),*(p_hbid+2));	
}

void Exec_Get_Binding_st_(unsigned short cmdret)
{
	Message_Send_4halfword(cmdret,Main_Head_Binding_flag,Main_Head_Binding_is_fail?0:1,0);	
}

void Exec_read_main_flash()
{
	int ret;
	ret = EE_Read(MAIN_BOARD_ID_FLASH_ADDR,(unsigned short *)Main_Board_id_flash,6);
	if (ret)
	{
		memset((void *)Main_Board_id_flash,0,12);
	}
}

void Exec_read_binding_flag()
{
	unsigned short bf=0;
	int ret = EE_Read(MAIN_BOARD_ID_FLASH_ADDR+6,&bf,1);
	if (ret)
	{
		Main_Head_Binding_flag = 0;
	}
	else
		Main_Head_Binding_flag =(bf==MAIN_BINDING_FLAG_ENABLE)?1:0;
	
}


void Exec_read_app_last_ver()
{
	//unsigned short bf=0;
	unsigned short bf1=0;
	extern unsigned short lastAPP_ver;
	
	int ret= EE_Read(MAIN_BOARD_ID_FLASH_ADDR+8,&bf1,1);
	if (ret)
	{
		/*没读到这个值，那要写一个进去*/
		goto write_flag_to_ee;
	}
	else
	{
		if (bf1==lastAPP_ver)
		{
			return;
		}
		else
			goto write_flag_to_ee;
	}

	//if (0)
	{
write_flag_to_ee:
		bf1= lastAPP_ver;
		//writeflash_index=WRTEFLASH_BASE+3;
		EE_Write(MAIN_BOARD_ID_FLASH_ADDR+8, &bf1, 1);
		
	}
}


void Exec_read_app_flag()
{
	//unsigned short bf=0;
	unsigned short bf1=0;
	
	int ret= EE_Read(MAIN_BOARD_ID_FLASH_ADDR+7,&bf1,1);
	if (ret)
	{
		/*没读到这个值，那要写一个进去*/
		goto write_flag_to_ee;
	}
	else
	{
		if (bf1==HEAD_APP_FLAG)
		{
			return;
		}
		else
			goto write_flag_to_ee;
	}

	//if (0)
	{
write_flag_to_ee:
		bf1= HEAD_APP_FLAG;
		//writeflash_index=WRTEFLASH_BASE+4;
		EE_Write(MAIN_BOARD_ID_FLASH_ADDR+7, &bf1, 1);
		
	}
}

void Exec_Read_main_id_from_flash()
{
	
	Exec_read_main_flash();
	
	Exec_read_binding_flag();

	//Exec_read_app_flag();	
	Exec_read_app_last_ver();

	Exec_read_Tryout_time_flash();
	
}


void Exec_main_head_board_id_init()
{
	memset((void *)Main_Board_id_,0,12);
	//memset((void *)Main_Board_id_timer,0,12);	
	memset((void *)Main_Board_id_flash,0,12);
	memset((void *)Head_Board_id_,0,12);
	memset((void *)&Main_Head_Binding_flag,0,2);
	GetMcu_IDCode();
	Exec_Read_main_id_from_flash();
}

void Set_check_mainbinding_timedelay(unsigned int delay_time_m)
{
	if (Main_Head_Binding_flag)
	{
		wait_time_ms_for_check_binding = delay_time_m*30000L;
	}
	else
		wait_time_ms_for_check_binding=0;

	return;
	
}
												/* 120*1*60*500--两小时*/
#define CHECK_BINGD_STATUS_BLANK_TIME_DEF	        3600000L

void Exec_check_is_binding_ok(unsigned char istimercheck,unsigned char ispass)
{
	
	if (Main_Head_Binding_flag)
	{
		if (ispass)
		{
			Main_Head_Binding_is_fail =0;
			wait_time_ms_for_check_binding = CHECK_BINGD_STATUS_BLANK_TIME_DEF;
			ack_ok =0;
		}
		else
		{	
			if(istimercheck)
			{
				//memset((void *)Main_Board_id_timer,0,12);	
				if(ack_ok)
				{
					ack_ok =0;
					Main_Head_Binding_is_fail =0;
					wait_time_ms_for_check_binding = CHECK_BINGD_STATUS_BLANK_TIME_DEF;
				}
				else
					{						
						Main_Head_is_lock=1;
						goto head_need_lock;
					}
				
			}
			else
			{		
				unsigned char *MBID_p;
				MBID_p = (unsigned char *)Main_Board_id_;

				if (memcmp((void *)MBID_p,(const void *)Main_Board_id_flash,12))
				{
head_need_lock:				
					/*有异常了*/
					alert_push(HEAD_BINDING_MAIN_ERR,0);
					Main_Head_Binding_is_fail =1;
					if(istimercheck)
						Main_Head_is_lock =1;					
				}
				else
				{
					Main_Head_Binding_is_fail =0;
					wait_time_ms_for_check_binding = CHECK_BINGD_STATUS_BLANK_TIME_DEF;
				}
			}
		}	
			
		

	}
	
}


void  Exec_Send_main_flash_id_(unsigned short cmdret,unsigned char whichid)
{
	unsigned short *p_hbid;
	Exec_read_main_flash();
	p_hbid = (unsigned short *)&Main_Board_id_flash[whichid?3:0];
	Message_Send_4halfword(cmdret,*p_hbid,*(p_hbid+1),*(p_hbid+2));	
}

unsigned char arch_need_close711_alert()
{
	extern volatile unsigned char Step_use_exBoard;
	extern unsigned short cpld_ver;
	
	if((!cpld_ver) &&(Step_use_exBoard))
	{
		return 1;
	}
	return 0;
}


unsigned int arch_StepMotor_Zero_mid_layer(unsigned int idx,unsigned char isnc)
{
	return isnc?(!arch_StepMotor_Zero(idx)):arch_StepMotor_Zero(idx);
}

unsigned int arch_StepMotor_Work_mid_layer(unsigned int idx,unsigned char isnc)
{
	return isnc?(!arch_StepMotor_Zero(idx)):arch_StepMotor_Zero(idx);
}

void Set_checkin_time(unsigned int delaytime_ms)   
{
	checkin_time_delay2ms_set = (delaytime_ms>>1);
	//if(checkin_time_delay2ms)
	checkin_time_delay2ms_delay =0;
}

void CheckIn_Timer()
{
	//static unsigned int delay_checkin=0;
	if(checkin_time_delay2ms_set)
	{
		checkin_time_delay2ms_delay++;
		if(checkin_time_delay2ms_delay>=checkin_time_delay2ms_set)
		{
			checkin_need_send =1;
			checkin_time_delay2ms_delay =0;
		}			
	}
}

void Checkin_need_send_data(unsigned short retcmd)
{
	if((checkin_need_send)&&(checkin_time_delay2ms_set))
	{
		checkin_need_send =0;
		Message_Send_4halfword(retcmd,0,checkin_index&0xFFFF,checkin_index>>16);
		checkin_index++;
	}
}

#ifdef READ_FLASH_DATA_FOR_BANDING

unsigned char start_2_read=0;
unsigned char readdata_timer=0;

unsigned short Flash_data_first_add __attribute__((at(0x0800c000)));
unsigned int Flash_add = (unsigned int)&Flash_data_first_add;

void set_start_to_read_flashdata(unsigned short onoff)
{
	start_2_read=onoff?1:0;
}

void read_flash_data()
{
	static unsigned short dataidx=0;
	unsigned short *paddr ;
	paddr=(unsigned short *)Flash_add;
	if((start_2_read)&&(readdata_timer==0))
	{
		unsigned short xx[3]={0,0,0};
		int i;
		for (i=0;i<3;i++)
		{
			xx[i]=*(paddr++);
		}
		Flash_add=(unsigned int)paddr;
		Message_Send_4halfword(dataidx++,xx[0],xx[1],xx[2]);
		readdata_timer=6;
		if(dataidx>=0x4000)
		{
			dataidx =0;
			start_2_read =0;
		}
		
	}
}

void arch_read_data_timer250us()
{
	if(readdata_timer) readdata_timer--;
}

#endif
