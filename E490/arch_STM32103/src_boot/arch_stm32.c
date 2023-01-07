/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f2xx.h"
#include "platform_config.h"
#include "alert.h"

#ifdef BOOT_CODE_JOIN_TEST_CODE
#include "step.h"
#endif

typedef unsigned long  DWORD;
typedef unsigned char  BYTE;
//typedef unsigned long  uint32_t;
typedef void (*pFunction)(void);

//volatile unsigned short writeflash_index=0;

volatile DWORD step_speed[10];
volatile BYTE led_status = 0;
volatile BYTE led_disable = 0;
volatile DWORD led_delay = 0;
volatile DWORD led_delay_setup = 0;
volatile unsigned int wait_time_ms = 0;
volatile unsigned int wait_time_us = 0;
volatile unsigned int arch_mode = 0;
volatile unsigned int arch_board_id = 0;

//volatile  int Check_APP_isOK=-1;
volatile unsigned short last_app_ver;


#define TEST_CODE_EMF_JQD		(0)
#define TEST_CODE_EMF_ACT		(1)
#define TEST_CODE_EMF_YARN	(2)
#define TEST_CODE_EMF_TYPE_COUNT	(TEST_CODE_EMF_YARN+1)
/*//(4*8+8+2*6)  4个8刀选针器，8个纱嘴，2个6刀三角电磁铁*/
#define TEST_CODE_EMF_MAX_ELEMENT	(52)   

/*存放设定的时间参数*/
volatile unsigned char Test_arg_EMF_clear_Time[TEST_CODE_EMF_TYPE_COUNT];

/*存放延时计数值，在定时器里做--操作*/
volatile unsigned char Test_arg_EMF_TimeDelay[TEST_CODE_EMF_MAX_ELEMENT];

/*每2个bit位表示一个电磁铁的状态*/
/*          				状态表示方法如下*/
/*					bit1,bit0	*/
/*					0,	0	--clear_off    	表示已经完成动作，且动作为off*/		
/*					0,	1	--clear_on  	表示已经完成动作，且动作为on*/
/*					1,	0	--do_off    	表示正在做动作，且动作为off*/
/*					1,	1	--do_on		表示正在做动作，且动作为on*/
volatile unsigned char Test_arg_EMF_sts[TEST_CODE_EMF_MAX_ELEMENT>>2] ;
#define TEST_CODE_STEP_TTYPE_1_4		(0)
#define TEST_CODE_STEP_TTYPE_5_6		(1)
#define TEST_CODE_STEP_TTYPE_7_8		(2)
#define TEST_CODE_STEP_TTYPE_9_12		(3)
#define TEST_CODE_STEP_TTYPE_COUNT	(TEST_CODE_STEP_TTYPE_9_12+1)
volatile unsigned int Test_code_Step_speed_HZ[TEST_CODE_STEP_TTYPE_COUNT];

unsigned short cpld_name=0;
unsigned short cpld_ver=0;
unsigned short cpld_expID=0;

volatile unsigned int DC24N_ALARM_set=1320;	//1320 mv=1320/55=24A  
volatile unsigned int DC24P_ALARM_set=1320;	//1320 mv=1320/55=24A  
volatile unsigned char SYS_is_PowerOn=0;

#ifdef BOOT_CODE_JOIN_TEST_CODE

#define STEP_ERR_CODE(_idx,_type)	((_type==MOTOR_TYPE_DENSITY) ? (STEP_ERR + _idx) : \
									((_type==MOTOR_TYPE_SKINER) ? (SINKER1_ERR + _idx) :\
									((_type==MOTOR_TYPE_ACTION) ? (TRIANGLE1_ERR+_idx):\
									((_type==MOTOR_TYPE_FEET) ? (FEET1_ERR + _idx):\
									((_type==MOTOR_TYPE_YARN)?(YARN_STEP1_ERR+_idx):\
									(DEVICE_CONFIG_NUM_ERROR))))))

#define FULL	0
#define HALF	1
#define QUARTER	2
#define EIGHTH	3

#define RESOLUTION	2


#define NEW_ZERO_DETECT_MODE


#define MAX_STEPS	2000L


#define ACC_STEPS_ACTION	(17)
#define ACC_STEPS_FEET  	(17)
#define ACC_STEPS_YARN  	(17)
#define ACC_STEPS_DM		(17)
#define ACC_STEPS_SINKER	(17)




volatile unsigned char Head_Mode_=0;

unsigned short Test_code_ver=0x0001;

STEP_TYPE STEPMOTOR[12];

volatile unsigned char StepMotor_Count_MAX=0; //系统支持的电机数

volatile unsigned int stepmotor_speed[MOTOR_TYPE_COUNT][2];	//速度值
volatile unsigned int stepmotor_speed_isset[MOTOR_TYPE_COUNT][2];	//是否已经设置

volatile unsigned int stepmotor_steps[MOTOR_TYPE_COUNT][2]; //步数，部分电机用到

volatile unsigned int stepmotor_AccSteps[MOTOR_TYPE_COUNT]; //加减速步数，
volatile unsigned int step_base_speed[MOTOR_TYPE_COUNT];

volatile unsigned int step_resolution[MOTOR_TYPE_COUNT];

volatile int step_alert_detect_setup[MOTOR_TYPE_COUNT];	//电机失步报警精度(按照电机类型来)

volatile unsigned char Steps_ID_NO[MOTOR_TYPE_COUNT+1][16];		//0-idall,1-idself_d,2-idself_s,3-idself_a,4-idself_f

volatile unsigned int emf_status[3];

volatile int sinker_zero_area;
volatile int step_zero_adj;				//到了0位之后，还要走这么些步数

volatile unsigned int step_run_mode;
volatile unsigned int sinker_add_speed; //hlc 20140725

volatile unsigned int step_interval;

volatile unsigned int step_work_sign_alarmenable;


#ifdef E480_BOARD_V10
volatile unsigned int POS_Yarn_DO=200;
#endif

#ifdef ZERO2_SUPPORT
//volatile unsigned int step_zero2_enable;
volatile int step_setup_zero2_detect_steps;
void StepMotor_Detect_Zero2(unsigned int stepno);
#endif


void Test_code_arch_clear_EMF(unsigned char emfid);
void Test_code_StepMotor_Init();
void Test_code_StepMotor_Isr(unsigned int stepno);
void Test_code_StepMotor_exec(unsigned int stepno, short pos, int mode);
void Test_code_StepMotor_Reset(unsigned int stepno);
void Test_code_hook_time_250us(void) ;	
void Test_code_check_do_emf_clear();
void arch_StepMotor_Stop(unsigned int stepno);
void arch_StepMotor_Start(unsigned int stepno);

#endif
extern unsigned int error_active;
extern int can_receive_count;
extern volatile unsigned char Yarn_use_Step;

void GPIO_Configuration(void);
void USART_Configuration(void);
void CAN_Configuration(void);
void TIM_Init(void);
void FSMC_Init(void);
void FLASH_Unlock(void);
u16 EE_Init(void);
int EE_Read(int addr, u16 *Data, int len);
int EE_Write(int addr, u16 *Data, int len);
void EXTI_Config(void);
void EXTI_Enable(u32 EXTI_Line);
void EXTI_Disable(u32 EXTI_Line);
void time_2ms_prog(void);
void time_250us_prog(void);
void wait_ms(unsigned int ms);
void wait_us(unsigned int us);
void Shell_Init(void);
void myprintf(const char *format, ...);
void IWDG_Init(void);
void IWDG_Enable(void);
void shell_hook_time(void);
void arch_StepMotor_Disable(void);
int Exec_check_app_isok_from_flash();
unsigned int Exec_check_app_last_ver();


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

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div4);

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

	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
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
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00000);
#endif
	
}

u8 myStrCmp(char *s0, char *s1)
{
	for (; *s0 == *s1; ++s0, ++s1)
		if (*s0 == '\0')
			return 0;

	return (*(unsigned char *)s0 < *(unsigned char *)s1 ? -1 : +1);
}

void delayus()
{
	volatile int us = 1;
	while(--us);
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

void arch_LED_Setup(DWORD time)
{
	led_delay_setup = time;
}

#if 0
void arch_LED_timer(void)
{
	if(led_delay_setup == 0) return;
	if(led_delay) {
		led_delay --;
		return ;
	}

	led_delay = led_delay_setup;
	led_status = !led_status;

	if(led_status) arch_LED_On();
	else arch_LED_Off();
}


void arch_LED_Start(void)
{
	led_disable = 0;
}

void arch_LED_Stop(void)
{
	led_disable = 1;
}
#endif

#ifdef BOOT_CODE_JOIN_TEST_CODE

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
	else
		{
		return stepidex;
		}
	
	#endif
	return stepidex;

}

#endif

void arch_YARNEMF_AllClear()
{
	int i;
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
	

}

void arch_ACTEMF_AllClear()
{
	int i;
	
		for(i = 0; i < 6; i ++) {
		OUTPUT(ADDR_ACTBOARD1 + i, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD1 + i + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD2 + i, ACT_OUT_IS_NOT?0:1);
		OUTPUT(ADDR_ACTBOARD2 + i + ADDR_DIS_ACT_IN_EN, ACT_OUT_IS_NOT?0:1);
	}
}

void arch_Jacquard_AllClear()
{
	

	int i;
	unsigned char jqdsts;
	for(i = 0; i < MAX_BLADE_HARDWARE* 2; i ++) 
	{					
		jqdsts=0;
		OUTPUT(ADDR_JQDBOARD1 +i, jqdsts);
		delayus();
		OUTPUT(ADDR_JQDBOARD1 + i + ADDR_DIS_JQD_IN_EN, 0);
		delayus();
		jqdsts=0;
		OUTPUT(ADDR_JQDBOARD2 +i, jqdsts);
		delayus();
		OUTPUT(ADDR_JQDBOARD2 + i + ADDR_DIS_JQD_IN_EN, 0);
		delayus();	
						
	}
}

#ifdef BOOT_CODE_JOIN_TEST_CODE
unsigned int StepMotor_get_no_with_IDself(unsigned int step_IDself,unsigned char step_type)
{
	STEP_TYPE *Step;
	int i;

	if (step_type<=MOTOR_TYPE_COUNT)
		return Steps_ID_NO[step_type][step_IDself & 0x0f];
	return 0xff;


}


void arch_YARN_Step_Setup(int yno, int onoff)
{
	unsigned int step_id;
	unsigned short tmp_stus;

	unsigned short last_stus;

	int moto_pos;	

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
			
			if(STEPMOTOR[step_id].is_poweron ==0)
			{
			     
			         Test_code_StepMotor_Reset(step_id);
				 STEPMOTOR[step_id].need_2_pos_after_reset = moto_pos;
			}
			else
			{
			         Test_code_StepMotor_exec(step_id,moto_pos,1);    
				      					   
			}
		}
	}
	

}



void Test_code_arch_JQDEMF_clear(int jqdindex)
{
		unsigned long addr;
		int jqdno;
		int blade;
		jqdno = jqdindex >>3;
		blade = jqdindex & 0x07;
		addr = ADDR_JQDBOARD1;		
		jqdno &= 0x3;
		addr += jqdno * (8) + blade;

		OUTPUT(addr + ADDR_DIS_JQD_IN_EN, 0);			
		delayus();
}


void Test_code_arch_YARNEMF_Clear(int yno)
{
	int addr;

	if(yno >= MAX_YARN) {
		return;
	}

#ifdef E480_BOARD_V10
	if (Yarn_use_Step)
	{
		return;
	}
#endif
	
	//arch_Clear_check_JQD_YARN_ACT(CHECK_DC24_YARN);
	
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


void Test_code_arch_ACTEMF_Clear(int actno)
{

	unsigned long addr;

	if(actno >= MAX_ACTEMF) {
		return;
	}
	if(actno < 12) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 12;
		addr = ADDR_ACTBOARD2;
	}

	addr += actno;

	OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
	delayus();
	OUTPUT(addr + ADDR_DIS_JQD_IN_EN, ACT_OUT_IS_NOT?0:1);

}

void Test_code_arch_JQDEMF_set(int jqdindex,unsigned char onoff)
{
		unsigned long addr;
		int jqdno;
		int blade;
		jqdno = jqdindex >>3;
		blade = jqdindex & 0x07;
		addr = ADDR_JQDBOARD1;		
		jqdno &= 0x3;
		addr += jqdno * (8) + blade;

		OUTPUT(addr, onoff?0:1);			//in	
		delayus();
		OUTPUT(addr + ADDR_DIS_JQD_IN_EN, 1);			//en

}

void Test_code_arch_YARNEMF_set(int yno,unsigned char onoff)
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
	
	if(onoff) {
		yarn1 = 1;
		yarn2 = 0;
	}
	else {
		yarn1 = 0;
		yarn2 = 1;
	}
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
	
}



void Test_code_arch_ACTEMF_set(int actno,unsigned char onoff)
{
	unsigned long addr;

	if(actno >= MAX_ACTEMF) {
		return;
	}
	if(actno < 12) {
		addr = ADDR_ACTBOARD1;
	}
	else {
		actno -= 12;
		addr = ADDR_ACTBOARD2;
	}
	addr += actno;
	if(onoff) {
		OUTPUT(addr, ACT_OUT_IS_NOT?0:1);
		delayus();
		OUTPUT(addr + ADDR_DIS_JQD_IN_EN, ACT_OUT_IS_NOT?1:0);
	}
	else {
		OUTPUT(addr + ADDR_DIS_JQD_IN_EN, ACT_OUT_IS_NOT?0:1);
		delayus();
		OUTPUT(addr, ACT_OUT_IS_NOT?1:0);
	}	

}

void Test_code_arch_clear_EMF(unsigned char emfid)
{
	
	int blade;
	int jqdno;
	
	if (emfid<32)   //选针
	{		
		Test_code_arch_JQDEMF_clear(emfid);			
	}
	else
	if (emfid<40)	//纱嘴
	{
		Test_code_arch_YARNEMF_Clear(emfid-32);
	}
	else
	if (emfid<52)	//三角
	{
		Test_code_arch_ACTEMF_Clear(emfid-40);
	}
	jqdno = emfid>>2;
	blade = emfid & 0x03;
	Test_arg_EMF_sts[jqdno] &=~((unsigned char)0x01<<((blade*2)+1));
	
}

void Test_code_arch_Set_EMF(unsigned char emfid,unsigned char onoff)
{
	int blade;
	int jqdno;
	
	if (emfid<32)   //选针
	{		
		Test_arg_EMF_TimeDelay[emfid] =Test_arg_EMF_clear_Time[TEST_CODE_EMF_JQD] ;
		Test_code_arch_JQDEMF_set(emfid,onoff);	
		
	}
	else
	if (emfid<40)	//纱嘴
	{
		Test_arg_EMF_TimeDelay[emfid] =Test_arg_EMF_clear_Time[TEST_CODE_EMF_YARN] ;
		Test_code_arch_YARNEMF_set(emfid-32,onoff);
	}
	else
	if (emfid<52)	//三角
	{
		Test_arg_EMF_TimeDelay[emfid] =Test_arg_EMF_clear_Time[TEST_CODE_EMF_ACT] ;
		Test_code_arch_ACTEMF_set(emfid-40,onoff);
	}
	jqdno = emfid>>2;
	blade = emfid & 0x03;
	Test_arg_EMF_sts[jqdno] |=((unsigned char)0x01<<((blade*2)+1));

	if (onoff)
	{
		Test_arg_EMF_sts[jqdno] |=((unsigned char)0x01<<((blade*2)));
	}
	else
	{
		Test_arg_EMF_sts[jqdno] &=~((unsigned char)0x01<<((blade*2)));
	}
	
	
}


void Test_code_arch_EMF_init()
{
	int i;
	for (i=0;i<(TEST_CODE_EMF_MAX_ELEMENT);i++)
	{
		Test_arg_EMF_sts[i>>2] = 0;
		Test_arg_EMF_TimeDelay[i] = 0;
	}
	
	{
		Test_arg_EMF_clear_Time[TEST_CODE_EMF_JQD] 	=8;		// 2ms
		Test_arg_EMF_clear_Time[TEST_CODE_EMF_ACT] 	=120;   	//30ms
		Test_arg_EMF_clear_Time[TEST_CODE_EMF_YARN] 	=120;  	//30ms
		
	}
	
	for (i=0;i<TEST_CODE_STEP_TTYPE_COUNT;i++)
	{
		Test_code_Step_speed_HZ[i] =3000;
	}
	

	
}

#endif

#if 0
void arch_Exti_disable()
{
	EXTI_Disable(EXTI_Line0);
	EXTI_Disable(EXTI_Line1);
	EXTI_Disable(EXTI_Line5);
	EXTI_Disable(EXTI_Line6);
	EXTI_Disable(EXTI_Line8);
	EXTI_Disable(EXTI_Line12);
	EXTI_Disable(EXTI_Line13);
	EXTI_Disable(EXTI_Line14);	
	EXTI_Disable(EXTI_Line15);
}
#endif



void arch_Power_Off(void)
{
	//myprintf("Power Off ____\r\n");
	//arch_Exti_disable();

	
	POWER_XZ_POSITIVE24V_OUTPUT(0); 
	POWER_XZ_NEGATIVE24V_OUTPUT(1);
	POWER_EXT_3V3_OUTPUT(1);	
	//POWER_XZ_NEGATIVE24V_OUTPUT(1);
	//POWER_SJ_NEGATIVE24V_OUTPUT(1);
	//POWER_SZ_NEGATIVE24V_OUTPUT(1);
	
	

}


void arch_Power_On(void)
{
	//myprintf("Power Off ____\r\n");
	//arch_Exti_disable();

	POWER_EXT_3V3_OUTPUT(0);
	POWER_XZ_POSITIVE24V_OUTPUT(1);  
	POWER_XZ_NEGATIVE24V_OUTPUT(0);
	//POWER_SJ_NEGATIVE24V_OUTPUT(1);
	//POWER_SZ_NEGATIVE24V_OUTPUT(1);
	

}


unsigned int arch_Get_Board_Type(void)
{


#ifdef E475_480_BOARD

switch (cpld_name)
{
	case BOARD_NAME_E475:

	return 0x8007;
	break;
	case BOARD_NAME_E480:	
	{
		return 0x8008;
	}
	break;
	case BOARD_NAME_E490:	
	{
		return 0x8009;
	}
	break;
	
	default:
	{
		return 0x0000;
		break;		
	}
}

	//return 0x8006;
#endif
}

void WatchDog_Kick(void)
{
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
	myprintf("\r\nGoto Application\r\n");

	//check app is empty   
	{
		int i;
		for(i=0;i<2;i++)
		{
			if(*(volatile uint32_t*) (APP_START_ADDRESS + i*4)!=0xFFFFFFFF)
			{
				break;
			}
			else
				continue;
		}
		if(i==2) //app is empty
		{
			return ;
		}
	}

	
	JumpAddress = *(volatile uint32_t*) (APP_START_ADDRESS + 4);
	Jump_To = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__set_MSP(*(volatile uint32_t*) APP_START_ADDRESS);
	Jump_To();
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
	return 0;//(FLASH_GetReadOutProtectionStatus() == SET);
}
#if 0
void wait_ms(unsigned int ms)
{
	//wait_time_ms = ms << 1;
	wait_time_ms = ms >> 1;
	while(wait_time_ms);
}

void wait_us(unsigned int us)
{
	wait_time_us = (us + 511) >> 9;
	while(wait_time_us);
}


void hook_time_2ms(void)
{
	if(wait_time_ms > 0) wait_time_ms --;

	//time_2ms_prog();
	arch_LED_timer();
}

#endif

#if 0
void hook_time_50us(void)
{
};

void hook_time_250us(void)
{
#ifdef BOOT_CODE_JOIN_TEST_CODE
	Test_code_hook_time_250us();
#endif

};



unsigned int arch_GetKey(void)
{
	//return STATUS_KEY_INPUT();
	return 0;
}

unsigned int arch_need_current_add(unsigned int currentdata)
{
	return currentdata;
}
#endif

unsigned int arch_GetBoardID(void)
{
	unsigned int ret = 0;
	ret= BOARD_ID_INPUT_0();
	ret |=(BOARD_ID_INPUT_1()<<1);
	ret |=(BOARD_ID_INPUT_2()<<2);
	ret |=(BOARD_ID_INPUT_3()<<3);	
	return ret;
}

unsigned int arch_GetBoardID_E480(void)
{
	unsigned int ret = 0;
	ret= BOARD_ID_INPUT_2();
	ret |=(BOARD_ID_INPUT_3()<<1);
	return ret;
}

unsigned int arch_Get_Mode(void)
{
	return arch_mode ;
}


unsigned int arch_Get_ID(void)
{
	return arch_board_id;
}


#if 0
void	arch_8844_Reset_begin()
{

	XZ_RESET_OUTPUT(0);	
	delay(10);
	XZ_RESET_OUTPUT(1);
	delay(10);	
}




void	arch_8844_Reset_end()
{

	XZ_RESET_OUTPUT(0);	
	
}

#endif

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

#if 0
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

#endif
unsigned short Get_CPLD_Ver_E490()
{
	unsigned short ret=0;
	ret= E490_BOARD_VER0();
	ret |=(E490_BOARD_VER1()<<1);
	ret |=(E490_BOARD_VER2()<<2);	
	ret |=(E490_BOARD_VER3()<<3);
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
	int i;
	int k;
	unsigned short cpld_name_temp=BOARD_NAME_E490;
	unsigned short cpld_ver_temp=0;
	unsigned short cpld_expID_temp=0;

	k=0;
	if (cpld_name_temp !=cpld_name)
	{
		cpld_name =cpld_name_temp;		
	}

	cpld_ver_temp=Get_CPLD_Ver_E490();	
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

unsigned int Scan_error(void)
{
	return 0;
}


#ifdef BOOT_CODE_JOIN_TEST_CODE





void Test_code_hook_time_250us(void)  //
{
	static unsigned int us_250=0;
	
	if(wait_time_us > 0) wait_time_us --;

	Test_code_check_do_emf_clear();
	
	if ((us_250 & 0x3)==0)  // 4的倍数说明到1ms了
	{
		//do 1ms thing;
	}
	if ((us_250 & 0x7)==0)  // 8的倍数说明到2ms了
	{
		//do 2ms thing;
	}
	
	
	//time_250us_prog();
	//shell_hook_time();
}



void Test_code_check_do_emf_clear()
{	
	unsigned char i;
	for (i=0;i<TEST_CODE_EMF_MAX_ELEMENT;i++)
	{
		if (Test_arg_EMF_TimeDelay[TEST_CODE_EMF_MAX_ELEMENT])
		{
			Test_arg_EMF_TimeDelay[TEST_CODE_EMF_MAX_ELEMENT]--;
			if (Test_arg_EMF_TimeDelay[TEST_CODE_EMF_MAX_ELEMENT]==0)
			{
				Test_code_arch_clear_EMF(i);
			}
			else
				continue;
		}
		else
			continue;
	}
	
}

void Test_code_hook_TIM2_isr(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		Test_code_StepMotor_Isr(0);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

		Test_code_StepMotor_Isr(1);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

		Test_code_StepMotor_Isr(2);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

		Test_code_StepMotor_Isr(3);
	}
}

void Test_code_hook_TIM3_isr(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		Test_code_StepMotor_Isr(4);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		Test_code_StepMotor_Isr(5);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		Test_code_StepMotor_Isr(6);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		Test_code_StepMotor_Isr(7);
	}
}

void Test_code_hook_TIM8_isr(void)
{
	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

		Test_code_StepMotor_Isr(8);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

		Test_code_StepMotor_Isr(9);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);

		Test_code_StepMotor_Isr(10);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);

		Test_code_StepMotor_Isr(11);
	}
}



void Test_Code_TIM_Config(TIM_TypeDef* TIMx,unsigned char isapb2)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;

	
	TIM_TimeBaseStructure.TIM_Prescaler = isapb2?119:59;

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 10;

	TIM_OC1Init(TIMx, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC3Init(TIMx, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC4Init(TIMx, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}


void Test_code_adc_init()
{
	Time_PWM_init();
	ADC_HardInit();
	DAC_HardInit();
	
}

void Test_code_TIM_Init()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable TIM clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  


	Test_Code_TIM_Config(TIM2,0);		//电机1-4
	Test_Code_TIM_Config(TIM3,0);		//电机5-8
	Test_Code_TIM_Config(TIM8,1);		//电机9-12

	
	//TIM_Config5(TIM5);
	//Test_Code_TIM_Config(TIM6,0);
	//TIM_Config7(TIM7);
	

	/* TIM */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //脭脢脨铆脰脨露脧 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

	//NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	//NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   



	//BSP_CLK_SysTick_Config(120000);
	SysTick_Config(30000);//  250us   =120000000/4000 
	//Time_start();
}


void Test_code_Can_isr_Enable()
{	
	extern CAN_Enable_isr(void);

	CAN_Enable_isr();

}

void Test_code_Get_Board_Data()
{
	Get_Cpld_Name_Ver();
	
}

void Test_code_Step_Motor_init()
{
	Test_code_StepMotor_Init();
	arch_StepMotor_Init();
}


void Test_code_Printf()
{
	myprintf("\r\n =====================");
	myprintf("\r\n ==    Test_Code Start   ==");
	myprintf("\r\n =====================");
	myprintf("\r\n\r\n");
}

void Test_code_arch_init()
{
	// 1、timer init
	Test_code_TIM_Init();

	Test_code_adc_init();

	// 2、Can_init
	Test_code_Can_isr_Enable();

	// 3、Board Data
	Test_code_Get_Board_Data();
	// 4、Step_motor_init()
	Test_code_Step_Motor_init();

	//

	// 5、Printf()
}
#endif

void arch_init(void)
{
	/* Configure the system clocks */
	RCC_Configuration();
	
	/* NVIC Configuration */
	NVIC_Configuration();

	GPIO_Configuration_default();

	//FSMC_Init();
	//GPIO_Configuration_8844RESET();
	
	//arch_8844_Reset();

	GPIO_Configuration_powerctr();

	arch_Power_Off();
	

	/* Configure the GPIO ports */
	GPIO_Configuration();

	#ifdef E480_BOARD_V10

	Yarn_use_Step =SWITCH_YARN_BOARD();
	Gpio_cfg_yarn_(Yarn_use_Step);
		
	#endif

	arch_LED_Off();

	FSMC_Init();

	

	Jtag_Security();
	
	Get_Cpld_Name_Ver();
	
	arch_mode = 0;//arch_GetKey() & 0x1;

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

	/* CAN Init */
	CAN_Configuration();

	/* USART Init */
	USART_Configuration();

	//TIM_Init();

	myprintf("\r\n =====================");
	myprintf("\r\n ==    Boot Start  %d ==",*(vu32*)(0x1fff7A20)  );
	myprintf("\r\n =====================");
	myprintf("\r\n\r\n");

	arch_StepMotor_Disable();
	//arch_StepMotor_Init();

	//arch_CPLD_Reset();
	
	arch_8803_Reset();
	arch_Jacquard_AllClear();
	arch_ACTEMF_AllClear();
	arch_YARNEMF_AllClear();

	FLASH_Unlock();
	EE_Init();
	last_app_ver = Exec_check_app_last_ver();
	//Check_APP_isOK = Exec_check_app_isok_from_flash();
	FLASH_Lock();
	arch_LED_On();
}


unsigned int arch_Set_CANID(unsigned int isbroadcastID)
{
	if (isbroadcastID)
	{
		return 0x361;				//测试用的ID
	}
	else
		return 0x362+arch_board_id+1;
}


void EXTI_CHECK_iscomming()
{
	return;
}


void hook_EXTI_isr(){}
void StepMotor_Isr(unsigned int stepno)
{
return;
}

void StepMotor_Isr_call(unsigned int stepno)
{
return;
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
	if (1)
	{
		MOTO9_ENABLE_OUTPUT(1);
		MOTO10_ENABLE_OUTPUT(1);
		MOTO11_ENABLE_OUTPUT(1);
		MOTO12_ENABLE_OUTPUT(1);
	}
	if (1)
	{
		MOTO13_ENABLE_OUTPUT(1);
		MOTO14_ENABLE_OUTPUT(1);
	}
	
}

//void alert_cpu_fatal(){}
#ifdef BOOT_CODE_JOIN_TEST_CODE


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
		if (cpld_name==BOARD_NAME_E490)
		{
			MOTO9_12_ENABLE_OUTPUT_E490(0);
		}
		else
		MOTO9_12_ENABLE_OUTPUT(0);
	}
	#else
	MOTO1_ENABLE_OUTPUT(0);
	MOTO2_ENABLE_OUTPUT(0);
	MOTO3_ENABLE_OUTPUT(0);
	MOTO4_ENABLE_OUTPUT(0);
	MOTO5_ENABLE_OUTPUT(0);
	MOTO6_ENABLE_OUTPUT(0);
	#endif
}


void arch_StepMotor_Half(int stepno, int onoff)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);

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

		#endif
		default :
			break;
	}
}

void arch_StepMotor_Dir(int stepno, int onoff)
{
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
		#endif
		default :
			break;
	}
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
		#endif
		default :
			break;
	}
}


#define STEPMODE_FULL	0
#define STEPMODE_HALF	1
#define STEPMODE_QUARTER	2
#define STEPMODE_EIGHTH	3

void arch_StepMotor_Set_UMS(unsigned int mode,unsigned int mototype)  //这里需要根据mode 设置???
{

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



void arch_StepMotor_Init()
{

	int i;
	for(i = 0; i < StepMotor_Count_MAX; i ++) {
		arch_StepMotor_Half(i, 1);
		arch_StepMotor_Dir(i, 0);
		arch_StepMotor_Pulse(i, 0);
	}
	arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_DENSITY-1],MOTOR_TYPE_DENSITY);
	#ifdef E480_BOARD_V10
		arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_ACTION-1],MOTOR_TYPE_ACTION);
	#else	
		arch_StepMotor_Set_UMS(step_resolution[MOTOR_TYPE_SKINER-1],MOTOR_TYPE_SKINER);
	#endif
}


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


unsigned int arch_StepMotor_work(unsigned int stepno)
{
	return arch_StepMotor_Zero(stepno);
}


int is_ACTION_Step(int stepno)
{
	if (stepno>=StepMotor_Count_MAX) return 0;
		if (STEPMOTOR[stepno].moto_remap_config.moto_attr.bit.moto_type_config== MOTOR_TYPE_ACTION)
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


#ifdef ZERO2_SUPPORT
int is_zero2_support(unsigned int stepno)
{
	return STEPMOTOR[stepno].zero2_mode;
}
void StepMotor_Detect_Zero2(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int i;
	int steps;

	if(stepno >= StepMotor_Count_MAX) return;

	Step = &STEPMOTOR[stepno];

	
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;

	if (Step->moto_remap_config.moto_attr.bit.moto_type_config!=MOTOR_TYPE_ACTION) return;

	if(Step->position <= sinker_zero_area &&
	   !arch_StepMotor_Zero(stepno)) {
		goto exec_it;
	}

	//if(arch_StepMotor_Zero(stepno)) {
	//	StepMotor_Reset(stepno);
	//	return ;
	//}

exec_it:
	arch_StepMotor_Stop(stepno);

	// by xhl 2010/08/26
	Step->is_poweron =1;

	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	Step->speed = 0;
	Step->pos_2 = 0;
	// by xhl 2010/05/18
	Step->alarm_step = 0;
	// by xhl 2012/03/08
	//Step->speed_div = 0;

	steps = step_setup_zero2_detect_steps;
	Step->steps = steps;
	Step->acc_steps = (steps + 1) >> 1;

	arch_StepMotor_Half(stepno, 0);

	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
	Step->step_st.bit.dir = 1;
	//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
	//Step->step_st.bit.dir = 0;
	Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
	Step->state_par = DETECT_ZERO2;

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 200/*50*/;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	//myprintf("start dir %d, step %d\r\n", Step->step_st.bit.dir, Step->steps);
	arch_StepMotor_Start(stepno);
	//Step_Zero_Check_Delay[stepno] = 2/*4*/;
#ifdef ENCODER_SUPPORT
	encoder_init &= ~(0x1 << stepno);
	Step->step_delay = ENCODER_CHECK_DELAY_RST;
	Step->enc_err_count = 0;
#endif
	Step->step_wait_time = 0;
	Step->step_st.bit.zero2 = arch_StepMotor_work(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
	Step->step_st.bit.zero2_count = 0;
}
#endif

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



unsigned int Test_code_StepMotor_get_IDall_with_no(unsigned int step_no)
{
	STEP_TYPE *Step;
	
	
	if (step_no>=StepMotor_Count_MAX) return 0xff;

	Step = &STEPMOTOR[step_no];

	return Step->moto_remap_config.moto_remap_id_all;
}


int Test_code_StepMotor_calc_speed(int stepno)
{
	STEP_TYPE *Step;
	int acc;

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
		 
	switch(stepindex) {
	case 0:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare1(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
		break;
	case 1:
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
	#ifdef E480_BOARD_V10
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
	#endif
	default:
		break;
	}
}


void arch_StepMotor_Set_Speed(unsigned int stepno, unsigned int speed)
{

	step_speed[stepno] = speed;
}

void arch_StepMotor_Start(unsigned int stepno)
{
	u16 capture = 10;
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);


	switch(stepindex) {
	case 0:
		capture += TIM_GetCounter(TIM2);
		TIM_SetCompare1(TIM2, capture&0xffff);
		TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
		break;
	case 1:
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
	#ifdef E480_BOARD_V10
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
	#endif
	
	default:
		break;
	}
}

void arch_StepMotor_Stop(unsigned int stepno)
{
	unsigned int stepindex=0xffff;
	stepindex = arch_Check_isYarn_step_withstepindex(stepno);



	switch(stepindex) {
	case 0:
		TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
		break;
	case 1:
		TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
		break;
	case 2:
		TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);
		break;
	case 3:
		TIM_ITConfig(TIM2, TIM_IT_CC4, DISABLE);
		break;
	case 4:
		TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
		break;
	case 5:
		TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
		break;
	#ifdef E480_BOARD_V10
	case 6:
		TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
		break;
	case 7:
		TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
		break;
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
	#endif
	
	default:
		break;
	}
}


void Test_code_Stepmotor_remape_withmode(unsigned char sysmode)
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
		Step->step_reset_delay_time =10;
		Step->moto_remap_config.moto_zero_input_index=i;
		Step->moto_remap_config.moto_attr.bit.moto_work_input_index=0xff;
		Step->moto_remap_config.moto_attr.bit.is_fast_mode =0;
		Step->moto_remap_config.moto_attr.bit.is_verystep_enable= 0 ;//为1的话，就是可变电机		
		Step->moto_remap_config.moto_attr.bit.is_activestep_enable =1; //为1 表示使能
		Step->moto_zero_width = 20;
		Step->moto_zero_width_self= 66;
		Step->need_2_pos_after_reset = 0;
		Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_UNDEF;
		
		if (i<4)
		{
			Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_DENSITY;		//后面再设置默认的电机类型
			Step->moto_remap_config.moto_remap_id_self = i;
		}
		else
		#ifdef E480_BOARD_V10
			if (i<6)
			{
				if ((sysmode==0)||(sysmode==1))
				{
					Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_ACTION;//MOTOR_TYPE_SKINER;	
					Step->moto_remap_config.moto_remap_id_self = i-4;
					Step->moto_remap_config.moto_attr.bit.moto_work_input_index=i+8;		
				}
				else
					if ((sysmode==2)||(sysmode==3))
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;	
						Step->moto_remap_config.moto_remap_id_self = i-4;
					}
					
						
				
			}
			else
				if (i<8)
				{
					if ((sysmode==0)||(sysmode==3))
					{
						Step->moto_remap_config.moto_attr.bit.moto_type_config=MOTOR_TYPE_SKINER;//MOTOR_TYPE_ACTION;
						Step->moto_remap_config.moto_remap_id_self = i-((sysmode==3)?4:6);
					}
					
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
			
		Step->is_poweron =0;		//0表示还没动过
		Step->error_count_check = 0;	//timer 中检测是否到位的计数器
		Step->input_errorstep = 0;	//默认无误差;//还没有算出误差
		
	
		if(arch_StepMotor_Zero(Step->moto_remap_config.moto_zero_input_index)) {
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
			Step->dir_steps =0;
			if (!( i & 0x01 ))
			{
				Step->step_st.bit.dir_High = 0;
			}
			else
			{
				Step->step_st.bit.dir_High = 1;
			}
			
		}
		Step->step_max = MAX_STEPS/*1000*/;

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




void Test_code_StepMotor_Init()
{
	switch (cpld_name)
	{	
		case BOARD_NAME_E475:
			Head_Mode_ = 2;				
			break;
		case BOARD_NAME_E480:
			Head_Mode_ = 0;				
			break;
		case BOARD_NAME_E490:
			Head_Mode_ = 0;				
			break;	
		default:
			Head_Mode_ = 2;				
	}
	Test_code_Stepmotor_remape_withmode(Head_Mode_);
	step_resolution[MOTOR_TYPE_DENSITY-1]  = QUARTER;
	step_resolution[MOTOR_TYPE_SKINER-1]  = QUARTER;	
	step_resolution[MOTOR_TYPE_FEET-1]  = QUARTER;
	#ifdef E480_BOARD_V10
	step_resolution[MOTOR_TYPE_ACTION-1]  = QUARTER;
	step_resolution[MOTOR_TYPE_YARN-1]  = QUARTER;
	stepmotor_AccSteps[MOTOR_TYPE_ACTION-1] = ACC_STEPS_ACTION -1;
	stepmotor_AccSteps[MOTOR_TYPE_YARN-1] = ACC_STEPS_YARN -1;
	stepmotor_speed[MOTOR_TYPE_ACTION-1][0] = STEP_LOW_SPD;
	stepmotor_speed[MOTOR_TYPE_ACTION-1][1]= (ACT_MAX_SPEED << step_resolution[MOTOR_TYPE_ACTION-1]) >> QUARTER;
	stepmotor_speed[MOTOR_TYPE_YARN-1][0] = STEP_LOW_SPD;
	stepmotor_speed[MOTOR_TYPE_YARN-1][1]= (YARN_MAX_SPEED << step_resolution[MOTOR_TYPE_YARN-1]) >> QUARTER;
	step_alert_detect_setup[MOTOR_TYPE_ACTION-1] = 20;//10;
	step_alert_detect_setup[MOTOR_TYPE_YARN-1] = 20;//10;	
	#endif
	step_run_mode = 0;
	stepmotor_AccSteps[MOTOR_TYPE_DENSITY-1] = ACC_STEPS_DM -1;
	stepmotor_AccSteps[MOTOR_TYPE_SKINER-1] = ACC_STEPS_SINKER -1;	
	stepmotor_AccSteps[MOTOR_TYPE_FEET-1] = ACC_STEPS_FEET -1;		
	stepmotor_speed[MOTOR_TYPE_DENSITY-1][0] = STEP_LOW_SPD;
	stepmotor_speed[MOTOR_TYPE_DENSITY-1][1]  = (STEP_MAX_SPD << step_resolution[MOTOR_TYPE_DENSITY-1]) >> QUARTER;
	#ifdef STEP_MOTOR_DDDM_SUPPORT
	step_base_speed[MOTOR_TYPE_DENSITY-1] 	= STEP_START_SPEED_HZ_DM;
	step_base_speed[MOTOR_TYPE_SKINER-1] 	= STEP_START_SPEED_HZ_SK;
	step_base_speed[MOTOR_TYPE_ACTION-1] 	= STEP_START_SPEED_HZ_ACT;
	step_base_speed[MOTOR_TYPE_FEET-1] 	= STEP_START_SPEED_HZ_FEET;
	step_base_speed[MOTOR_TYPE_YARN-1] 	= STEP_START_SPEED_HZ_YARN;
	#else
	step_base_speed = 0/*STEP_LOW_SPD*/;
	#endif
	stepmotor_speed[MOTOR_TYPE_SKINER-1][0] = STEP_LOW_SPD;
	stepmotor_speed[MOTOR_TYPE_SKINER-1][1]= (SK_MAX_SPEED << step_resolution[MOTOR_TYPE_SKINER-1]) >> QUARTER;
	sinker_add_speed= 0;   //hlc 2014-07-25
	step_zero_adj = 4;//;
	step_alert_detect_setup[MOTOR_TYPE_DENSITY-1] = 20;//10;
	step_alert_detect_setup[MOTOR_TYPE_SKINER-1] = 20;//10;	
	step_alert_detect_setup[MOTOR_TYPE_FEET-1] = 20;//10;
	sinker_zero_area = 60;    //10/*0*/;0;
#ifdef ZERO2_SUPPORT
	step_setup_zero2_detect_steps = 20;
#endif
	step_interval = 0;
	step_work_sign_alarmenable =0;	
}



void Test_code_StepMotor_exec(unsigned int stepno, short pos, int mode)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	int zero_work_area;
	int zero;
	int work;
	int ZeroPos_WorkST;


	if(stepno >= StepMotor_Count_MAX) return;


	Step = &STEPMOTOR[stepno];

	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config>MOTOR_TYPE_COUNT) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config==MOTOR_TYPE_UNDEF) return;
	
	zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_zero_input_index);
	work = arch_StepMotor_work(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
	ZeroPos_WorkST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno) && is_ACTION_Step(stepno) && zero)
	{
		
		if(work != ZeroPos_WorkST) {
			zero = 0;
		}
	}
#endif

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
		if((!Step->is_poweron) 
			|| (zero && Step->position != 0) 
			|| (!zero && Step->position == 0)) {
			Test_code_StepMotor_Reset(stepno);
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

	Step->pos_2 = pos;			//记录到达的目的地
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	//Step->speed = step_base_speed;
	Step->state_par = 0;				//just run
	// by xhl 2010/05/18
	Step->alarm_step = 0;   
	Step->step_st.bit.IS_Reset_ex =0;

	zero_work_area = step_zero_adj + Step->input_errorstep;

	zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);


	if(Step->position == pos) {

		if(!zero && (Step->position < step_zero_adj)) {

			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1001);
			
		}
		if(zero && (Step->position > zero_work_area)) {
			
			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 0x1002);
			
		}
		return ;
	}


	



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
		//fast_mode = 0;				//这里有疑问
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

		//Message_Send_4halfword_debug(0x0909,stepno,0xEEEE,0xffff);
		
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
		//Message_Send_4halfword_debug(0x0909,stepno,0xDDDD,0xffff);
	}

speed_set:
	Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];

#ifdef STEP_MOTOR_DDDM_SUPPORT
	
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];//2000;
	Step->speed = Step->low_speed;

#else
	Step->speed =step_base_speed;
#endif




	// Half On
	arch_StepMotor_Half(stepno, 0);

	if(Step->position > pos) {
		arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		Step->steps = Step->position - pos;
		Step->step_st.bit.dir = 0;
	}
	else {
		arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
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
			arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
		}
		else {
			Step->position = pos + Step->steps;
			arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
		}
	}
#endif

		if(Step->steps >= ((stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1]+1) << 1)) {
			Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
		}
		else {
			Step->acc_steps = Step->steps >> 1;
		}
#ifdef STEP_MOTOR_DDDM_SUPPORT
	Step->speed_acc_dec =(Step->max_speed > Step->low_speed)?((Step->max_speed -Step->low_speed)  /Step->acc_steps):0 ;
#endif

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	arch_StepMotor_Start(stepno);


	if(mode & 0x400) {
		Step->state_par = 0x80;
		Step->pos_2 = 0;

		Step->steps_go_temp = 0;

	}

#ifdef ENCODER_SUPPORT
	Step->step_delay = ENCODER_CHECK_DELAY_RUN;
	Step->enc_err_count = 0;

	if((encoder_status & (0x1 << stepno)) == 0) {
		coder_err = 0xe1 + stepno;
		//myprintf("coder_err 0x%x\r\n", coder_err);
	}
#endif
	//Step_Zero_Check_Delay[stepno] = 2/*4*/;
#ifdef STEP_TEST_IN_NDL
	Step->needle = Needle_pos_Get();
#endif

	Step->step_wait_time = 0;

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




void Test_code_StepMotor_Reset(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int rst_speed;
	unsigned int iszero;
	unsigned int iswork;
	

	if(stepno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	if (!Step->moto_remap_config.moto_attr.bit.is_activestep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.is_verystep_enable) return;
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config > MOTOR_TYPE_COUNT) return;

	arch_StepMotor_Stop(stepno);

	Step->is_poweron =1;		//变成1 说明电机动过了.
	Step->step_st.bit.IS_Reset_ex =1;
	Step->step_st.bit.level = 0;
	Step->step_st.bit.phase = 0;
	Step->speed = 0;
	Step->pos_2 = 0;
	Step->alarm_step = 0;
	Step->steps = Step->step_max;	
	Step->state_par = STEP_RESET;

	Step->acc_steps = stepmotor_AccSteps[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];  // ACC_STEPS - 1;
	rst_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
	#ifdef STEP_MOTOR_DDDM_SUPPORT
	if (rst_speed>stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1])
	#else
	if (rst_speed<stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1])
	#endif	
		rst_speed =stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][1];	
	Step->max_speed = rst_speed;
	
	arch_StepMotor_Half(stepno, 0);//全流
	
	iszero = arch_StepMotor_Zero(Step->moto_remap_config.moto_zero_input_index);
	iswork = arch_StepMotor_Zero(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
	if(iszero) {		
		Step->step_st.bit.dir = 1;
		Step->position = 0;	
		if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
		{
			Step->state_chi =5;
		}
		else
			Step->state_chi =2;
	}
	else {
		Step->step_st.bit.dir = 0;			
		Step->position = Step->step_max;
		Step->state_chi =0;
	}
	arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);

#ifdef STEP_MOTOR_DDDM_SUPPORT
	Step->low_speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];  //启动频率
	Step->speed_acc_dec =(Step->max_speed>Step->low_speed)?((Step->max_speed -Step->low_speed)  /Step->acc_steps):0 ;		
	Step->speed = (Step->max_speed>Step->low_speed) ? Step->low_speed :Step->max_speed;												//当前速度
#endif

	arch_StepMotor_Stop(stepno);
	Step->step_check_interval = 50;
	Step->step_st.bit.running = 1;
	Step->step_st.bit.check_delay_count = 0;
	arch_StepMotor_Start(stepno);

	Step->step_wait_time = 0;
	//Step->done_steps = 0;				//已经执行的步数 针对 三角电机
	//Step->last_rest= 0;	
	Step->change_dir_count = 0;
	Step->dir_steps =0;

#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno)) {
		if(is_ACTION_Step(stepno)) {
			Step->step_st.bit.zero2 = iswork;
		}
		if(Step->steps > step_setup_zero2_detect_steps)
		Step->step_st.bit.zero2_count = 0;
	}
#endif
}




#ifdef STEP_MOTOR_DDDM_SUPPORT

#define DRV8711		1

#endif

void Test_code_StepMotor_Isr(unsigned int stepno)
{
	STEP_TYPE *Step/* = &STEPMOTOR[stepno]*/;
	unsigned int speed;
	unsigned int step_alert_detect_setup_local;	
	int zero = 0;
	int work = 0;
	int ZeroPos_WorkST;
	int errno = 0;
	int step_alert_detect;
	int zero_work_area;

	if(stepno >= StepMotor_Count_MAX) return;
	Step = &STEPMOTOR[stepno];
	if (Step->moto_remap_config.moto_attr.bit.moto_type_config)
	{
		step_alert_detect_setup_local = step_alert_detect_setup[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
	}
	else
	{
		step_alert_detect_setup_local = 0;
	}


	if(Step->step_st.bit.running == 0 || Step->steps == 0) {          //说明走完了，该停下来了。
		Step->step_st.bit.running = 0;
		arch_StepMotor_Half(stepno ,1);//半流
		arch_StepMotor_Stop(stepno);
		goto exit;
	}
	//Step->step_check_interval = 50;
	if(Step->step_wait_time) {
		#ifdef STEP_MOTOR_DDDM_SUPPORT
		arch_StepMotor_Set_Speed(stepno, 1000/(Step->step_wait_time*2));
		#else
		arch_StepMotor_Set_Speed(stepno, PULSE_LOW);

		#endif
		goto done;
	}

	if(Step->step_st.bit.level == 0) {
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


	zero = arch_StepMotor_Zero(Step->moto_remap_config.moto_zero_input_index);
	work = arch_StepMotor_work(Step->moto_remap_config.moto_attr.bit.moto_work_input_index);
	ZeroPos_WorkST = Step->moto_remap_config.moto_attr.bit.zeroPos_Work_ST;
	
#ifdef ZERO2_SUPPORT
	if(is_zero2_support(stepno) && is_ACTION_Step(stepno) && zero )
	{
		if ((work!=ZeroPos_WorkST)&&(!Step->step_st.bit.IS_Reset_ex) )
		{
			zero = 0;
		}
	}
	if(is_ACTION_Step(stepno)) 
	{
		int zero2 = work;
		if(Step->step_st.bit.zero2 != zero2) 
		{
			Step->step_st.bit.zero2 = zero2;		
			Step->step_st.bit.zero2_count ++;		
		}
	}

	if(Step->state_par == DETECT_ZERO2) {
		goto pos_end;
	}
#endif


	if(Step->step_st.bit.dir) {
		Step->position ++;
	}
	else {
		Step->position --;
	}

	
#ifdef ZERO2_SUPPORT
pos_end:
#endif


	Step->steps --;


	switch(Step->state_par) {
		case JUST_RUN:
			{

			#ifdef ZERO2_SUPPORT
			if (is_zero2_support(stepno) && is_ACTION_Step(stepno) && (Step->steps == step_setup_zero2_detect_steps))
			{				
				Step->state_par = GOTO_ZERO2;
				Step->steps *= 2;			
			}
			#endif

				
			if(Step->step_st.bit.running == RUNNING_OVER) 
			{
				Step->steps = 0;
				goto exit;
			}
		
			}
			 break;	
			 
		case STEP_RESET:
			{
				switch (Step->state_chi)
				{
					case 0:		//第一次进0位
						{
							if (zero)   //找到0位了，那就继续走几步
							{
								int steps ;
								if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
								{
									steps = 400;																	
									Step->state_chi = 2; 
									Step->moto_zero_width_self = 0;
								}
								else
								{
									steps = step_zero_adj;
									Step->state_chi = 1; 
								}
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = steps/*0*/;
								
							}
							else
							{
								if (Step->steps==0) //走完了，还没找到0位，那么要报警了
								{
									errno = 1;
									goto alarm_and_exit;
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

								Step->steps = Step->step_max;//200;
								Step->position = 0;
								Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
								#ifdef STEP_MOTOR_DDDM_SUPPORT
								Step->speed = step_base_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1];
								#else
								Step->speed = step_base_speed;
								#endif
								Step->step_st.bit.phase = 0;
								Step->step_st.bit.dir = !Step->step_st.bit.dir ;
								Step->step_wait_time = Step->step_reset_delay_time;
								arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 2; 
								
							}
							if (!zero)  //0位信号突然没了。那要报警了
							{
								errno = 2;	
								goto alarm_and_exit;
							}
						}
						break;
					case 2:
						{
							if (!zero)   //说明离开了。那么记录当前的点位
							{
								int steps = STEPS_LEAVEZERO;						
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								Step->steps = steps/*0*/;
								Step->position = 0/*0*/;								
								Step->state_chi = 3; 
							}
							if (Step->steps==0)   //走完了，还没有离开0位，那有问题了
							{
								errno = 3;
								goto alarm_and_exit;
							}
							if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
							{
								Step->moto_zero_width_self++;
							}

						}
					
						break;

					case 3:
						{
							if (Step->steps==0)  //走完了，那么可以调头了
							{
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
								Step->step_st.bit.dir = !Step->step_st.bit.dir ;
								Step->step_wait_time = Step->step_reset_delay_time;
								arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 4; 
								
							}
							if (zero)  //0位信号突然又有了。那要报警了
							{
								errno = 4;
								goto alarm_and_exit;
							}
						}
						break;
					case 4:
						{
							if (zero)		//关键的到了//
							{	
								int steps = step_zero_adj;				
								steps = stepspeed_Q2R(step_zero_adj,Step->moto_remap_config.moto_attr.bit.moto_type_config);

								Step->input_errorstep = (Step->step_max-Step->steps) - STEPS_LEAVEZERO;//可正可负
								Step->position = steps/*0*/;

								if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
								{
									//steps =Step->moto_zero_width;
									steps = Step->moto_zero_width_self - Step->input_errorstep -Step->moto_zero_width;		//回到中心位置
									Step->position = 0-steps/*0*/;
								}
								
								Step->steps = steps/*0*/;
								//Step->position = steps/*0*/;
									//走回来的步数			- 走出去的步数		

								Step->state_par = JUST_RUN;

							}
							else
								{
									if (Step->steps==0)	 //走完了，还没找到0位，那么要报警了
									{
										errno = 5;
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
								errno = 3;
								goto alarm_and_exit;
							}							

						}
						break;
					case 6:
						{
							if (Step->steps==0)  //走完了，那么可以调头了
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
								Step->step_st.bit.dir = !Step->step_st.bit.dir ;
								Step->step_wait_time = Step->step_reset_delay_time;
								arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High :!Step->step_st.bit.dir_High);
								Step->state_chi = 0; 
								
							}
							if (zero)  //0位信号突然又有了。那要报警了
							{
								errno = 4;
								goto alarm_and_exit;
							}							

						}
						break;	
						
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
					int steps = step_zero_adj;
				
					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					Step->steps = steps/*0*/;
					Step->position = steps/*0*/;
					Step->state_par = JUST_RUN;
					if ((step_alert_detect_setup_local)&&(Step->alarm_step))
					{
						if (Step->alarm_step-1+step_zero_adj >step_alert_detect_setup_local)
						{
						errno = 7	;//上一次多走了step_alert_detect_setup这么些
						//可以自动纠正
						}
					}

					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno) && is_ACTION_Step(stepno) && (Step->step_st.bit.zero2_count==0) )
					{
						
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 3);  //doing...
						
					}
					#endif
					
		
				}
				else
				{
					if (Step->steps == 0)  //走完了都还没找到0位那说明0位传感器异常
					{
						errno = 8;			//和复位过程中的错误1 一样，//无法自动纠正
						goto alarm_and_exit;
					}
					if(Step->alarm_step) 
					{
						Step->alarm_step ++;				
					}
					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno)&&is_ACTION_Step(stepno)&&(Step->alarm_step == step_alert_detect_setup_local) )
					{
						
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 7);//doing...
						
					}
					#endif
					
				}			
			}
			break;
		case MOVETO_ZERO:
			if(zero) {
					int steps;
					
					if (step_alert_detect_setup_local)
					{
						if (Step->steps>=step_zero_adj)
						{
							if (Step->steps - step_zero_adj >step_alert_detect_setup_local)
							{
								errno = 6	;//上一次失步了step_alert_detect_setup这么些
							}
						}
						else
						{
							if (step_zero_adj - Step->steps  >step_alert_detect_setup_local)
							{
								errno = 7	;//上一次多走了step_alert_detect_setup这么些
							}
						}
							
					}


					if (Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)
					{
						if (!Step->step_st.bit.dir)
						{
							steps = Step->moto_zero_width;
							steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
							Step->position = steps/*0*/;
							Step->steps =steps;
							
						}
						else
						{
						
							steps = Step->moto_zero_width_self - Step->input_errorstep -Step->moto_zero_width;
							if (steps<0)
							{
								steps *= -1;			
								steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
								steps *= -1;
							}
							Step->position = 0-steps/*0*/;
							Step->steps =steps;
							
						}
						//Step->steps = Step->position;  //steps/*0*/;
					}
					else
						{
							steps = step_zero_adj;
							steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);
							Step->position = steps/*0*/;
							Step->steps = steps/*0*/;
						}
					//Step->steps = steps/*0*/;
					
					Step->state_par = JUST_RUN;
					#ifdef ZERO2_SUPPORT
					if (is_zero2_support(stepno) && is_ACTION_Step(stepno) && (Step->step_st.bit.zero2_count==0) )
					{
						alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config), 3 + 10);
					
					}
					#endif
	

					
				
			}
			else {
				if(Step->steps == 0/*ACC_STEP*/) {//说明上一次走多了，这次提前结束了。至于是否要报警，那还要看下面到底还有多少步到达0位
					int steps = 200;

					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					Step->steps = steps;//200;//Step->step_max;
					Step->position = steps;//200;//Step->step_max;

					//if(step_alert_detect)
					Step->alarm_step = 1/*step_alert_detect*//*ZERO_DETECT*//*5*/;
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
					Step->state_par = GOTO_ZERO;
					if(Step->step_st.bit.phase == 2) {
						Step->step_st.bit.phase = 0;
					}
				}
			}
			break;
		case LEAVE_ZERO:			
			if(!zero) {
				int steps =STEPS_LEAVEZERO;	
			

				if(Step->alarm_step) //那说明没离开0位的时候就已经走完步数了
				{
					if (Step->alarm_step>(step_zero_adj+Step->input_errorstep)) //上次走多了
					{
						if (Step->alarm_step -(step_zero_adj+Step->input_errorstep) >step_alert_detect_setup_local)
						{
							errno =10;//上一次走到负方向的时候走多了，可纠正
						}
						
					}
					else				// 上一次失步了
					{
						if ((step_zero_adj+Step->input_errorstep)-Step->alarm_step >step_alert_detect_setup_local)
						{
							errno =11;//上一次走到负方向的时候走少了，可纠正
						}
					}

				}
				else		//离开了0位还没走完
				{
					if (Step->steps +(step_zero_adj+Step->input_errorstep) >step_alert_detect_setup_local)
					{
						errno =12;//上一次走到负方向的时候走少了，可纠正，少了很多
					}

				}


				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

				Step->steps = steps;//50/*100*/;
				Step->position = 0;
				Step->state_par = LEAVE_STEP;

		
			}
			else
			{
				if (Step->steps==0) // 说明走完了还没离开0位 那么上一次有可能走多了，要不要报警，要看下面的
				{
					int steps = 200;

					steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

					Step->steps = steps;//200;//Step->step_max;
					Step->position = steps;//200;//Step->step_max;					
					Step->alarm_step = 1;
					Step->max_speed = stepmotor_speed[Step->moto_remap_config.moto_attr.bit.moto_type_config-1][0];
				}
				if(Step->alarm_step) 
				{
					Step->alarm_step ++;
					if (Step->steps==0)   //又走完了，还没离开0位 ,那对不起，要报警了 不可纠正
					{
						errno= 9;
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
				arch_StepMotor_Dir(stepno, Step->step_st.bit.dir ? Step->step_st.bit.dir_High : !Step->step_st.bit.dir_High);
				Step->state_par = GOTO_ZERO;
			}
			break;
		case FEET_STEP_ISWORK:
		{
			
			if(work) {

				if(Step->steps_go_temp++ > 10) 

				{

					Step->state_par = 0;
					//myprintf("Step isr %d, %d, %d\r\n", Step->steps, Step->position, Step->pos_2);
					if(Step->pos_2 >= 0)
						Step->steps = Step->pos_2;	// ????
					else if(Step->pos_2 < 0) {
						//arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
						Step->steps = -Step->pos_2;	// ????
					}
					Step->pos_2 += Step->position;
					Step->steps = step_zero_adj;
				}
			}
			
		}
			break;
	#ifdef ZERO2_SUPPORT
		case DETECT_ZERO2:
			Step->step_wait_time = 1;
			if(!work && !Step->step_st.bit.dir) {
				//Step->steps = 0;
				//break;
				int steps = 4/*step_zero_adj*/;
				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

				// by xhl 2013/03/21
				if(zero) {
					steps = 0;
				}
				Step->steps = steps/*0*/;
				Step->step_st.bit.dir = 2;
			}
			if((Step->steps == 0) && work && (Step->step_st.bit.dir != 2)) {
				if(!Step->step_st.bit.dir) break;
				//arch_StepMotor_Dir(stepno, Step->step_st.bit.dir_High);
				//Step->step_st.bit.dir = 1;
				arch_StepMotor_Dir(stepno, !Step->step_st.bit.dir_High);
				Step->step_st.bit.dir = 0;
				Step->steps = step_setup_zero2_detect_steps * 2;
				Step->step_wait_time = 10;
			}
			if((Step->steps == 0) && (Step->step_st.bit.dir == 2)) {
				Step->steps = 0;
				break;
			}
			break;
		case GOTO_ZERO2:
			if(zero) {
				Step->position = 0;
				Step->state_par = 0;
				Step->steps = 0;
				Step->pos_2 = 0;
				break;
			}
			//if(!arch_StepMotor_Zero(stepno + 2)) { }
			if(!work && (Step->position > step_setup_zero2_detect_steps * 2)) 
			{
				int steps = 4/*step_zero_adj*/;
				steps = stepspeed_Q2R(steps,Step->moto_remap_config.moto_attr.bit.moto_type_config);

				Step->steps = steps/*0*/;
				Step->state_par = 0;
				if(Step->step_st.bit.dir) {
					Step->pos_2 = Step->position + steps;
				}
				else {
					Step->pos_2 = Step->position - steps;
				}
			}
			break;
	#endif
		default:
			break;
		}

		if (errno)
		{
			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno);
			errno =0;
			
		}

		if(Step->steps > 0) {

#ifdef STEP_MOTOR_DDDM_SUPPORT

		speed = Test_code_StepMotor_calc_speed(stepno);		

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

	}
	else {
		Step->step_st.bit.running = 0;
		arch_StepMotor_Stop(stepno);
		arch_StepMotor_Half(stepno, 1);

		#ifdef STEP_MOTOR_DDDM_SUPPORT
		
		if(Step->step_st.bit.step_flags) 
		{			
			void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
			#ifdef STEP_DEBUG_DDM
			Message_Send_4halfword((0x08 << 8) | 0x03,Test_code_StepMotor_get_IDall_with_no(stepno), Step->position,Step->step_last_postion);
			Step->step_last_postion =0;
			#else
				Message_Send_4halfword((0x08 << 8) | 0x03,Test_code_StepMotor_get_IDall_with_no(stepno),Step->position,0);
			#endif			
		}
		Step->step_is_cnt =0;
		Step->step_is_cnt_old =0;
		#endif


		if (((step_work_sign_alarmenable>>Step->moto_remap_config.moto_remap_id_all) & 0x01)){//说明需要判断传感器2是否有效
			if (!work)   {//说明没有检测到那就报警
				
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),0x9222);
			
			}			
			step_work_sign_alarmenable  &=  ~(0x01<<Step->moto_remap_config.moto_remap_id_all) ;//清除检测标志
		}


		zero_work_area = step_zero_adj + Step->input_errorstep;
		

		zero_work_area = stepspeed_Q2R(zero_work_area,Step->moto_remap_config.moto_attr.bit.moto_type_config);



#ifdef NEW_ZERO_DETECT_MODE
		if ((zero && (Step->position > zero_work_area /*step_zero_adj + ZERO_DETECT*//*5*/)) ||
		    (!zero && (Step->position <= 0))) {
			Step->step_st.bit.check_delay_count = 20/*10*/;
			Step->step_st.bit.check_delay_count += 15;
			errno = 0;
			// by xhl 2012/07/12
			Step->pos_2 = Step->position;
			Step->alarm_step = 0;
		}
#endif

#ifdef ZERO2_SUPPORT
		if (is_zero2_support(stepno) && is_ACTION_Step(stepno)  && !zero )
		{
			Step->step_st.bit.check_delay_count = 20;
			Step->step_st.bit.check_delay_count += 15;
			if (Step->step_st.bit.zero2_count == 0)
			{
				errno = 3 + 20;
			}
		}	
		
#endif


		if(errno) {
			
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno);
			
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
	
	if(Step->steps == 0) {
		if(Step->state_par == FEET_STEP_ISWORK) {
			if(!work) {
				
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),3);
				
			}
			return ;
		}
		
		if(Step->state_par) {
			if(Step->state_par == LEAVE_ZERO ||
			   Step->state_par == GOTO_ZERO) {
			
				alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),0x8001);
				
				Step->position = 0;
				Step->pos_2 = 0;
			}
			return ;
		}
		if(Step->pos_2 != Step->position) {
			Test_code_StepMotor_exec(stepno, Step->pos_2, 0);
				{
					unsigned int add_speed_temp;	
					if (is_sinker(stepno))
						add_speed_temp = sinker_add_speed;
					else
						add_speed_temp =0;
	
				arch_StepMotor_Active(stepno,add_speed_temp);
				}
			// by xhl 2012/07/12
			Step->step_wait_time = 10;
		}

		if ((Step->step_st.bit.IS_Reset_ex)&&(Step->moto_remap_config.moto_attr.bit.moto_type_config == MOTOR_TYPE_YARN)) 
		{
			if (Step->need_2_pos_after_reset !=0)
			{
				extern void arch_StepMotor_Active(unsigned int stepno,unsigned int  add_speed);
				
				Test_code_StepMotor_exec(stepno, Step->need_2_pos_after_reset, 1);
				
				arch_StepMotor_Active(stepno,0);
			
				Step->step_wait_time = 10;
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
		arch_StepMotor_Half(stepno, 1);

		if (errno)
			alert_push(STEP_ERR_CODE(Step->moto_remap_config.moto_remap_id_self,Step->moto_remap_config.moto_attr.bit.moto_type_config),errno);
		return ;
	
}


#endif
void shell_rcv_char(){}

#define MAIN_BOARD_ID_FLASH_ADDR		90
#define MAIN_BINDING_FLAG_ENABLE		0xBABA			/*绑定*/
#define MAIN_BINDING_FLAG_DISABLE		0xABAB			/*解除*/
#define HEAD_APP_FLAG					0x0D3A			/*写入E方的特定值，表示具备某一种功能*/




int Exec_check_app_isok_from_flash()
{
	unsigned short bf=0;
	unsigned short bf1=0;
	int fg;
	
	int ret;

	
	ret = EE_Read(MAIN_BOARD_ID_FLASH_ADDR+6,&bf,1);
	if (ret)
	{
		fg = 0;
	}
	else
		fg =(bf==MAIN_BINDING_FLAG_ENABLE)?1:0;

	if (fg)
	{
		ret = EE_Read(MAIN_BOARD_ID_FLASH_ADDR+7,&bf1,1);
		if (ret)
		{
			/*没读到这个值，那要写一个进去*/
			goto APP_flag_is_err;
		}
		else
		{
			if (bf1==HEAD_APP_FLAG)
			{
				return 1;		
			}
			else
				goto APP_flag_is_err;
		}
	}
	else
		return 0;
	if (0)
	{
APP_flag_is_err:
		return -1;		
	}
		
	
}



unsigned int Exec_check_app_last_ver()
{
	unsigned short bf=0;
	unsigned short bf1=0;
	//int fg;
	
	int ret;

	
	ret = EE_Read(MAIN_BOARD_ID_FLASH_ADDR+8,&bf,1);
	if (ret)
	{
		bf = 0;
	}
	return bf;
}



