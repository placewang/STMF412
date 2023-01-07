
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "config.h"
#include "Platform_config.h"

void hook_time_2ms(void);
void hook_time_250us(void);
void StepMotor_Isr(int);
void StepMotor_Isr_call(unsigned int );
extern unsigned int arch_need_current_add(unsigned int currentdata);
extern void arch_JQD_Isr_call(unsigned int whichjqd,unsigned char clear);
					
void TimeStart(void);
unsigned long TimeGet(void);

#ifdef E480_BOARD_V10
int get_CCR_Val_with_PWM_A(int Step_A);
#endif

unsigned long SYSHCLK_AHB=120000000;


#ifdef ECODE_USE_MT6813_PWM
#define TIME_ISR_CNT_IS_GETCODE_OK		2

unsigned int Time3_isr_cnt=0;
unsigned int Time3_isr_cnt_ed=0;
unsigned char Sys_is_MT6813=0;

unsigned int IC1Value;
unsigned int DutyCycle;
unsigned int Frequency;
unsigned int T3_Pwm_timeout;
unsigned char GetEcode_Start =0;

#define T3_PWM_TIME_OUT_DEF_MS	20

#endif

void Time_start()
{
	//unsigned long capture;

	//capture = TIM_GetCounter(TIM5) + 100;
	//TIM_SetCompare1(TIM5, capture);
	//TIM_SetCompare2(TIM5, capture);
	#ifdef JQD_ISR_TIMER_CNT_50US
	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);// TIM_IT_CC1|TIM_IT_CC2
	#else
	//TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);// TIM_IT_CC1|TIM_IT_CC2
	#endif
#if 0
	capture = TIM_GetCounter(TIM1) + 100;
	TIM_SetCompare2(TIM1, capture);
	//TIM_SetCompare2(TIM5, capture);
	TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
	#endif
	
}


void Time_stop_time5()
{
	TIM_ITConfig(TIM10, TIM_IT_Update , DISABLE);
	//TIM_ITConfig(TIM1, TIM_IT_CC2 , DISABLE);
}

void Time_start_time7()
{
	TIM_ITConfig(TIM7, TIM_IT_Update , ENABLE);

}
void Time_stop_time7()
{
	TIM_ITConfig(TIM7, TIM_IT_Update , DISABLE);
}

void TIM_Config7(TIM_TypeDef* TIMx)
{
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 50;

	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}

void TIM3_PWM_input_Config()
{
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure ;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;

	TIM_TimeBaseStructure.TIM_Prescaler = 5;//59/*=1M , apb1=30M timerclk=apb1*2 */

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);                          

	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);	

	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1); 

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); 

	TIM_ClearFlag(TIM3, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);

	Ecode_cs_select_all();/*片选有效，判断数据是否进来*/
}



void TIM_Config(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;

	if(TIMx ==TIM6)
	TIM_TimeBaseStructure.TIM_Prescaler = 5999;
	else
	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */

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


void TIM_Config5(TIM_TypeDef* TIMx)
{
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1999;

	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);	

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}

void TIM_Config10(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	#ifdef JQD_ISR_TIMER_CNT_50US
	TIM_TimeBaseStructure.TIM_Period = 49;
	#else
	TIM_TimeBaseStructure.TIM_Period = 65535;
	#endif

	TIM_TimeBaseStructure.TIM_Prescaler = 119;//59/*=1M , apb1=30M timerclk=apb1*2 */

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);	

	TIM_OCInitStructure.TIM_Pulse = 10;

	TIM_OC1Init(TIMx, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);
	

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}

void TIM_Config12(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */   1m,10khz--100US=0.1MS

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);	

	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 10;

	TIM_OC1Init(TIMx, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}


void TIM_Config9_ex(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 119;//59/*=1M , apb1=30M timerclk=apb1*2 */   1m,10khz--100US=0.1MS

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);	

	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 10;

	TIM_OC1Init(TIMx, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}


void TIM_Config9(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 11119;//59/*=1M , apb1=30M timerclk=apb1*2 */   1m,10khz--100US=0.1MS

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);	

	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 10;

	TIM_OC1Init(TIMx, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);

	TIM_ClearFlag(TIMx, TIM_FLAG_Update); 

	/* TIM enable counter */
	TIM_Cmd(TIMx, ENABLE);
}

void TIM_Config6(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb2=60M 
	
	
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



void TIM_Config8(TIM_TypeDef* TIMx)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 119;//59/*=1M , apb2=60M 
	
	
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




void TIM_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable TIM clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE); 
	

	TIM_Config(TIM2);
	#ifdef ECODE_USE_MT6813_PWM
	TIM3_PWM_input_Config();
	#else
	TIM_Config(TIM3);
	#endif
	TIM_Config(TIM6);
	//TIM_Config(TIM4);
	//TIM_Config5(TIM5);
	//#ifdef JQD_ISR_TIMER_ONE
	//TIM_Config6(TIM6);
	//#else
	
	//#endif
	//TIM_Config(TIM1);
	TIM_Config7(TIM7);
	TIM_Config8(TIM8);
	TIM_Config8(TIM1);
	TIM_Config10(TIM10);
	#ifdef ECODE_USE_MT6813_PWM
	TIM_Config9_ex(TIM9);
	TIM_Config12(TIM12);
	#else
	TIM_Config9(TIM9);
	#endif
	#ifdef JQD_ISR_TIMER_ONE
	//TIM_Config9(TIM12);	
	#endif


	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	#ifdef ECODE_USE_MT6813_PWM
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //脟脌脮录脫脜脧脠录露1 

	#else
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   //脟脌脮录脫脜脧脠录露1 

	#endif
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //脭脢脨铆脰脨露脧 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   
	
	

	

	/* TIM */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1+1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //脭脢脨铆脰脨露脧 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

#ifndef ECODE_USE_MT6813_PWM
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

#endif	
#ifdef ECODE_USE_MT6813_PWM
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

#endif

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   


	//NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	//NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   
/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   
*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1+1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);    

#if 0
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);     
	
#endif

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1+1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);   
	//TIM_ITConfig(TIM5, TIM_IT_CC1 | TIM_IT_CC2/* | TIM_IT_CC3 | TIM_IT_CC4*/, ENABLE);

#ifdef JQD_ISR_TIMER_ONE
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM_ITConfig(TIM5, TIM_IT_CC1 | TIM_IT_CC2/* | TIM_IT_CC3 | TIM_IT_CC4*/, ENABLE);
#endif

	//BSP_CLK_SysTick_Config(120000);
	
	SysTick_Config(SYSHCLK_AHB  /4000);//  250us
	
	Time_start();
}



void hook_TIM2_isr(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		StepMotor_Isr_call(1);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

		StepMotor_Isr_call(0);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

		StepMotor_Isr_call(2);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

		StepMotor_Isr_call(3);
	}
}

#ifdef DEBUG_MT6813_PWM
unsigned int Debug_led_cnt=0;
unsigned char Debug_led_alarmit=0;


void Debug_led_cnt_isr()
{
	if(Debug_led_cnt)
		Debug_led_cnt--;
}

int Check_Debug_Led_next_Off()
{
	if(Debug_led_alarmit)
	{
		if(Debug_led_cnt==0)
		{
			Debug_led_alarmit =0;
			return 1;
			
		}
		else
			return 0;
	}
	else
		return 0;
}

#endif

void hook_TIM3_isr(void)
{
	static unsigned int DutyCycle_last;
	static unsigned int Frequency_last;
	unsigned int last_new_code=0;
#ifndef ECODE_USE_MT6813_PWM
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		StepMotor_Isr_call(4);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		StepMotor_Isr_call(5);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		StepMotor_Isr_call(6);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		StepMotor_Isr_call(7);
	}
#else
if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
{

	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);                //清除TIM的中断待处理位
	IC1Value = TIM_GetCapture1(TIM3);                         //读取IC1捕获寄存器的值，即为PWM周期的计数值
	if (IC1Value != 0)
	{
		DutyCycle = (TIM_GetCapture2(TIM3))/* * 100) / IC1Value*/;         //读取IC2捕获寄存器的值，并计算占空比
		Frequency = /*72000000 /*/ IC1Value;                                          //计算PWM频率。
		T3_Pwm_timeout=T3_PWM_TIME_OUT_DEF_MS;
	}
	else
	{
		DutyCycle = 0;
		Frequency = 0;
	}
	Time3_isr_cnt_ed++;
	if((IC1Value<3000)||(DutyCycle==0))  /*最大400us，计数是0.1us单位*/
	{
		return;	
	}

	
	//if(last_new_code)
	
	
	Time3_isr_cnt++;

	if(Sys_is_MT6813)
	{
		if(Time3_isr_cnt>TIME_ISR_CNT_IS_GETCODE_OK)
		{
			last_new_code = abs(DutyCycle_last - DutyCycle);
			if((last_new_code<=3850)&&(last_new_code>=50))
			{
				DutyCycle_last = DutyCycle;
				Frequency_last=Frequency;
				Time3_isr_cnt-=2;
			}
			else
			{
				Set_Ecode_data(DutyCycle,Frequency);
				Ecode_CS_for_MT6813();
			}
		}
		else
		{
			DutyCycle_last = DutyCycle;
			Frequency_last=Frequency;
		}
	}
}
#endif
	
}

#ifdef ECODE_USE_MT6813_PWM


void Set_is_MT6813PWM_Mode()
{
	
	Sys_is_MT6813 = (Time3_isr_cnt_ed>TIME_ISR_CNT_IS_GETCODE_OK)?1:0;

	//return Sys_is_MT6813;
}

int Check_is_MT6813PWM_Mode()
{
	return Sys_is_MT6813;
}


void Set_Duty_Fre_reset()
{
	DutyCycle = 0;
	Frequency = 0;
	GetEcode_Start =1;
	Time3_isr_cnt =0;
	Time3_isr_cnt_ed =0;
	T3_Pwm_timeout=T3_PWM_TIME_OUT_DEF_MS;
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); 
}

int Get_Duty_Fre_(unsigned int *duty,unsigned  int *freq)
{
	int ret;	
	if((DutyCycle==0)||(Frequency==0)||(Time3_isr_cnt<=TIME_ISR_CNT_IS_GETCODE_OK))
	{
		ret =0;	
	}
	else
	{
		if(duty)
		{
			*duty= DutyCycle;			
		}
		if(freq)
		{
			*freq= Frequency;			
		}
		ret =1;
		
	}
	if(T3_Pwm_timeout==0)
		ret=-1;
	return ret;
}


void time3_pwm_timeout_250us_cnt()
{
	if(T3_Pwm_timeout)
		T3_Pwm_timeout--;
}

void tim3_pwm_updata_loop()
{
	static unsigned int DutyCycle_LOC=0;
	static unsigned int Frequency_LOC=0;
	unsigned int DutyCycle_new;
	unsigned int Frequency_new;
	unsigned int DDabs=0;
	unsigned int FFabs=0;
	
	int ret;

	return;
	if(GetEcode_Start)
	{
		ret  = Get_Duty_Fre_(&DutyCycle_new,&Frequency_new);
		if(ret>0)
		{
		#if 0
			if((DutyCycle_LOC!=DutyCycle_new)||(Frequency_LOC!=Frequency_new))
			{
				DDabs = abs(DutyCycle_LOC-DutyCycle_new);
				FFabs = abs(Frequency_LOC-Frequency_new);
				DutyCycle_LOC = DutyCycle_new;
				Frequency_LOC= Frequency_new;	
				if((FFabs>=3)||(DDabs>=3))
				Message_Send_4halfword(0x2f01,DutyCycle_new,Frequency_new,Get_Curr_eid());
			}
			#endif
			Message_Send_4halfword(0x2f01,DutyCycle_new,Frequency_new,Get_Curr_eid());
			
		}
		else
			if(ret<0)
			{
				unsigned short T3_itenst;
				//T3_itenst = 
				Message_Send_4halfword(0x2e01,0,0,Get_Curr_eid());
				//GetEcode_Start=0;
			}
		
	}
}

void tim3_pwm_timeout_loop()
{
	
	unsigned int DutyCycle_new;
	unsigned int Frequency_new;
	static unsigned char eid_last=0;
	unsigned char eid_c=0;
	unsigned long ed=0;
	
	int ret;
	if((GetEcode_Start)&&(Check_is_MT6813PWM_Mode()))
	{
		ret  = Get_Duty_Fre_(&DutyCycle_new,&Frequency_new);
		if(ret<0)
		{
			Set_Ecode_st(1);
			Ecode_CS_for_MT6813();
		}	
		//Send_last_ecode_data();
		
	}
}



#endif

void hook_TIM1_isr(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

		StepMotor_Isr_call(12);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

		StepMotor_Isr_call(13);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

		StepMotor_Isr_call(14);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

		StepMotor_Isr_call(15);
	}
}

#if 0
void hook_TIM4_CCR_isr(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

		arch_JQD_Isr_call(0,0);
	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

		arch_JQD_Isr_call(1,0);
	}
}


void hook_TIM8_updata()
{
	if (TIM_GetFlagStatus(TIM8,TIM_FLAG_Update)!=RESET)
	{
		TIM_ClearFlag(TIM8,TIM_FLAG_Update);		
		arch_JQD_Isr_call(2);		
	}
}

#endif
void hook_TIM8_isr(void)
{
#ifndef JQD_ISR_TIMER_CNT_50US
#ifdef JQD_NEXT_DO_FIFO
if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

		arch_JQD_Isr_call(0,0);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

		arch_JQD_Isr_call(1,0);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);

		arch_JQD_Isr_call(2,0);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);

		arch_JQD_Isr_call(3,0);
	}
#endif
#else

	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

		StepMotor_Isr_call(8);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

		StepMotor_Isr_call(9);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);

		StepMotor_Isr_call(10);
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);

		StepMotor_Isr_call(11);
	}
#endif
}

void hook_TIM10_isr(void)
{
	//static u16 is2ms =0;
	//u16 capture = 0;
	#ifdef JQD_ISR_TIMER_CNT_50US
	if (TIM_GetFlagStatus(TIM10,TIM_FLAG_Update)!=RESET)
	{
		TIM_ClearFlag(TIM10,TIM_FLAG_Update);		
		arch_JQD_isr_timer_cnt_50us();		
	}

	#else
	#if 0
	if (TIM_GetFlagStatus(TIM10,TIM_FLAG_CC1)!=RESET)
	{
		TIM_ClearFlag(TIM10,TIM_FLAG_CC1);		
		//hook_time_2ms();
		arch_jqd_isr_50_20_us();
	}
	#endif
	#endif

}



void hook_TIM6_isr(void)
{
}



void hook_TIM7_isr(void)
{
	if (TIM_GetFlagStatus(TIM7,TIM_FLAG_Update)!=RESET)
	{
		extern void hook_time_50us(void);
		TIM_ClearFlag(TIM7,TIM_FLAG_Update);		
		hook_time_50us();		
	}
	return;
}


void hook_TIM4_isr(void)
{
	static char i;
	if (TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)!=RESET)
	{
		//extern void hook_time_50us(void);
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);		
		//hook_time_50us();
		if (i)
		{
			hook_time_2ms();	
			
		}
		i=!i;
	}
	return;
}



#ifdef  ECODE_USE_MT6813_PWM
void hook_TIM12_isr()
{
	//extern volatile unsigned int ms01_loop_cnt;
	//static int ms0001=0;

	if (TIM_GetITStatus(TIM12, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC1);

		StepMotor_Isr_call(6);
	}
	if (TIM_GetITStatus(TIM12, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC2);

		StepMotor_Isr_call(7);
	}
	return;
}

#else
void hook_TIM12_isr()
{
	//extern volatile unsigned int ms01_loop_cnt;
	//static int ms0001=0;

	if (TIM_GetITStatus(TIM12, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC1);

		//StepMotor_Isr_call(6);
	}
	if (TIM_GetITStatus(TIM12, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC2);

		//StepMotor_Isr_call(7);
	}
	return;
}

#endif

void hook_TIM9_isr()
{

	#ifndef ECODE_USE_MT6813_PWM

	extern volatile unsigned int ms01_loop_cnt;
	//static int ms0001=0;
	if(TIM_GetFlagStatus(TIM9,TIM_FLAG_Update)!=RESET)
	{
		TIM_ClearFlag(TIM9,TIM_FLAG_Update);
		//ms0001++
		ms01_loop_cnt++;
	}

	#else
	if (TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);

		StepMotor_Isr_call(4);
	}
	if (TIM_GetITStatus(TIM9, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);

		StepMotor_Isr_call(5);
	}
	

	#endif

	#if 0
	if (TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);

		arch_JQD_Isr_call(0);
	}
	if (TIM_GetITStatus(TIM9, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);

		arch_JQD_Isr_call(1);
	}
	#endif
	return;
}
extern void EXTI_CHECK_iscomming(void);
void hook_TIM_sysclk_isr(void)
{
		//hook_time_250us();
	#ifdef JQD_ISR_TIMER_ONE
		hook_time_250us();
		EXTI_CHECK_iscomming();
		#ifdef DEBUG_MT6813_PWM
			Debug_led_cnt_isr();
		#endif
	#else
		#ifdef JQD_ISR_TIMER_50US		
		{
			static char i=0;
			if(i++>=5)
			{
				i=0;
				hook_time_250us();
			}
			JQD_clearblade_timer_250us();

		}
		
		#else
		hook_time_250us();
		#endif
		EXTI_CHECK_iscomming();
	#endif
		
	
}

volatile unsigned long timecount;
void TimeStart()
{
	timecount = TIM_GetCounter(TIM2);
}

unsigned long TimeGet()
{
	unsigned long time = TIM_GetCounter(TIM2);

	if(timecount > time)
		return time + 65535 - timecount;
	else
		return time - timecount;

}

unsigned int max_counter_us[11];
unsigned int last_counter[11];

unsigned int Get_now_time_counter()
{
//	return TIM_GetCounter(TIM6);
}

void Get_main_loop_time_us(unsigned char whichi,unsigned int lastct)
{
#if 0
	//static unsigned char isnotfirst=0;
	//static unsigned int last_counter=0;
	unsigned int nowcounter_err=0;
	unsigned int nowcounter=TIM_GetCounter(TIM6);
	

	//if (isnotfirst)
	{	
		if (lastct>nowcounter)
			nowcounter_err = 	nowcounter+65535-lastct;
		else
			nowcounter_err = 	nowcounter-lastct;
		last_counter[whichi]=nowcounter_err;
		if (nowcounter_err>max_counter_us[whichi])
			max_counter_us[whichi] = nowcounter_err;
		if (max_counter_us[whichi+1]==0)
		{
			max_counter_us[whichi+1]=nowcounter_err;
		}
		else
		{
			if (nowcounter_err<max_counter_us[whichi+1])
			max_counter_us[whichi+1]=nowcounter_err;
		}

	}	
	//last_counter[whichi]=nowcounter;
	//if (!isnotfirst)
	//	isnotfirst =1;

	#endif

}

void Send_debug_Get_main_loop()
{
#if 0
int i;
	for (i=0;i<2;i++)
	Message_Send_4halfword(0x00F2,max_counter_us[i],last_counter[i],i );
#endif
}

void arch_delay_us(int us)
{
	TimeStart();
	while(TimeGet() < us);
}

#ifdef E490_V10_BOARD

void Step_PWMDA_Init_exp(unsigned char whichpwmmask)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	u16 CCR1_Val = 500;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
  	/* PWM1 Mode configuration: Channel1 */
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if (whichpwmmask&0x01)		//抬针
	{
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);	
		TIM_OC2FastConfig(TIM5,TIM_OCFast_Enable); 
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
	}
	if (whichpwmmask&0x02)		//纱嘴
	{		
	  	TIM_OC3Init(TIM5, &TIM_OCInitStructure);	
		TIM_OC3FastConfig(TIM5,TIM_OCFast_Enable); 
	  	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
	}
	if (whichpwmmask&0x04)		//生克
	{		
	  	TIM_OC4Init(TIM5, &TIM_OCInitStructure);	
		TIM_OC4FastConfig(TIM5,TIM_OCFast_Enable); 
	  	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);		
	}
	
  	TIM_OC1Init(TIM5, &TIM_OCInitStructure);	
	TIM_OC1FastConfig(TIM5,TIM_OCFast_Enable); 
  	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

  	TIM_ARRPreloadConfig(TIM5, ENABLE);
  	

	TIM_ITConfig(TIM5,TIM_IT_Update,DISABLE);   //关中断,	
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);   	//必须先清除配置时候产生的更新标志,	
	TIM_Cmd(TIM5,ENABLE);  
}

void Yarn_step_PWMDA_Init()
{
	return;
}


void Yarn_step_PWMDA_Set_val(unsigned int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;

	TIM_SetCompare3(TIM5,temp_cap);

}
void SKER_step_PWMDA_Set_val(unsigned int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;

	TIM_SetCompare4(TIM5,temp_cap);

}
void LIFT_step_PWMDA_Set_val(unsigned int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;

	TIM_SetCompare2(TIM5,temp_cap);

}

void LIFT_step_PWMDA_Set_val_test(unsigned int duty)
{
	if (duty>99)
	{
		duty = 99;
	}
	else if (duty<=0)
	{
		duty =1;
	}
	duty *=10;

	TIM_SetCompare2(TIM5,duty);
}


#else

#ifdef E480_BOARD_V10
void Yarn_step_PWMDA_Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	u16 CCR1_Val = 500;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 119;//59/*=1M , apb1=30M timerclk=apb1*2 */
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
  	/* PWM1 Mode configuration: Channel1 */
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	
	TIM_OC1FastConfig(TIM9,TIM_OCFast_Enable); 
  	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

  	TIM_ARRPreloadConfig(TIM9, ENABLE);
  	

	TIM_ITConfig(TIM9,TIM_IT_Update,DISABLE);   //关中断,	
	TIM_ClearFlag(TIM9,TIM_FLAG_Update);   	//必须先清除配置时候产生的更新标志,	
	TIM_Cmd(TIM9,ENABLE);  
}


void Yarn_step_PWMDA_Set_val(unsigned int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;

	TIM_SetCompare1(TIM9,temp_cap);

}



#endif
#endif





void Time_PWM_init(unsigned char isshockboard)
{
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
u16 CCR1_Val = 500;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 999;

	TIM_TimeBaseStructure.TIM_Prescaler = 59;//59/*=1M , apb1=30M timerclk=apb1*2 */

  	
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
  	/* PWM1 Mode configuration: Channel1 */
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if (isshockboard)
	{
  	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3FastConfig(TIM4,TIM_OCFast_Enable); 
  	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	#ifdef ALARM_SHOCK_DOUBLE
	//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4FastConfig(TIM4,TIM_OCFast_Enable); 
  	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	#endif
	}

	#ifdef E490_V10_BOARD

	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 10;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
	
	#endif


	TIM_ARRPreloadConfig(TIM4, ENABLE);

	
	
  	//TIM_Cmd(TIM4, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1+1;   //脟脌脮录脫脜脧脠录露1 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //脧矛脫娄脫脜脧脠录露1 
	NVIC_Init(&NVIC_InitStructure);                             //脨麓脠毛脡猫脰脙   


	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);   //关中断,	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);   	//必须先清除配置时候产生的更新标志,	
	TIM_Cmd(TIM4,ENABLE);   
	
	
}

#ifdef E692_STEPMOTOR_EX
void EX_step_PWMDA_Set_val(unsigned char whichstep,unsigned int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;
	if (whichstep==0)
	{
		TIM_SetCompare3(TIM4,temp_cap);
	}
	else
		if (whichstep==1)
		{
			TIM_SetCompare4(TIM4,temp_cap);
		}
	

}

#endif


#ifdef E480_BOARD_V10

#ifdef E490_V10_BOARD

volatile unsigned short PWM_A[] = {
	880,878,876,873,870,868,866,863,860,858,856,853,850,848,846,843,840,838,836,
	833,830,828,826,823,820,818,816,813,810,808,806,803,800,798,796,793,790,788,
	786,783,780,778,776,773,770,768,766,763,760,758,756,753,750,748,746,744,742,
	740,738,736,733,730,728,726,723,720,718,716,713,710,708,706,703,700,698,696,
	693,690,688,686,683,680,678,676,673,670,668,666,663,660,658,656,653,650,648,
	646,643,640,648,646,643,630,628,626,623,620,618,616,614,612,610,606,603,600,
	598,596,593,590,588,586,584,582,580,578,576,573,570,568,566,563,560,558,556,
	553,550,548,546,543,540,538,536,533,530,528,526,523,520,518,516,513,510,508,
	506,503,500,498,496,493,490,488,486,483,480,478,476,473,470,468,466,463,460,
	457,454,452,450,447,444,441,439,436,433,430,427,424,421,418,415,412,409,407,
	404,401,399,396,393,390,387,384,381,378,376,373,370,367,364,361,358,355,352};


#else

volatile unsigned short PWM_A[] = {
	826,823,820,817,813,810,807,804,800,797,793,790,787,783,780,777,773,770,767,
	764,760,757,753,750,748,746,743,740,738,736,734,732,730,728,726,724,722,720,
	719,717,716,715,713,711,710,709,708,706,705,704,702,700,698,696,695,694,692,
	690,689,688,686,685,683,681,680,678,676,675,674,672,670,668,666,664,662,660,
	658,656,653,650,647,643,640,637,633,630,627,623,620,617,613,610,605,600,595,
	590,585,580,575,570,567,563,560,555,550,545,540,535,530,525,520,515,510,505,
	500,495,490,485,480,475,470,465,460,455,450,445,440,435,430,425,420,415,410,
	405,400,395,390,385,380,375,370,365,360,355,350,345,340,335,330,325,320,315,
	310,305,300,295,290,285,280,275,270,265,260,255,250,245,240,235,230,225,220};
#endif

int get_CCR_Val_with_PWM_A(int Step_A)
{
	//int base_val;
	//int base_A;
	//int cd;
	//int cd1;
	//int cz=0;
	int isn=1;
#if 1
	
	if (Step_A>2580/*2200*/)
		{Step_A=2580;}
	if (Step_A<500)
		{Step_A=500;}

	isn = (Step_A / 10 -50);
	if ((isn>=0)&&(isn<= 208/*170*/))		
	{
		return PWM_A[isn];
	}
	else
		return 734;
	
#else	
	int base_val;
	int base_A;
	int cd;
	int cd1;
	int cz=0;

	
	if (Step_A<=57)
	{
		Step_A =57;
	}
	else
		if (Step_A>=2553)
		{
			Step_A =2553;
		}

	if (Step_A<=730)
	{
		
		base_A = 440;
		base_val = 850;
		cd =30;	
		cd1 =29;
	
	}else if (Step_A>=1400)
	{

		base_A = 1740;
		base_val = 450;
		cd =21;
		cd1=19;
	}
	else
	{
		base_A = 1200;
		base_val = 670;
		cd =55;
		cd1=38;
	}

		cz= Step_A-base_A;
		
		if (cz<0)
		{
			cz *=(-1);
			isn =-1;
		}

		return base_val - (cz*10 / ((isn==1)?cd1:cd))*isn;
#endif
	
}




void alert_set_Step_PWM_A(int Step_A)
{
	int temp_cap;
	Step_A=arch_need_current_add(Step_A);
	temp_cap = get_CCR_Val_with_PWM_A(Step_A);
	
	if(temp_cap > 980)
		temp_cap = 980;
	else if(temp_cap <= 20)
		temp_cap = 20;
	//else temp_cap = Step_A;
#ifdef E490_V10_BOARD
	TIM_SetCompare4(TIM5,temp_cap);
#else	
	TIM_SetCompare1(TIM4, temp_cap);
#endif
}


#endif


void alert_set_shock_PWM_1(int shock_sensitivity)
{
	int temp_cap;
	if(shock_sensitivity > 999)
		temp_cap = 999;
	else if(shock_sensitivity <= 0)
		temp_cap = 0;
	else temp_cap = shock_sensitivity;

		TIM_SetCompare3(TIM4, temp_cap);
	

}

#ifdef ALARM_SHOCK_DOUBLE
void alert_set_shock_PWM_2(int shock_sensitivity)
{
	int temp_cap;
	if(shock_sensitivity > 999)
		temp_cap = 999;
	else if(shock_sensitivity <= 0)
		temp_cap = 0;
	else temp_cap = shock_sensitivity;

		TIM_SetCompare4(TIM4, temp_cap);
	

}
#endif

void alert_set_shock_PWM(int shock_sensitivity,int whichshock)
{
	//shock_sensitivity = 700;
	switch (whichshock)
	{
		case 0:
			alert_set_shock_PWM_1(shock_sensitivity);
			#ifdef ALARM_SHOCK_DOUBLE			
			alert_set_shock_PWM_2(shock_sensitivity);
			#endif
			break;
		case 1:
			alert_set_shock_PWM_1(shock_sensitivity);
			break;
		#ifdef ALARM_SHOCK_DOUBLE			
		case 2:
			alert_set_shock_PWM_2(shock_sensitivity);
			break;
		#endif	
		default:
			break;
	}

	
}

#ifndef ECODE_USE_MT6813_PWM


unsigned int Get_systick_32()
{
	extern  volatile unsigned int ms01_loop_cnt;
	return TIM_GetCounter(TIM9)|(ms01_loop_cnt<<16);
}

u16 Get_systick()
{
	return TIM_GetCounter(TIM9);
	
}

#endif


unsigned short Get_systime_100us()
{
	return TIM_GetCounter(TIM6);

	
}

unsigned short Get_interval_time_100us(unsigned short last100us)
{
	unsigned long time = TIM_GetCounter(TIM6);

	if(last100us > time)
		return time + 65535 - last100us;
	else
		return time - last100us;
}


