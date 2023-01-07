/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "platform_config.h"
//#include "arch.h"

extern void arch_Power_ctrl_ex(unsigned char whichpower,unsigned char onoff,unsigned char whichcp);
extern void arch_delay_us(int us);
extern int Overload_is_come(unsigned int overloadindex);
extern unsigned char arch_need_close711_alert();
void CheckOverloadBranch(void);
extern volatile unsigned char Yarn_use_Step;
extern volatile unsigned char Step_use_exBoard;


#define OVERLOAD_CHECKAGAIN_TIMEOUT	 (50)   //25us


#ifdef FILTER_1ST_2ND_JQD_OVERLOAD
//extern volatile unsigned char isfastjqd;
extern volatile unsigned char jqd_last_blade[MAX_JACQUARD];
extern volatile unsigned char jqd_isfirst_do[MAX_JACQUARD][MAX_BLADE];/*确认是否是第一次动作0--第一次动*/
extern volatile unsigned short needresetjqd;
#endif


extern unsigned long Overload_status_JQD_ACT_YARN;
void EXTI_Enable(u32 EXTI_Line)
{
	EXTI_ClearITPendingBit(EXTI_Line);
	EXTI->IMR |= EXTI_Line;
	//EXTI->EMR |= EXTI_Line;
}

void EXTI_Disable(u32 EXTI_Line)
{
	EXTI->IMR &= ~EXTI_Line;
	EXTI->EMR &=  ~EXTI_Line;
}


unsigned char EXTI_CHECK_justcomming()
{
		unsigned char ret=0;
	
		return ret;
}

void EXTI_CHECK_iscomming()
{
	 unsigned char ret=0;
		ret = EXTI_CHECK_justcomming();
		if (ret)
		{
			CheckOverloadBranch();
		}
}

#ifdef E490_V10_BOARD

void EXTI_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	#ifndef E693_TEN_BLADE_NEEDLE_SELECTOR
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);		
	#endif
		
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource11);	
	

	#ifdef E692_STEPMOTOR_EX
	if(!arch_need_close711_alert())
	{
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource8);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);
		if (!Yarn_use_Step)
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);	
		
	}
	#endif

	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource0);

	// 纱嘴过流中断

	//EXTI_DeInit();
	
	//EXTI_StructInit(&EXTI_InitStruct);

	#ifndef E693_TEN_BLADE_NEEDLE_SELECTOR
	{
		EXTI_InitStruct.EXTI_Line = EXTI_Line15;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_Disable(EXTI_Line15);	
	}
	#endif

	
	EXTI_InitStruct.EXTI_Line = EXTI_Line11;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line11);

	#ifdef E692_STEPMOTOR_EX
	if(!arch_need_close711_alert())
	{
		EXTI_InitStruct.EXTI_Line = EXTI_Line8;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);	
		EXTI_Disable(EXTI_Line8);

			// 三角1过流报警
		EXTI_InitStruct.EXTI_Line = EXTI_Line12;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);	
		EXTI_Disable(EXTI_Line12);
		
		if (!Yarn_use_Step)
		{
			EXTI_InitStruct.EXTI_Line = EXTI_Line7;
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStruct);	
			EXTI_Disable(EXTI_Line7);
		}
	}
	#endif

	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line3);
	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line2);	

		
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line1);
// 选针3 +24V 过流报警
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line0);
// 选针3 +24V 过流报警

	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);

	//if (!Yarn_use_Step)
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;    //更新事件 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
		NVIC_Init(&NVIC_InitStructure);
	}
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);



}



#else

void EXTI_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource6);

	#ifdef E499_BOARD_SUPPORT_
	if (!arch_is_EMF_2_SK_board())
	#endif
	{
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource14);
	}
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource8);
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);


	// 纱嘴过流中断

	//EXTI_DeInit();
	
	//EXTI_StructInit(&EXTI_InitStruct);

	#ifdef E499_BOARD_SUPPORT_
	if (!arch_is_EMF_2_SK_board())
	#endif
	{
		EXTI_InitStruct.EXTI_Line = EXTI_Line15;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_Disable(EXTI_Line15);

		EXTI_InitStruct.EXTI_Line = EXTI_Line14;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);	
		EXTI_Disable(EXTI_Line14);
	}
	// 三角1过流报警


	
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line13);
	// 三角2过流报警
	
	

	EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line12);
	
	// 选针1 -24V 过流报警
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line11;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line11);
	
	// 选针1 +24V 过流报警

	EXTI_InitStruct.EXTI_Line = EXTI_Line8;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line8);
	
/*
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line6);
	*/
		
// 选针3 -24V 过流报警
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line3);
// 选针3 +24V 过流报警

	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);	
	EXTI_Disable(EXTI_Line2);	
	


	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    //更新事件 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //响应优先级1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断 
	NVIC_Init(&NVIC_InitStructure);



}

#endif

extern volatile unsigned int whichOverload_in_progress_mask;

#ifdef E490_V10_BOARD

extern unsigned char JQD_overload_alerted;

void hook_EXTI_isr()
{
	int ol=0;
	void EXTI_Disable(u32 EXTI_Line);
	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		
		if (XZ1_POSITIVE24V_ALARM_INPUT())
		{	
			#ifdef FILTER_1ST_2ND_JQD_OVERLOAD
			if (jqd_isfirst_do[0][jqd_last_blade[0]]<=2)
			{
				needresetjqd |= (0x01<<0);
				//arch_8844_Reset_one(0);
			}
			else
			#endif
			{		
				//POWER_XZ_NEGATIVE24V_OUTPUT(1);
				ol=Overload_is_come(OVERLOAD_MASK_XZ1);
			
				Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ1;
				whichOverload_in_progress_mask |=0x03;
				JQD_overload_alerted = 1;
			}
		}	
		
		
	}
	
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line1);
		#ifdef ACT_OVERLOAD_ALERT
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (SJ2_NEGATIVE24V_ALARM_INPUT())
		{
			ol=Overload_is_come(OVERLOAD_MASK_ACT2);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_ACT2;
			whichOverload_in_progress_mask |=0x02;
		}
		#endif
		
	}

	// 纱嘴过流
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line2);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		
		if (XZ2_POSITIVE24V_ALARM_INPUT())
		{	
			#ifdef FILTER_1ST_2ND_JQD_OVERLOAD	
			if (jqd_isfirst_do[1][jqd_last_blade[1]]<=2)
			{
				//arch_8844_Reset_one(1);
				needresetjqd |= (0x01<<1);
			}
			else
			#endif
			{
				ol=Overload_is_come(OVERLOAD_MASK_XZ2);
				//POWER_XZ_NEGATIVE24V_OUTPUT(1);
				Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ2;
				whichOverload_in_progress_mask |=0x03;
				JQD_overload_alerted = 2;
			}
		}	
		
		
	}
	// 选针1过流
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line3);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (XZ4_POSITIVE24V_ALARM_INPUT())
		{
			#ifdef FILTER_1ST_2ND_JQD_OVERLOAD	
			if (jqd_isfirst_do[3][jqd_last_blade[3]]<=2)
			{
				//arch_8844_Reset_one(3);
				needresetjqd |= (0x01<<3);
			}
			else
			#endif
			{
				ol=Overload_is_come(OVERLOAD_MASK_XZ4);
				Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ4;
				whichOverload_in_progress_mask |=0x03;
				JQD_overload_alerted = 4;
			}
		}
	
	}
	

	if(EXTI_GetITStatus(EXTI_Line11) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line11);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (XZ3_POSITIVE24V_ALARM_INPUT())
		{
		#ifdef FILTER_1ST_2ND_JQD_OVERLOAD	
			if (jqd_isfirst_do[2][jqd_last_blade[2]]<=2)
			{
				//arch_8844_Reset_one(2);
				needresetjqd |= (0x01<<2);
			}
			else
		#endif
			{
				ol=Overload_is_come(OVERLOAD_MASK_XZ3);
				Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ3;
				whichOverload_in_progress_mask |=0x03;
				JQD_overload_alerted = 3;
			}
		}
	
	}

#ifndef NOCHECK_ASC711FLAUT_ISR


	if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line7);
		#ifndef TEST_EMC_711_PN
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (SZ1_NEGATIVE24V_ALARM_INPUT())
		{
			//Overload_is_come(OVERLOAD_MASK_YARN);
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,38);
			ol=Overload_is_come(OVERLOAD_MASK_YARN_F);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_YARN_F;
			whichOverload_in_progress_mask |=0x02;
		}
		#endif
	}


	
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line8);
		#ifndef TEST_EMC_711_PN
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT<<3);
		if (FAULT_NGATIVE24_CURRENT_INPUT())
		{
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);	
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,39);
			ol=Overload_is_come(OVERLOAD_MASK_24V_C_N);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_24V_C_N;
			whichOverload_in_progress_mask |=0x01;
		}
		#endif
		
	}	

	if(EXTI_GetITStatus(EXTI_Line12) != RESET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		#ifndef TEST_EMC_711_PN
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (FAULT_POSITIVE24_CURRENT_INPUT())
		{
			//POWER_XZ_POSITIVE24V_OUTPUT(0);	
			arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,40);
			ol=Overload_is_come(OVERLOAD_MASK_24V_C_P);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_24V_C_P;
			whichOverload_in_progress_mask |=0x02;
		}
		#endif
	}
	
	
#endif
	

	if(EXTI_GetITStatus(EXTI_Line15) != RESET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line15);
		#ifdef ACT_OVERLOAD_ALERT
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		
		if (SJ1_NEGATIVE24V_ALARM_INPUT())
		{
			ol=Overload_is_come(OVERLOAD_MASK_ACT1);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_ACT1;
			whichOverload_in_progress_mask |=0x02;
		}
		#endif
		
	}	

	if (ol)
		CheckOverloadBranch();
}


#else


void hook_EXTI_isr()
{
	
	void EXTI_Disable(u32 EXTI_Line);

	// 纱嘴过流
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line2);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		
		if (XZ2_POSITIVE24V_ALARM_INPUT())
		{		
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ2;
			whichOverload_in_progress_mask |=0x03;
		}		
		
	}
	// 选针1过流
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line3);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (XZ4_POSITIVE24V_ALARM_INPUT())
		{
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ4;
			whichOverload_in_progress_mask |=0x03;
		}
	}
	// 选针2过流
/*
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line6);
		Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_YARN;
	}

*/
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line8);
		//arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (FAULT_NGATIVE24_CURRENT_INPUT())
		{
		//	POWER_XZ_NEGATIVE24V_OUTPUT(1);	
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,41);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_24V_C_N;
			whichOverload_in_progress_mask |=0x01;
		}
	}

	

	if(EXTI_GetITStatus(EXTI_Line11) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line11);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (XZ3_POSITIVE24V_ALARM_INPUT())
		{
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ3;
			whichOverload_in_progress_mask |=0x03;
		}
	}

	if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line12);
		//arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (FAULT_POSITIVE24_CURRENT_INPUT())
		{
			//POWER_XZ_POSITIVE24V_OUTPUT(0);
			arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,42);
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_24V_C_P;
			whichOverload_in_progress_mask |=0x02;
		}
	}
	

	if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line13);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (XZ1_POSITIVE24V_ALARM_INPUT())
		{
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_XZ1;
			whichOverload_in_progress_mask |=0x03;
		}
	}
	
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line14);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (SJ2_NEGATIVE24V_ALARM_INPUT())
		{
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_ACT2;
			whichOverload_in_progress_mask |=0x02;
		}
	}
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line15);
		arch_delay_us(OVERLOAD_CHECKAGAIN_TIMEOUT);
		if (SJ1_NEGATIVE24V_ALARM_INPUT())
		{
			Overload_status_JQD_ACT_YARN |= OVERLOAD_MASK_ACT1;
			whichOverload_in_progress_mask |=0x02;
		}
	}	

	CheckOverloadBranch();
}

#endif


