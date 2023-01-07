#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

#include "Config.h"
#include "Stm32f2xx_gpio.h"

//#define V24_CURRENT_711_NO   /*711芯片供货不及，取消FAULT报警*/

//#define DEBUG_MT6813_PWM 


#define ECODE_USE_MT6813_PWM



#define POWER_ON_NP_24 1
#define POWER_OFF_NP_24 0
#define POWER_CTR_N_24 0
#define POWER_CTR_P_24 1






#define MAX_JACQUARD		8
#define MAX_BLADE		10



#define MAX_YARN		8
#define MAX_ACTEMF		12


#define USE_ADC2_	(1)


/*支持三角电磁铁板转换成生克电机板(E499)*/
#define E499_BOARD_SUPPORT_	(1)

#define E490_V10_BOARD		(1)		/*支持E490V1.0板子，改动比较大*/

//#define YARN_ZERO_CHECK	/*支持纱嘴零位检查(采样)*/


/*2018 11 29 支持电机扩展板*/
#define E692_STEPMOTOR_EX 	(1)  


#ifdef SUPPORT_1S
 
#define E475_480_BOARD

#define BOARD_NAME_E475	(0xE475)
#define BOARD_NAME_E480	(0xE480)
#define BOARD_NAME_E490	(0xE490)



#define ALARM_SHOCK_DOUBLE	// 支持两个撞针信号


#define MAX_DC24_COUNT		(700)

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
#define MAX_DC24_DATA_COUNT (80)
#else
#define MAX_DC24_DATA_COUNT (72)
#endif

#define MAX_DC24_A_BASE_DATA_COUNT 	(30)	
										/*基准值用采样之前的平均值*/

  #undef MAX_YARN
  #define MAX_YARN	8

  #undef MAX_ACTEMF
  #define MAX_ACTEMF	12

  #undef MAX_JACQUARD
  #define MAX_JACQUARD	8
#endif	// end SUPPORT_1S

//#define WDG_ENABLE

#ifdef STEP_CURR_DEF
#define DEFAULT_DA_CURRENT_DEN_MA		(2400)	/*度目默认电流值*/
#define DEFAULT_DA_CURRENT_SK_MA			(800)
#define DEFAULT_DA_CURRENT_ACT_MA		(2400)
#define DEFAULT_DA_CURRENT_YARN_MA		(1500)
#define DEFAULT_DA_CURRENT_LIFT_MA		(800)
#define DEFAULT_DA_CURRENT_FEET_MA		(800)


#else
#ifdef CX_DM_CURRENT_DEFAULT_1800
#define DEFAULT_DA_CURRENT_DEN_MA		(1800)	/*度目默认电流值*/
#else

#define DEFAULT_DA_CURRENT_DEN_MA		(800)	/*度目默认电流值*/
#endif
#define DEFAULT_DA_CURRENT_SK_MA			(800)
#define DEFAULT_DA_CURRENT_ACT_MA		(800)
#define DEFAULT_DA_CURRENT_YARN_MA		(1500)/*纱嘴电机做升降用，电流需要开放设置*/
#define DEFAULT_DA_CURRENT_LIFT_MA		(800)
#define DEFAULT_DA_CURRENT_FEET_MA		(800)


#endif




#define BOOT_START_ADDRESS	((u32)0x08000000)
#define APP_START_ADDRESS		((u32)0x08020000)

#define Bank1_SRAM1_ADDR		(0x60000000L)

#ifdef E490_V10_BOARD

#define ADDR_MAINBOARD		(0x00000000L)
#define ADDR_YARN_DCT			(0x000000C0L)
#define ADDR_JQDBOARD1			(0x00000000L)
#define ADDR_ACTBOARD1			(0x00000020L)
#define ADDR_JQDBOARD2			(0x00000040L)
#define ADDR_ACTBOARD2			(0x00000060L)
#define ADDR_STEPBOARD1		(0x00000100L)
#define ADDR_STEPBOARD2		(0x00000140L)
#define ADDR_STEPBOARD3		ADDR_YARN_DCT 		//纱嘴电机
#define ADDR_STEPBOARD4		(0x00000080L)		//抬针电机

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
#define ADDR_JQDBOARD_EX1		(0x00000030L)
#define ADDR_JQDBOARD_EX2		(0x00000070L)
#define ADDR_DIS_JQD_IN_EN_EX		(0x04)
#endif

#define ADDR_JQDBOARD1_RESET	(0x00000026L)
#define ADDR_ACTBOARD1_RESET	(0x00000027L)
#define ADDR_JQDBOARD2_RESET	(0x00000066L)
#define ADDR_ACTBOARD2_RESET	(0x00000067L)

#define ADDR_DIS_JQD_IN_EN		(0x10)
#define ADDR_DIS_ACT_IN_EN		(0x08)

#define ACT_OUT_IS_NOT			(1)


#else

#define ADDR_MAINBOARD		(0x00000000L)
#define ADDR_YARN_DCT			(0x00000000L)
#define ADDR_JQDBOARD1			(0x00200000L)
#define ADDR_ACTBOARD1			(0x00200020L)
#define ADDR_JQDBOARD2			(0x00400000L)
#define ADDR_ACTBOARD2			(0x00400020L)
//#define ADDR_STEPBOARD		(0x00800000L)
#define ADDR_STEPBOARD			(0x00600000L)

#define ADDR_DIS_JQD_IN_EN		(0x40)
#define ADDR_DIS_ACT_IN_EN		(0x40)
#endif


#define OUTPUT(addr, Data)	(*(volatile unsigned char *)(Bank1_SRAM1_ADDR + (addr))) = Data
#define INPUT(addr)		(*(volatile unsigned char *)(Bank1_SRAM1_ADDR + (addr)))



#if 1
#ifdef SUPPORT_1S
 #define JQD1_ERR1_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 0))
 #define JQD1_ERR2_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 1))
 #define JQD2_ERR1_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 2))
 #define JQD2_ERR2_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 3))
 #define STEP_ERR_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 4))
 #define V24_PERR_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 5))
 #define V12_PERR_INPUT()	(~INPUT(ADDR_MAINBOARD + 0xA0) & (0x1 << 6))
 #define STATUS_LINE1_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
 #define STATUS_LINE2_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
 #define EXT_OVERLOAD_YARN()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0))
 #define EXT_OVERLOAD_JQD1()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))
#endif

#endif


#if 1

#define ENCODER_DMA_SUPPORT			0

//#define ENCODER_IC_IS_MT6815CT		/*定义另外的编码器芯片*/
//#define ENCODER_IC_IS_AEAT8800		/*定义另外的编码器芯片*/
//
#define ENCODER_IC_IS_5045OR8800		/*定义两种编码器芯片同时支持*/

// for E534 2.0 
#define SSI_READ_DATA()		(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))
#define SSI_CLK_LOW()		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
#define SSI_CLK_HIGHT()		GPIO_SetBits(GPIOB, GPIO_Pin_3);

#define SSI_WRITE_DATA(x)		((x)?GPIO_SetBits(GPIOB, GPIO_Pin_4):GPIO_ResetBits(GPIOB, GPIO_Pin_4))
//#define 



#define SSI_Select(idx)		SSI_Select_ecode(idx)
#define SSI_DeSelect(idx)		SSI_DeSelect_ecode(idx)

#endif




#define OVERLOAD_MASK_BOARD1	0x01
#define OVERLOAD_MASK_BOARD2	0x02


#ifdef E475_480_BOARD


#define EMF_INDEX_YARN		(0x02)
#define EMF_INDEX_ACTION_B		(0x00)
#define EMF_INDEX_ACTION_F		(0x01)

#define ERROR_ACTIVE_MASK_DC24_P	(0x00000001)
#define ERROR_ACTIVE_MASK_DC24_N	(0x00000002)
#define ERROR_ACTIVE_MASK_DC12_P	(0x00000004)
#define ERROR_ACTIVE_MASK_SW1		(0x00000008)
#define ERROR_ACTIVE_MASK_SW2		(0x00000010)
#define ERROR_ACTIVE_MASK_MOTO1	(0x00000020)
#define ERROR_ACTIVE_MASK_MOTO2	(0x00000040)
#define ERROR_ACTIVE_MASK_MOTO3	(0x00000080)
#define ERROR_ACTIVE_MASK_MOTO4	(0x00000100)
#define ERROR_ACTIVE_MASK_MOTO5	(0x00000200)
#define ERROR_ACTIVE_MASK_MOTO6	(0x00000400)
#define ERROR_ACTIVE_MASK_SHOCK	(0x00000800)
#define ERROR_ACTIVE_MASK_TEMP	(0x00001000)

//主控界面上显示的机头板过流[aa][bb],aa表示具体内容--低8位，bb表示系统号


#define OVERLOAD_MASK_24V_C_N		(0x00000001)
#define OVERLOAD_MASK_24V_C_P		(0x00000002)
#define OVERLOAD_MASK_RESERVE1	(0x00000004)
#define OVERLOAD_MASK_RESERVE2	(0x00000008)
#define OVERLOAD_MASK_XZ1		(0x00000010)
#define OVERLOAD_MASK_XZ2		(0x00000020)
#define OVERLOAD_MASK_XZ3		(0x00000040)
#define OVERLOAD_MASK_XZ4		(0x00000080)
#define OVERLOAD_MASK_ACT1	(0x00000100)
#define OVERLOAD_MASK_ACT2	(0x00000200)
//
//
#define OVERLOAD_MASK_YARN	(0x00001000)
#define OVERLOAD_MASK_YARN_F	(0x00002000)

#define OVERLOAD_MASK_AD_24_N	(0x00004000)
#define OVERLOAD_MASK_AD_24_P	(0x00008000)




#define OVERLOAD_MASK	(OVERLOAD_MASK_24V_C_N | OVERLOAD_MASK_24V_C_P | OVERLOAD_MASK_RESERVE1 |OVERLOAD_MASK_RESERVE2 \
						| OVERLOAD_MASK_XZ1 | OVERLOAD_MASK_XZ2 | OVERLOAD_MASK_XZ3 | OVERLOAD_MASK_XZ4 \
						| OVERLOAD_MASK_ACT1 | OVERLOAD_MASK_ACT2 | OVERLOAD_MASK_YARN|OVERLOAD_MASK_YARN_F |OVERLOAD_MASK_AD_24_N|OVERLOAD_MASK_AD_24_P)

#define OVERLOAD_MASK_NOYARN	(OVERLOAD_MASK_XZ1 | OVERLOAD_MASK_XZ2 | OVERLOAD_MASK_XZ3 | OVERLOAD_MASK_XZ4 |OVERLOAD_MASK_AD_24_N|OVERLOAD_MASK_AD_24_P)


						
#endif



//E534-3.3
#define BOARD_CPLD_VER		0xF8
#define GET_CPLD_VER1()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER))
#define GET_CPLD_VER2()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 1))
#define GET_CPLD_VER3()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 2))
#define GET_CPLD_VER4()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 3))

//#define BOARD_CPLD_VER_NEW 	0x50
#define GET_CPLD_NAME_NEW(i)	(INPUT(ADDR_JQDBOARD1 + 0x50+i))
#define GET_CPLD_VER_NEW(i)	(INPUT(ADDR_JQDBOARD1 + 0x30+i))
#define GET_CPLD_EXP_NEW(i)	(INPUT(ADDR_JQDBOARD1 + 0x70+i))

//#define GET_CPLD_VER_NEW2()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER_NEW+1))


#ifdef E475_480_BOARD

#ifdef E480_BOARD_V10
#define STEP_NUM			16
#else
#define STEP_NUM			6
#endif

//E457-V00
//MOTO_ALL_RESET
#ifdef E490_V10_BOARD

#define MOTO1_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+2, x))
#define MOTO2_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+6, x))
#define MOTO3_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+2, x))
#define MOTO4_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+6, x))
#define MOTO5_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+10, x))
#define MOTO6_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+14, x))
#define MOTO7_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+10, x))
#define MOTO8_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+14, x))
#define MOTO9_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+2, x))
#define MOTO10_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+6, x))
#define MOTO11_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+10, x))
#define MOTO12_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+14, x))
#define MOTO13_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+2, x))
#define MOTO14_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+6, x))
#define MOTO15_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+10, x))
#define MOTO16_RESET_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+14, x))
#else

#define MOTOALL_RESET_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_9)))
#ifdef E499_BOARD_SUPPORT_
#define MOTO_13_14_RESET_OUTPUT(x) ((x!=0) ? (OUTPUT(ADDR_ACTBOARD1 +11 , 0)) : (OUTPUT(ADDR_ACTBOARD1 +11 , 1)))	
#endif
#endif
//MOTO_HALF
#ifdef E490_V10_BOARD

#define MOTO1_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+3, x))
#define MOTO2_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+7, x))
#define MOTO3_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+3, x))
#define MOTO4_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+7, x))
#define MOTO5_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+11, x))
#define MOTO6_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+15, x))
#define MOTO7_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+11, x))
#define MOTO8_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+15, x))
#define MOTO8_HALF_OUTPUT_E490(x) (OUTPUT(ADDR_STEPBOARD2+15, x))
#define MOTO9_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+3, x))
#define MOTO10_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+7, x))
#define MOTO11_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+11, x))
#define MOTO12_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+15, x))
#define MOTO13_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+3, x))
#define MOTO14_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+7, x))
#define MOTO15_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+11, x))
#define MOTO16_HALF_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+15, x))


#else

#define MOTO1_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_0)))
#define MOTO2_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_5)))
#define MOTO3_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_4)))
#define MOTO4_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_2)))
#define MOTO5_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_10)))
#define MOTO6_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_11)))
#ifdef E480_BOARD_V10

#define MOTO7_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_12)))
#define MOTO8_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_4)))
#define MOTO8_HALF_OUTPUT_E490(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_1)))


#define MOTO9_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_3)))
#define MOTO10_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_0)))
#define MOTO11_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_1)))
#define MOTO12_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_0)))
#ifdef E499_BOARD_SUPPORT_
#define MOTO13_HALF_OUTPUT(x)  ((x==0) ? (OUTPUT(ADDR_ACTBOARD1 +10 , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +10 , ACT_OUT_IS_NOT?1:0)))
#define MOTO14_HALF_OUTPUT(x)  ((x==0) ? (OUTPUT(ADDR_ACTBOARD1 +10+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +10+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?1:0)))

#endif
#endif
#endif


//MOTO_DIR

#ifdef E490_V10_BOARD
#define MOTO1_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD1+1, x))
#define MOTO2_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD1+5, x))
#define MOTO3_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD2+1, x))
#define MOTO4_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD2+5, x))
#define MOTO5_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD1+9, x))
#define MOTO6_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD1+13, x))
#define MOTO7_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD2+9, x))
#define MOTO8_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD2+13, x))
#define MOTO9_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD3+1, x))
#define MOTO10_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD3+5, x))
#define MOTO11_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD3+9, x))
#define MOTO12_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD3+13, x))
#define MOTO13_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD4+1, x))
#define MOTO14_DIR_OUTPUT(x) 	 (OUTPUT(ADDR_STEPBOARD4+5, x))
#define MOTO15_DIR_OUTPUT(x)  	 (OUTPUT(ADDR_STEPBOARD4+9, x))
#define MOTO16_DIR_OUTPUT(x) 	 (OUTPUT(ADDR_STEPBOARD4+13, x))
#else
#define MOTO1_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_10)))
#define MOTO2_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_13)))
#define MOTO3_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_8)))
#define MOTO4_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_3)))
#define MOTO5_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_11)))
#define MOTO6_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_8)))
#ifdef E480_BOARD_V10
#define MOTO7_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_9)))
#define MOTO8_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_5)))

#define MOTO9_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_1)))
#define MOTO10_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_4)))
#define MOTO11_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_10)))
#define MOTO12_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_9)))
#ifdef E499_BOARD_SUPPORT_
#define MOTO13_DIR_OUTPUT(x)  ((x==0) ? (OUTPUT(ADDR_ACTBOARD1 +9 , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +9 , ACT_OUT_IS_NOT?1:0)))
#define MOTO14_DIR_OUTPUT(x)  ((x==0) ? (OUTPUT(ADDR_ACTBOARD1 +9+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +9+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?1:0)))

#endif
#endif
#endif

//MOTO_ENABLE

#ifdef E490_V10_BOARD

#define MOTO1_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+0, x))
#define MOTO2_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+4, x))
#define MOTO3_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+0, x))
#define MOTO4_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+4, x))
#define MOTO5_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+8, x))
#define MOTO6_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD1+12, x))
#define MOTO7_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+8, x))		
#define MOTO8_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD2+12, x))		//SKER

#define MOTO9_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+0, x))
#define MOTO10_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+4, x))
#define MOTO11_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+8, x))
#define MOTO12_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD3+12, x))

#define MOTO13_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+0, x))			//抬针
#define MOTO14_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+4, x))			//抬针
#define MOTO15_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+8, x))			//抬针
#define MOTO16_ENABLE_OUTPUT(x)  (OUTPUT(ADDR_STEPBOARD4+12, x))			//抬针

#else


#ifdef E480_BOARD_V10
#define MOTO1_8_ENABLE_OUTPUT(x)  ((x==0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_6)))
#define MOTO9_12_ENABLE_OUTPUT(x)  ((x==0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_3)))
#define MOTO9_12_ENABLE_OUTPUT_E490(x)  ((x==0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_0)))
#ifdef E499_BOARD_SUPPORT_
#define MOTO13_ENABLE_OUTPUT(x)  ((x!=0) ? (OUTPUT(ADDR_ACTBOARD1 +8 , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +8, ACT_OUT_IS_NOT?1:0)))
#define MOTO14_ENABLE_OUTPUT(x)  ((x!=0) ? (OUTPUT(ADDR_ACTBOARD1 +8+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?0:1)) : (OUTPUT(ADDR_ACTBOARD1 +8+ADDR_DIS_ACT_IN_EN , ACT_OUT_IS_NOT?1:0)))

#endif


#else

#define MOTO1_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_11)))
#define MOTO2_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_12)))
#define MOTO3_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_5)))
#define MOTO4_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_13)))
#define MOTO5_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_12)))
#define MOTO6_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_9)))


#endif
#endif

//MOTO_Pulse
#ifdef E490_V10_BOARD
#define MOTO1_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_6)))
#define MOTO2_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_7)))
#define MOTO3_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_8)))
#define MOTO4_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_9)))
#define MOTO5_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_6)))
#define MOTO6_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_7)))
#define MOTO7_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_10)))
#define MOTO8_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_11)))

#define MOTO9_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_9)))
#define MOTO10_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_11)))
#define MOTO11_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_13)))
#define MOTO12_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_14)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_14)))

#define MOTO13_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_5)))
#define MOTO14_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_6)))

#define MOTO15_PULSE_OUTPUT(x)  ((x!=0) ? (1) :(0))		/* 预留*/
#define MOTO16_PULSE_OUTPUT(x)  ((x!=0) ? (1) :(0))		/* 预留*/



#else


#define MOTO1_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_6)))
#define MOTO2_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_7)))
#define MOTO3_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_8)))
#define MOTO4_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_9)))
#define MOTO5_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_6)))
#define MOTO6_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_7)))

#ifdef E480_BOARD_V10
#define MOTO7_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_13)))
#define MOTO8_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_11)))

#define MOTO9_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_6)))
#define MOTO10_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_7)))
#define MOTO11_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_7)))
#define MOTO12_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_8)))
#ifdef E499_BOARD_SUPPORT_
#define MOTO13_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_15)))
#define MOTO14_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_14)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_14)))


#endif
#endif
#endif

//MOTO_USM01
#define MOTO_DM_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_0)))
#define MOTO_DM_USM1(x)   ((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_1)))
#ifdef E480_BOARD_V10
#define MOTO_SK_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_0)))
#define MOTO_SK_USM1(x)   ((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_1)))
#define MOTO_SJ_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define MOTO_SJ_USM1(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))
#else
#define MOTO_SK_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define MOTO_SK_USM1(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))
#endif

//MOTO_ZERO
#ifdef E490_V10_BOARD
#define MOTO1_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))
#define MOTO2_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9))
#define MOTO3_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10))
#define MOTO4_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11))
#define MOTO5_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
#define MOTO6_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
#define MOTO7_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))
#define MOTO8_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))

#define MOTO9_ZERO_INPUT()		(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8))
#define MOTO10_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9))
#define MOTO11_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))
#define MOTO12_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))

#define MOTO13_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8))
#define MOTO14_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
#ifdef E692_STEPMOTOR_EX

#define MOTO15_ZERO_INPUT()			(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))
#define MOTO16_ZERO_INPUT()			(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))

#else

#define MOTO15_ZERO_INPUT()	(!1)		/*预留*/
#define MOTO16_ZERO_INPUT()	(!1)	      /*预留*/

#endif

#define SW1_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_5))
#define SW2_INPUT()		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_8))


#ifdef YARN_ZERO_CHECK
#define YARN0_ZERO_INPUT()		MOTO7_ZERO_INPUT()
#define YARN1_ZERO_INPUT()		SW1_INPUT()
#define YARN2_ZERO_INPUT()		MOTO8_ZERO_INPUT()
#define YARN3_ZERO_INPUT()		SW2_INPUT()
#define YARN4_ZERO_INPUT()		(!1)		/*预留*/
#define YARN5_ZERO_INPUT()		(!1)		/*预留*/
#define YARN6_ZERO_INPUT()		(!1)		/*预留*/
#define YARN7_ZERO_INPUT()		(!1)		/*预留*/
#endif



#else

#define MOTO1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))
#define MOTO2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9))
#define MOTO3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10))
#define MOTO4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11))
#define MOTO5_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
#define MOTO6_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))

#ifdef E480_BOARD_V10
#define SW1_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
#define SW2_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_8))

#define MOTO9_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
#define MOTO10_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
#define MOTO11_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14))
#define MOTO12_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13))

#define MOTO7_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))
#define MOTO8_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))

#ifdef E499_BOARD_SUPPORT_
#define MOTO13_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))
#define MOTO14_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))
#endif
#endif

#endif

//ALARM_XZ_+24V	

#ifdef E490_V10_BOARD
#define XZ1_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_0))
#else
#define XZ1_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13))
#endif
#define XZ2_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
#define XZ3_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11))
#define XZ4_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3))


//ALARM_SJ_-24V
#define SJ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15))
#ifdef E490_V10_BOARD
#define SJ2_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1))
#else
#define SJ2_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14))
#endif
//ALARM_SZ_-24V

#ifdef E490_V10_BOARD
#define SZ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7))
#endif
//OTHER_IO_INPUT

#define ERROR1_INPUT()			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_7))
#define ERROR2_INPUT() 			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_6))


#define FAULT_NGATIVE24_CURRENT_INPUT()  	(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_8))
#define FAULT_POSITIVE24_CURRENT_INPUT()  	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))

#ifdef E490_V10_BOARD
#define E490_BOARD_VER0()		(GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_2))	
#define E490_BOARD_VER1()		(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
#define E490_BOARD_VER2()		(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
#define E490_BOARD_VER3()		(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10))

#define E490_BOARD_EXP0()		(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10))
#define E490_BOARD_EXP1()		(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11))


#define E490_TZSTEP_CS()		(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15))

#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
#define E693_TEN_NS_CS()		SJ1_NEGATIVE24V_ALARM_INPUT()
#endif
	
#endif

//BoaedID_input
#define BOARD_ID_INPUT_0()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
#define BOARD_ID_INPUT_1()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
#define BOARD_ID_INPUT_2()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
#define BOARD_ID_INPUT_3()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))

#define GET_PB9_ST()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))

//Other_IO_output


#define POWER_XZ_POSITIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_4)))
#define POWER_XZ_NEGATIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_7)))

#ifdef E490_V10_BOARD

#define POWER_EXT_3V3_OUTPUT(x)  			((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_1)))

#else
#define POWER_EXT_3V3_OUTPUT(x)  			((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_6)))

#endif
//
//#define LED_OUTPUT(x)  						((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_12)))
#define LED_OUTPUT(x)  						((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_2)))
#define XZ_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_9)))
#define SJ_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_15)))
#define CPLD_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_4)))

#define FAN_CTR_OPEN(x)					((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_13)))	
#define FAN_JC_INPUT()					(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_12))

//SZ_output
#ifdef E490_V10_BOARD
#define SZ1_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+0, x))
#define SZ1_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+1, x))
#define SZ2_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+2, x))
#define SZ2_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+3, x))
#define SZ3_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+4, x))
#define SZ3_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+5, x))
#define SZ4_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+6, x))
#define SZ4_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+7, x))
#define SZ5_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+8, x))
#define SZ5_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+9, x))
#define SZ6_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+10, x))
#define SZ6_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+11, x))
#define SZ7_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+12, x))
#define SZ7_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+13, x))
#define SZ8_Q0_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+14, x))
#define SZ8_Q1_OUTPUT(x) (OUTPUT(ADDR_YARN_DCT+15, x))

#else

#define SZ1_Q0_OUTPUT(x) ((x!=0) ?  (GPIO_SetBits(GPIOA, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_2)))
#define SZ1_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_1)))
#define SZ2_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_6)))
#define SZ2_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_3)))
#define SZ3_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_4)))
#define SZ3_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_7)))
#define SZ4_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_0)))
#define SZ4_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_10)))
#define SZ5_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_7)))
#define SZ5_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_1)))
#define SZ6_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_9)))
#define SZ6_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_8)))
#define SZ7_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_11)))
#define SZ7_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_10)))
#define SZ8_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_14)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_14)))
#define SZ8_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_13)))
#endif

//选针
#define EXT_OVERLOAD_JQD1_P()	XZ1_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD2_P()	XZ2_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD3_P()	XZ3_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD4_P()	XZ4_POSITIVE24V_ALARM_INPUT() 

//三角
 #define EXT_OVERLOAD_ACT1()	SJ1_NEGATIVE24V_ALARM_INPUT()
 #define EXT_OVERLOAD_ACT2() 	SJ2_NEGATIVE24V_ALARM_INPUT()
//纱嘴
 //#define EXT_OVERLOAD_YARN()	SZ1_NEGATIVE24V_ALARM_INPUT()


#define SHOCK_INPUT()			(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))
#define SHOCK_ENABLE(x)			((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define SHOCK_PULS_OUTPUT(x)			((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_8)))

#ifdef E480_BOARD_V10
#define SWITCH_YARN_BOARD() 	(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15))
#endif

#ifdef ALARM_SHOCK_DOUBLE
#define SHOCK_DOUBLE_INPUT() 	(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10))
#define SHOCK_2_INPUT()			(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))
#define SHOCK_2_ENABLE(x)			((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_6)))
#define SHOCK_2_PULS_OUTPUT(x)			((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_15)))
	
#endif



//Ecode_IO_OUTPUT
#define ECODE_SELECT_CODE_1(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define ECODE_SELECT_CODE_2(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))
#define ECODE_SELECT_CODE_3(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOB, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_8)))
#define ECODE_SELECT_CODE_4(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOD, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_1)))
#define ECODE_SELECT_CODE_5(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define ECODE_SELECT_CODE_6(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOD, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_6)))
#define ECODE_SELECT_CODE_7(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOC, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_12)))
#define ECODE_SELECT_CODE_8(x)		 ((x!=0) ?  (GPIO_SetBits(GPIOD, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_15)))


#define E495_OUT1(x) 		((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define E495_OUT2(x) 		((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))
#define E495_OUT3(x) 		((x!=0) ?  (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define E495_OUT4(x) 		((x!=0) ?  (GPIO_SetBits(GPIOD, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_6)))



#ifdef E692_STEPMOTOR_EX
#define MOTO_EX1_RESET_OUTPUT()
#define MOTO_EX2_RESET_OUTPUT()

#define MOTO_EX1_ENABLE_OUTPUT(x)	((x==0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_10)))
#define MOTO_EX2_ENABLE_OUTPUT(x)	((x==0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_10)))

#define MOTO_EX1_ZERO_IN()			(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))
#define MOTO_EX2_ZERO_IN()			(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))

#define MOTO_EX1_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define MOTO_EX2_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_6)))

#define MOTO_EX1_PULSE_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define MOTO_EX2_PULSE_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))

#define MOTO_EX1_HALF_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_11)))
#define MOTO_EX2_HALF_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_3)))

#define TANZHEN_EX_IN()				(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))

#endif


#endif

#endif

