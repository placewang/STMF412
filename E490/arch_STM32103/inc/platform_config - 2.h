#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

#include "Config.h"
#include "Stm32f2xx_gpio.h"

#define MAX_JACQUARD		8
#define MAX_BLADE		10
/*  //实际机头箱上面的选针器段数 只有8段 另外2段在别的地方*/
#define MAX_BLADE_HARDWARE	8      
#define MAX_YARN		8
#define MAX_ACTEMF		12

#ifndef FEET_SUPPORT_4
#define FEET_SUPPORT_4
#endif

//#define SUPPORT_1S
//#define SUPPORT_2S
//#define ACTAUTOCLEAR
//#define JQDAUTOCLEAR

#ifdef SUPPORT_1S
  //#define E600_BOARD		// for 1-SYS CPUBoard
  //#define E535_BOARD		// for 1-SYS Jacquard
  //#define E591_BOARD		// for 1-SYS MainBoard
 // #define E592_BOARD		// for 1-SYS MotorBoard


	

 // #define E457_BOARD	
  //#define E475_BOARD



#define E475_480_BOARD

#define E_BOARD_V10		//硬件版本1.0

#define ALARM_SHOCK_DOUBLE	// 支持两个撞针信号



#define MAX_DC24_COUNT		(32)
#define MAX_DC24_DATA_COUNT (72)


  #undef MAX_YARN
  #define MAX_YARN	8

  #undef MAX_ACTEMF
  #define MAX_ACTEMF	12

  #undef MAX_JACQUARD
  #define MAX_JACQUARD	8
#endif	// end SUPPORT_1S

#ifdef SUPPORT_2S
  //#define E534_BOARD		// for 2-SYS CPUBoard
  //#define E535_BOARD		// for 2-SYS Jacquard
  //#define E536_BOARD		// for 2-SYS MotorBoard
#endif	// end SUPPORT_2S

//#define WDG_ENABLE

#define 	DCT_HV_OUT_ENABLE    	(0x40)
#define 	DCT_HV_10MS       	(0x40)
#define 	DCT_HV_12MS       	(0x41)
#define 	DCT_HV_14MS       	(0x42)
#define 	DCT_HV_16MS       	(0x43)
#define 	DCT_HV_18MS       	(0x44)
#define 	DCT_HV_20MS       	(0x45)
#define 	DCT_HV_22MS       	(0x46)
#define 	DCT_HV_24MS       	(0x47)
#define 	DCT_HV_26MS       	(0x48)
#define 	DCT_HV_28MS       	(0x49)
#define 	DCT_HV_30MS       	(0x4A)
#define 	DCT_HV_32MS       	(0x4B)
#define 	DCT_HV_34MS       	(0x4C)
#define 	DCT_HV_36MS       	(0x4D)
#define 	DCT_HV_38MS       	(0x4E)
#define 	DCT_HV_40MS       	(0x4F)

#define		XZQ_HV_OUT_ENABLE	(0x30)
#define		XZQ_HV_1200us		(0x30)
#define		XZQ_HV_1300us		(0x31)
#define		XZQ_HV_1400us		(0x32)
#define		XZQ_HV_1500us		(0x33)
#define		XZQ_HV_1600us		(0x34)
#define		XZQ_HV_1700us		(0x35)
#define		XZQ_HV_1800us		(0x36)
#define		XZQ_HV_1900us		(0x37)
#define		XZQ_HV_2000us		(0x38)

#define BOOT_START_ADDRESS		((u32)0x08000000)
#define APP_START_ADDRESS		((u32)0x08020000)

#define Bank1_SRAM1_ADDR		(0x60000000L)

#define ADDR_MAINBOARD			(0x00000000L)
#define ADDR_YARN_DCT			(0x00000000L)
#define ADDR_JQDBOARD1			(0x00200000L)
#define ADDR_ACTBOARD1			(0x00200020L)
#define ADDR_JQDBOARD2			(0x00400000L)
#define ADDR_ACTBOARD2			(0x00400020L)
//#define ADDR_STEPBOARD		(0x00800000L)
#define ADDR_STEPBOARD			(0x00600000L)

#ifdef E536_BOARD
  #define ADDR_STEP_EN			(ADDR_STEPBOARD + 0x5F)
  #define ADDR_STEP_DIR			(ADDR_STEPBOARD + 0x40)
  #define ADDR_STEP_PLUSE		(ADDR_STEPBOARD + 0x41)
  #define ADDR_STEP_HALF		(ADDR_STEPBOARD + 0x42)
  #define STEP_ADD			3/*5*/
  
//#ifdef FEET_SUPPORT_4
  #define STEP_NUM			12
//#else
//  #define STEP_NUM			10
//#endif
  
#endif	// end E536_BOARD

#ifdef E592_BOARD
  #define ADDR_STEP_EN			(ADDR_STEPBOARD + 0x5F)
  #define ADDR_STEP_DIR			(ADDR_STEPBOARD + 0x40)
  #define ADDR_STEP_PLUSE		(ADDR_STEPBOARD + 0x41)
  #define ADDR_STEP_HALF		(ADDR_STEPBOARD + 0x42)
  #define STEP_ADD			3
//#ifdef FEET_SUPPORT_4
  #define STEP_NUM			12
//#else
//  #define STEP_NUM			10
//#endif
#endif //end E592_BOARD

#ifdef E457_BOARD
  #define STEP_NUM			6
#endif

#ifdef E475_480_BOARD
  #define STEP_NUM			6
#endif



#define OUTPUT(addr, Data)	(*(volatile unsigned char *)(Bank1_SRAM1_ADDR + (addr))) = Data
#define INPUT(addr)		(*(volatile unsigned char *)(Bank1_SRAM1_ADDR + (addr)))

/* input */
#define STEP1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))
#define STEP2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13))
#define STEP3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14))
#define STEP4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15))
#define STEP5_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2))
#define STEP6_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))
#define STEP7_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8))
#define STEP8_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9))
#ifdef SUPPORT_2S
//#ifdef FEET_SUPPORT_4 //单系统压脚，需改这里的引脚定义
#define FEET_SINK1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))
#define FEET_SINK3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
#define FEET_SINK4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
#ifdef FEET_SUPPORT_OLD_SENSOR
#define FEET_SINK2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
#define FEET_STATUS_DIR_INPUT2()	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))
#else
#define FEET_SINK2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))
#define FEET_STATUS_DIR_INPUT2()	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
#endif
#define FEET_STATUS_DIR_INPUT1()	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
#define FEET_STATUS_DIR_INPUT3()	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
#define FEET_STATUS_DIR_INPUT4()	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4))

//#else
#define SINK1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))
#define SINK2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))
#define SINK3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
#define SINK4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))

#define STATUS_DIR_INPUT1()	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))
#define STATUS_DIR_INPUT2()	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))
//#endif
#endif
#ifdef SUPPORT_1S
#define SINK1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))
#define SINK2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))
#define SINK3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
#define SINK4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))

#define STATUS_DIR_INPUT1()	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))
#define STATUS_DIR_INPUT2()	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))

#endif


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

#ifdef SUPPORT_2S
 #define JQD1_ERR1_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
 #define JQD1_ERR2_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
 #define JQD2_ERR1_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
 #define JQD2_ERR2_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
 #define STEP_ERR_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
 #define V24_PERR_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10))
 #define V12_PERR_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))
 #define STATUS_LINE1_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
 #define STATUS_LINE2_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
 #define EXT_OVERLOAD_YARN()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0))
 #define EXT_OVERLOAD_JQD1()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))
 #define EXT_OVERLOAD_JQD2()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
#endif
#endif


// 
//#define BOARD_MODE_INPUT()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2))


//#define STATUS_KEY_INPUT()	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
/* end input */

#if 0
// for E534 1.0 
#define SSI_READ_DATA()		(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
#define SSI_CLK_LOW()		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#define SSI_CLK_HIGHT()		GPIO_SetBits(GPIOA, GPIO_Pin_2);
#else
// for E534 2.0 
#define SSI_READ_DATA()		(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))
#define SSI_CLK_LOW()		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
#define SSI_CLK_HIGHT()		GPIO_SetBits(GPIOA, GPIO_Pin_5);
#endif


#define SSI_Select(idx)		OUTPUT(0x40 + idx, 0)
#define SSI_DeSelect(idx)	OUTPUT(0x4F, 0)

//#define COMMAND_PWR	0xF8
#define COMMAND_PWR	0x80

// 0: 子板1
// 1: 子板2
// 2: 子板3
// 3: YARN overload2
// 4: YARN overload1
#define OVERLOAD_STATUS		(0x93)

#define OVERLOAD_MASK_BOARD1	0x01
#define OVERLOAD_MASK_BOARD2	0x02
//#define OVERLOAD_MASK_BOARD3	0x04
#ifdef E600_BOARD
#define OVERLOAD_MASK_YARN	0x10
#endif
#ifdef E534_BOARD
#define OVERLOAD_MASK_YARN	0x18
#define E534_YARN_ACT_ERR_REPORT
#endif

#ifdef E457_BOARD


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



#define OVERLOAD_MASK_XZ1_P	(0x00000001)
#define OVERLOAD_MASK_XZ2_P	(0x00000002)
#define OVERLOAD_MASK_XZ3_P	(0x00000004)
#define OVERLOAD_MASK_XZ4_P	(0x00000008)
#define OVERLOAD_MASK_XZ1		(0x00000010)
#define OVERLOAD_MASK_XZ2		(0x00000020)
#define OVERLOAD_MASK_XZ3		(0x00000040)
#define OVERLOAD_MASK_XZ4		(0x00000080)
#define OVERLOAD_MASK_ACT1	(0x00000100)
#define OVERLOAD_MASK_ACT2	(0x00000200)
//
//
#define OVERLOAD_MASK_YARN	(0x00001000)




#define OVERLOAD_MASK	(OVERLOAD_MASK_XZ1_P | OVERLOAD_MASK_XZ2_P | OVERLOAD_MASK_XZ3_P |OVERLOAD_MASK_XZ4_P \
						| OVERLOAD_MASK_XZ1 | OVERLOAD_MASK_XZ2 | OVERLOAD_MASK_XZ3 | OVERLOAD_MASK_XZ4 \
						| OVERLOAD_MASK_ACT1 | OVERLOAD_MASK_ACT2 | OVERLOAD_MASK_YARN )
						
#endif

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

#define OVERLOAD_MASK_AD_24_N	(0x00004000)
#define OVERLOAD_MASK_AD_24_P	(0x00008000)




#define OVERLOAD_MASK	(OVERLOAD_MASK_24V_C_N | OVERLOAD_MASK_24V_C_P | OVERLOAD_MASK_RESERVE1 |OVERLOAD_MASK_RESERVE2 \
						| OVERLOAD_MASK_XZ1 | OVERLOAD_MASK_XZ2 | OVERLOAD_MASK_XZ3 | OVERLOAD_MASK_XZ4 \
						| OVERLOAD_MASK_ACT1 | OVERLOAD_MASK_ACT2 | OVERLOAD_MASK_YARN |OVERLOAD_MASK_AD_24_N|OVERLOAD_MASK_AD_24_P)

#define OVERLOAD_MASK_NOYARN	(OVERLOAD_MASK_XZ1 | OVERLOAD_MASK_XZ2 | OVERLOAD_MASK_XZ3 | OVERLOAD_MASK_XZ4 |OVERLOAD_MASK_AD_24_N|OVERLOAD_MASK_AD_24_P)


						
#endif

#ifdef E534_YARN_ACT_ERR_REPORT
#define YARN_ACT_ERR_INPUT()  (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)) 
#endif

//E534-3.3
#define BOARD_CPLD_VER		0xF8
#define GET_CPLD_VER1()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER))
#define GET_CPLD_VER2()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 1))
#define GET_CPLD_VER3()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 2))
#define GET_CPLD_VER4()	(INPUT(ADDR_MAINBOARD + BOARD_CPLD_VER + 3))

#ifdef E457_BOARD

#define STEP_NUM			6


//E457-V00
//MOTO_ALL_RESET
#define MOTOALL_RESET_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_1)))

//MOTO_HALF
#define MOTO1_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_0)))
#define MOTO2_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_13)))
#define MOTO3_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_6)))
#define MOTO4_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_2)))
#define MOTO5_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_13)))
#define MOTO6_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_15)))

//MOTO_DIR
#define MOTO1_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_10)))
#define MOTO2_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define MOTO3_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_4)))
#define MOTO4_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_10)))
#define MOTO5_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_10)))
#define MOTO6_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_10)))

//MOTO_ENABLE
#define MOTO1_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_11)))
#define MOTO2_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_9)))
#define MOTO3_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_3)))
#define MOTO4_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_11)))
#define MOTO5_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_12)))
#define MOTO6_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_3)))


//MOTO_Pulse
#define MOTO1_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_6)))
#define MOTO2_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_7)))
#define MOTO3_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_8)))
#define MOTO4_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_9)))
#define MOTO5_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_6)))
#define MOTO6_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_7)))

//MOTO_ZERO
#define MOTO1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13))
#define MOTO2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8))
#define MOTO3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14))
#define MOTO4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
#define MOTO5_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_8))
#define MOTO6_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))

//MOTO_FAULT
#define MOTO1_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9))
#define MOTO2_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))
#define MOTO3_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_5))
#define MOTO4_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9))
#define MOTO5_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
#define MOTO6_FAULT_INPUT()	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))


//ALARM_XZ_+24V	
#define XZ1_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5))
#define XZ2_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6))
#define XZ3_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_6))
#define XZ4_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))

//ALARM_XZ_-24V
#define XZ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))
#define XZ2_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_8))
#define XZ3_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))
#define XZ4_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0))

//ALARM_SJ_-24V
#define SJ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
#define SJ2_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))

//ALARM_SZ_-24V
#define SZ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))

//OTHER_IO_INPUT
#define SW1_INPUT() 				(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
#define SW2_INPUT() 				(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
#define MOTO7_ZERO_INPUT() 		(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
#define MOTO8_ZERO_INPUT() 		(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
#define ERROR1_INPUT()			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_12))
#define ERROR2_INPUT() 			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_7))
#define IO_NGATIVE24_INPUT()  	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))

//BoaedID_input
#define BOARD_ID_INPUT_0()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
#define BOARD_ID_INPUT_1()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
#define BOARD_ID_INPUT_2()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
#define BOARD_ID_INPUT_3()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))

//Other_IO_output

#define POWER_XZ_POSITIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_4)))
#define POWER_XZ_NEGATIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_7)))
#define POWER_SJ_NEGATIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_9)))
#define POWER_SZ_NEGATIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_10)))
#define POWER_EXT_3V3_OUTPUT(x)  			((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_3)))
#define LED_OUTPUT(x)  						((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_2)))
#define CPLD_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_4)))


//SZ_output
#define SZ1_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_2)))
#define SZ1_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_1)))
#define SZ2_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_2)))
#define SZ2_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_3)))
#define SZ3_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_6)))
#define SZ3_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_7)))
#define SZ4_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_4)))
#define SZ4_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_11)))
#define SZ5_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_13)))
#define SZ5_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_14)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_14)))
#define SZ6_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_15)))
#define SZ6_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_0)))
#define SZ7_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_1)))
#define SZ7_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_7)))
#define SZ8_Q0_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_8)))
#define SZ8_Q1_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_9)))


//选针
#define EXT_OVERLOAD_JQD1_P()	XZ1_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD2_P()	XZ2_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD3_P()	XZ3_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD4_P()	XZ4_POSITIVE24V_ALARM_INPUT() 
#define EXT_OVERLOAD_JQD1()	XZ1_NEGATIVE24V_ALARM_INPUT() 
#define EXT_OVERLOAD_JQD2()	XZ2_NEGATIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD3()	XZ3_NEGATIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD4()	XZ4_NEGATIVE24V_ALARM_INPUT()
//三角
 #define EXT_OVERLOAD_ACT1()	SJ1_NEGATIVE24V_ALARM_INPUT()
 #define EXT_OVERLOAD_ACT2() 	SJ2_NEGATIVE24V_ALARM_INPUT()
//纱嘴
 #define EXT_OVERLOAD_YARN()	SZ1_NEGATIVE24V_ALARM_INPUT()


#define SHOCK_INPUT()			(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10))
#define SHOCK_ENABLE(x)			((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_11)))

#define SHOCK_PULS_OUTPUT(x)			((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_8)))


#endif

#ifdef E475_480_BOARD

#define STEP_NUM			6


//E457-V00
//MOTO_ALL_RESET
#define MOTOALL_RESET_OUTPUT(x) ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_9)))

//MOTO_HALF
#define MOTO1_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_0)))
#define MOTO2_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_5)))
#define MOTO3_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_4)))
#define MOTO4_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_2)))
#define MOTO5_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_10)))
#define MOTO6_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_11)))
#define MOTO7_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_12)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_12)))
#define MOTO8_HALF_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_4)))



//MOTO_DIR
#define MOTO1_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_10)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_10)))
#define MOTO2_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_13)))
#define MOTO3_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOA, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOA, GPIO_Pin_8)))
#define MOTO4_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_3)))
#define MOTO5_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_11)))
#define MOTO6_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_8)))
#define MOTO7_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_9)))
#define MOTO8_DIR_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOG, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOG, GPIO_Pin_5)))

//MOTO_ENABLE
#define MOTO1_8_ENABLE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_6)))


//MOTO_Pulse
#define MOTO1_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_6)))
#define MOTO2_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_7)))
#define MOTO3_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_8)))
#define MOTO4_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_9)))
#define MOTO5_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_6)))
#define MOTO6_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_7)))
#define MOTO7_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_13)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_13)))
#define MOTO8_PULSE_OUTPUT(x)  ((x!=0) ? (GPIO_SetBits(GPIOC, GPIO_Pin_11)) :(GPIO_ResetBits(GPIOC, GPIO_Pin_11)))

//MOTO_USM01
#define MOTO_DM_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_0)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_0)))
#define MOTO_DM_USM1(x)   ((x!=0) ?  (GPIO_SetBits(GPIOE, GPIO_Pin_1)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_1)))
#define MOTO_SK_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define MOTO_SK_USM1(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))
#define MOTO_SJ_USM0(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_2)))
#define MOTO_SJ_USM1(x)   ((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_3)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_3)))


//MOTO_ZERO
#define MOTO1_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))
#define MOTO2_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9))
#define MOTO3_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10))
#define MOTO4_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11))
#define MOTO5_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
#define MOTO6_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
#define MOTO7_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
#define MOTO8_ZERO_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_8))

#define MOTO5_WORK_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))
#define MOTO6_WORK_INPUT()	(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))


//ALARM_XZ_+24V	
#define XZ1_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13))
#define XZ2_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
#define XZ3_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11))
#define XZ4_POSITIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3))



//ALARM_SJ_-24V
#define SJ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15))
#define SJ2_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14))

//ALARM_SZ_-24V
#define SZ1_NEGATIVE24V_ALARM_INPUT() (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6))

//OTHER_IO_INPUT
#define SW1_INPUT() 				(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
#define SW2_INPUT() 				(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_8))
//#define MOTO7_ZERO_INPUT() 		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15))
//#define MOTO8_ZERO_INPUT() 		(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14))
#define ERROR1_INPUT()			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_7))
#define ERROR2_INPUT() 			(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_6))
#define FAULT_NGATIVE24_CURRENT_INPUT()  	(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_8))
#define FAULT_POSITIVE24_CURRENT_INPUT()  	(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))



//BoaedID_input
#define BOARD_ID_INPUT_0()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
#define BOARD_ID_INPUT_1()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
#define BOARD_ID_INPUT_2()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
#define BOARD_ID_INPUT_3()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))

//Other_IO_output

#define POWER_XZ_POSITIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOE, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOE, GPIO_Pin_4)))
#define POWER_XZ_NEGATIVE24V_OUTPUT(x)  	((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_7)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_7)))
#define POWER_EXT_3V3_OUTPUT(x)  			((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_6)))
#define LED_OUTPUT(x)  						((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_2)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_2)))
#define XZ_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_9)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_9)))
#define SJ_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOF, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOF, GPIO_Pin_15)))
#define CPLD_RESET_OUTPUT(x)				((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_4)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_4)))



//SZ_output
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


//选针
#define EXT_OVERLOAD_JQD1_P()	XZ1_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD2_P()	XZ2_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD3_P()	XZ3_POSITIVE24V_ALARM_INPUT()
#define EXT_OVERLOAD_JQD4_P()	XZ4_POSITIVE24V_ALARM_INPUT() 

//三角
 #define EXT_OVERLOAD_ACT1()	SJ1_NEGATIVE24V_ALARM_INPUT()
 #define EXT_OVERLOAD_ACT2() 	SJ2_NEGATIVE24V_ALARM_INPUT()
//纱嘴
 #define EXT_OVERLOAD_YARN()	SZ1_NEGATIVE24V_ALARM_INPUT()


#define SHOCK_INPUT()			(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))
#define SHOCK_ENABLE(x)			((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_5)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_5)))
#define SHOCK_PULS_OUTPUT(x)			((x!=0) ? (GPIO_SetBits(GPIOB, GPIO_Pin_8)) :(GPIO_ResetBits(GPIOB, GPIO_Pin_8)))

#ifdef ALARM_SHOCK_DOUBLE

#define SHOCK_DOUBLE_INPUT() 	(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10))
#define SWITCH_YARN_BOARD() 	(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15))


#define SHOCK_2_INPUT()			(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))
#define SHOCK_2_ENABLE(x)			((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_6)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_6)))
#define SHOCK_2_PULS_OUTPUT(x)			((x!=0) ? (GPIO_SetBits(GPIOD, GPIO_Pin_15)) :(GPIO_ResetBits(GPIOD, GPIO_Pin_15)))
	
#endif


#endif

#endif

