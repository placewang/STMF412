#ifndef __CONFIG_H__
#define __CONFIG_H__

//#define SHIELD_LIFT_ALARM	/*20221125 特殊版本，屏蔽推针电机报警*/


//#define STEP_CUR_HALF_RUN 

//#define READ_FLASH_DATA_FOR_BANDING	/*读取绑定信息*/

	//#define CX_FEET_INPUT_NC	/*慈星压脚传感器信号常闭*/

	//#define LX_ACT_SPECIAL	/*连兴特殊版本--处理动作电机负数回零失步的问题*/

	//#define TEST_EMC_711_PN	/*20181218 by hlc 可靠性实验室测试屏蔽711报警*/

	#define FLASH_READ_PROTECTED		/*2018 12 13 flash 读保护使能*/

	#define E490_BOOT_ID	0x4900

	#define E693_TEN_BLADE_NEEDLE_SELECTOR		/*支持10段选针器*/

	#define BOOT_UPGRADE_CHECK_CRC

	#define WRTEFLASH_BASE	20


	/*20220209 定义必沃度目电机慢启动，多加减速步数*/
	//#define FOR_BW_DM_SLOWLYRUN 


	//#define DEBUG_ALERT_65	/*强隆程序调试一个报警*/

	#define JQD_MAX_TIMER_US	10000 /*20181204 选针器最大通电时间设置为10ms*/
	#define JQD_MAX_TIMER_FAST_US	999    /*2018 12 04 快速选针器的最大通电时间，拿这个表示是快速选针还是慢速选针器*/ 


	#ifndef CLR_JQD_DELAY_TIME
	#define CLR_JQD_DELAY_TIME	50	//500
	#endif

	#ifndef CLR_EMF_DELAY_TIME
	#define CLR_EMF_DELAY_TIME	150	//500
	#endif

	#ifndef DCT_TIME
	#define DCT_TIME	21 //  40-42ms  ()				//15	// 30ms
	#endif
	
	#ifndef XZQ_TIME
	#define XZQ_TIME	5	// 5/4=1.25ms
	#endif

	#ifndef XZQ_TIME_2
	#define XZQ_TIME_2	5
	#endif

	#define XZQ_TIME_TEST	10	// 10/4=2.5ms
	#define EMF_TIME_TEST	15   // 15*2=30 ms
	#define EMF_ACT_TIME_TEST	10   // 10*2=20 ms
	
	#define PULSE_LOW	32	// 100us
	#define PULSE_START	32	// 100US    

	
	#define ADD_DEC_STEP	15    
	#define STEP_LOW_SPD	156
	#define STEP_MAX_SPD	16//32//40//43
	#define SK_MAX			2000
	#define SK_MAX_SPEED	 60/*32*/

	#define ACT_MAX_SPEED	 60/*32*/

	#define YARN_MAX_SPEED	 60/*32*/

	#define MAX_BLADE_HARDWARE_8 		8
	#define MAX_BLADE_HARDWARE_10		10
	
	
	#ifndef MAX_STEPS
	#define MAX_STEPS	2000L
	#endif
	#ifndef MAX_BLADE
	/*//8 //10    //hlc 逻辑上最大支持10段 另外两段分到单独的小板子上去了*/
	#define MAX_BLADE  10	
 	#endif
	#ifndef MAX_BLADE_HARDWARE
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		/*  //hlc 物理上最大支持10段*/
			#define MAX_BLADE_HARDWARE  MAX_BLADE_HARDWARE_10	 
		#else		
		/*  //hlc 物理上最大支持8段*/
			#define MAX_BLADE_HARDWARE  MAX_BLADE_HARDWARE_8	 
		#endif
 	#endif

	
	#ifndef MAX_JACQUARD
	#define MAX_JACQUARD	8
	#endif
	#ifndef MAX_YARN
	#define MAX_YARN	8
	#endif
	#ifndef MAX_ACTEMF
	#define MAX_ACTEMF	12
	#endif

	#ifndef SINKER_BASE
	#define SINKER_BASE	4
	#endif


#define LOG_DEBUG_FOR_LX_AT_CH		/*byhlc 20180709在澄海调试连兴各种电机报警增加日志功能*/


#define DC_ERR_FN_ALERT	/*保险丝报警的时候直接报F1，或2*/

#define LX_JQD_MASK_ENABLE		/*连兴的选针器测试命令加屏蔽功能该功能兼容普通命令*/



#define JQD_ISR_TIMER_CNT_50US  /*通过50us定时器的计数来实现选针器清刀功能*/



#ifndef JQD_ISR_TIMER_CNT_50US
#define JQD_NEXT_DO_FIFO 
#endif

#define JQD_ISR_TIMER_50US	/*20181201 选针器定时器改成50us*/


#define JQD_ISR_TIMER_ONE /*20181203 选针器定时器改成动作触发*/

#ifdef JQD_NEXT_DO_FIFO
#define FILTER_1ST_2ND_JQD_OVERLOAD	/*过滤前两次报警*/

#define JQD_IS_FAST_MODE_2_STEPS   /*2018 12 08 选针器分两段打*/

#endif
//#define STEP_MOTOR_RUN_HALF_MODE		1

//#define DEBUG_STEP_DDM_CNT			

//#define NOTALART_ACTSTEP

//#define JQD_XZQ_NEEDLE_REDO_MODE		1


//#define HF_		/*浩丰的机器，模式设置应该是2，度目加生克加推针*/


#define LX_SET_			/*连兴特殊版本*/


//#define LX_DEBUG	

#ifdef LX_SET_

//#define LX_DM_SPECIAL_

#endif


#define DMSTEP_MOVERTOZERO_DEC_ADV	/*度目电机回零的时候提前减速到复位速度值*/

//#define TEST_CX_ACT  /* 测试慈星的动作电机零位缺口改过了*/


//#define LX_DM_TEST		/*测试连兴度目电机，只做一个方向修正*/

//#define STEP_CURR_DEF	(1)   /*电机电流写死*/


//#define LX_ACTION_STEP_SPECIAL    /*连兴的动作电机比较特殊*/

//#define LX_SK_SPEED_5K5	

#define LX_JQD_CHECK_ENABLE	/*选针器上下导通检查命令*/

#define YARN_OPT_CHECK_ENABLE		/*沙嘴电磁铁上下导通检查*/

#define ACT_OPT_CHECK_ENABLE		/*动作电磁铁上下导通检查*/

//#define DEBUG_STEP_RESET_FAST_SPEED	/*byhlc 调试电机高速归零导致失步报警，或电机一直动*/
//#define TRY_ZERO_MIDD		/*宏定义，支持，电机零位在中间，电机左右摆动寻找零位*/

//#define ACT_OVERLOAD_ALERT 


//#define DEBUG_STEP_OPERATE_GONO_ERROR	/*测试两个命令接续的问题*/
//#endif

//#define JQD_DO_CAN_ISR	/*选针动作在CAN中断里做*/

#define CAN_SEND_ISR_ENABLE	/*CAN数据使用发送FIFO,用中断来实现发送*/

//#define TEST_STEPMOTOR_AUTOREPORT	

#define JQD_ISR_CNT_ENABLE

//#define CHECK_XZQ_ISR_INIT	/*检查选针器命令和初始化命令是否搅合在一起*/

//#define NEW_ALARM_STYLE

#define CHECK_CAN_CNT		/*检查CAN数据是否正确*/

//#define CHECK_24_ALERT_		/*判断正负24伏保险丝失效的报警*/

//#define NOCHECK_ASC711FLAUT_ISR	/*忽略711芯片的报警信号*/

//#define BOOT_CODE_JOIN_TEST_CODE	(1)

#define ENCODER_SUPPORT	

//#define TZ_NOT_HALF_	(1)

#define QL_ 

//#define QL_STEPMOTOR_START_LOW			//表示是强隆的版本

//#define NO_CHECKINPUT_FOR_LXACT_REALTIME  /*20210713 特殊版本，取消动作电机实时检查功能*/
//#define PWM_ECODE_ERROR_AUTO_ADJ

#ifdef QL_STEPMOTOR_START_LOW

//#define CODER_ALERT_DEF_0		/*强隆默认编码器失步报警为0*/

#define DM_START_2K_HZ

	#define SK_STEP_LOST_ALERT_0

	#define PWM_ECODE_ERROR_AUTO_ADJ	

	#ifdef XZQ_TIME
	#undef XZQ_TIME	
	#endif
	#define XZQ_TIME	 5	// 5/4=1.25ms

	#ifdef XZQ_TIME_2
	#undef XZQ_TIME_2
	#endif
	#define XZQ_TIME_2	5
	
#else

//#define SK_STEP_LOST_ALERT_0 

/*电机状态自动上报功能2018-1-2 by hlc*/
//#define MOTOR_ST_AUTO_REPORT		

#endif



//#define TZ_LOST_ALARM_STEPS_50

//#define LX_DM_MOVETOZERO_FAST_SPEED	/*定义连兴度目电机回零的时候快速回零*/

//#define CX_DM_CURRENT_DEFAULT_1800  /*定义慈星度目电机的默认电流1800*/

//#define QL_DEBUG_STEP_MOTOR_ECORD

//#define YF_

#ifdef YF_

//#define ACT_ALARM_ALWAYS

#endif

//#define STEP_DEBUG_HZ

//#define GUOLV_SANJIAO_ALARM

#define E480_BOARD_V10		//硬件版本1.0

#ifdef E480_BOARD_V10

#define ZERO2_SUPPORT

//#define ACT_USE_8803	

#endif

/*支持动态度目*/
#define STEP_MOTOR_DDDM_SUPPORT		(1)	


#define STEP_ALARM_SEND_LOSTSTEPS (1)


#ifdef STEP_MOTOR_DDDM_SUPPORT
	#ifdef STEP_LOW_SPD
	#undef STEP_LOW_SPD
	#endif

	#ifdef STEP_MAX_SPD
	#undef STEP_MAX_SPD
	#endif

	#ifdef ACT_MAX_SPEED
	#undef ACT_MAX_SPEED
	#endif
	#ifdef SK_MAX_SPEED
	#undef SK_MAX_SPEED
	#endif
	#ifdef YARN_MAX_SPEED
	#undef YARN_MAX_SPEED
	#endif	
	#ifdef LIFT_MAX_SPEED
	#undef LIFT_MAX_SPEED
	#endif	
	
	#define STEP_LOW_SPD 		(2000)
	#define STEP_MAX_SPD		(3000)
#ifdef QL_STEPMOTOR_START_LOW
	#define ACT_MAX_SPEED		(2000)
#else
	

#ifdef QL_


	#ifdef YF_
	#define ACT_MAX_SPEED		(3000)
	#else
	#ifdef STEP_CURR_DEF
	#define ACT_MAX_SPEED		(1000)
	#else
	//(4000)   //20161225 改成5K。之前一直是4K
	#define ACT_MAX_SPEED		(4000) //(4000)
	
	#endif
	#endif
	
#else
	
	#define ACT_MAX_SPEED		(3000)
#endif	
#endif

#ifdef LX_SK_SPEED_5K5
	#define SK_MAX_SPEED		(4500)
#else
	#define SK_MAX_SPEED		(3000)
#endif	
	#define YARN_MAX_SPEED	 	(3000)
#ifdef QL_STEPMOTOR_START_LOW
	#define LIFT_MAX_SPEED		(5000)
	#else	
	#define LIFT_MAX_SPEED		(3500)
	#endif
	#define FEET_MAX_SPEED		(3000)

#define ACT_MAX_SPEED_SMP		(2000) //(4000)
#define ACT_BASE_SPEED_SMP		(1000) //(4000)


//#define ACT_MAX_SPEED_LX		(5000)	
#define ACT_MAX_SPEED_LX		(5500)	

#define ACT_SLOW1_SPEED_LX		(1800)			/*连兴动作电机短行程的时候按照慢速跑*/
#define ACT_SLOW2_SPEED_LX		(2700)			/*连兴动作电机短行程的时候按照慢速跑*/

#define ACT_SLOW3_SPEED_LX		(3600)			/*连兴动作电机短行程的时候按照慢速跑*/
#define ACT_SLOW4_SPEED_LX		(4250)
#define ACT_SLOW5_SPEED_LX		(5000)

#ifdef QL_STEPMOTOR_START_LOW

#ifdef DM_START_2K_HZ
#define STEP_START_SPEED_HZ_DM	(2000)  
#else
#define STEP_START_SPEED_HZ_DM	(1100)     //(2000)   //强隆要求度目启动速度为1.5K
#endif
#define STEP_START_SPEED_HZ_SK	(1100)
#define STEP_START_SPEED_HZ_ACT	(1100)
#define STEP_START_SPEED_HZ_ACT_LX	(1100)
#define STEP_START_SPEED_HZ_FEET	(1100)
#define STEP_START_SPEED_HZ_YARN	(1500)
#define STEP_START_SPEED_HZ_LIFT	(1100)
	

#else	

//#define STEP_START_SPEED_HZ_DM	(800) 
 #ifdef  FOR_BW_DM_SLOWLYRUN 
#define STEP_START_SPEED_HZ_DM	(1000)	//(1500) 
 #else

#define STEP_START_SPEED_HZ_DM	(1500)	//(1500)     //(2000)   //强隆要求度目启动速度为1.5K
#endif
#define STEP_START_SPEED_HZ_SK	(1500)
#ifdef QL_

#ifdef STEP_CURR_DEF
#define STEP_START_SPEED_HZ_ACT	(1000)
#else  //(1100)
#define STEP_START_SPEED_HZ_ACT	(1100)
#endif
#else
#define STEP_START_SPEED_HZ_ACT	(1500)
#endif

#define STEP_START_SPEED_HZ_ACT_LX	(1500)

#define STEP_START_SPEED_HZ_FEET	(1500)
#define STEP_START_SPEED_HZ_YARN	(1500)
#define STEP_START_SPEED_HZ_LIFT	(1000)

	
#endif

#endif

#define EMF_YARN_ACT_TEST_USEPWM	(1)

#define APPRUNNING_AFTERBOOT	(1)

#define DEF_FEET_ERR_CHECK_TIME	2

#define DEF_TRIANGLE_ERR_CHECK_TIME (2)


//#define DO_XZQ_ALL_2STEP	(1)  //单个选针器全上全下的时候 分4刀4刀动作 //20141231
//#endif
#ifdef  DO_XZQ_ALL_2STEP
	#define XZQ_KEEP_TIME_INDEX	(0)
	#define XZQ_STS_INDEX	(1)
	#define XZQ_KEEPTIME_STS_CNT	(2)
	#define XZQ_SECOND_STEP_TIME_DEFAULT (80)//  20ms   定时器250us为单位
	
#endif


#define JQD_INIT_DELAY_ENABLE 	(1)

		

//#define LX_ACT_DONOT_ADJ   /*连兴动作电机不修正*/



#endif

