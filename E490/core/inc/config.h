#ifndef __CONFIG_H__
#define __CONFIG_H__

//#define SHIELD_LIFT_ALARM	/*20221125 ����汾����������������*/


//#define STEP_CUR_HALF_RUN 

//#define READ_FLASH_DATA_FOR_BANDING	/*��ȡ����Ϣ*/

	//#define CX_FEET_INPUT_NC	/*����ѹ�Ŵ������źų���*/

	//#define LX_ACT_SPECIAL	/*��������汾--�����������������ʧ��������*/

	//#define TEST_EMC_711_PN	/*20181218 by hlc �ɿ���ʵ���Ҳ�������711����*/

	#define FLASH_READ_PROTECTED		/*2018 12 13 flash ������ʹ��*/

	#define E490_BOOT_ID	0x4900

	#define E693_TEN_BLADE_NEEDLE_SELECTOR		/*֧��10��ѡ����*/

	#define BOOT_UPGRADE_CHECK_CRC

	#define WRTEFLASH_BASE	20


	/*20220209 ������ֶ�Ŀ�������������Ӽ��ٲ���*/
	//#define FOR_BW_DM_SLOWLYRUN 


	//#define DEBUG_ALERT_65	/*ǿ¡�������һ������*/

	#define JQD_MAX_TIMER_US	10000 /*20181204 ѡ�������ͨ��ʱ������Ϊ10ms*/
	#define JQD_MAX_TIMER_FAST_US	999    /*2018 12 04 ����ѡ���������ͨ��ʱ�䣬�������ʾ�ǿ���ѡ�뻹������ѡ����*/ 


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
	/*//8 //10    //hlc �߼������֧��10�� �������ηֵ�������С������ȥ��*/
	#define MAX_BLADE  10	
 	#endif
	#ifndef MAX_BLADE_HARDWARE
		#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
		/*  //hlc ���������֧��10��*/
			#define MAX_BLADE_HARDWARE  MAX_BLADE_HARDWARE_10	 
		#else		
		/*  //hlc ���������֧��8��*/
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


#define LOG_DEBUG_FOR_LX_AT_CH		/*byhlc 20180709�ڳκ��������˸��ֵ������������־����*/


#define DC_ERR_FN_ALERT	/*����˿������ʱ��ֱ�ӱ�F1����2*/

#define LX_JQD_MASK_ENABLE		/*���˵�ѡ����������������ι��ܸù��ܼ�����ͨ����*/



#define JQD_ISR_TIMER_CNT_50US  /*ͨ��50us��ʱ���ļ�����ʵ��ѡ�����嵶����*/



#ifndef JQD_ISR_TIMER_CNT_50US
#define JQD_NEXT_DO_FIFO 
#endif

#define JQD_ISR_TIMER_50US	/*20181201 ѡ������ʱ���ĳ�50us*/


#define JQD_ISR_TIMER_ONE /*20181203 ѡ������ʱ���ĳɶ�������*/

#ifdef JQD_NEXT_DO_FIFO
#define FILTER_1ST_2ND_JQD_OVERLOAD	/*����ǰ���α���*/

#define JQD_IS_FAST_MODE_2_STEPS   /*2018 12 08 ѡ���������δ�*/

#endif
//#define STEP_MOTOR_RUN_HALF_MODE		1

//#define DEBUG_STEP_DDM_CNT			

//#define NOTALART_ACTSTEP

//#define JQD_XZQ_NEEDLE_REDO_MODE		1


//#define HF_		/*�Ʒ�Ļ�����ģʽ����Ӧ����2����Ŀ�����˼�����*/


#define LX_SET_			/*��������汾*/


//#define LX_DEBUG	

#ifdef LX_SET_

//#define LX_DM_SPECIAL_

#endif


#define DMSTEP_MOVERTOZERO_DEC_ADV	/*��Ŀ��������ʱ����ǰ���ٵ���λ�ٶ�ֵ*/

//#define TEST_CX_ACT  /* ���Դ��ǵĶ��������λȱ�ڸĹ���*/


//#define LX_DM_TEST		/*�������˶�Ŀ�����ֻ��һ����������*/

//#define STEP_CURR_DEF	(1)   /*�������д��*/


//#define LX_ACTION_STEP_SPECIAL    /*���˵Ķ�������Ƚ�����*/

//#define LX_SK_SPEED_5K5	

#define LX_JQD_CHECK_ENABLE	/*ѡ�������µ�ͨ�������*/

#define YARN_OPT_CHECK_ENABLE		/*ɳ���������µ�ͨ���*/

#define ACT_OPT_CHECK_ENABLE		/*������������µ�ͨ���*/

//#define DEBUG_STEP_RESET_FAST_SPEED	/*byhlc ���Ե�����ٹ��㵼��ʧ������������һֱ��*/
//#define TRY_ZERO_MIDD		/*�궨�壬֧�֣������λ���м䣬������Ұڶ�Ѱ����λ*/

//#define ACT_OVERLOAD_ALERT 


//#define DEBUG_STEP_OPERATE_GONO_ERROR	/*���������������������*/
//#endif

//#define JQD_DO_CAN_ISR	/*ѡ�붯����CAN�ж�����*/

#define CAN_SEND_ISR_ENABLE	/*CAN����ʹ�÷���FIFO,���ж���ʵ�ַ���*/

//#define TEST_STEPMOTOR_AUTOREPORT	

#define JQD_ISR_CNT_ENABLE

//#define CHECK_XZQ_ISR_INIT	/*���ѡ��������ͳ�ʼ�������Ƿ������һ��*/

//#define NEW_ALARM_STYLE

#define CHECK_CAN_CNT		/*���CAN�����Ƿ���ȷ*/

//#define CHECK_24_ALERT_		/*�ж�����24������˿ʧЧ�ı���*/

//#define NOCHECK_ASC711FLAUT_ISR	/*����711оƬ�ı����ź�*/

//#define BOOT_CODE_JOIN_TEST_CODE	(1)

#define ENCODER_SUPPORT	

//#define TZ_NOT_HALF_	(1)

#define QL_ 

//#define QL_STEPMOTOR_START_LOW			//��ʾ��ǿ¡�İ汾

//#define NO_CHECKINPUT_FOR_LXACT_REALTIME  /*20210713 ����汾��ȡ���������ʵʱ��鹦��*/
//#define PWM_ECODE_ERROR_AUTO_ADJ

#ifdef QL_STEPMOTOR_START_LOW

//#define CODER_ALERT_DEF_0		/*ǿ¡Ĭ�ϱ�����ʧ������Ϊ0*/

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

/*���״̬�Զ��ϱ�����2018-1-2 by hlc*/
//#define MOTOR_ST_AUTO_REPORT		

#endif



//#define TZ_LOST_ALARM_STEPS_50

//#define LX_DM_MOVETOZERO_FAST_SPEED	/*�������˶�Ŀ��������ʱ����ٻ���*/

//#define CX_DM_CURRENT_DEFAULT_1800  /*������Ƕ�Ŀ�����Ĭ�ϵ���1800*/

//#define QL_DEBUG_STEP_MOTOR_ECORD

//#define YF_

#ifdef YF_

//#define ACT_ALARM_ALWAYS

#endif

//#define STEP_DEBUG_HZ

//#define GUOLV_SANJIAO_ALARM

#define E480_BOARD_V10		//Ӳ���汾1.0

#ifdef E480_BOARD_V10

#define ZERO2_SUPPORT

//#define ACT_USE_8803	

#endif

/*֧�ֶ�̬��Ŀ*/
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
	//(4000)   //20161225 �ĳ�5K��֮ǰһֱ��4K
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

#define ACT_SLOW1_SPEED_LX		(1800)			/*���˶���������г̵�ʱ����������*/
#define ACT_SLOW2_SPEED_LX		(2700)			/*���˶���������г̵�ʱ����������*/

#define ACT_SLOW3_SPEED_LX		(3600)			/*���˶���������г̵�ʱ����������*/
#define ACT_SLOW4_SPEED_LX		(4250)
#define ACT_SLOW5_SPEED_LX		(5000)

#ifdef QL_STEPMOTOR_START_LOW

#ifdef DM_START_2K_HZ
#define STEP_START_SPEED_HZ_DM	(2000)  
#else
#define STEP_START_SPEED_HZ_DM	(1100)     //(2000)   //ǿ¡Ҫ���Ŀ�����ٶ�Ϊ1.5K
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

#define STEP_START_SPEED_HZ_DM	(1500)	//(1500)     //(2000)   //ǿ¡Ҫ���Ŀ�����ٶ�Ϊ1.5K
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


//#define DO_XZQ_ALL_2STEP	(1)  //����ѡ����ȫ��ȫ�µ�ʱ�� ��4��4������ //20141231
//#endif
#ifdef  DO_XZQ_ALL_2STEP
	#define XZQ_KEEP_TIME_INDEX	(0)
	#define XZQ_STS_INDEX	(1)
	#define XZQ_KEEPTIME_STS_CNT	(2)
	#define XZQ_SECOND_STEP_TIME_DEFAULT (80)//  20ms   ��ʱ��250usΪ��λ
	
#endif


#define JQD_INIT_DELAY_ENABLE 	(1)

		

//#define LX_ACT_DONOT_ADJ   /*���˶������������*/



#endif

