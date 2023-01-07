
 //#define BOOTVER	0x00000002
 //#define BOOTVER	0x00000003
#define BOOTVER_SUPPORT_BINDING	0x00000004
#define BOOTVER	0x00000005


/*�����ʽ�ĳ�����:*/
/*
ͨ�������ֽ�����ʾ����(��ǰ��һ���ֽ�)
���е�һ���ֽڱ�ʾ������࣬�ڶ����ֽڱ�ʾС����
��������ֽڱ�ʾ������
���һ���ֽڱ�ʾindex.(ͨѶ���)

CAN ������8���ֽ�
Data0	Data1	data2	data3	Data4	data5	Data6	Data7
CMDtype	cmd		arg1L	arg1H	arg2L	arg2H	arg3L	index
cmdtype ��Ҫ�ּ�����
1:����
2:ϵͳ����
3:��Ŀ���
4:�������(��ˣ�ѹ�ţ������ȵ�)
5:�����(���Ǻ�ɴ��)
6:ѡ����
7:�������
8:��������
*/

#define CMDTYPE_UNDEFINE			(0x00)
#define CMDTYPE_PERIPHERAL			(0x01)
#define CMDTYPE_SYSSET				(0x02)
#define CMDTYPE_MOTOR_DENSITY		(0x03)
#define CMDTYPE_MOTOR_OTHER		(0x04)
#define CMDTYPE_DCT				(0x05)
#define CMDTYPE_XZQ				(0x06)
#define CMDTYPE_XZQ_EX			(0x16)

#define CMDTYPE_ALARM				(0x07)
#define CMDTYPE_OTHERSET			(0x08)
#define CMDTYPE_CHECKONLINE		(0x09)		/*������Ƚ����⣬�������ж��д���*/
#define CMDTYPE_DATASTORE			(0x0A)		 


#define CMDTYPE_TEST_CODE			(0x0F)		/*���Թ�װ��Ҫ������*/


/*CMDTYPE_PERIPHERAL*/


//new cmd
#define CMD_UNDEFINE				(0x00) //�޶���
#define CMD_POWER					(0x01)// ����ӵ� �ϵ�arg1(0- �ϵ�!0 �ӵ�)
#define CMD_STEP_ENABLE			(0x02)// ��������ϵ�ϵ�,arg1(0- �ϵ�!0 �ӵ�)
#define CMD_GET_DIR					(0x03)// ȡ��ͷ���򴫸���״̬
#define CMD_DIR_ISR					(0x04)// ���û�ͷ��������
#define CMD_GET_STEP_INPUT		(0x05)//ȡ��ͷ�������λ״̬arg1l��ʾ�������(0-,1 ��Ŀ,2 ����,3 ���� ,4 ѹ��)arg1h(0- ȡ0λ1-ȡ����λ)
#define CMD_GET_STEP_INPUT_LOG	(0x25)	// �����߼�λ��ȡ������Ĵ�����״̬	
#define CMD_KEYDOG_READ			(0x06)	//		
#define CMD_KEYDOG_CHECK			(0x07)	//
#define CMD_ENCODER_SETUP			(0x08)	//	�����������������,ARG1=0 ���ã�=1����
#define CMD_ENCODER_CHECK			(0x09)	//	 ���ظ���������״̬
#define CMD_STEP_ENCODER_CHECK	(0x0A) /* ����ָ�������������״̬arg1=�����(0-15) */
#define CMD_STEP_WORK_CURRENT	(0x0B)  /*����*/
#define CMD_SET_SYS_ENABLE		(0x0C)  /*���õ�ǰϵͳʹ�ܻ��ǽ���*/
#define CMD_GET_SIG_PH				(0x0D)  /*ȡ����ϵͳ�ĸ�����������״̬����������λ�÷���*/
#define CMD_GET_AD_CUR				(0x0E)  /*��ȡ��ǰAD����ֵ*/
#define CMD_CHECK_RES_DATA		(0x0F)  /*���Ե�ǰ���迹ֵ*/
#define CMD_GET_RES_DATA			(0x10)  /*��ȡ��ǰ���迹ֵ */	
#define CMD_SET_TEST_MODE_PWM	(0x11)  /*���ò���ģʽѡ��PWM��ʽ*/
#define CMD_SET_PWM_DA_OR_DA		(0x12)  /*���õ���ĵ���ֵ*/

#define CMD_POWER_CTRL_EX			(0x13)	/*�ֱ����+-24V*/

#define CMD_SET_STEP_CHECK_TYPE	(0x14)	/*���õ����λ��������(0--Ĭ�ϣ�1--��������2--��������3--������+������)*/

//by zhuqiwei 160524
#define CMD_ENCODER_ZERO_MASK		(0x15)	/* ��ǵ����ǰλ��Ϊ��λ arg1 -- ����� */
#define CMD_ENCODER_ZERO_SETVAL		(0x16)	/* ֱ�����õ����λֵ arg1--����� arg2--���õ���λ����ֵ[0-4095] */
#define CMD_ENCODER_ZERO_READ		(0x17)	/* ���λ�ü��ƫ�Χ���ֵ���� arg1--����� */
#define CMD_ENCODER_DIR_SETUP		(0x18)	/* �����ǰ����ֵ��ȡ arg1--����� */
#define CMD_ENCODER_POS_TODO		(0x19)	/* ������λ�ò��� -- arg1--����� */
#define CMD_ENCODER_ENABLE_MASK	(0x1A)	/*�����������������,ARG1=mask(bit0-7)    0 ���ã�=1����*/

#define CMD_ENCODER_GET_STATE	(0x1B) /*20161104 ��ѯ��������״̬*/
#define CMD_ENCODER_GET_MAX_STEPS (0x1C)	/*20161120 ��ȡ���������г̣����г����Զ�����֮���ֵ*/
#define CMD_ENCODER_SET_GOBACK_STEPS	(0x1D) /*20161120 �����Զ�����֮�󷵻ز���*/
#define CMD_ENCODER_SET_MIN_ECODE_RANG	(0x1E)/*20170209 by hlc ���õ���Զ������ʱ����С�ļ���г�*/

#define CMD_STEP_MOTOR_ISOK_RETURN	(0x1F)	/*20170718 by hlc ������֮���Զ��ظ�����*/

#define CMD_STEP_ALERT_ST_REPORT		(0x20) /*2017 12 27 by hlc ������౨��״̬*/


#define CMD_STEP_PARAMETER_SET_WITH_MOTORTYPE		(0x21)  /*2018 01 11 by hlc ����������ã�������������*/

#define CMD_STEP_SET_TRY_STEPS_TO_ZERO				(0x22) /*2018 01 23 by hlc �������λ��ʱ���Զ����ذڶ�ÿ�����ӵķ�ֵ*/
#define CMD_STEP_SET_ACC_STEPS						(0x23) /*2018 01 29 by hlc ����Ӽ�����󲽷�*/

#define CMD_DATA_STORE_SYS_KEY						(0x24)/*20180224 by hlc */

#define CMD_SYS_POWER_LOST							(0x26)/*20180803 by hlc  ���������ʱ�رյ������*/

#define CMD_ENCODER_CS								(0x27) /*2020 1104 by hlc ������ʹ��*/
//Old cmd

#define KEYDOG_READ		0x33
#define KEYDOG_CHECK		0x35

#define ENCODER_SETUP		0x45 /*
				      * �����������������
				      * 0: ����
				      * 1: ����
				      */
#define ENCODER_CHECK		0x46 /*
				      * ���ظ���������״̬
				      */
					
#define STEP_ENCODER_CHECK	0x55 /*
				      * ����ָ�������������״̬
				      */


#define POWER_ON		0x00	
#define POWER_OFF		0x01	
#define STEP_ON			0x02	
#define STEP_OFF		0x03	
#define GET_DIR			0x08	
#define DIR_ISR			0x10	
#define GET_ZERO_OTHER 		0x5B	// ȡ��ͷ�����������λ״̬�����˶�Ŀ������
										/*arg1 ��ʾ�������
										*/


/*CMDTYPE_SYSSET*/
//new cmd
#define CMD_SYSSET_MODE_ISR				(0x01)	// ��ϵͳģʽ????
#define CMD_SYSSET_MOTOR_CONFIG_IDTYPE	(0x02)	/* ����������� 1-,����id&type��
							* ����1L: ��8λ��ʾ������NO�ţ�									(0-5)
							* ����1H: ��8λ��ʾtype													(0-4)
							* ����2L: ��8λ��ʾ����ID�ţ�����ڶ�Ŀ����ӿ�				(0-15)
							* ����2H: ��8λ��ʾ����ID�ţ�������������͵ĵ���ӿ�	(0-15)
							*/
#define CMD_SYSSET_MOTOR_CONFIG_INPUTID	(0x03)	/* ����������� 2-����inputid
							* ����1L: ��8λ��ʾ������NO�ţ�	(0-5)
							* ����1H: ��8λbit0��bit1 ���ڱ�ʾ0λ���͹���λ������ID�Ƿ���Ч(1��Ч��0��Ч)
							* ����2L: ��8λ��ʾ0λ������ID		(0-15)
							* ����2H: ��8λ��ʾ����λ������ID	(0-15)
							*/	
#define CMD_SYSSET_HEAD_MODE_SET			(0x04) /*  ��ͷ������������� */
								/*arg1_l=0 ��ʾ���ã�=1 ��ʾ��ȡ*/
								/* arg1_h  ��ֵ
								*0��Ĭ�����á�4��Ŀ+2����+2����+4ɴ�죨����ǵ��ɳ��Ļ�����			
								*1�����׵���4��Ŀ+2����+2�հ�+4ɴ�죨����ǵ��ɳ��Ļ�����			
								*2����ͨ�4��Ŀ+2����+2�հ�+4ɴ�죨����ǵ��ɳ��Ļ�����			
								*3�����˿4��Ŀ+4����+4ɴ�죨����ǵ��ɳ��Ļ�����			
								*/
#define CMD_SYSSET_SYSTYPE		(0x05)	/* ���õ�ǰ��ͷ���ϵͳ������Ҫ�Ǳ�����ʱ����Ҫ�õ� */

#define CMD_SYSSET_LIFTSTEP_AFTER_JQD_ARG	(0x07)

#define CMD_SYSSET_SET_ZEROTYPE_WITH_MAINID	(0x08) /*���õ����λ���ͣ�														
														* ����1L: ��8λ��ʾ������NO�ţ�	(0-15)
														* ����1H: 0--������ȫ����1--�м�һ�����ģʽ
													*/

#define CMD_TEST_STEP_JUSTRUN				(0x0F)  /*���ø������������������������*/

#define CMD_SYSSET_STEP_RESET				(0x10)	/*�������������λ*/
#define CMD_SYSSET_STEP_OPT				(0x11)	/*���������������*/

#define CMD_SYSSET_STEP_GETPOS				(0x12)	/*�����������ȡλ��*/
#define CMD_SYSSET_STEP_SETPOS				(0x13)	/*���������������λ��*/
#define CMD_SYSSET_GET_ZERO_ST				(0x14)	/*�����������ȡ��λ״̬*/



//old cmd

#define SET_MODE_ISR		0x2E	// ��ϵͳģʽ????
#define NEW_STEP_TYPE_SET		0x80 



#define SET_STEPS_TYPE	(0x76)
						/*	��������Զ���
						*  	����1: ������ܺ�(0-�޶���,1-��Ŀ���(���8��),2-��˵��(���4��), 3-�������(���4��),4-ѹ�ŵ��(���4��))
						*	����2:�����ֽڣ�bit0-bit3 ��������0 map����ͷ�ĵ����,
						*							bit4-bit7 ��������1 map����ͷ�ĵ����,
						*							bit8-bit11 ��������2 map����ͷ�ĵ����,
						*							bit12-bit15 ��������3 map����ͷ�ĵ����,
						*							
						*     ����3: �����ֽڣ�bit0-bit3 ��������4 map����ͷ�ĵ����,
						*							bit4-bit7 ��������5 map����ͷ�ĵ����,
						*							bit8-bit11 ��������6 map����ͷ�ĵ����,
						*							bit12-bit15 ��������7 map����ͷ�ĵ���š�
						*	
						*    ע��1: Ŀǰ��ͷ���������Ϊ14�������ԣ�����Щϵͳ��(��ϵͳ)
						*							û�ж�Ӧ��ϵ�ĵ����Ӧ�����ó�0XF��
						*		   2:���õ��������ظ��ˣ���ô��������յ�������ָ��ִ�С�
						*		   3:����Ŵ�0��ʼ��
						*		   4:��ͷ�����˳������:0,1,��ϵͳ�󴲶�Ŀ���
						*										 2,3,��ϵͳǰ����Ŀ���
						*										 4,5,��ϵͳ��/ǰ�����˵��
						*										 6,7,��ϵͳ�󴲶�Ŀ���
						*										 8,9,��ϵͳǰ����Ŀ���
						*										 10,11,��ϵͳ��/ǰ�����˵��
						*/

#define GET_STEPS_TYPE (0x77)
						/* ��ȡ��������Զ���*/
						/*	����1 ���ܺ�: (0-�޶���,1-��Ŀ���(���8��),2-��˵��(���4��), 3-�������(���4��),4-ѹ�ŵ��(���4��))*/
						/* 	*/




/*CMDTYPE_MOTOR_DENSITY*/
//new cmd

#define CMD_DENSITY_MOTOR_RST		(0x01)	// ���ж�Ŀ�������arg1=(0--all,1--left,2--right,3--sig) arg2 =(0-15)
#define CMD_DENSITY_GET_ZERO_WORK_BUSY		(0x02)	// ȡ��ͷ��Ŀ״̬arg1l(0-zero,1-work,2-busy,3-pos) arg1h(0-all,1-sig),arg2(0-15)
#define CMD_DENSITY_STEP_ISR		(0x03)	//ִ�ж�Ŀ����	
											/*		��Ŀ����					
												*		arg1_l	-��Ӧ��Ŀ�����
												*		arg2 -�������
												*		arg1_h - �Ƿ��鴫����2��״̬( 0--����飬!0--Ҫ��� )
											*/
#define CMD_DENSITY_SET_MOT_POS	(0x04)	// ���û�ͷָ����Ŀλ��arg1_l(0--����pos��1-����inputerr) arg1_h ��ʾ��Ŀ��(0-15) arg2 ��ʾ�����ֵ
#define CMD_DENSITY_SET_SPEED		(0x05)	//�����ٶ�arg1(0-��λ�ٶ�,1-����ٶ�,2,other) ,arg2 �ٶ�ֵ
#define CMD_DENSITY_SET_ATTRIBUTE 	(0x06)	// ���ö�Ŀ����arg1(0--enable,1--dir,2--fastmode,3--verystep,4--��⾫��,5--ϸ��6,--��λ����֮�����ߵĲ���) .arg2(attr_mask)
#define CMD_DENSITY_SET_DELAYTIME	(0x07)	// ���ö�Ŀ��λ����ʱʱ��arg1 �����(0-15)��arg2(ʱ��ֵms)
#define CMD_DENSITY_STEP_ISR_DD		(0x08)    //��̬��Ŀ����

#define CMD_DENSITY_STEP_ZEROCHECK_STEPS	(0x09) /*���ö�Ŀ�����λ�������Χֵ*/

#define CMD_DENSITY_SET_SPEED_EX		(0x0F)	//�����ٶ�,�µĽӿڣ��趨ֵΪƵ��ֵ()


#define CMD_DENSITY_STEP_ISR_BIND		(0x13)	/*20220927 ��Ŀ�����������
													���Ұ����ҵ���⣬����һ����������㿴�����Բ��ԣ�
													1����ͷ����֯��
													2��ִ�е��1����
													3��ִ�е��2����
													4���ȴ����1�͵��2�ֱ�ִ����ɣ���λ����
													5��ִ�е��2������λ
													6��ִ�е��1������λ���У�2��3���������ͬʱ����5.6���������ͬʱ����
													�����԰ɣ�*/

#define CMD_DENSITY_SET_STEP_CHECK_TYPE	(0x14)	/*���õ����λ��������(0--Ĭ�ϣ�1--��������2--��������3--������+������)*/

#define CMD_DENSITY_SET_DDM_CHECK		(0x15)  /*�Ƿ�����̬��Ŀ��ʱ������һ�������Ƿ����*/

#define CMD_DENSITY_SET_ZERODATA 		(0x18)  /*���ö�Ŀ�����λ��Ⱥ����ֵ*/
#define CMD_DENSITY_GET_ZERODATA 		(0x19)  /*��ȡ��Ŀ�����λ��Ⱥ����ֵ*/

#define CMD_DENSITY_SET_MAX_CW_STEPS	(0x1A)	/*20191028 by hlc ���ö�Ŀ����������λ��ֵ��������������,Ĭ��70*/
#define CMD_DENSITY_SET_DDM_MIN_ECODE_RANG	(0x1B) /*2070209 by hlc ���ö�Ŀ�������С����г�*/

#define CMD_DENSITY_SET_DM_TYPE			(0x1C) /*20170815 by hlc ���ö�Ŀ��������ͣ��޸��е㲻һ�� */

#define CMD_DENSITY_SET_CHECK_AREA		(0x1D) /*20170816 by hlc ���ö�Ŀ��鴫�����ķ�Χ(�������پͲ����0λ�źţ���Ҫ�Ǿ޸���е����Ҫ)*/

#define CMD_DENSITY_GET_ALL_POSTION			(0x1E)  /*20171024 by hlc ��ȡ��ǰ��Ŀ�����λ��ֵ*/
#define CMD_DENSITY_GET_ALL_2POS				(0x1F) /*20171024  by hlc ��ȡ��ǰ��Ŀ�����Ŀ��λ��ֵ*/


#define CMD_DENSITY_SET_ZERO_CHECK		(0x20)	/*20190226 by hlc ���ö�Ŀ������������򳬹���ֵ�������λ״̬*/


#define CMD_DENSITY_STEP_ISR_DD_CFG		(0x28) /*��̬��Ŀ�����������յ����*/ 
#define CMD_DENSITY_STEP_ISR_DD_RUN		(0x38) /*��̬��������ִ�ж�������*/ 
#define CMD_DENSITY_STEP_ISR_DD_CFG_POWEROFF		(0xA8) /*��̬��Ŀ�����������յ����,�ϵ���֯������һ�εģ�������������*/ 

#define CMD_DENSITY_MAXSPEED_HZ_FOR_DD	(0x2F)	/*���ö�̬��Ŀ����ٶ�*/		
#define CMD_DENSITY_SET_SPEED_MAX_DD	(0x40)	//���ö�̬���������ٶ�


//old cmd

	
#define STEP_ALL_RST	0x04	// ���ж�Ŀ�������
#define GET_ZERO 		0x0B	// ȡ��ͷ��Ŀ��λ״̬
#define GET_MOT 		0x0E	// ȡ��ͷ��Ŀλ��
#define GET_STEP_BUSY		0x22	// ����Ŀ���æ״̬
#define STEP_ISR			0x1C	// ��Ŀ����					
				/*		��Ŀ����					
				*		arg1	-��Ӧ��Ŀ�����
				*		arg2 -�������
				*		arg3 - �Ƿ��鴫����2��״̬( 0--����飬!0--Ҫ��� )
				*/
#define SET_STEP_ISR		0x28	// ���ö�Ŀλ��/*�������Ҳ����ͨ������ӿ�����*/
#define LEFT_STEP_RST_ISR	0x29	// ����Ŀ����
#define RIGHT_STEP_RST_ISR	0x2A 	// �Ҳ��Ŀ����
#define STEP_RST_SPD_ISR	0x2B	// ��Ŀ�����ٶ�
#define STEP_MAX_SPD_ISR	0x2C	// ��Ŀ�����ٶ�
#define SET_STEP_ENABLE		0x2F	// ���ö�Ŀ��������(Ĭ��ֵ0xFF)
#define SIG_STEP_RST			0x34	// ������Ŀ�������
#define SET_STEP_MODE		0x36	// ���ò����������ģʽ			??
#define SET_SETP_DIR		0x38	/* ���ò����������(Ĭ��ֵ0x0155)
					* arg1:ûʹ��
					*arg2 ����ֵdir_mask:
					 * ֧�ְ汾(0x2012)
					 */
#define SET_STEP_RESOLUTION	0x39	/* ���ò��������ϸ����
					 * (1, 2, 4, 8 ...)
					 * ֧�ְ汾(0x2013)
					 */
#define SET_STEP_CHECK		0x43 /*
				      * by xhl 2010/11/15
				      * ���õ��λ�ü�⾫��
				      * �����е����Ч
				      */

#define SET_STEP_FAST_MODE	0x44 /*	
				      * ���������г̵���Ŀ��λ��
				      * arg1 :����ţ���Ŀ���(0-15)
				      */

#define CMD_STEP_RESET_DELAY_TIME	0x4B /*
				      * ���������λʱ��ʱ����
				      *	����1: �����
				      *	����2: ��ʱʱ�䣨2ms��
				      */


#define CMD_SETUP_STEP		0x4C /*
				      * ������������������ýӿ�
				      * ����1: ��������
				      * ����2: ����ֵ
				      *
				      * ��������=
				      *		1: ��Ŀ��λ��������
				      *		2: ������λ��������
				      *		3: ��Ŀ��λ����
				      */
#define SET_STEPMOTOR_MODE_FAST		0x53 	/* by wf 2013/11/25
					* �ɱ��Ŀ����������������ú�ʹ��������������Ŀ,���ˣ�ѹ�ŵ��
					* bit0 = 1; sti���������� 0 ����ʹ��ԭ���ķ�ʽ
					* bit1 = 1; sk
					* bit2 = 1; yj
					* 2014/4 ��Ϊÿ��bit��Ӧ1�����
					*/
					
#define GET_STEPMOTOR_ZERO_SIG		0x54	/* by wf 2013/11/25
					* ��ȡ������Ŀ�������λ������arg1: ������1 - 8
					*/

#define GET_STEPMOTOR_BUSY_STATUS	0x56 /* by wf 2013/11/26
					  * ��Ŀ ���æ״̬��飬
					  * ����arg1: ������
					  */

//2014-03-05 
#define	  SET_MOTOR_ENABLE		(0x68)			//����ʹ�ܵ��
#define	  GET_MOTOR_NEWSTATE	(0x69)			//ȡ�����ӵĴ�������״̬




/*CMDTYPE_MOTOR_OTHER*/

enum {
	MOTOR_OTHER_SINKER = 0,	// 0
	MOTOR_OTHER_ACTION,		// 1
	MOTOR_OTHER_FEET,		// 2
	MOTOR_OTHER_YARN,		// 3
	MOTOR_OTHER_LIFT,		// 4
	MOTOR_OTHER_DM,		//5	
	MOTOR_OTHER_OTHER,	//6
	MOTOR_OTHER_UPDOWN,	//7,����ɳ�����¶��������ÿ��ϵͳ4��
	MOTOR_OTHER_MAX,	
};

//new cmd
#define CMD_OTHER_ISR				(0x01)	//�������arg1L(0-���,1--����,2--ѹ��3--other,) arg1H(������ID��(0-15))��arg2 posֵ
#define CMD_OTHER_RST				(0x02)	//�����λarg1L(0-���,1--����,2--ѹ��3--other,) arg1H(������ID��(0-15))    arg2(0-��λ���� ,!0-��λarg1h��ʾ�ĵ��)
#define CMD_OTHER_GET_ZERO_WORK_BUSY		(0x03)	// ��ȡ�����λ״̬(arg1L(0-���,1--����,2--ѹ��3--other,)arg1H(0-zero,1-����,2-busy��3-pos)),arg2l(0-15,�����) arg2h(0-���У�!0-����)
#define CMD_OTHER_GET_ZEROWORK	(0x04)	// ��ȡָ�����������״̬(arg1L(0-���,1--����,2--ѹ��3--other,)arg1H(������ID��(0-15)))      (����ֵ��bit0Ϊ0λ��bit1Ϊ����λ)
#define CMD_OTHER_SET_POS			(0x05) 	//���õ��������λ��ֵ(arg1L(0-���,1--����,2--ѹ��3--other,)arg1H(������ID��(0-15))),arg2(posֵ)
#define CMD_OTHER_SET_SPEED		(0x06)	 //���õ���������ٶ�(arg1L(0-���,1--����,2--ѹ��3--other,)arg1H(0- ��λ�ٶ�1-�����ٶ�,2-other)),arg2(�ٶ�ֵ)
#define CMD_OTHER_SET_STEPS		(0x07)	//���õ�������в���(arg1L(0-���,1--����,2--ѹ��3--other,)arg2(����))
#define CMD_OTHER_SET_OTHER		(0x08)    /*�������������һЩ�������������磬ʧ����ⷶΧ�����ϸ����*/
//#define CMD_OTHER_SET_WORK_ENABLE (0x09)   /*���õ������λ�������Ƿ���Ч*/
#define CMD_OTHER_SET_ACTION_STEP (0x09)   /*���� ���ǵ������*/

#define CMD_OTHER_YARN_STEP		(0x0A)   /*ɴ�������������*/
#define CMD_OTHER_SET_ACTION_HP_STEP (0x0B)  /*HP���ǵ�����*/
#define CMD_OTHER_SET_SPEED_EX	(0x0F)	//�����ٶ�,�µĽӿڣ��趨ֵΪƵ��ֵ()
#define CMD_OTHER_SET_FEET_VAL	(0x10) /*����ѹ�ŵ�һЩ����*/

#define CMD_OTHER_SET_STEP_CHECK_TYPE	(0x14)	/*���õ����λ��������(0--Ĭ�ϣ�1--��������2--��������3--������+������)*/

#define CMD_OTHER_SET_STEP_SET_CHECK_POS	(0x17) /*���ö���λ�������ļ��POS*/

#define CMD_OTHER_SET_STEP_ZERO_WIDE 	(0x18) /*���������������λ���(�ϵ���֯��ʱ����Ҫ�õ�)*/
#define CMD_OTHER_GET_STEP_ZERO_WIDE 	(0x19) /*��ȡ�����������λ���(�ϵ���֯��ʱ����Ҫ�õ�,�ػ�ǰȡ��ȥ)*/

#define CMD_OTHER_SET_MOTOR_MAX_STEPS_CW	(0x1A) /*20170207 by hlc �����������õ�����������,����������*/
#define CMD_OTHER_SET_MOTOR_MIN_RANG		(0x1B)/* 20170209 by hlc �����������õ���������Զ��������С��Χ*/

#define CMD_OTHER_SET_MOTOR_ZERO_ADJ		(0x1C) /*20170216byhlc ���յ������������λ����֮���߽�ȥ�Ĳ���ֵ*/

#define CMD_OTHER_SET_MOTOR_ZERO_ADJ_EX		(0x1D) /*20180410 by hlc �����λ����ֵ����ֵ��Ҫ����LX�Ķ�������ϣ�Ĭ��3.*/

#define CMD_OTHER_SET_ZERO_DETECT (0x1E)	//20170318 ������λ��鷶Χ����ֵ

#define CMD_OTHER_SET_ZERO_IS_POSITIVE_DIR	(0x20)/*2017.11.08 byhlc ���õ����λ��װ����(0--Ĭ�ϸ�����,1--��������)*/

#define CMD_OTHER_SET_MOTOR_ZERO_TYPE		(0x1F)/*2017.313byhlc ֧�ֺƷỻ�����Ĵ�������������*/

#define CMD_OTHER_SET_MOTOR_POSTION_EX		(0x21)	/*������������λ��*/
#define CMD_SK_MOTOR_TO_WHICH_POSTION	(0x22)	/*���˵����ǰӦ���ڵ�λ��*/
#define CMD_ACT_MOTOR_TO_WHICH_POSTION	(0x23)	/*���ǵ����ǰӦ���ڵ�λ��*/
#define CMD_FEET_MOTOR_TO_WHICH_POSTION	(0x24)	/*ѹ�ŵ����ǰӦ���ڵ�λ��*/
#define CMD_YARN_L_MOTOR_TO_WHICH_POSTION	(0x25)	/*ɳ��1.2.3ϵͳ�����ǰӦ���ڵ�λ��*/
#define CMD_YARN_R_MOTOR_TO_WHICH_POSTION	(0x26)	/*ɳ��5,6,7ϵͳ�����ǰӦ���ڵ�λ��*/
#define CMD_LIFT_MOTOR_TO_WHICH_POSTION	(0x27)	/*��������ǰӦ���ڵ�λ��*/

#define CMD_OTHER_SET_MOTOR_ZEROPOS_WORKST	(0x28)   /*��������λλ�õ�ʱ�򣬹���λ��������״̬*/

#define CMD_OTHER_SET_MOTOR_2ED_SPEED		(0x29)	/*�������˵ڶ��ٶȣ����ٶ�ֵ��������ǰ�������ٶ�*/

#define CMD_OTHER_SET_PAR_EX					(0x2F)  /*���������������չ����*/	
#define CMD_OTHER_ISR_EX_WITH_SPEED			(0x31)  /*����������ٶ�����ִ��*/			
#define CMD_OTHER_ISR_LX_ACT_RUN_NORMAL		(0x41) /*by hlc 2019 07 01 ���˶���������е�0 ֱ�ӻ��㣬������������*/


#define CMD_OTHER_GET_SPEED_DATA			(0x30)  /*by hlc 20181129 ��ȡ��ص�����ٶ�����*/			
#define CMD_OTHER_SET_ACT_ADJ_ENABLE		(0x32)/*by hlc 20190124  �������˶�������Ƿ��Զ�����ʹ��,Ĭ��ʹ��*/

#define CMD_OTHER_CHECK_OTHER_STEP_ALERT		(0x33)/*by hlc 20190130  ������������ı�����飨�������Ƿ��б�����*/
#define CMD_OTHER_SET_INPUT_NC_OR_NO			(0x34) /*by hlc 20190221 ������������źų��ճ�����0��ʾĬ��֮ǰ�ĳ�����1--����*/
#define CMD_OTHER_SET_ALERT_DELAY_CNT			(0x35)/*by hlc 20190306 ��������������˱����ӳٴ���������ģʽ����Ĭ��5�Ρ�*/

#define CMD_OTHER_SET_ACT_WORK_ZEROPOS_DISABLE	(0x36)/*by hlc 20191031*/

#define CMD_OTHER_SET_MOTOR_SIGNAL_EDG			(0x37) /*by hlc 20191119 ���õ���źű��ط�������*/



//oldcmd

#define SK_ISR			0x1F	// ���˵������
#define SK_RST_ISR		0x20	// ���˵������
#define GET_SK_BUSY		0x21	// ������˵��æ״̬
#define GET_SK_ISR		0x26	// ȡ����λ��
								/*
								*	arg1 bit7=1�Ļ�������ȡѹ�ŵ�λ��
								*		����Ļ�������ȡ���˵�λ��
								* 	arg2 ��Ϊ0�Ļ����Ǿ���arg2���������˵ı��+1	(1-16);
								*		����Ļ�������arg1���������˵ı��(0-15)
								*/
#define SET_SK_ISR		0x27	// ��������λ��

#define SINKMOTOR_MAX_SPEED	0x48 /*
				      * ���˵�������ٶ�
				      */


#define CMD_FEET_SETUP		0x49 /*
				      * ѹ�ŵ����������
				      * ����1: ���÷���0 -- ���ò���,2 --ִ�ж���,3--����ѹ������
				      * ����2: ����
				      */
#define CMD_FEET_RUN		0x4A /*
				      * ѹ�ŵ������֧��
				      *	����1: �����
				      *	����2: 16bitָ��
				      *	  bit0~bit5:ѹ������
				      *	  bit14: 1->ѹ������ 0->ѹ��̧��
				      *	  bit15: 1->ѹ�Ÿ�λ
				      */
#define SET_SINKER_NUM			0x51 /* �������˵������ */


#define TRIANGLE_STEP_ISR		0x73 /* ���ǵ���������� 
							* ����1: ���ǵ����(0-3 Ȼ�󱾵�����ת����10-13).
							* ����2: 	arg2   bit15=0,		��ʾλ��ֵ(bit0,bit1)(01--left,11--zero,10--right,0--reset)
							*				  bit15 =1��	bit0-14��ʾʵ����Ҫ�ߵ���λ�á�		
							*����3:   	arg3   bit0,bit1��ʾ��ʱ�Ƿ���Ҫ�����Ӧ��������״̬				
							
							*/
#define TRIANGLE_STEP_GET 	0x74
							/* ���ǵ�״̬��ȡ
							* ����1: ���ǵ����(0-3 Ȼ�󱾵�����ת����10-13).
							* ����2:arg2>0  ��ʾ��ȡ���ǵ����ʵ��postion ֵ��
							*			���򣬻�ȡ���ǵ����ǰλ�õĴ������ź�.							
							*/

#define TRIANGLE_STEP_SET 	0x75
							/* ����λ������
							* ����1: ���ǵ����(0-3 Ȼ�󱾵�����ת����10-13).
							* ����2:����ֵ
							* ����3:arg3>0  ��ʾ�������ǵ����ʵ��postion ֵ��
							*			�����������ǵ����ǰλ�õĴ������ź�.							
							*/

/*CMDTYPE_DCT*/

//new cmd
#define CMD_DCT_ISR			(0x01)	//���������arg1L(0-ɴ��,1--������,2,--ǰ������) arg1H(0-7�������),arg2(0,!0)
#define CMD_DCT_GET_STS	(0x02) 	//ȡ״̬arg1(0-ɴ��״̬,1-������״̬,2-ǰ������״̬)
#define CMD_DCT_SET_HV		(0x03)	//���õ������ѹ(arg1,0- ��ѹ1��1--��ѹ2,)	arg2 ��ѹֵ
#define CMD_DCT_WORKMODE	(0x04)	//���õ��������ģʽ arg1L(0-ɴ��,1--������,2,--ǰ������) arg1H(0-7�������),arg2(0,!0--��ͨ��)

#define CMD_DCT_TEST_RESISTANCE (0x05)//���㵥����������迹ֵ��

#define CMD_DCT_GET_RESISTANCE (0x06)//��ȡ������������迹ֵ��

#define CMD_DCT_TEST_RESISTANCE7		(0x07)

#define CMD_DCT_GET_YARN_ZERO_ST		(0x08)	/*��ȡɴ����λ״̬*/


//old cmd

#define GET_DCT			0x09	// ȡɴ��״̬
#define GET_ACT 			0x0C		// ȡ��ͷ����״̬
#define DCT_HV_ISR 		0x1A	// ���õ������ѹ
#define DCT_ISR			0x1E	// ���������
#define DCT_HV1_ISR		0x35	// ���õ������ѹ2
#define SET_EMF_WORK_MODE	0x41 /*
				      *	by xhl 2010/06/17 
				      * ֧�ְ汾(0x2016)
				      * �����������ǵ�����Ƿ����ڳ�ͨ��
				      * ģʽ��
				      * ������ + ��������(arg1:0-11) + ����ģʽ(arg2:0,!0)
				      * ��arg2 Ϊ����ʱ���õ���������ڳ�ͨ��ģʽ��(��ͨIO)
				      */

/*CMDTYPE_XZQ*/

//new cmd


#define CMD_XZQ_INIT			(0x01)	//����ѡ������ʼ��
#define CMD_XZQ_ZHEN_ISR 		(0x02)	// ����ѡ������.�������ɵ���canID ���͸�ϵͳ���н���arg1l- ������,arg1h ѡ����mask ,arg2,arg3 ����ϵͳ
#define CMD_XZQ_ZHEN_ISR_EX 	(0x82)	// ����ѡ������.�������ɵ���canID ���͸�ϵͳ���н���arg1l- ������,arg1h ѡ����mask ,arg2,arg3 ����ϵͳ

#define  CMD_XZQ_TEST_ISR		(0x03)	//ѡ��������(��ͷ���Խ������) arg1(0-7 ѡ���) arg2 (ÿ��״̬sts(bit0-7))
#define CMD_XZQ_GET_STS 		(0x04)	// ȡ��ͷѡ����״̬(arg1(0-3)��ʾ)
#define CMD_XZQ_GET_DAO_ISR	 (0x05)	// ȡ��ǰ��
#define CMD_XZQ_GET_READY_BLADE 	(0x06) // ȡ��ͷѡ������ǰ�����Ĺ�����ͷ arg1(0-7 ѡ���)


#define CMD_XZQ_SET_START		(0x07)  //����ѡ��������arg1(0-7) arg2(��id��0-7),
#define CMD_XZQ_SET_HLV		(0x08) //����ѡ�����ĸ�ѹ���ѹʱ��arg1(0-��ѹ��1-��ѹ) arg2(����ʱ��ֵ)
#define CMD_XZQ_SET_FAST_HLV		(0x28) //����ѡ�����ĸ�ѹ���ѹʱ��arg1(0-��ѹ��1-��ѹ) arg2(����ʱ��ֵ)

#define CMD_XZQ_SET_DAO_ISR	(0x09) // ���õ�ǰ�� arg1(��ǰ��)
#define CMD_XZQ_SET_DAO_MAX	(0x0A) // �����ܵ���arg1(0-15) ��ʾ1-16��
#define CMD_XZQ_SET_WORKMODE	(0x0B) //����ѡ��������ģʽarg1(0-����ģʽ��!0--һ��ѡ�������������)
#define CMD_XZQ_SET_NEEDLE_STEP (0x0C)// ѡ�������ж� ���� �ӼӼ�1��Ϊ�Ӽ�2��arg1(��ʾ���岽��)


#define CMD_XZQ_TEST_RESISTANCE (0x0D)//���㵥��ѡ�������迹ֵ��
#define CMD_XZQ_GET_RESISTANCE (0x0E) //��ȡ����ѡ�������迹ֵ��

#define CMD_XZQ_SET_START_ALL123		(0x0F)	/*����ѡ�������𵶣������������0x07����*/
#define CMD_XZQ_SET_START_ALL456		(0x10)	/**/
#define CMD_XZQ_SET_REMAP				(0x11)	/*����ѡ�������ض��򣬵�һ����������ڼ�����˳���ǵ���*/
#define CMD_XZQ_SET_REDO_CNT			(0x12) 	/*����ѡ��������ʱ����ظ����*/
//#define CMD_XZQ_SET_START_EX		(0x17)		//ͬʱ����ѡ��������arg1(1ϵͳ) arg2(2ϵͳ)��arg3(3ϵͳ)

#define CMD_XZQ_SET_START_ALL123_EX		(0x13)	/*����ѡ�������𵶣������������0x0f����,������Ч*/
#define CMD_XZQ_SET_START_ALL456_EX		(0x14)	/*����ѡ�������𵶣������������0x10����,������Ч*/

#define CMD_XZQ_SET_START_ALL123_55AA		(0x15)	/*����ѡ�������𵶣������������0x0f����,�ȴ�6-1������Ч*/
#define CMD_XZQ_SET_START_ALL456_55AA		(0x16)	/*����ѡ�������𵶣������������0x10����,�ȴ�6-1������Ч*/

#define CMD_XZQ_SET_JQD_MODE				(0x17) 	/*����ѡ��������Ȧ����0-Ĭ�ϣ�1-����*/
#define CMD_XZQ_SET_JQD_LX				(0x18) 	/*����ѡ�����Ķ����ӳ�ʱ��*/


#define HP3G_NEEDLE_2STEP		0x72 /* �������������������
							* ѡ�������ж� ���� �ӼӼ�1��Ϊ�Ӽ�2��
							*/
#define ST_INIT			0x05	// ����ѡ������ʼ��
#define GET_XZQ 			0x0D		// ȡ��ͷѡ����״̬
#define ZHEN_ISR			0x0F	// ����ѡ������
#define ST1_ISR			0x11	// ����1��ѡ������
#define ST2_ISR			0x12	// ����2��ѡ������
#define ST3_ISR			0x13	// ����3��ѡ������
#define ST4_ISR			0x14	// ����4��ѡ������
#define ST5_ISR			0x15	// ����5��ѡ������
#define ST6_ISR			0x16	// ����6��ѡ������
#define ST7_ISR			0x17	// ����7��ѡ������
#define ST8_ISR			0x18	// ����8��ѡ������
#define XZQ_HV_ISR 		0x19	// ����ѡ������ѹ
#define XZQ_LV_ISR		0x1B	// ����ѡ������ѹ
#define XZQ_ISR			0x1D	// ѡ��������, test
#define GET_DAO_ISR		0x23	// ȡ��ǰ��
#define DAO_ISR			0x24	// ���õ�ǰ��
#define DAO_MAX_ISR		0x25	// �����ܵ���
									/* arg1 ��ʾ�ܵ���-1.	(0-15) ��ʵ���ܵ���(1-16)
									*
									*/	
#define SET_XZQ_MODE_F		0x31	// ����ǰ��ѡ������
#define SET_XZQ_MODE_B		0x32	// ���ͺ�ѡ������
#define SET_XZQ_MODE_TYPE	0x33	/*
					 * ����ǰ�󴲹���ѡ����
					 * ������ + ǰ�� + ��
					 */
#define GET_JQD_BLADE		0x42 /*
				      * by xhl 2010/08/09
				      * ȡ��ͷѡ������ǰ�����Ĺ�����ͷ
				      * ������ + ѡ�������(0 -- 7)
				      */
#define CMD_PARA_SETUP		0x4D /*
				      * �����������ýӿ�
				      * ����1: ��������
				      * 0x10: ѡ��������ӳ����ʼ��
				      * 0x18: ɴ�쿪����ʱ��
				      * 0x19:ɴ��ر���ʱ��
				      */

#define JAQ_WORKMODE_8_16_SET		0x4E /*
				      * ѡ���������������ýӿ�
				      * ����1: ��������
				      * 1: ѡ����������8_16ģʽ��
				      * 0: ѡ�����ظ���8·ģʽ
				      */

#define ST9_ISR			0x60	// ����9��ѡ������
#define ST10_ISR		0x61	// ����10��ѡ������
#define ST11_ISR		0x62	// ����11��ѡ������
#define ST12_ISR		0x63	// ����12��ѡ������
#define ST13_ISR		0x64	// ����13��ѡ������
#define ST14_ISR		0x65	// ����14��ѡ������
#define ST15_ISR		0x66	// ����15��ѡ������
#define ST16_ISR		0x67	// ����16��ѡ������



/*CMDTYPE_ALARM*/
//new cmd


#define CMD_ALARM_CLR	(0x01)	//���
#define CMD_ALARM_GET	(0x02)	// ȡ��ͷ����״̬arg1(0-��ͷ����״̬,1-���򴫸�������״̬  2-���ش�����)
#define CMD_ALARM_SET_ENABLEMASK	(0x03)	// ���û�ͷ��������(arg1��bit0-15 Ϊ0��ʾ���Σ�Ϊ1��ʾ����)
#define CMD_ALARM_SET_OVERLOADENABLE	(0x04)  // ������⹦������arg1(0-��ֹ,!0--����)
#define CMD_ALARM_SET_TANZHEN		(0x05)	//arg1L(0-�ر�,1-����1��,2-����2��,3-����1�ź�2��) arg1H(0-�رգ�!0��ʾ��)arg2(�ӳ�ʱ��)
#define CMD_ALARM_GET_TANZHEN		(0x15)	//��ȡ̽��״̬������


#define CMD_ALARM_CHECK				(0x06)	//arg1l(0-��ʼ��!0-����) arg1H(bitmask) arg2(����)
#define CMD_ALARM_SHOCK				(0x07)  //ײ����������arg1_l(0--��λ��1--�Ƿ�ʹ�ܣ�2--����������)��arg1_h(0-��ʹ��,!0-ʹ��),(0-100��ʾ������) ײ��������0-�����ڹر�.

#define CMD_ALARM_GETALERTSTR_CH		(0x08) 		//��ȡ��������arg1(alert_code),arg2(alert_arg,(--����)
#define CMD_ALARM_GETALERTSTR_EN		(0x09) 	//��ȡ��������arg1(alert_code),arg2(alert_arg,(--Ӣ��)

#define CMD_ALARM_SET_OVERLOADDATA	(0x0A)	//���ù�������ֵ
#define CMD_ALARM_SHOCK_NEW			(0x0B)  //��������������0X07 ֻ��������һ��ײ���arg2_l��=0��1 ��2(0--ȫ����1��ʾ1�ţ�2��ʾ2��)

#define CMD_ALARM_NEW_ALARM_FN		(0x0C) /*��ȡ��ǰϵͳ�Ƿ�֧�ֱ���˿ֱ�ӱ���*/
#define CMD_ALARM_SET_TEMP_DATA		(0x0D)  /*���ó��±���ֵ*/
#define CMD_ALARM_ENABLE_LOG			(0x0E) /*������ر���־����*/
#define CMD_ALARM_ENABLE_LOG_LX		(0x0F) /*������ر���־����_LX*/

#define CMD_ALARM_ENABLE_RUNTIME_CHECK	(0x11) /*������ر�ʵʱ���*/



#define GET_DIR_ERR		0x06	
#define CLR_ERR			0x07	// �����ͷ����
#define GET_ERR			0x0A	// ȡ��ͷ����״̬
#define SET_ERR_ENABLE_ISR	0x2D	// ���û�ͷ��������

#define OVERLOAD_SETUP		0x47 /*
				      * ������⹦������
				      * 0: ����
				      * 1: ����
				      */
#define SET_TANZHEN_KNIT_DELAY	0x52	/* by wf 2013 11 05
					* ���ٻ�ת������̽�뱨���Ŀ��ƣ�
					* ����1 0:�ر�����̽�뱨����1:����1�ţ�2:����2�ţ�3:����1��2��
					* ����2 arg1>0ʱ��=0���رձ�����=1���򿪱�����>1:��ʱʱ�䣬ʱ����100msΪ��λ
					*/    
#define GET_ERROR_CODE		0x59 /*
				      * ���ش�����
				      */


#define TANZHEN_SETUP		0x6a
#define TANZHEN_NEW_SET	0x6b


#define CHECK_STATUS_START	0x70	/* ��ʼ��ѯ 
						* bit0: ���򴫸����¼�ⷽʽ��(Խ�� 1 - 0 - 1)
						*/
#define CHECK_STATUS_STOP		0x71	/* ������ѯ 
						* arg1:	bit0: ���򴫸����¼�ⷽʽ��Խ��
						* 		bit1: ���˵����ѯ
						*		bit2: ���Ŀ left sti prepared 0/2/4/6/
						*		bit3: �Ҷ�Ŀ right sti prepared	1/3/5/7/
						*		bit4: ɴ��״̬ yarn zero
						* arg2: param;
						*/


/*CMDTYPE_OTHERSET*/

#define CMD_OTHERSET_GET_VER		(0x01) //ȡ��ͷ����汾��
#define CMD_OTHERSET_GET_TYPE		(0x02) //ȡ��ͷ������
#define CMD_OTHERSET_GET_FUNC	(0x03) //ȡ��ͷ�幦��
#define CMD_OTHERSET_GET_BTIME	(0x04) //��ͷ��������ʱ��
#define CMD_OTHERSET_DEBUG		(0x05) //����

#define CMD_OTHERSET_REQUEST		(0x06) //��������
#define CMD_OTHERSET_GETCPLD		(0x07)//��ȡCPLD ����Լ��汾��Ϣ
#define CMD_OTHERSET_GETCANERR		(0x08)  //��ȡCAN ������Ϣ

#define CMD_OTHERSET_GETSTEPDEBUGMSG		(0x09)  //��ȡ�����̬����������

#define CMD_OTHERSET_GET_HEAD_CONFIG	(0x0A)	/*��ȡ��ͷ���ϵ�һЩ������Ϣ()*/
#define CMD_OTHERSET_GET_HEAD_SUP_REPORT (0x0B)		/*��ȡ��ͷ���Ƿ�֧�ֱ�ͣģʽ*/

#define CMD_OTHERSET_RETURN_YARN_ZERO_ST	(0x0C)	/*�Զ��ϱ�ɴ����λ״̬*/

#define CMD_OTHERSET_MAIN_BOARD_ID_WRITE_1	(0x0D)		/*20180227 by hlc  ����ID��1д��*/
#define CMD_OTHERSET_MAIN_BOARD_ID_WRITE_2	(0x0E)		/*20180227 by hlc  ����ID��2д��*/

#define CMD_OTHERSET_MAIN_BOARD_ID_SET_1	(0x0F)		/*20180227 by hlc  ����ID��1�·�*/
#define CMD_OTHERSET_MAIN_BOARD_ID_SET_2	(0x10)		/*20180227 by hlc  ����ID��2�·�*/

#define CMD_OTHERSET_HEAD_BOARD_ID_GET_1	(0x11)		/*20180227 by hlc  ��ȡ��ͷID��1*/
#define CMD_OTHERSET_HEAD_BOARD_ID_GET_2	(0x12)		/*20180227 by hlc  ��ȡ��ͷID��2*/

#define CMD_OTHERSET_MAIN_BOARD_HEAD_BINDING	(0x13)		/*20180227 by hlc �󶨻���*/

#define CMD_OTHERSET_GET_MAIN_BOARD_HEAD_BINDING	(0x14)		/*20180227 by hlc��ѯ �󶨻���*/

#define CMD_OTHERSET_SET_TRYOUT_TIME					(0x15)  	/*20181031 by hlc ������ʱ����ʱ��*/

#define CMD_OTHERSET_SET_CHECKIN_TIME					(0x1A)	/*20190301 by hlc  �������������ϱ�ʱ����(Ĭ��0�����ϱ�)*/




/*CMDTYPE_TEST_CODE*/
#define CMD_TESTCODE_START		(0x01)//��ʼ����
#define CMD_TESTCODE_END			(0x02)//��������
#define CMD_TESTCODE_STEPMOTO	(0x03)//�����ת
#define CMD_TESTCODE_DEBUG		(0x04)//��������
#define CMD_TESTCODE_DEBUG_STEP		(0x05)//���Ե��

#define CMD_TESTCODE_DEBUG_CAN_CNT	(0x06)//��ȡcan���͵��ܼ�¼��

#define CMD_TESTCODE_DEBUG_GET_ADCCNT 0x07

#define CMD_TESTCODE_DEBUG_GET_MAINLOOP 0x08



/*CMDTYPE_DATA_STORE*/
#define CMD_DATA_STORE_READ_DATA	0x01
#define CMD_DATA_STORE_WRITE_DATA	0x02




#define GET_VER_ISR		0x30	// ȡ��ͷ����汾��
#define HEAD_DEBUG		0x37	// ���Խӿ�
#define CMD_HEAD_LOG_ENABLE	0xF1 /* wf, 2013/10/21
						* ����arg1:
						* 0x01	��ѡ�������		0x81	�ر�ѡ�������
						* 0x02	�����ǵ�������	0x82	�ر����ǵ�������
						* 0x03	��ɴ���������	0x83	�ر�ɴ���������
						* 0x04	�򿪶�Ŀ��������	0x84	�رն�Ŀ��������
						* 0x05	�����˵�������	0x85	�ر����˵�������
						* 0x06	��ѹ�ŵ�������	0x87	�ر�ѹ�ŵ�������
						* 0x0F	������			0x8F	�ر�����
						*/



#define UPGRADE_REQUEST		0x3A	/* �������� */ 
#define UPGRADE_DATA		0x3B	/* �������� */ 
#define UPGRADE_ENDDATA		0x3C	/* �������� */ 
#define UPGRADE_DATA_RECEIVED	0x3D	/* �������� */ 
#define UPGRADE_SUCCESS		0x3E	/* �������� */ 
#define UPGRADE_FORCE_REQ	0x3F	/* �������� */ 

#define UPGRADE_DATA_RECEIVEDERR	0x013D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPVER_LOW	0x023D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR	0x033D	/* ��������--CRCУ����� */ 


#define BOOT_ACTIVE_CMD_0701	0x0107   
#define BOOT_ACTIVE_CMD_0802	0x0208   
#define BOOT_ACTIVE_CMD_0806	0x0608   

//#ifdef  BOOT_CODE_JOIN_TEST_CODE

#define TEST_CODE_CMD_START	(0x5C)			//�������ģʽ

//#endif

#define GET_BOARD_TYPE		0x40	// ֧�ְ汾(0x2015)

#define GET_BOARD_FUNC_TYPE		0x4F	/* ֧�ְ汾 0x2065
					  * ��ͷ�书������
					  * bit1-bit0: ϵͳ����1-4
					  * bit2 = 1: ֧�ֱ�����
					  * bit3 = 1: ֧��ѹ��
					  */


#define GET_BOARD_BUILD_TIME		0x50    /* ֧�ְ汾 0x2065
					  * ��ͷ��������ʱ��
					  */




  #define TYPE_LF2407		0x8001
  #define TYPE_LPC2919		0x8002
  #define TYPE_F2812			0x8003
  #define TYPE_STM32E457	0x8006


#define TYPE_NUM	0x80							
#define TYPE_STI	0x01
#define TYPE_SK		0x02
#define TYPE_ACT	0x03
#define TYPE_YJ		0x04
#define TYPE_DETECT1	0x05
#define TYPE_DETECT2	0x06
#define TYPE_DIR1		0x07
#define TYPE_DIR2		0x08
#define TYPE_ENABLE	0x80



#define LOGOUT_XZQ_MASK	0x01
#define LOGOUT_DCT_MASK	0x02
#define LOGOUT_YARN_MASK	0x04
#define LOGOUT_STI_MASK	0x08
#define LOGOUT_SK_MASK		0x10
#define LOGOUT_YJ_MASK		0x20



/*
*	wf, 2013/10/21
*	��ͷlog��� cmd
*/
#define LOG_OUT_XZQ		0x10
#define LOG_OUT_DCT		0x11
#define LOG_OUT_YARN		0x12
#define LOG_OUT_STI1		0x13
#define LOG_OUT_STI2		0x14
#define LOG_OUT_STI3		0x15
#define LOG_OUT_STI4		0x16
#define LOG_OUT_STI5		0x17
#define LOG_OUT_STI6		0x18
#define LOG_OUT_STI7		0x19
#define LOG_OUT_STI8		0x1A
#define LOG_OUT_SK1		0x1B
#define LOG_OUT_SK2		0x1C
#define LOG_OUT_SK3		0x1D
#define LOG_OUT_SK4		0x1E
#define LOG_OUT_YJ1		0x1F	//ѹ��
#define LOG_OUT_YJ2		0x20
#define LOG_OUT_YJ3		0x21
#define LOG_OUT_YJ4		0x22
/* log out end */

// ˵��:
// �汾��Ϣ���4λ����ϵͳ��
// ��:
//    �汾Ϊ0x2011->֧�ֵ�˫ϵͳ �汾��Ϊ0x0011
//    �汾Ϊ0x3011->֧����ϵͳ �汾��Ϊ0x0011
//
// ��ʹ�ð汾��Ϊ0x2010֮��ĳ���
// �����ð汾��ѯ����(0x30)��ѯ����汾
//
//
