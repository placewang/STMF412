#ifndef __ALERT_H__
#define __ALERT_H__

#include "config.h"

#define DSP_FATAL_ERR_ARG_TIMER	(0x0000)
#define DSP_FATAL_ERR_ARG_NMI		(0x0001)
#define DSP_FATAL_ERR_ARG_HardFault	(0x0002)
#define DSP_FATAL_ERR_ARG_MemManage	(0x0003)
#define DSP_FATAL_ERR_ARG_BusFault	(0x0004)
#define DSP_FATAL_ERR_ARG_UsageFault (0x0005)


#ifdef NEW_ALARM_STYLE
#define STEP_ERR_DM		0x00
#define STEP_ERR_SK		0x40
#define STEP_ERR_ACT	0x60
#define STEP_ERR_FEET	0x80
#define STEP_ERR_LEFT	0xA0
#define STEP_ERR_YARN	0xC0	




#define DIR_ERR_CODE_ARG(_i)				(0x0001,_i+1)
#define JQD_UNClEAR_CODE_ARG(_i,_sts)		(0x0002,(((_i+1)&0xff)<<8)|(_sts&0xff))
#define YARN_NOT_CLEAR_CODE_ARG(_sts)		(0x0003,_sts&0xff)
#define ACT_NOT_CLEAR_CODE_ARG(_sts)		(0x0004,(_sts&0xff))
#define DIR_CHECK_ERR_CODE_ARG(_sts)		(0x0005,(_sts&0xff))						/*��Ӧ֮ǰ��D0*/
#define SINKER_DONE_ERR_CODE_ARG(_i,_sts)	(0x0006,(((_i+1)&0x0f)<<12)|(_sts&0x0fff))
#define STI_DONE_ERR_CODE_ARG(_sts)		(0x0007,_sts&0xffff)
#define POWER_ERR_CODE_ARG(_i,_sts)		((0x01<<8)|(which_head_code<<12)|((_i+1)&0xff),_sts&0xffff)
#define OVERLOAD_ERR_CODE_ARG(_i,_sts)		((0x02<<8)|(which_head_code<<12)|((_i+1)&0xff),_sts&0xffff)
#define ENCODE_ERR_CODE_ARG(_sts)			((0x04<<8)|(which_head_code<<12)|(1&0xff),_sts&0xffff)
#define CMD_ERR_CODE_ARG(_i,_sts)			((0x05<<8)|(0<<12)|((_i+1)&0xff),_sts&0xffff)
#define OTHER_ERR_CODE_ARG(_i,_sts)			((0x08<<8)|(0<<12)|((_i+1)&0xff),_sts&0xffff)



#endif




// error define
#define STEP_ERR	0xB0	/* ��Ŀ���1-8�� ���� */
#define DIR1_ERR		0xB8	/* ���򴫸��� */
#define DIR2_ERR		0xB9	/* �ҷ��򴫸��� */
#define CAN_BUF_ERR		0xBA	/* CAN��������� */

#define SET_ERROR_ACTIVE	(0x10)/*���ñ�������λ*/
#define JQD_UNClEAR1		(0x31) /*1��ѡ����δ�嵶*/
#define JQD_UNClEAR2		(0x32) /*2��ѡ����δ�嵶*/
#define JQD_UNClEAR3		(0x33) /*3��ѡ����δ�嵶*/
#define JQD_UNClEAR4		(0x34) /*4��ѡ����δ�嵶*/
#define JQD_UNClEAR5		(0x35) /*5��ѡ����δ�嵶*/
#define JQD_UNClEAR6		(0x36) /*6��ѡ����δ�嵶*/
#define JQD_UNClEAR7		(0x37) /*7��ѡ����δ�嵶*/
#define JQD_UNClEAR8		(0x38) /*8��ѡ����δ�嵶*/


//#define DC_ERROR_ALL		(0x)

#define DC24_N_PERR			(0xBD) //0xC2	/* ����-24v����˿ʧЧ     *//* F1 -24V*/
#define DC12_P_PERR  		(0xBE) //0xCA	/* ����12v����˿ʧЧ     *//* F2 +12V*/
#define DC24_P_PERR			(0xBC)	///(0xBC) //0xBD	/* ��ͷ��+24v����˿ʧЧ     *//* F4 +24v*/
#define SHOCK_ALARM 		(0xE7)	/* ײ�뱨��*/

#define ALARM_HEAD_CAN	(0xF0)	/*�������ݵ�����ֵ(���ֽ�0xf0�����ֽ�Ϊϵͳ��(0-7))*/

#define ST_HEAD_CAN		(0xF1)	/*��ͷ״̬���ݵ�����ֵ(���ֽ�0xf1�����ֽ�Ϊϵͳ��(0-7))*/
#define LOG_HEAD_CAN		(0xF2)	/*��ͷ��־��Ϣ(���ֽ�0xf2�����ֽ�Ϊϵͳ��(0-7))*/


#if 0
#define DC24_P_PERR_BC_H	(0x9D)
#define DC24_P_PERR_BC_L	(0x9C)
#define DC24_P_PERR_BC_LL	(0x9B)
#endif

#define DSP_FATAL_ERR	(0xC0)	/* ��ͷ��ʧЧ */
#define TZ_L_ERR			(0xC1)	/* ̽���� */
#define TZ_R_ERR		(0xC2)	/* ̽�� ��*/

#define SINKER1_ERR	0xC3	/* �������1 ���� */
#define SINKER2_ERR	0xC4	/* �������2 ���� */
#define SINKER3_ERR	0xCE	/* �������3 ���� */
#define SINKER4_ERR	0xCF	/* �������4 ���� */

#define FEET1_ERR		0xC5	/* ѹ�����1���� */
#define FEET2_ERR		0xC6	/* ѹ�����2���� */
#define FEET3_ERR		0xC7	/* ѹ�����3���� */
#define FEET4_ERR		0xC8	/* ѹ�����4���� */

#define TRIANGLE1_ERR	(0xD9)   /*�������1����*/
#define TRIANGLE2_ERR	(0xDA)   /*�������2����*/


#define YARN_STEP1_ERR	(0xD5)   /*ɳ�����1����*/
#define YARN_STEP2_ERR	(0xD6)   /*ɳ�����2����*/
#define YARN_STEP3_ERR	(0xD7)   /*ɳ�����3����*/
#define YARN_STEP4_ERR	(0xD8)   /*ɳ�����4����*/

#define LIFT_STEP1_ERR	(0xE3)	/*������1����*/
#define LIFT_STEP2_ERR	(0xE4)	/*������2����*/
#define LIFT_STEP3_ERR	(0xE5)	/*������3����*/
#define LIFT_STEP4_ERR	(0xE6)	/*������4����*/

#define JDM_STEP1_ERR	(0xE8)		/*����Ŀ�������*/
#define JDM_STEP2_ERR	(0xE9)		/*����Ŀ�������*/

#define STEP_CHECK_ALERT	(0xEA)	/*�������ָ��λ�ñ���*/
#define STEP_CHECK_WORKPOS	(0xEB)	/*������ڹ���λ��*/





#if 0
#define HEAD_DC24_LOW	(0xDE)  /*��ͷ24����ԴǷѹ*/		
#define HEAD_DC24_HIGH	(0xDF)  /*��ͷ24����Դ��ѹ*/
#define HEAD_DC12_LOW	(0xE0)  /*��ͷ12����ԴǷѹ*/
#define HEAD_DC12_HIGH	(0xE1)  /*��ͷ12����Դ��ѹ*/
#endif

#define HEAD_BOOT_VER_ERR		(0xFA)	/*��ͷboot �汾̫��*/

#define HEAD_BINDING_MAIN_ERR	(0xFB)	/*��ͷ�����ذ��쳣*/

#define HEAD_DC24_CURR_OVERLOAD	(0xFC)	/*��ͷ24V�� ��������������*/
#define HEAD_POWER_ISOFF			(0xFE)//(0xFD)    /*��ͷ��Դ�رգ�������������������*/

#define ALARM_DC24_LOW_MASK	(0x00000001)
#define ALARM_DC24_HIGH_MASK	(0x00000002)
#define ALARM_DC12_LOW_MASK	(0x00000004)
#define ALARM_DC12_HIGH_MASK	(0x00000008)
#define ALARM_N_24_LOW_MASK	(0x00000010)
#define ALARM_N_24_HIGH_MASK	(0x00000020)
#define ALARM_C_24_LOW_MASK	(0x00000040)
#define ALARM_C_24_HIGH_MASK	(0x00000080)
#define ALARM_C_N_24_LOW_MASK	(0x00000100)
#define ALARM_C_N_24_HIGH_MASK	(0x00000200)
#define ALARM_DC24_LOW_LOW_MASK	(0x00000400)
#define ALARM_N_24_LOW_LOW_MASK	(0x00000800)



#define RIGHT_TANZHEN_ERR	0xC9	/* ��̽�� */
#define LEFT_RIGHT_TANZHEN_ERR 	(0xCA)    /*����̽��*/
#define LEFT_TANZHEN_ERR	0xCB	/* ��̽�� */



//#define STEP2_PERR	0xCB	/* ������24v����˿ʧЧ     *//* F7 +24V*/

#define OVERLOAD_HEAD	0xFD /* ��ͷ��������� */

#define OVERLOAD_24V	0x9F	/*��ͷ��24V�������������ͬ����ʾ��˼��ͬ*/
#define OVERLOAD_POWEROFF	0x9E	/*��ͷ��������µ�Դ�ر�*/


#define OVERLOAD_YARN	0xA0 /* ɴ����������
			      * ���� ��Ӧɴ���
			      */
#define OVERLOAD_JQD	0xA1 /* ѡ��������
			      * ����
			      *   �߰�λΪѡ������
			      *   �Ͱ�λΪ����ͷ��
			      */
#define OVERLOAD_ACT	0xA2 /* �������������
			      * ����
			      *   �߰�λΪ���
			      *   �Ͱ�λΪ�������
			      */
#define YARN_ACT_FAULT        0xA3 /*ɴ����������*/

#define YARN_STATE_NOT_CLEAR   0x30  /*ɴ��״̬δ����*/

#define ACT_STATE_NOT_CLEAR 0xAF

#ifdef LX_DEBUG
#define STEP_ALARM_DEBUG_1	0xAE
#define STEP_ALARM_DEBUG_2	0xAD
#endif



#define ACT_STEP_FAULT1	0xA4  /* �������1���� */
#define ACT_STEP_FAULT2	0xA5  /* �������2���� */
#define ACT_STEP_FAULT3	0xA6  /* �������3���� */
#define ACT_STEP_FAULT4	0xA7  /* �������4���� */

#define	DIR_ERR					0xD0	/* Խ�����µĻ����ⷽʽ��*/
#define 	LIFT_STEP_DONE_ERR	0xD1	/*����������δ���*/
#define	SINKER_DONE_ERR		0xD2	/* Խ�������˶���δ��� */
#define	STI_DONE_ERR			0xD3	/* Խ������Ŀ����δ��� */

//#define	ACT_DONE_ERR			0xCD/* Խ������Ŀ����δ��� */

#define DEVICE_CONFIG_NUM_ERROR	0xD4	/* ������� ������ ���� */

#define BOARD_APP_ERROR			(0x9D)		// ��ͷ������Ӳ���汾��ƥ��

#define JQD_CMD_REPEATED_ERROR	(0X9A)	/*20190819 ѡ���������ظ�*/

#define YARN_CMD_ERROR				(0xDB)			//���ɳ�������
#define FAN_STALL_ERROR				(0xDD)		//���ȶ�ת��
#define TEMP_OVER_ERROR			(0xDC)		//CUP���±���

#define ALERT_CODE_ENCODE			(0xE2)		/* �������౨�� */

#define JQD_OPERATE_ERROR			(0xEE)			/*ѡ�뵼ͨ*/

#define MOTOR_BIND_CMD_ERR		(0x96)		/*��Ŀ��������ִ�б���
													������:96��97��98��99
													����: 01-����δ���
														      02--ִ��δ���
												*/

#define LX_YARN_ZEROINPUT_CHECK_ERR	(0x95)	/*������λ�źż����󱨾�*/
#define LX_YARN_WORKINPUT_CHECK_ERR	(0x94)	/*���˹���λ�źż����󱨾�*/


void alert_init(void);
int alert_count(void);
int alert_push(int alert_code, int alert_arg);
int alert_pop(int *alert_code, int *alert_arg);
int alert_find(int alert_code, int alert_arg);
int alert_delete(int alert_code, int alert_arg);
void Alert_Clear(void);
void Alert_Poll(void);
int Send_alert_content_loop(unsigned char language);
int alert_Set_overloaddata(int p_d,int n_d);

#endif

