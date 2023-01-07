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
#define DIR_CHECK_ERR_CODE_ARG(_sts)		(0x0005,(_sts&0xff))						/*对应之前的D0*/
#define SINKER_DONE_ERR_CODE_ARG(_i,_sts)	(0x0006,(((_i+1)&0x0f)<<12)|(_sts&0x0fff))
#define STI_DONE_ERR_CODE_ARG(_sts)		(0x0007,_sts&0xffff)
#define POWER_ERR_CODE_ARG(_i,_sts)		((0x01<<8)|(which_head_code<<12)|((_i+1)&0xff),_sts&0xffff)
#define OVERLOAD_ERR_CODE_ARG(_i,_sts)		((0x02<<8)|(which_head_code<<12)|((_i+1)&0xff),_sts&0xffff)
#define ENCODE_ERR_CODE_ARG(_sts)			((0x04<<8)|(which_head_code<<12)|(1&0xff),_sts&0xffff)
#define CMD_ERR_CODE_ARG(_i,_sts)			((0x05<<8)|(0<<12)|((_i+1)&0xff),_sts&0xffff)
#define OTHER_ERR_CODE_ARG(_i,_sts)			((0x08<<8)|(0<<12)|((_i+1)&0xff),_sts&0xffff)



#endif




// error define
#define STEP_ERR	0xB0	/* 度目马达1-8号 故障 */
#define DIR1_ERR		0xB8	/* 左方向传感器 */
#define DIR2_ERR		0xB9	/* 右方向传感器 */
#define CAN_BUF_ERR		0xBA	/* CAN缓冲区溢出 */

#define SET_ERROR_ACTIVE	(0x10)/*设置报警屏蔽位*/
#define JQD_UNClEAR1		(0x31) /*1号选针器未清刀*/
#define JQD_UNClEAR2		(0x32) /*2号选针器未清刀*/
#define JQD_UNClEAR3		(0x33) /*3号选针器未清刀*/
#define JQD_UNClEAR4		(0x34) /*4号选针器未清刀*/
#define JQD_UNClEAR5		(0x35) /*5号选针器未清刀*/
#define JQD_UNClEAR6		(0x36) /*6号选针器未清刀*/
#define JQD_UNClEAR7		(0x37) /*7号选针器未清刀*/
#define JQD_UNClEAR8		(0x38) /*8号选针器未清刀*/


//#define DC_ERROR_ALL		(0x)

#define DC24_N_PERR			(0xBD) //0xC2	/* 主板-24v保险丝失效     *//* F1 -24V*/
#define DC12_P_PERR  		(0xBE) //0xCA	/* 主板12v保险丝失效     *//* F2 +12V*/
#define DC24_P_PERR			(0xBC)	///(0xBC) //0xBD	/* 机头板+24v保险丝失效     *//* F4 +24v*/
#define SHOCK_ALARM 		(0xE7)	/* 撞针报警*/

#define ALARM_HEAD_CAN	(0xF0)	/*报警数据的特征值(低字节0xf0，高字节为系统号(0-7))*/

#define ST_HEAD_CAN		(0xF1)	/*机头状态数据的特征值(低字节0xf1，高字节为系统号(0-7))*/
#define LOG_HEAD_CAN		(0xF2)	/*机头日志信息(低字节0xf2，高字节为系统号(0-7))*/


#if 0
#define DC24_P_PERR_BC_H	(0x9D)
#define DC24_P_PERR_BC_L	(0x9C)
#define DC24_P_PERR_BC_LL	(0x9B)
#endif

#define DSP_FATAL_ERR	(0xC0)	/* 机头板失效 */
#define TZ_L_ERR			(0xC1)	/* 探针左 */
#define TZ_R_ERR		(0xC2)	/* 探针 右*/

#define SINKER1_ERR	0xC3	/* 生克马达1 故障 */
#define SINKER2_ERR	0xC4	/* 生克马达2 故障 */
#define SINKER3_ERR	0xCE	/* 生克马达3 故障 */
#define SINKER4_ERR	0xCF	/* 生克马达4 故障 */

#define FEET1_ERR		0xC5	/* 压脚马达1故障 */
#define FEET2_ERR		0xC6	/* 压脚马达2故障 */
#define FEET3_ERR		0xC7	/* 压脚马达3故障 */
#define FEET4_ERR		0xC8	/* 压脚马达4故障 */

#define TRIANGLE1_ERR	(0xD9)   /*三角马达1故障*/
#define TRIANGLE2_ERR	(0xDA)   /*三角马达2故障*/


#define YARN_STEP1_ERR	(0xD5)   /*沙嘴马达1故障*/
#define YARN_STEP2_ERR	(0xD6)   /*沙嘴马达2故障*/
#define YARN_STEP3_ERR	(0xD7)   /*沙嘴马达3故障*/
#define YARN_STEP4_ERR	(0xD8)   /*沙嘴马达4故障*/

#define LIFT_STEP1_ERR	(0xE3)	/*推针电机1故障*/
#define LIFT_STEP2_ERR	(0xE4)	/*推针电机2故障*/
#define LIFT_STEP3_ERR	(0xE5)	/*推针电机3故障*/
#define LIFT_STEP4_ERR	(0xE6)	/*推针电机4故障*/

#define JDM_STEP1_ERR	(0xE8)		/*紧吊目电机报警*/
#define JDM_STEP2_ERR	(0xE9)		/*紧吊目电机报警*/

#define STEP_CHECK_ALERT	(0xEA)	/*电机不在指定位置报警*/
#define STEP_CHECK_WORKPOS	(0xEB)	/*电机不在工作位置*/





#if 0
#define HEAD_DC24_LOW	(0xDE)  /*机头24伏电源欠压*/		
#define HEAD_DC24_HIGH	(0xDF)  /*机头24伏电源过压*/
#define HEAD_DC12_LOW	(0xE0)  /*机头12伏电源欠压*/
#define HEAD_DC12_HIGH	(0xE1)  /*机头12伏电源过压*/
#endif

#define HEAD_BOOT_VER_ERR		(0xFA)	/*机头boot 版本太低*/

#define HEAD_BINDING_MAIN_ERR	(0xFB)	/*机头绑定主控板异常*/

#define HEAD_DC24_CURR_OVERLOAD	(0xFC)	/*机头24V正 负电流持续过大*/
#define HEAD_POWER_ISOFF			(0xFE)//(0xFD)    /*机头电源关闭，后续动作不能正常做*/

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



#define RIGHT_TANZHEN_ERR	0xC9	/* 右探针 */
#define LEFT_RIGHT_TANZHEN_ERR 	(0xCA)    /*左右探针*/
#define LEFT_TANZHEN_ERR	0xCB	/* 左探针 */



//#define STEP2_PERR	0xCB	/* 步进板24v保险丝失效     *//* F7 +24V*/

#define OVERLOAD_HEAD	0xFD /* 机头板过流报警 */

#define OVERLOAD_24V	0x9F	/*机头板24V报警后面参数不同，表示意思不同*/
#define OVERLOAD_POWEROFF	0x9E	/*机头板过流导致电源关闭*/


#define OVERLOAD_YARN	0xA0 /* 纱嘴电磁铁过流
			      * 参数 对应纱嘴号
			      */
#define OVERLOAD_JQD	0xA1 /* 选针器过流
			      * 参数
			      *   高八位为选针器号
			      *   低八位为刀并头号
			      */
#define OVERLOAD_ACT	0xA2 /* 动作电磁铁过流
			      * 参数
			      *   高八位为组号
			      *   低八位为电磁铁号
			      */
#define YARN_ACT_FAULT        0xA3 /*纱嘴电磁铁故障*/

#define YARN_STATE_NOT_CLEAR   0x30  /*纱嘴状态未清零*/

#define ACT_STATE_NOT_CLEAR 0xAF

#ifdef LX_DEBUG
#define STEP_ALARM_DEBUG_1	0xAE
#define STEP_ALARM_DEBUG_2	0xAD
#endif



#define ACT_STEP_FAULT1	0xA4  /* 动作马达1故障 */
#define ACT_STEP_FAULT2	0xA5  /* 动作马达2故障 */
#define ACT_STEP_FAULT3	0xA6  /* 动作马达3故障 */
#define ACT_STEP_FAULT4	0xA7  /* 动作马达4故障 */

#define	DIR_ERR					0xD0	/* 越发，新的换向检测方式，*/
#define 	LIFT_STEP_DONE_ERR	0xD1	/*推针电机动作未完成*/
#define	SINKER_DONE_ERR		0xD2	/* 越发，生克动作未完成 */
#define	STI_DONE_ERR			0xD3	/* 越发，度目动作未完成 */

//#define	ACT_DONE_ERR			0xCD/* 越发，度目动作未完成 */

#define DEVICE_CONFIG_NUM_ERROR	0xD4	/* 电机配置 命令数 出错 */

#define BOARD_APP_ERROR			(0x9D)		// 机头程序与硬件版本不匹配

#define JQD_CMD_REPEATED_ERROR	(0X9A)	/*20190819 选针器命令重复*/

#define YARN_CMD_ERROR				(0xDB)			//电机沙嘴命令错
#define FAN_STALL_ERROR				(0xDD)		//风扇堵转了
#define TEMP_OVER_ERROR			(0xDC)		//CUP超温报警

#define ALERT_CODE_ENCODE			(0xE2)		/* 编码器类报警 */

#define JQD_OPERATE_ERROR			(0xEE)			/*选针导通*/

#define MOTOR_BIND_CMD_ERR		(0x96)		/*度目联动命令执行报警
													报警号:96，97，98，99
													参数: 01-回零未完成
														      02--执行未完成
												*/

#define LX_YARN_ZEROINPUT_CHECK_ERR	(0x95)	/*连兴零位信号检查错误报警*/
#define LX_YARN_WORKINPUT_CHECK_ERR	(0x94)	/*连兴工作位信号检查错误报警*/


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

