
 //#define BOOTVER	0x00000002
 //#define BOOTVER	0x00000003
#define BOOTVER_SUPPORT_BINDING	0x00000004
#define BOOTVER	0x00000005


/*命令格式改成如下:*/
/*
通过两个字节来表示命令(以前是一个字节)
其中第一个字节表示命令大类，第二个字节表示小命令
后面五个字节表示参数；
最后一个字节表示index.(通讯序号)

CAN 数据中8个字节
Data0	Data1	data2	data3	Data4	data5	Data6	Data7
CMDtype	cmd		arg1L	arg1H	arg2L	arg2H	arg3L	index
cmdtype 主要分几大类
1:外设
2:系统设置
3:度目电机
4:其它电机(申克，压脚，动作等等)
5:电磁铁(三角和纱嘴)
6:选针器
7:报警相关
8:其它设置
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
#define CMDTYPE_CHECKONLINE		(0x09)		/*此命令比较特殊，主控在中断中处理*/
#define CMDTYPE_DATASTORE			(0x0A)		 


#define CMDTYPE_TEST_CODE			(0x0F)		/*测试工装需要的命令*/


/*CMDTYPE_PERIPHERAL*/


//new cmd
#define CMD_UNDEFINE				(0x00) //无定义
#define CMD_POWER					(0x01)// 外设加电 断电arg1(0- 断电!0 加电)
#define CMD_STEP_ENABLE			(0x02)// 步进电机上电断电,arg1(0- 断电!0 加电)
#define CMD_GET_DIR					(0x03)// 取机头换向传感器状态
#define CMD_DIR_ISR					(0x04)// 设置机头工作方向
#define CMD_GET_STEP_INPUT		(0x05)//取机头电机的零位状态arg1l表示电机类型(0-,1 度目,2 生克,3 三角 ,4 压脚)arg1h(0- 取0位1-取工作位)
#define CMD_GET_STEP_INPUT_LOG	(0x25)	// 按照逻辑位置取各电机的传感器状态	
#define CMD_KEYDOG_READ			(0x06)	//		
#define CMD_KEYDOG_CHECK			(0x07)	//
#define CMD_ENCODER_SETUP			(0x08)	//	电机编码器功能设置,ARG1=0 禁用，=1启用
#define CMD_ENCODER_CHECK			(0x09)	//	 返回各编码器的状态
#define CMD_STEP_ENCODER_CHECK	(0x0A) /* 返回指定电机编码器的状态arg1=电机号(0-15) */
#define CMD_STEP_WORK_CURRENT	(0x0B)  /*电流*/
#define CMD_SET_SYS_ENABLE		(0x0C)  /*设置当前系统使能还是禁能*/
#define CMD_GET_SIG_PH				(0x0D)  /*取单个系统的各个传感器的状态，按照物理位置返回*/
#define CMD_GET_AD_CUR				(0x0E)  /*获取当前AD采样值*/
#define CMD_CHECK_RES_DATA		(0x0F)  /*测试当前各阻抗值*/
#define CMD_GET_RES_DATA			(0x10)  /*获取当前各阻抗值 */	
#define CMD_SET_TEST_MODE_PWM	(0x11)  /*设置测试模式选择PWM方式*/
#define CMD_SET_PWM_DA_OR_DA		(0x12)  /*设置电机的电流值*/

#define CMD_POWER_CTRL_EX			(0x13)	/*分别控制+-24V*/

#define CMD_SET_STEP_CHECK_TYPE	(0x14)	/*设置电机到位检查的类型(0--默认，1--传感器，2--编码器，3--传感器+编码器)*/

//by zhuqiwei 160524
#define CMD_ENCODER_ZERO_MASK		(0x15)	/* 标记电机当前位置为零位 arg1 -- 电机号 */
#define CMD_ENCODER_ZERO_SETVAL		(0x16)	/* 直接设置电机零位值 arg1--电机号 arg2--设置的零位编码值[0-4095] */
#define CMD_ENCODER_ZERO_READ		(0x17)	/* 电机位置检查偏差范围最大值设置 arg1--电机号 */
#define CMD_ENCODER_DIR_SETUP		(0x18)	/* 电机当前编码值读取 arg1--电机号 */
#define CMD_ENCODER_POS_TODO		(0x19)	/* 编码器位置操作 -- arg1--电机号 */
#define CMD_ENCODER_ENABLE_MASK	(0x1A)	/*电机编码器功能设置,ARG1=mask(bit0-7)    0 禁用，=1启用*/

#define CMD_ENCODER_GET_STATE	(0x1B) /*20161104 查询编码器的状态*/
#define CMD_ENCODER_GET_MAX_STEPS (0x1C)	/*20161120 获取电机的最大行程，该行程是自动找零之后的值*/
#define CMD_ENCODER_SET_GOBACK_STEPS	(0x1D) /*20161120 设置自动归零之后返回步数*/
#define CMD_ENCODER_SET_MIN_ECODE_RANG	(0x1E)/*20170209 by hlc 设置电机自动找零的时候最小的检测行程*/

#define CMD_STEP_MOTOR_ISOK_RETURN	(0x1F)	/*20170718 by hlc 电机完成之后自动回复命令*/

#define CMD_STEP_ALERT_ST_REPORT		(0x20) /*2017 12 27 by hlc 电机各类报警状态*/


#define CMD_STEP_PARAMETER_SET_WITH_MOTORTYPE		(0x21)  /*2018 01 11 by hlc 电机参数设置，按照类型设置*/

#define CMD_STEP_SET_TRY_STEPS_TO_ZERO				(0x22) /*2018 01 23 by hlc 电机找零位的时候自动来回摆动每次增加的幅值*/
#define CMD_STEP_SET_ACC_STEPS						(0x23) /*2018 01 29 by hlc 电机加减速最大步幅*/

#define CMD_DATA_STORE_SYS_KEY						(0x24)/*20180224 by hlc */

#define CMD_SYS_POWER_LOST							(0x26)/*20180803 by hlc  掉电命令，此时关闭电机电流*/

#define CMD_ENCODER_CS								(0x27) /*2020 1104 by hlc 编码器使能*/
//Old cmd

#define KEYDOG_READ		0x33
#define KEYDOG_CHECK		0x35

#define ENCODER_SETUP		0x45 /*
				      * 电机编码器功能设置
				      * 0: 禁用
				      * 1: 启用
				      */
#define ENCODER_CHECK		0x46 /*
				      * 返回各编码器的状态
				      */
					
#define STEP_ENCODER_CHECK	0x55 /*
				      * 返回指定电机编码器的状态
				      */


#define POWER_ON		0x00	
#define POWER_OFF		0x01	
#define STEP_ON			0x02	
#define STEP_OFF		0x03	
#define GET_DIR			0x08	
#define DIR_ISR			0x10	
#define GET_ZERO_OTHER 		0x5B	// 取机头其他电机的零位状态；除了度目和生克
										/*arg1 表示电机类型
										*/


/*CMDTYPE_SYSSET*/
//new cmd
#define CMD_SYSSET_MODE_ISR				(0x01)	// 单系统模式????
#define CMD_SYSSET_MOTOR_CONFIG_IDTYPE	(0x02)	/* 电机配置命令 1-,配置id&type，
							* 参数1L: 低8位表示物理电机NO号，									(0-5)
							* 参数1H: 高8位表示type													(0-4)
							* 参数2L: 低8位表示主控ID号，相对于度目电机接口				(0-15)
							* 参数2H: 高8位表示主控ID号，相对于其它类型的电机接口	(0-15)
							*/
#define CMD_SYSSET_MOTOR_CONFIG_INPUTID	(0x03)	/* 电机配置命令 2-配置inputid
							* 参数1L: 低8位表示物理电机NO号，	(0-5)
							* 参数1H: 高8位bit0和bit1 用于表示0位，和工作位传感器ID是否有效(1有效，0无效)
							* 参数2L: 低8位表示0位传感器ID		(0-15)
							* 参数2H: 高8位表示工作位传感器ID	(0-15)
							*/	
#define CMD_SYSSET_HEAD_MODE_SET			(0x04) /*  机头箱简易配置命令 */
								/*arg1_l=0 表示设置，=1 表示获取*/
								/* arg1_h  的值
								*0、默认配置【4度目+2三角+2生克+4纱嘴（如果是电机沙嘴的话）】			
								*1、简易电机款【4度目+2三角+2空白+4纱嘴（如果是电机沙嘴的话）】			
								*2、普通款【4度目+2生克+2空白+4纱嘴（如果是电机沙嘴的话）】			
								*3、生克款【4度目+4生克+4纱嘴（如果是电机沙嘴的话）】			
								*/
#define CMD_SYSSET_SYSTYPE		(0x05)	/* 设置当前机头箱的系统数，主要是报警的时候需要用到 */

#define CMD_SYSSET_LIFTSTEP_AFTER_JQD_ARG	(0x07)

#define CMD_SYSSET_SET_ZEROTYPE_WITH_MAINID	(0x08) /*设置电机零位类型，														
														* 参数1L: 低8位表示物理电机NO号，	(0-15)
														* 参数1H: 0--负方向全亮，1--中间一个点的模式
													*/

#define CMD_TEST_STEP_JUSTRUN				(0x0F)  /*设置该命令用于启动电机测试命令*/

#define CMD_SYSSET_STEP_RESET				(0x10)	/*根据物理号来复位*/
#define CMD_SYSSET_STEP_OPT				(0x11)	/*根据物理号来运行*/

#define CMD_SYSSET_STEP_GETPOS				(0x12)	/*根据物理号来取位置*/
#define CMD_SYSSET_STEP_SETPOS				(0x13)	/*根据物理号来设置位置*/
#define CMD_SYSSET_GET_ZERO_ST				(0x14)	/*根据物理号来取零位状态*/



//old cmd

#define SET_MODE_ISR		0x2E	// 单系统模式????
#define NEW_STEP_TYPE_SET		0x80 



#define SET_STEPS_TYPE	(0x76)
						/*	电机功能自定义
						*  	参数1: 电机功能号(0-无定义,1-度目电机(最大8个),2-申克电机(最大4个), 3-动作电机(最大4个),4-压脚电机(最大4个))
						*	参数2:两个字节，bit0-bit3 代表主控0 map到机头的电机号,
						*							bit4-bit7 代表主控1 map到机头的电机号,
						*							bit8-bit11 代表主控2 map到机头的电机号,
						*							bit12-bit15 代表主控3 map到机头的电机号,
						*							
						*     参数3: 两个字节，bit0-bit3 代表主控4 map到机头的电机号,
						*							bit4-bit7 代表主控5 map到机头的电机号,
						*							bit8-bit11 代表主控6 map到机头的电机号,
						*							bit12-bit15 代表主控7 map到机头的电机号。
						*	
						*    注意1: 目前机头板最大电机数为14个，所以，在有些系统中(单系统)
						*							没有对应关系的电机号应该设置成0XF。
						*		   2:设置电机号如果重复了，那么按照最后收到的设置指令执行。
						*		   3:电机号从0开始。
						*		   4:机头电机号顺序如下:0,1,左系统后床度目电机
						*										 2,3,左系统前床度目电机
						*										 4,5,左系统后/前床生克电机
						*										 6,7,右系统后床度目电机
						*										 8,9,右系统前床度目电机
						*										 10,11,右系统后/前床生克电机
						*/

#define GET_STEPS_TYPE (0x77)
						/* 获取电机功能自定义*/
						/*	参数1 功能号: (0-无定义,1-度目电机(最大8个),2-申克电机(最大4个), 3-动作电机(最大4个),4-压脚电机(最大4个))*/
						/* 	*/




/*CMDTYPE_MOTOR_DENSITY*/
//new cmd

#define CMD_DENSITY_MOTOR_RST		(0x01)	// 所有度目电机归零arg1=(0--all,1--left,2--right,3--sig) arg2 =(0-15)
#define CMD_DENSITY_GET_ZERO_WORK_BUSY		(0x02)	// 取机头度目状态arg1l(0-zero,1-work,2-busy,3-pos) arg1h(0-all,1-sig),arg2(0-15)
#define CMD_DENSITY_STEP_ISR		(0x03)	//执行度目动作	
											/*		度目动作					
												*		arg1_l	-对应度目电机号
												*		arg2 -电机步数
												*		arg1_h - 是否检查传感器2的状态( 0--不检查，!0--要检查 )
											*/
#define CMD_DENSITY_SET_MOT_POS	(0x04)	// 设置机头指定度目位置arg1_l(0--设置pos，1-设置inputerr) arg1_h 表示度目号(0-15) arg2 表示具体的值
#define CMD_DENSITY_SET_SPEED		(0x05)	//设置速度arg1(0-复位速度,1-最大速度,2,other) ,arg2 速度值
#define CMD_DENSITY_SET_ATTRIBUTE 	(0x06)	// 设置度目属性arg1(0--enable,1--dir,2--fastmode,3--verystep,4--检测精度,5--细分6,--零位到了之后再走的步数) .arg2(attr_mask)
#define CMD_DENSITY_SET_DELAYTIME	(0x07)	// 设置度目复位的延时时间arg1 电机号(0-15)，arg2(时间值ms)
#define CMD_DENSITY_STEP_ISR_DD		(0x08)    //动态度目命令

#define CMD_DENSITY_STEP_ZEROCHECK_STEPS	(0x09) /*设置度目电机零位检查的最大范围值*/

#define CMD_DENSITY_SET_SPEED_EX		(0x0F)	//设置速度,新的接口，设定值为频率值()


#define CMD_DENSITY_STEP_ISR_BIND		(0x13)	/*20220927 度目电机联动命令
													那我按照我的理解，解释一下你的需求，你看看，对不对：
													1、机头出编织区
													2、执行电机1归零
													3、执行电机2归零
													4、等待电机1和电机2分别执行完成（零位亮）
													5、执行电机2到工作位
													6、执行电机1到工作位其中（2、3步骤的命令同时发，5.6步骤的命令同时发）
													这样对吧？*/

#define CMD_DENSITY_SET_STEP_CHECK_TYPE	(0x14)	/*设置电机到位检查的类型(0--默认，1--传感器，2--编码器，3--传感器+编码器)*/

#define CMD_DENSITY_SET_DDM_CHECK		(0x15)  /*是否做动态度目的时候检查上一个动作是否完成*/

#define CMD_DENSITY_SET_ZERODATA 		(0x18)  /*设置度目电机零位宽度和误差值*/
#define CMD_DENSITY_GET_ZERODATA 		(0x19)  /*读取度目电机零位宽度和误差值*/

#define CMD_DENSITY_SET_MAX_CW_STEPS	(0x1A)	/*20191028 by hlc 设置度目电机最大正数位置值，编码器换算用,默认70*/
#define CMD_DENSITY_SET_DDM_MIN_ECODE_RANG	(0x1B) /*2070209 by hlc 设置度目电机的最小检测行程*/

#define CMD_DENSITY_SET_DM_TYPE			(0x1C) /*20170815 by hlc 设置度目电机的类型，巨富有点不一样 */

#define CMD_DENSITY_SET_CHECK_AREA		(0x1D) /*20170816 by hlc 设置度目检查传感器的范围(超过多少就不检查0位信号，主要是巨富机械厂需要)*/

#define CMD_DENSITY_GET_ALL_POSTION			(0x1E)  /*20171024 by hlc 获取当前度目电机的位置值*/
#define CMD_DENSITY_GET_ALL_2POS				(0x1F) /*20171024  by hlc 获取当前度目电机的目标位置值*/


#define CMD_DENSITY_SET_ZERO_CHECK		(0x20)	/*20190226 by hlc 设置度目电机超负数方向超过该值不检查零位状态*/


#define CMD_DENSITY_STEP_ISR_DD_CFG		(0x28) /*动态度目配置命，按照电机号*/ 
#define CMD_DENSITY_STEP_ISR_DD_RUN		(0x38) /*动态跟随命令执行动作命令*/ 
#define CMD_DENSITY_STEP_ISR_DD_CFG_POWEROFF		(0xA8) /*动态度目配置命，按照电机号,断电续织起来第一次的，不检查后面的序号*/ 

#define CMD_DENSITY_MAXSPEED_HZ_FOR_DD	(0x2F)	/*设置动态度目最大速度*/		
#define CMD_DENSITY_SET_SPEED_MAX_DD	(0x40)	//设置动态跟随的最大速度


//old cmd

	
#define STEP_ALL_RST	0x04	// 所有度目电机归零
#define GET_ZERO 		0x0B	// 取机头度目零位状态
#define GET_MOT 		0x0E	// 取机头度目位置
#define GET_STEP_BUSY		0x22	// 检查度目电机忙状态
#define STEP_ISR			0x1C	// 度目动作					
				/*		度目动作					
				*		arg1	-对应度目电机号
				*		arg2 -电机步数
				*		arg3 - 是否检查传感器2的状态( 0--不检查，!0--要检查 )
				*/
#define SET_STEP_ISR		0x28	// 设置度目位置/*其它电机也可以通过这个接口设置*/
#define LEFT_STEP_RST_ISR	0x29	// 左侧度目归零
#define RIGHT_STEP_RST_ISR	0x2A 	// 右侧度目归零
#define STEP_RST_SPD_ISR	0x2B	// 度目归零速度
#define STEP_MAX_SPD_ISR	0x2C	// 度目运行速度
#define SET_STEP_ENABLE		0x2F	// 设置度目工作开关(默认值0xFF)
#define SIG_STEP_RST			0x34	// 单个度目电机归零
#define SET_STEP_MODE		0x36	// 设置步进电机工作模式			??
#define SET_SETP_DIR		0x38	/* 设置步进电机方向(默认值0x0155)
					* arg1:没使用
					*arg2 方向值dir_mask:
					 * 支持版本(0x2012)
					 */
#define SET_STEP_RESOLUTION	0x39	/* 设置步进电机板细分数
					 * (1, 2, 4, 8 ...)
					 * 支持版本(0x2013)
					 */
#define SET_STEP_CHECK		0x43 /*
				      * by xhl 2010/11/15
				      * 设置电机位置检测精度
				      * 对所有电机生效
				      */

#define SET_STEP_FAST_MODE	0x44 /*	
				      * 电机以最短行程到达目标位置
				      * arg1 :电机号，度目编号(0-15)
				      */

#define CMD_STEP_RESET_DELAY_TIME	0x4B /*
				      * 步进电机复位时延时设置
				      *	参数1: 电机号
				      *	参数2: 延时时间（2ms）
				      */


#define CMD_SETUP_STEP		0x4C /*
				      * 步进电机工作参数设置接口
				      * 参数1: 参数类型
				      * 参数2: 参数值
				      *
				      * 参数类型=
				      *		1: 度目零位工作区间
				      *		2: 生克零位工作区间
				      *		3: 度目零位修正
				      */
#define SET_STEPMOTOR_MODE_FAST		0x53 	/* by wf 2013/11/25
					* 可变度目设置命令；此命令设置后，使用驱动器驱动度目,生克，压脚电机
					* bit0 = 1; sti用驱动器； 0 正常使用原来的方式
					* bit1 = 1; sk
					* bit2 = 1; yj
					* 2014/4 改为每个bit对应1个电机
					*/
					
#define GET_STEPMOTOR_ZERO_SIG		0x54	/* by wf 2013/11/25
					* 获取单个度目电机的零位；参数arg1: 电机序号1 - 8
					*/

#define GET_STEPMOTOR_BUSY_STATUS	0x56 /* by wf 2013/11/26
					  * 度目 电机忙状态检查，
					  * 参数arg1: 电机序号
					  */

//2014-03-05 
#define	  SET_MOTOR_ENABLE		(0x68)			//设置使能电机
#define	  GET_MOTOR_NEWSTATE	(0x69)			//取新增加的传感器的状态




/*CMDTYPE_MOTOR_OTHER*/

enum {
	MOTOR_OTHER_SINKER = 0,	// 0
	MOTOR_OTHER_ACTION,		// 1
	MOTOR_OTHER_FEET,		// 2
	MOTOR_OTHER_YARN,		// 3
	MOTOR_OTHER_LIFT,		// 4
	MOTOR_OTHER_DM,		//5	
	MOTOR_OTHER_OTHER,	//6
	MOTOR_OTHER_UPDOWN,	//7,慈星沙嘴上下动作电机，每个系统4个
	MOTOR_OTHER_MAX,	
};

//new cmd
#define CMD_OTHER_ISR				(0x01)	//电机动作arg1L(0-申克,1--三角,2--压脚3--other,) arg1H(具体电机ID号(0-15))，arg2 pos值
#define CMD_OTHER_RST				(0x02)	//电机复位arg1L(0-申克,1--三角,2--压脚3--other,) arg1H(具体电机ID号(0-15))    arg2(0-复位所有 ,!0-复位arg1h标示的电机)
#define CMD_OTHER_GET_ZERO_WORK_BUSY		(0x03)	// 获取电机零位状态(arg1L(0-申克,1--三角,2--压脚3--other,)arg1H(0-zero,1-工作,2-busy，3-pos)),arg2l(0-15,电机号) arg2h(0-所有，!0-单个)
#define CMD_OTHER_GET_ZEROWORK	(0x04)	// 获取指定电机传感器状态(arg1L(0-申克,1--三角,2--压脚3--other,)arg1H(具体电机ID号(0-15)))      (返回值中bit0为0位，bit1为工作位)
#define CMD_OTHER_SET_POS			(0x05) 	//设置单个电机的位置值(arg1L(0-申克,1--三角,2--压脚3--other,)arg1H(具体电机ID号(0-15))),arg2(pos值)
#define CMD_OTHER_SET_SPEED		(0x06)	 //设置电机的运行速度(arg1L(0-申克,1--三角,2--压脚3--other,)arg1H(0- 复位速度1-动作速度,2-other)),arg2(速度值)
#define CMD_OTHER_SET_STEPS		(0x07)	//设置电机的运行步数(arg1L(0-申克,1--三角,2--压脚3--other,)arg2(步数))
#define CMD_OTHER_SET_OTHER		(0x08)    /*设置其它电机的一些其它参数，比如，失步检测范围，电机细分数*/
//#define CMD_OTHER_SET_WORK_ENABLE (0x09)   /*设置电机工作位传感器是否生效*/
#define CMD_OTHER_SET_ACTION_STEP (0x09)   /*设置 三角电机参数*/

#define CMD_OTHER_YARN_STEP		(0x0A)   /*纱嘴电机的相关命令*/
#define CMD_OTHER_SET_ACTION_HP_STEP (0x0B)  /*HP三角的命令*/
#define CMD_OTHER_SET_SPEED_EX	(0x0F)	//设置速度,新的接口，设定值为频率值()
#define CMD_OTHER_SET_FEET_VAL	(0x10) /*设置压脚的一些操作*/

#define CMD_OTHER_SET_STEP_CHECK_TYPE	(0x14)	/*设置电机到位检查的类型(0--默认，1--传感器，2--编码器，3--传感器+编码器)*/

#define CMD_OTHER_SET_STEP_SET_CHECK_POS	(0x17) /*设置动作位传感器的检查POS*/

#define CMD_OTHER_SET_STEP_ZERO_WIDE 	(0x18) /*设置其它电机的零位宽度(断电续织的时候需要用到)*/
#define CMD_OTHER_GET_STEP_ZERO_WIDE 	(0x19) /*获取其它电机的零位宽度(断电续织的时候需要用到,关机前取回去)*/

#define CMD_OTHER_SET_MOTOR_MAX_STEPS_CW	(0x1A) /*20170207 by hlc 按照类型设置电机正向最大步数,编码器上用*/
#define CMD_OTHER_SET_MOTOR_MIN_RANG		(0x1B)/* 20170209 by hlc 按照类型设置电机编码器自动找零的最小范围*/

#define CMD_OTHER_SET_MOTOR_ZERO_ADJ		(0x1C) /*20170216byhlc 按照电机类型设置零位到了之后走进去的步数值*/

#define CMD_OTHER_SET_MOTOR_ZERO_ADJ_EX		(0x1D) /*20180410 by hlc 电机零位修正值，该值主要用在LX的动作电机上，默认3.*/

#define CMD_OTHER_SET_ZERO_DETECT (0x1E)	//20170318 设置零位检查范围修正值

#define CMD_OTHER_SET_ZERO_IS_POSITIVE_DIR	(0x20)/*2017.11.08 byhlc 设置电机零位安装方向(0--默认负方向,1--正数方向)*/

#define CMD_OTHER_SET_MOTOR_ZERO_TYPE		(0x1F)/*2017.313byhlc 支持浩丰换向电机的传感器类型设置*/

#define CMD_OTHER_SET_MOTOR_POSTION_EX		(0x21)	/*设置推针电机的位置*/
#define CMD_SK_MOTOR_TO_WHICH_POSTION	(0x22)	/*生克电机当前应该在的位置*/
#define CMD_ACT_MOTOR_TO_WHICH_POSTION	(0x23)	/*三角电机当前应该在的位置*/
#define CMD_FEET_MOTOR_TO_WHICH_POSTION	(0x24)	/*压脚电机当前应该在的位置*/
#define CMD_YARN_L_MOTOR_TO_WHICH_POSTION	(0x25)	/*沙嘴1.2.3系统电机当前应该在的位置*/
#define CMD_YARN_R_MOTOR_TO_WHICH_POSTION	(0x26)	/*沙嘴5,6,7系统电机当前应该在的位置*/
#define CMD_LIFT_MOTOR_TO_WHICH_POSTION	(0x27)	/*推针电机当前应该在的位置*/

#define CMD_OTHER_SET_MOTOR_ZEROPOS_WORKST	(0x28)   /*设置在零位位置的时候，工作位传感器的状态*/

#define CMD_OTHER_SET_MOTOR_2ED_SPEED		(0x29)	/*设置生克第二速度，该速度值用于区分前后床生克速度*/

#define CMD_OTHER_SET_PAR_EX					(0x2F)  /*设置其它电机的扩展参数*/	
#define CMD_OTHER_ISR_EX_WITH_SPEED			(0x31)  /*其它电机带速度下来执行*/			
#define CMD_OTHER_ISR_LX_ACT_RUN_NORMAL		(0x41) /*by hlc 2019 07 01 连兴动作电机运行到0 直接回零，不做过零修正*/


#define CMD_OTHER_GET_SPEED_DATA			(0x30)  /*by hlc 20181129 获取相关电机的速度数据*/			
#define CMD_OTHER_SET_ACT_ADJ_ENABLE		(0x32)/*by hlc 20190124  设置连兴动作电机是否自动修正使能,默认使能*/

#define CMD_OTHER_CHECK_OTHER_STEP_ALERT		(0x33)/*by hlc 20190130  设置其它电机的报警检查（立马检查是否有报警）*/
#define CMD_OTHER_SET_INPUT_NC_OR_NO			(0x34) /*by hlc 20190221 设置其它电机信号常闭常开，0表示默认之前的常开，1--常闭*/
#define CMD_OTHER_SET_ALERT_DELAY_CNT			(0x35)/*by hlc 20190306 设置其它电机生克报警延迟次数，连兴模式下面默认5次。*/

#define CMD_OTHER_SET_ACT_WORK_ZEROPOS_DISABLE	(0x36)/*by hlc 20191031*/

#define CMD_OTHER_SET_MOTOR_SIGNAL_EDG			(0x37) /*by hlc 20191119 设置电机信号边沿反馈功能*/



//oldcmd

#define SK_ISR			0x1F	// 生克电机动作
#define SK_RST_ISR		0x20	// 生克电机归零
#define GET_SK_BUSY		0x21	// 检查生克电机忙状态
#define GET_SK_ISR		0x26	// 取生克位置
								/*
								*	arg1 bit7=1的话，就是取压脚的位置
								*		否则的话，就是取生克的位置
								* 	arg2 不为0的话，那就是arg2当成是生克的编号+1	(1-16);
								*		否则的话，就是arg1当成是生克的编号(0-15)
								*/
#define SET_SK_ISR		0x27	// 设置生克位置

#define SINKMOTOR_MAX_SPEED	0x48 /*
				      * 生克电机运行速度
				      */


#define CMD_FEET_SETUP		0x49 /*
				      * 压脚电机控制设置
				      * 参数1: 设置分类0 -- 设置步数,2 --执行动作,3--设置压脚数量
				      * 参数2: 参数
				      */
#define CMD_FEET_RUN		0x4A /*
				      * 压脚电机控制支持
				      *	参数1: 电机号
				      *	参数2: 16bit指令
				      *	  bit0~bit5:压脚修正
				      *	  bit14: 1->压脚作用 0->压脚抬起
				      *	  bit15: 1->压脚复位
				      */
#define SET_SINKER_NUM			0x51 /* 设置生克电机个数 */


#define TRIANGLE_STEP_ISR		0x73 /* 三角电机动作命令 
							* 参数1: 三角电机号(0-3 然后本地自行转换到10-13).
							* 参数2: 	arg2   bit15=0,		表示位置值(bit0,bit1)(01--left,11--zero,10--right,0--reset)
							*				  bit15 =1，	bit0-14表示实际需要走到的位置。		
							*参数3:   	arg3   bit0,bit1表示此时是否需要检测相应传感器的状态				
							
							*/
#define TRIANGLE_STEP_GET 	0x74
							/* 三角电状态获取
							* 参数1: 三角电机号(0-3 然后本地自行转换到10-13).
							* 参数2:arg2>0  表示获取三角电机的实际postion 值，
							*			否则，获取三角电机当前位置的传感器信号.							
							*/

#define TRIANGLE_STEP_SET 	0x75
							/* 三角位置设置
							* 参数1: 三角电机号(0-3 然后本地自行转换到10-13).
							* 参数2:设置值
							* 参数3:arg3>0  表示设置三角电机的实际postion 值，
							*			否则，设置三角电机当前位置的传感器信号.							
							*/

/*CMDTYPE_DCT*/

//new cmd
#define CMD_DCT_ISR			(0x01)	//电磁铁动作arg1L(0-纱嘴,1--后床三角,2,--前床三角) arg1H(0-7电磁铁号),arg2(0,!0)
#define CMD_DCT_GET_STS	(0x02) 	//取状态arg1(0-纱嘴状态,1-后床三角状态,2-前场三角状态)
#define CMD_DCT_SET_HV		(0x03)	//设置电磁铁高压(arg1,0- 高压1。1--高压2,)	arg2 高压值
#define CMD_DCT_WORKMODE	(0x04)	//设置电磁铁工作模式 arg1L(0-纱嘴,1--后床三角,2,--前床三角) arg1H(0-7电磁铁号),arg2(0,!0--常通电)

#define CMD_DCT_TEST_RESISTANCE (0x05)//计算单个电磁铁的阻抗值。

#define CMD_DCT_GET_RESISTANCE (0x06)//获取单个电磁铁的阻抗值。

#define CMD_DCT_TEST_RESISTANCE7		(0x07)

#define CMD_DCT_GET_YARN_ZERO_ST		(0x08)	/*获取纱嘴零位状态*/


//old cmd

#define GET_DCT			0x09	// 取纱嘴状态
#define GET_ACT 			0x0C		// 取机头三角状态
#define DCT_HV_ISR 		0x1A	// 设置电磁铁高压
#define DCT_ISR			0x1E	// 电磁铁动作
#define DCT_HV1_ISR		0x35	// 设置电磁铁高压2
#define SET_EMF_WORK_MODE	0x41 /*
				      *	by xhl 2010/06/17 
				      * 支持版本(0x2016)
				      * 用于设置三角电磁铁是否工作在常通电
				      * 模式下
				      * 命令码 + 电磁铁序号(arg1:0-11) + 工作模式(arg2:0,!0)
				      * 当arg2 为非零时，该电磁铁工作在常通电模式下(普通IO)
				      */

/*CMDTYPE_XZQ*/

//new cmd


#define CMD_XZQ_INIT			(0x01)	//所有选针器初始化
#define CMD_XZQ_ZHEN_ISR 		(0x02)	// 发送选针数据.该命令由单独canID 发送各系统自行解析arg1l- 针数据,arg1h 选针器mask ,arg2,arg3 三个系统
#define CMD_XZQ_ZHEN_ISR_EX 	(0x82)	// 发送选针数据.该命令由单独canID 发送各系统自行解析arg1l- 针数据,arg1h 选针器mask ,arg2,arg3 三个系统

#define  CMD_XZQ_TEST_ISR		(0x03)	//选针器动作(机头测试界面调用) arg1(0-7 选针号) arg2 (每刀状态sts(bit0-7))
#define CMD_XZQ_GET_STS 		(0x04)	// 取机头选针器状态(arg1(0-3)表示)
#define CMD_XZQ_GET_DAO_ISR	 (0x05)	// 取当前刀
#define CMD_XZQ_GET_READY_BLADE 	(0x06) // 取机头选针器当前就绪的工作刀头 arg1(0-7 选针号)


#define CMD_XZQ_SET_START		(0x07)  //设置选针器的起刀arg1(0-7) arg2(起刀id号0-7),
#define CMD_XZQ_SET_HLV		(0x08) //设置选针器的高压或低压时间arg1(0-高压，1-低压) arg2(具体时间值)
#define CMD_XZQ_SET_FAST_HLV		(0x28) //设置选针器的高压或低压时间arg1(0-高压，1-低压) arg2(具体时间值)

#define CMD_XZQ_SET_DAO_ISR	(0x09) // 设置当前刀 arg1(当前刀)
#define CMD_XZQ_SET_DAO_MAX	(0x0A) // 设置总刀数arg1(0-15) 表示1-16刀
#define CMD_XZQ_SET_WORKMODE	(0x0B) //设置选针器工作模式arg1(0-正常模式，!0--一个选针器拆成两个用)
#define CMD_XZQ_SET_NEEDLE_STEP (0x0C)// 选针器针中断 计算 从加减1改为加减2；arg1(表示具体步长)


#define CMD_XZQ_TEST_RESISTANCE (0x0D)//计算单个选针器的阻抗值。
#define CMD_XZQ_GET_RESISTANCE (0x0E) //获取单个选针器的阻抗值。

#define CMD_XZQ_SET_START_ALL123		(0x0F)	/*设置选针器的起刀，类似于上面的0x07命令*/
#define CMD_XZQ_SET_START_ALL456		(0x10)	/**/
#define CMD_XZQ_SET_REMAP				(0x11)	/*设置选针器的重定向，第一刀定向到物理第几刀，顺序还是倒序*/
#define CMD_XZQ_SET_REDO_CNT			(0x12) 	/*设置选针器工作时候的重复序号*/
//#define CMD_XZQ_SET_START_EX		(0x17)		//同时设置选针器的起刀arg1(1系统) arg2(2系统)，arg3(3系统)

#define CMD_XZQ_SET_START_ALL123_EX		(0x13)	/*设置选针器的起刀，类似于上面的0x0f命令,立即生效*/
#define CMD_XZQ_SET_START_ALL456_EX		(0x14)	/*设置选针器的起刀，类似于上面的0x10命令,立即生效*/

#define CMD_XZQ_SET_START_ALL123_55AA		(0x15)	/*设置选针器的起刀，类似于上面的0x0f命令,等待6-1命令生效*/
#define CMD_XZQ_SET_START_ALL456_55AA		(0x16)	/*设置选针器的起刀，类似于上面的0x10命令,等待6-1命令生效*/

#define CMD_XZQ_SET_JQD_MODE				(0x17) 	/*设置选针器的线圈方向0-默认，1-反向*/
#define CMD_XZQ_SET_JQD_LX				(0x18) 	/*设置选针器的动作延迟时间*/


#define HP3G_NEEDLE_2STEP		0x72 /* 慈星三针机，新增命令
							* 选针器针中断 计算 从加减1改为加减2；
							*/
#define ST_INIT			0x05	// 所有选针器初始化
#define GET_XZQ 			0x0D		// 取机头选针器状态
#define ZHEN_ISR			0x0F	// 发送选针数据
#define ST1_ISR			0x11	// 设置1号选针器起刀
#define ST2_ISR			0x12	// 设置2号选针器起刀
#define ST3_ISR			0x13	// 设置3号选针器起刀
#define ST4_ISR			0x14	// 设置4号选针器起刀
#define ST5_ISR			0x15	// 设置5号选针器起刀
#define ST6_ISR			0x16	// 设置6号选针器起刀
#define ST7_ISR			0x17	// 设置7号选针器起刀
#define ST8_ISR			0x18	// 设置8号选针器起刀
#define XZQ_HV_ISR 		0x19	// 设置选针器高压
#define XZQ_LV_ISR		0x1B	// 设置选针器低压
#define XZQ_ISR			0x1D	// 选针器动作, test
#define GET_DAO_ISR		0x23	// 取当前刀
#define DAO_ISR			0x24	// 设置当前刀
#define DAO_MAX_ISR		0x25	// 设置总刀数
									/* arg1 表示总刀数-1.	(0-15) 。实际总刀数(1-16)
									*
									*/	
#define SET_XZQ_MODE_F		0x31	// 发送前床选针数据
#define SET_XZQ_MODE_B		0x32	// 发送后床选针数据
#define SET_XZQ_MODE_TYPE	0x33	/*
					 * 设置前后床工作选针器
					 * 命令码 + 前床 + 后床
					 */
#define GET_JQD_BLADE		0x42 /*
				      * by xhl 2010/08/09
				      * 取机头选针器当前就绪的工作刀头
				      * 命令码 + 选针器序号(0 -- 7)
				      */
#define CMD_PARA_SETUP		0x4D /*
				      * 工作参数设置接口
				      * 参数1: 参数类型
				      * 0x10: 选针器重新映射起始刀
				      * 0x18: 纱嘴开保持时间
				      * 0x19:纱嘴关保持时间
				      */

#define JAQ_WORKMODE_8_16_SET		0x4E /*
				      * 选针器工作参数设置接口
				      * 参数1: 参数类型
				      * 1: 选针器工作在8_16模式下
				      * 0: 选针器回复到8路模式
				      */

#define ST9_ISR			0x60	// 设置9号选针器起刀
#define ST10_ISR		0x61	// 设置10号选针器起刀
#define ST11_ISR		0x62	// 设置11号选针器起刀
#define ST12_ISR		0x63	// 设置12号选针器起刀
#define ST13_ISR		0x64	// 设置13号选针器起刀
#define ST14_ISR		0x65	// 设置14号选针器起刀
#define ST15_ISR		0x66	// 设置15号选针器起刀
#define ST16_ISR		0x67	// 设置16号选针器起刀



/*CMDTYPE_ALARM*/
//new cmd


#define CMD_ALARM_CLR	(0x01)	//清错
#define CMD_ALARM_GET	(0x02)	// 取机头错误状态arg1(0-机头错误状态,1-换向传感器错误状态  2-返回错误码)
#define CMD_ALARM_SET_ENABLEMASK	(0x03)	// 设置机头错误屏蔽(arg1中bit0-15 为0表示屏蔽，为1表示报警)
#define CMD_ALARM_SET_OVERLOADENABLE	(0x04)  // 过流检测功能设置arg1(0-禁止,!0--启用)
#define CMD_ALARM_SET_TANZHEN		(0x05)	//arg1L(0-关闭,1-设置1号,2-设置2号,3-设置1号和2号) arg1H(0-关闭，!0表示打开)arg2(延长时间)
#define CMD_ALARM_GET_TANZHEN		(0x15)	//读取探针状态的命令


#define CMD_ALARM_CHECK				(0x06)	//arg1l(0-开始，!0-结束) arg1H(bitmask) arg2(参数)
#define CMD_ALARM_SHOCK				(0x07)  //撞针的相关设置arg1_l(0--复位，1--是否使能，2--灵敏度设置)，arg1_h(0-不使能,!0-使能),(0-100表示灵敏度) 撞针灵敏度0-类似于关闭.

#define CMD_ALARM_GETALERTSTR_CH		(0x08) 		//获取报警内容arg1(alert_code),arg2(alert_arg,(--中文)
#define CMD_ALARM_GETALERTSTR_EN		(0x09) 	//获取报警内容arg1(alert_code),arg2(alert_arg,(--英文)

#define CMD_ALARM_SET_OVERLOADDATA	(0x0A)	//设置过流报警值
#define CMD_ALARM_SHOCK_NEW			(0x0B)  //命令内容类似与0X07 只是增加了一个撞针号arg2_l，=0或1 或2(0--全部，1表示1号，2表示2号)

#define CMD_ALARM_NEW_ALARM_FN		(0x0C) /*获取当前系统是否支持保险丝直接报警*/
#define CMD_ALARM_SET_TEMP_DATA		(0x0D)  /*设置超温报警值*/
#define CMD_ALARM_ENABLE_LOG			(0x0E) /*开启或关闭日志功能*/
#define CMD_ALARM_ENABLE_LOG_LX		(0x0F) /*开启或关闭日志功能_LX*/

#define CMD_ALARM_ENABLE_RUNTIME_CHECK	(0x11) /*开启或关闭实时检查*/



#define GET_DIR_ERR		0x06	
#define CLR_ERR			0x07	// 清除机头错误
#define GET_ERR			0x0A	// 取机头错误状态
#define SET_ERR_ENABLE_ISR	0x2D	// 设置机头错误屏蔽

#define OVERLOAD_SETUP		0x47 /*
				      * 过流检测功能设置
				      * 0: 禁用
				      * 1: 启用
				      */
#define SET_TANZHEN_KNIT_DELAY	0x52	/* by wf 2013 11 05
					* 快速回转，增加探针报警的控制；
					* 参数1 0:关闭两个探针报警；1:设置1号；2:设置2号；3:设置1和2号
					* 参数2 arg1>0时；=0，关闭报警；=1，打开报警；>1:延时时间，时间以100ms为单位
					*/    
#define GET_ERROR_CODE		0x59 /*
				      * 返回错误码
				      */


#define TANZHEN_SETUP		0x6a
#define TANZHEN_NEW_SET	0x6b


#define CHECK_STATUS_START	0x70	/* 开始查询 
						* bit0: 方向传感器新检测方式，(越发 1 - 0 - 1)
						*/
#define CHECK_STATUS_STOP		0x71	/* 结束查询 
						* arg1:	bit0: 方向传感器新检测方式，越发
						* 		bit1: 生克电机查询
						*		bit2: 左度目 left sti prepared 0/2/4/6/
						*		bit3: 右度目 right sti prepared	1/3/5/7/
						*		bit4: 纱嘴状态 yarn zero
						* arg2: param;
						*/


/*CMDTYPE_OTHERSET*/

#define CMD_OTHERSET_GET_VER		(0x01) //取机头程序版本号
#define CMD_OTHERSET_GET_TYPE		(0x02) //取机头板类型
#define CMD_OTHERSET_GET_FUNC	(0x03) //取机头板功能
#define CMD_OTHERSET_GET_BTIME	(0x04) //机头箱代码编译时间
#define CMD_OTHERSET_DEBUG		(0x05) //调试

#define CMD_OTHERSET_REQUEST		(0x06) //升级重启
#define CMD_OTHERSET_GETCPLD		(0x07)//获取CPLD 板号以及版本信息
#define CMD_OTHERSET_GETCANERR		(0x08)  //获取CAN 错误信息

#define CMD_OTHERSET_GETSTEPDEBUGMSG		(0x09)  //获取电机动态的运行数据

#define CMD_OTHERSET_GET_HEAD_CONFIG	(0x0A)	/*获取机头板上的一些配置信息()*/
#define CMD_OTHERSET_GET_HEAD_SUP_REPORT (0x0B)		/*获取机头板是否支持边停模式*/

#define CMD_OTHERSET_RETURN_YARN_ZERO_ST	(0x0C)	/*自动上报纱嘴零位状态*/

#define CMD_OTHERSET_MAIN_BOARD_ID_WRITE_1	(0x0D)		/*20180227 by hlc  主控ID号1写入*/
#define CMD_OTHERSET_MAIN_BOARD_ID_WRITE_2	(0x0E)		/*20180227 by hlc  主控ID号2写入*/

#define CMD_OTHERSET_MAIN_BOARD_ID_SET_1	(0x0F)		/*20180227 by hlc  主控ID号1下发*/
#define CMD_OTHERSET_MAIN_BOARD_ID_SET_2	(0x10)		/*20180227 by hlc  主控ID号2下发*/

#define CMD_OTHERSET_HEAD_BOARD_ID_GET_1	(0x11)		/*20180227 by hlc  获取机头ID号1*/
#define CMD_OTHERSET_HEAD_BOARD_ID_GET_2	(0x12)		/*20180227 by hlc  获取机头ID号2*/

#define CMD_OTHERSET_MAIN_BOARD_HEAD_BINDING	(0x13)		/*20180227 by hlc 绑定或解绑*/

#define CMD_OTHERSET_GET_MAIN_BOARD_HEAD_BINDING	(0x14)		/*20180227 by hlc查询 绑定或解绑*/

#define CMD_OTHERSET_SET_TRYOUT_TIME					(0x15)  	/*20181031 by hlc 设置延时锁定时间*/

#define CMD_OTHERSET_SET_CHECKIN_TIME					(0x1A)	/*20190301 by hlc  设置外设主动上报时间间隔(默认0，不上报)*/




/*CMDTYPE_TEST_CODE*/
#define CMD_TESTCODE_START		(0x01)//开始测试
#define CMD_TESTCODE_END			(0x02)//结束测试
#define CMD_TESTCODE_STEPMOTO	(0x03)//电机旋转
#define CMD_TESTCODE_DEBUG		(0x04)//调试命令
#define CMD_TESTCODE_DEBUG_STEP		(0x05)//调试电机

#define CMD_TESTCODE_DEBUG_CAN_CNT	(0x06)//获取can发送的总记录数

#define CMD_TESTCODE_DEBUG_GET_ADCCNT 0x07

#define CMD_TESTCODE_DEBUG_GET_MAINLOOP 0x08



/*CMDTYPE_DATA_STORE*/
#define CMD_DATA_STORE_READ_DATA	0x01
#define CMD_DATA_STORE_WRITE_DATA	0x02




#define GET_VER_ISR		0x30	// 取机头程序版本号
#define HEAD_DEBUG		0x37	// 调试接口
#define CMD_HEAD_LOG_ENABLE	0xF1 /* wf, 2013/10/21
						* 参数arg1:
						* 0x01	打开选针器输出		0x81	关闭选针器输出
						* 0x02	打开三角电磁铁输出	0x82	关闭三角电磁铁输出
						* 0x03	打开纱嘴电磁铁输出	0x83	关闭纱嘴电磁铁输出
						* 0x04	打开度目电磁铁输出	0x84	关闭度目电磁铁输出
						* 0x05	打开生克电磁铁输出	0x85	关闭生克电磁铁输出
						* 0x06	打开压脚电磁铁输出	0x87	关闭压脚电磁铁输出
						* 0x0F	打开所有			0x8F	关闭所有
						*/



#define UPGRADE_REQUEST		0x3A	/* 升级命令 */ 
#define UPGRADE_DATA		0x3B	/* 升级命令 */ 
#define UPGRADE_ENDDATA		0x3C	/* 升级命令 */ 
#define UPGRADE_DATA_RECEIVED	0x3D	/* 升级命令 */ 
#define UPGRADE_SUCCESS		0x3E	/* 升级命令 */ 
#define UPGRADE_FORCE_REQ	0x3F	/* 升级命令 */ 

#define UPGRADE_DATA_RECEIVEDERR	0x013D	/* 升级命令--接收数据出错 */ 
#define UPGRADE_DATA_RECEIVEDAPPVER_LOW	0x023D	/* 升级命令--接收数据出错 */ 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR	0x033D	/* 升级命令--CRC校验出错 */ 


#define BOOT_ACTIVE_CMD_0701	0x0107   
#define BOOT_ACTIVE_CMD_0802	0x0208   
#define BOOT_ACTIVE_CMD_0806	0x0608   

//#ifdef  BOOT_CODE_JOIN_TEST_CODE

#define TEST_CODE_CMD_START	(0x5C)			//进入测试模式

//#endif

#define GET_BOARD_TYPE		0x40	// 支持版本(0x2015)

#define GET_BOARD_FUNC_TYPE		0x4F	/* 支持版本 0x2065
					  * 机头箱功能类型
					  * bit1-bit0: 系统数，1-4
					  * bit2 = 1: 支持编码器
					  * bit3 = 1: 支持压脚
					  */


#define GET_BOARD_BUILD_TIME		0x50    /* 支持版本 0x2065
					  * 机头箱代码编译时间
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
*	机头log输出 cmd
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
#define LOG_OUT_YJ1		0x1F	//压脚
#define LOG_OUT_YJ2		0x20
#define LOG_OUT_YJ3		0x21
#define LOG_OUT_YJ4		0x22
/* log out end */

// 说明:
// 版本信息最高4位代表系统数
// 如:
//    版本为0x2011->支持单双系统 版本号为0x0011
//    版本为0x3011->支持三系统 版本号为0x0011
//
// 请使用版本号为0x2010之后的程序
// 可以用版本查询命令(0x30)查询程序版本
//
//
