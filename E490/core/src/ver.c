/*
 * by xhl 2010/08/24
 */
#include "stdio.h"
#include "config.h"

//#include "Platform_config.h"
#include "string.h"

#if 0

#define VER80 	0x0080  /*by hlc 20150616 新款三系统机头箱*/ 
#define VER51 	0x0051  /*by hlc 20150715 新款三系统机头箱stm32f205*/ 

#define VER52 	0x0052  /*by hlc 20150826 新款三系统机头箱系统频率为120M*/ 

#define VER53 	0x0053  /*by hlc 20150910 选针过流不报警的解决*/ 

#define VER54 	0x0054  /*by hlc 20151228支持两路撞针信号*/ 

#define VER55 	0x0055  /*by hlc 20160220 E480板子AD采样基本OK,PWM方式输出*/ 

#define VER56 	0x0056		/*基本调试完成，E475和E480程序合并*/

#define VER57 		0x0057		/*by hlc 20160328支持动态度目*/

#define VER58 		0x0058		/*by hlc 20160418 ADC 分两个通道，ADC1和adc2*/

#define VER59 		0x0059		/*by hlc 20160428  新增获取CPLD 信息接口*/

#define VER60		0x0060		/*by hlc 20160507 新增测试电机命令*/

#define VER61		0x0061		/*by hlc 20160517正负24V过流的时候，靠过流检测来切断电源*/

#define VERf2		0x00F2		/*by hlc 20160518支持E490板子*/

#define VER63		0x0063		/*by hlc 20160704 支持E490 v1.0板子*/

#define VER64		0x0064		/*by hlc 20160708 支持压脚，支持编码器使能单个设置*/

#define VER65		0x0065		/*by hlc 20160708 支持压脚，支持编码器使能单个设置*/

#define VER66			0x0066		/*by hlc 20160716 解决动态度目的时候死机，初步怀疑是发送应答信号的时候，CAN死机,
									后面做了一个，出错指示灯。*/
									
#define VER67			0x0067		/*by hlc 20160813 加了一个动态的时候来不及的话检查并报警*/
									
#define VER68			0x0068		/*by hlc 20160901 来不及的情况续接的地方改了一下。*/

#define VER69			0x0069		/*by hlc 20161026生克零位范围之前没有用到，现在加上去*/

#define VER70			0x0070		/*by hlc 20161028过流报警只报一次，并且电源不关闭*/

#define VER71				0x0071		/*by hlc 20161101 增加一个接口用于取回当前机头板上的配置情况
									* 增加了编码器报警的时候报具体错误 
									*增加保险丝报警的时候具体保险丝号	
									*/

#endif
#if 0

//#ifdef LX_ACTION_STEP_SPECIAL

#define VER72				0x0072		/*by hlc 20161109 连兴动作电机结构复杂化,config.h中有一个宏定义#define LX_ACTION_STEP_SPECIAL  
									* 并且在初始值上面加用了连兴特有的主要就是零位宽度这里
									*/

//#else
#define VER				0x0073		/*by hlc 20161117  普通连兴程序
									*/

//#endif

#else

#if 0
//#define VER				0x0071		/*by hlc 20161101  依然是71版本*/

#define VER72				0x0072		/*by hlc 20161120 在桐乡，修改了几个功能，1，增加失败值显示，2、增加行程检查，3、自动找零降速*/

#define VER73				0x0073		/*by hlc 20161125 强隆五工位动作电机和三工位动作电机程序兼容*/

#define VER74				0x0074			/*by hlc 20161127选针分两次动，和不分两次动。LX不一样的效果*/

//#define VER				0x0075			/*by hlc 20161210 过流关闭第一次*/
//#define VER				0x00F5			/*by hlc 20161220 电机动作接续的改进，测试版本*/
#define VER76				       0x0076			/*by hlc 20161225 连兴动作电机过零修正的时候加了报警，失步值25
												运行完了之后的位置检查延时改了一下。20
												动作电机的启动频率由1100，改成了1500*/

#define VER77					0x77		/*20161229 by hlc 过流检测做了大的调整
												度目走过头的情况下复位的时候一边走走看，再调头*/

#define VER78					0x78		/*20170103 by hlc 模式5下面，推针不用编码器默认浩丰模式。
												20170104		编码器报警这里有个do while(1)时间太长导致CAN溢出*/

#define VER79						0x79  			/*20170105 by hlc LX选针的问题以及LX动作出零位修正,
													*20170113 byhlc 电机复位的时候，就算走过了零位也不做检查。
													*/

#define VER80						0x80			/* 20170116 byhlc   电压欠压报警延迟10ms(做滤波)再报警    */
												//模式5下面，推针不用编码器不默认浩丰模式。与78版本相反
#define VER81						0x81			/* 20170117 byhlc  电源控制加了清报警开电功能 */
													//2017 01 19 by hlc 连兴要求度目电机可以走到负方向目前由机头宏定义
													//2017 0207 by hlc 连兴动作5.5K  版本F3
													//2017 0207 byhlc 越发支持动作电机范围正负不对称 F4
													//2017 0209 byhlc 设置电机自动找零的时候最小范围
													//2017 0213byhlc 传感器进出误差限制在一定范围内。 F6_V81

#define VER82							0x82			//20170302 byhlc 电机启动时间延迟10太小改100us
													//20170302byhlc 连杆电机断电续织，失步检查范围放大。
													//20170307byhlc 编码器读数的时候，不要立即更新，当它正确的时候再赋值
													//20170307 by hlc 强隆，三角电机要求2K。度目启动速度降到1.5K，实际1.1K
													//20170307by hlc 加了命令支持的检查功能
													//20170307by hlc 编码器位置检查的时候加了一个编码器状态的条件
													//20170313by hlc 浩丰模式的推针电机可以配置

#define VER83							0x83			//20170317 byhlc 国光支持压脚，支持连兴三角电机，位置接在
													//20170320 byhlc	测阻抗的时候，如果选择刀片数量设置不为8的话，关闭选针那里会漏关。
													// 导致阻抗检查一直通不过。
													// 电源检查这里，掉到20V以下的报警时间由原来的10ms改成15ms。
													//20170321by hlc 生克过零位的时候自动修正。超过报警值的时候报警 

#define VER84							0x84			//20170331 byhlc 电机命令接续的问题改了一下。
													//20170401 byhlc HP款动作电机的到位检查改了一下。
													//20170408 byhlc 生克电机重复命令的时候，零位状态被更新，导致过零检查失效。电机走到底

#define VER85							0x85			//20170411 LX  测试238报警的原因版本由原来的84改成F4
													//20170411 连兴测试生克有问题


#define VER86							0x86			//20170417 byhlc 连兴生克电机，负数到0，再到正数的接续问题还是要和之前的做法一样，检测到零位直接换向，不带减速
													//20170419 byhlc  纱嘴清除检查这里改成了带参数

#define VER87								0x87			//20170419 byhlc 增加测试命令F6
														//20170424by hlc 增加连兴动作电机多工位切换矫正功能。	F7
														//动作电机走1980步还没停下来的话报警停下。

#define VER88								0x88		//20170425byhlc 测试撞针
													//20170426 byhlc 连兴要求纱嘴电磁铁动作后面加时间参数
													//20170427 by hlc 机头低电流持续输出，报警
													//20170502 by hlc 改成正式版本88


#define VER89								0x89      //20170519 byhlc 必沃测试机头上带撞针功能
												//20170525byhlc 选针可以重定向第一刀的位置。
												//2017 0607 byhlc  超温报警默认120℃
												//2017 0612 by hlc 12V短路报警
												//2017 0612 by hlc 普通运行半流模式版本f9
												// 2017 0612 by hlc 推针电机带传感器的，归零过程加了零位滤波
												//2017 0624 byhlc 度目电机最小检查精度，放大默认20个步。


#define VER90									0x90   //20170628 by hlc APP自我诊断是否与硬件版本冲突。
												  //2017 06 29 by hlc电机带编码器失步的时候，再做一次表示补救一次。
												  
#define VERF2									0xF2//0x91 // 20170630 by hlc 动作电机工作位检查报警区分开
													/*F2 强隆半流测试程序*/

#define VER91									0x91  /*byhlc 20170714子号改一下，主要是生克归零的时候间隙可能比较大*/

#define 	VERF3									0xF3	/*by hlc20170718 加了电机完成之后的应答,以及日志开关*/

//#define VER										0xE8	/*by hlc 20170721 EE报警加到日志*/ /*选针数据还原到主循环做，CAN发送使用中断模式*/
//#define VER										0xEA		/*by hlc 翱翔客户动作电机失步报警放到100*/

#define VER92										0x92	/*byhlc 20170815 几个保险丝失效的报警判断时间加大了200ms*/

#define VER93										0x93		/*by hlc 20170822 三个711的报警全部屏蔽掉711不可靠，CAN发送不用中断模式*/

#define VER93_02									0x93		/*by hlc 20170822  电机回零的时候提前减速*/

#define VERFE											0xFE				/*by hlc 20170823 支持飞虎动作电机*/

#define VER94											0x94			/*by hlc 20170826 711报警不能屏蔽，会出现烧板子的问题*/

#define VER95												0x95_	/*by hlc 201708231 CAN发送使用队列在定时器里发送*/
//#define VER												0xfe	/*by hlc 20170905 选针器清高压这里和选针设置起刀与初始化的命令时机做到无关*/
#define VERFD												0xfd	/*by hlc 20170912 电机设置位置的时候需要把pos_2_main一起设置掉。不然默认0的话走零的命令不执行.
																				顺便把默认值设置为2000*/
#define VER95												0x95      /*by hlc 20170913 0XFD的基础上出的正式版*/
																/*by hlc 20170913 电磁铁自检这里有一个bug一直没有测试出来。*/

//#define VER												0xfc	/*by hlc 20170913 慈星测试*/

#define VER96												0x96	/*by hlc 20170927 支持风扇控制*/

#define VER9696												0x96	/*by hlc 20171018 支持IO模式的电磁铁不显示自检数据*/
																/*by hlc 20171024 支持同时取电机位置的命令*/
#define VER96													0x96	/*20171024 by hlc 关闭EF报警*/

//#define VER													0x97	/*20171101 电机电流设置加大10%*/
#define VERF7													0xF7	/*20171110推针电机加了零位方向设置功能*/
																	/*20171114 选针设起始刀的命令，立即生效的模式与6.1命令冲突。*/				
																	/*2017 11 15 by hlc 把671和670命令之间的关联取消掉。谁来了就做谁*/
																	/*2017 11 16 by hlc 把24伏电源检查报警这里做了上电时候的延时操作，正负各自区分*/
																	/*2017 11 21 by hlc 把纱嘴未清零的报警0x30,屏蔽掉，直接把未清的状态改变过来*/
																	/*0xF7*/		/*2017 11 22 by hlc 机头模式增加0x0A*/
//#define VER							0xF7							/*V97 20171127 by hlc 连兴动作电机，工位之间切换的时候修正的功能最后走到位的时候采用归零速度(2kHZ)*/
																	/*2017 11 27 by hlc 修改了压脚一个bug*/
																	/*2017 11 28 by hlc 推针电机是否到位检查*/
																	/*2017 11 28 by hlc 新增AN款机头箱支持推针电机(放在生克后面)*/		
																	/*2017 12 01 by hlc 新增纱嘴零位信号检查宏定义YARN_ZERO_CHECK*/
#define 	VER98								0x98						/*2017 12 01 by hlc 连兴山板的动作电机小行程的时候电机速度降低到3.5K，减少失步的可能性*/
																	/*2017 12 19 by hlc 连兴山板的动作电机，命令接续的时候跑错分支了*/
																	/*2017 12 20 by hlc 浩丰模式的推针电机，过零修正的时候需要计算到正中间 */
																	/*2017 12 29 by hlc 0xED测试代码里的报警信息关闭*/
																	/*2018 01 03 by hlc can发送那里，延时等待的时间10ms太长，遇到特殊情况下，CAN接收缓冲区会溢(0.1ms一个数据包，10ms就100条数据了)*/
																	/*2018 01 04 by hlc 连兴度目电机过零修正，不报警。*/
#define 	VER99								0x99					/*2018 01 12 by hlc 连兴动作电机过零修正的时候发现走不出零位，之前的做法是加走200步，实际会导致电机反走。需要去掉这个200*/
#define 	VERF9								0xF9					/*2018 01 16 BY hlc 临时版本，推针失步值设置为50，通过宏定义TZ_LOST_ALARM_STEPS_50*/							
#define 	VERF8								0xF8					/*2018 01 17 by hlc 连兴度目电机要求回零速度加快到6K。失步报警设置成100   LX_DM_MOVETOZERO_FAST_SPEED*/
#define 	VERf7 								0xF7					/*2018 01 17 by hlc 慈星度目电机电流默认1800,宏定义CX_DM_CURRENT_DEFAULT_1800*/
#define 	VERF6 								0xF6					/*2018 01 17 by hlc */
#define 	VER99 								0x99					/*2018 01 29 by hlc 两个命令接续的时候，后续速度变慢导致下一条回零命令出错。*/

#endif
#define 	VER									0x99					/*2018 01 29 by hlc 温度检测报警这里，之前定义成无符号了。出现零下的时候，就会报警,9A版本直接关闭该功能*/

#ifdef DEBUG_MT6813_PWM
#define DEBUG_VER
#endif


//#define FH_TEST_ACT_SAMPLE

#ifdef FH_TEST_ACT_SAMPLE
#define DEBUG_VER
#endif

#ifdef DEBUG_VER

#ifdef VER
	#undef 	VER	
#endif

#define 	VER 								0xFF /*测试版本*/

#endif


#ifdef STEP_MOTOR_RUN_HALF_MODE
	#ifdef VER
		#undef VER
		#define VER	0xF9
	#endif

#endif

//#define VER							0xf8


#ifdef LX_DEBUG
#define VER							0xF5			//2017 0214byhlc 连兴动作电机运行结束后的状态检查，之前改坏了，提前设置后面又初始化了。82
													//2017 02 15 by hlc 连兴度目一直报警F2
													//2017 02 22 by hlc 连兴度目一直报警,0-0-0的情况之前状态不对就自动归零了现在不归零
													//2017 02 28 度目走到负数，判断负数的来源
													// 2017 0310 度目确实出现-1的值，动作的时候，判断值由0改成-5   版本F5
													
													
													
#else
#define VER84							0x84			//20170216 byhlc 加了一个接口设置零位走进去的步数
													//20170220byhlc 把BC报警细分
													//20170223byhlc 把  动作默认速度3K--强隆F4 		
													////20170223byhlc 把  动作默认速度2K--强隆F5
													// 84版本测试探针反不反
#endif
													
#endif







//#define test_debug
#ifdef test_debug
	#define VER 0x0002
#endif

//#define TEST_VER	0x1250
#ifdef EMF_HALF	
  #define EMF_OFF_HALF	0x1
#else
  #define EMF_OFF_HALF	0x0
#endif

#ifdef VERTEST
#define VERSION	((EMF_OFF_HALF << 8) | \
		 (VERTEST))
#else
#define VERSION	((EMF_OFF_HALF << 8) | \
		 (VER))
#endif

#define HEAD_BUILD_YEAR 	(2016)
#define HEAD_BUILD_MON		(3)
#define HEAD_BUILD_DAY		(28)

#define HEAD_BUILD_HOUR	(12)
#define HEAD_BUILD_MIN		(0)
#define HEAD_BUILD_SEC		(0)

#include "arch.h"	 

extern volatile unsigned int BootVer_int;



unsigned int arch_System(void);
int Jtag_Security_State(void);

unsigned int Get_VER_data()
{
	return VER;
}


unsigned int Get_Version()
{
	unsigned int version = VERSION;

	version |= arch_System() << 12;

	if(Jtag_Security_State() == 0) {
		version |= 0x0800;
	}

	if (BootVer_int)
	{
		version |=((BootVer_int & 0x0F)<<8);	
	}
	
#ifdef TEST_VER
	return TEST_VER;
#endif
	return version;
}

#if 0
unsigned int  Get_Build_Time(void)
{
	unsigned int  buf =0;
	
	buf |= ((HEAD_BUILD_YEAR - 2000) << 9);
	buf |= (HEAD_BUILD_MON << 5);
	buf |= (HEAD_BUILD_DAY);
	return buf;
}
#else

void Get_SYS_Data_Time(unsigned char *Year, unsigned char *Month, unsigned char *Day,unsigned char *Hh,unsigned char *Mm,unsigned char *Ss)
{    	const char *pMonth[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};    
	const char Date[12] = __DATE__;//取编译日期
	const char CTime[12] = __TIME__;//取编译时间
	
	unsigned char i;     
	for(i = 0; i < 12; i++)
	{
		if(memcmp(Date, pMonth[i], 3) == 0)
		{
			*Month = i + 1;
			i = 12;
		}	
	}
	*Year = (unsigned char)atoi(Date + 9); //Date[9]为２位年份，Date[7]为完整年份    
	*Day = (unsigned char)atoi(Date + 4);
	*Hh = (unsigned char)atoi(CTime);
	*Mm = (unsigned char)atoi(CTime+3);
	*Ss = (unsigned char)atoi(CTime+6);
	
}

unsigned int  Get_Build_Time(unsigned int *buf)
{
	unsigned char month, day, year,Hh,Mm,Ss;
	unsigned int time=0;
	Get_SYS_Data_Time(&year,&month,&day,&Hh,&Mm,&Ss);
	time = ((year) << 9);
	time |= (month << 5);
	time |= (day);
	time |= (Hh <<27);
	time |= (Mm <<21);
	time |= ((Ss>>1) <<16);

	if(buf)
	{
		*buf = time;
	}	
	return time;
}

#endif

#if 0
#define VER_CH01				0x01		/*20170607 by hlc 子版本号，占用1个字节，用BCD码表示(00-99)*/
#define VER_CH02				0x02		/*20170624 by hlc 子版本号*/
#define VER_CH03				0x03		/*20170714 by hlc 子版本号,改了生克归零的时候间隙大的问题*/
#define VER_CH04				0x04		/*20180131 by hlc 子版本号,超温报警,直接关闭掉了。*/
#define VER_CH05				0x05		/*20180206 by hlc 位置相同的情况下，发命令，不检查传感器状态*/
#define VER_CH06				0x06		/*20180209 by hlc 4-9-2命令bug修改。*/
#define VER_CH07				0x07		/*20180223 by hlc  电机里面的特性做了合并处理(度目电机根据零位信号是否走过头分连兴和非连兴，
																							三角电机分HP款，F型，普通款，
																							推针分浩丰款，非浩丰款)。*/
#define VER_CH08					0x08		/*2018 03 02 by hlc 机头主控绑定功能*/
#define VER_CH09					0x09		/*2018 03 08 by hlc 推针电机断电续织设置传感器宽度问题*/
#define VER_CH0A					0x0A		/*2018 03 10 by hlc 新增一个类型，通过物理ID号定义电机位置*/
#define VER_CH						0x0B		/*2018 03 12 by hlc 新类型的电机，零位检查范围加大20个点。浩丰推针电机，过零需要检查信号状态切换*/


//#define VER_CH						0x0C		/*2018 03 27 by hlc 浩丰推针电机,连续走零命令导致归零动作，并且加了位置检查*/

//#define VER_CH						0x0D		/*2018 03 30 by hlc 连兴款度目电机做动态度目的时候，状态检查报1-1报警，把这个检查取消*/

//#define VER_CH						0x0E		/*2018 04 02 by hlc 连兴其它电机动作的时候带速度值下来*/

//#define VER_CH						0x0F			/*2018 04 02 by hlc 浩丰的推针电机，需要实时查一下零位状态*/

//#define VER_CH						0x10		/*2018 04 08 by hlc 把纱嘴导通，三角电磁铁导通这两个报警放到日志里去F2开头
//																LX动作电机MOVETOZERO的时候提前减速，导致来不及，需要放开，到20步的时候减速*/
														
//#define VER_CH						0x11		/*2018 04 09 by hlc 生克电机支持零位设置
												//2018 04 09 by hlc  连兴动作电机零位修正值，LX默认是3，慈星结构有改，应该是(-1)*/
												//2018 04 09 by hlc  连兴动作电机9-4报警，做了优化	
												//
//#define VER_CH						0x12		//2018 04 12 by hlc 强隆要求AN款做简易双系统
												//2018 04 19 by hlc 回零的时候如果是简易电机，就不报失步

//#define VER_CH						0x13	       //2018 0422 by hlc 连兴动作电机根据行程自动减速功能恢复

//#define VER_CH						0x14	       //2018 0428 by hlc 慈星连杆电机带修正的时候，提前到零位了，最终需要修正到正确位置。 


//#define VER_CH						0x15	       //2018 0509 by hlc 连兴生克特殊（零位在中间，宽度比较大）

//#define VER_CH						0x16	       //2018 0602 by hlc 0XAF报警屏蔽

//#define VER_CH						0x17	       //2018 0607 by hlc 22版本有一个测试代码在，需要改用23版本
											//2018 06 09 by hlc   电机动作完成的应答加了零位状态。
											//2018 06 15 by hlc	简易双系统动作电机修正值由20改成4

//#define VER_CH						0x18	       //2018 0615 by hlc FF-23-03  之后的正式版本
											//2018 0619 by hlc 连兴动作电机报警比较多， 实时检查关闭掉--测试版本
											//2018 0620 by hlc 连兴动作电机根据行程自动匹配速度的表格多了两项--测试版本

//#define VER_CH						0x19	       //2018 06 21 by hlc 连兴普通生克电机按照步数走，5次累计报警之后才报警
//#define VER_CH						0x1A		//2018 06 26 byhlc 电机在修正位置的时候，由于STEP=0。此时POSTION值不应该变化
//#define VER_CH						0x1B		//2018 00704byhlc 生克最大值加大到1200
//#define VER_CH						0x1C		     //2018 00705byhlc 连兴动作电机，0位走出去离不开零位的时候加走90步。
												//20180706 by hlc 连兴生克命令接续的问题
												//20180706 by hlc 推针电机的电流调节上限加到2.58A
//#define VER_CH						0x1D			//20180707 by hlc 连兴生克一直在零位内运动,过零修正的时候失步报警统一100

//#define VER_CH						0x1E			//20180707 by hlc 连兴生克普通生克过零修正失步报警值200
//#define VER_CH						0x1F			//20180708 by hlc 连兴动作电机补偿值加到200
//#define VER_CH						0x20			//201807010 by hlc 连兴加了日志功能。
												//20180713 by hlc 把失步检查全部默认成0.版本不变

#define VER_CH						0x21			//20180716 by hlc 新增HP款机型带编码器的度目电机工作位完成应答功能
												//20180716 by hlc 	新增探针状态获取
												//20180716byhlc 生克速度可以带下来。
												//20180723 by hlc 度目零位走进去步数由10步改成6步(模式8).连兴舢板的非零区域宽度加大(100->150)

#define VER_CH						0x22			//20180808 by hlc 必沃普通动作电机也要加翻针传感器检查 

#define VER_CH						0x23			//20180816 by hlc 连兴新的浮沙电机模式。
												//20180824 byhlc 拉杆反复启停导致度目电机方向错的问题
												//20180824 byhlc 简易双几个参数调整。4，2000，


#define VER_CH						0x24			//20180905 by hlc 新增设起刀命令0x15，0x16。表示0X55AA	
#define VER_CH						0x25			//20181102 by hlc 压脚工作位置的值根据电机号子分开。04-07-02-命令扩展
#define VER_CH						0x26			//20181112 by hlc 机头与主控之间的绑定做了优化升级。 



#define VER_CH						0x27			//20181220 by hlc 机头箱在断电情况下电机不再继续执行需要急停。
#define VER_CH						0x27			/*2018 12 27 by hlc 之前的39版本选针器自检会常通，导致烧器件	*/
												/*考虑到发出去的版本比较少，可控范围内。所以目前改过的版本号不抬升*/
												/*20190109 慈星测试，工作过程中零位没信号的时候，不做归零动作*/
												/*2018 12 27 by hlc 生克电机最大行程由原来的1200改成2000，满足国盛的使用*/
#define VER_CH						0x28			/*20190110 by hlc 关闭711报警*/
#define VER_CH						0x29			/*20190124 by hlc 开放动作电机自动修正的使能设置0x04-0x32命令*/
#define VER_CH						0x2A			/*20190128 by hlc  PB12配置悬空输入，纱嘴过流保护设置12A检测10次过流报警*/
#define VER_CH						0x2B			/*20190130 by hlc  十段选针支持*/



#define VER_CH						0x2C		/*20190221 by hlc 1、动作电机实时检查报警状态命令支持
															2、其他电机输入信号可以设置常开常闭
															3、支持度目电机负数方向零位信号不检查
															4、20190301 增加设置自动上报的命令*/
#define VER_CH						0x2D		/*20190304 by hlc 慈星压脚工作方式特殊*/	
											/*2019 03 21 by hlc 1.5A过流切电延时由之前10s改成5s*/

															
#define VER_CH						0x2E		/*20190327 by hlc 浩丰推针电机归零报警0-4 修复*/	
#define VER_CH						0x2F		/*20190409 by hlc 1、慈星全成行机型，支持选针器方向互换
															2、连兴选针动作可以设置延时
															3、连兴度目电机宽度与零位误差需要可读可设置
															4、20190410 推针电机走多了不报警--过零修正的位置
															5、20190410 动作电机命令接续的问题*/	




#define VER_CH	    					0x30		/*20190422 by hlc  选针器阻抗测试带时间参数*/
											/*20190430 by hlc 新增新的推针模式*/


#define VER_CH	    					0x31 	/*20190511 by hlc  模式2的时候推针电机零位方向没有初始化*/
#define VER_CH	    					0x32      /*20190611 by hlc 电机中断设置的时候关闭总中断*/
#endif

#ifdef LX_ACT_SPECIAL
#define VER_CH34	    					0x34		/*20190621 连兴特殊版本号*/ 
#define VER_CH35	    					0x35		/*20190701 连兴特殊版本号支持动作电机回零不走修正*/ 
#define VER_CH38						0x38		/*20190810 连兴特殊版本号6.3命令来的急的情况下之前会选针出错*/
#define VER_CH3F						0x3F		/*20191108 连兴特殊版本号6.3 状态和6.2命令保持一致*/
#define VER_CH44						0x44		/*20200622 0x43的连兴版本*/
#define VER_CH46						0x46			/*20200810 信龙F型山板缺口支持15步*/
#define VER_CH4D						0x4D			/*20210223连兴动作三角命令接续的时候走零，也要能区分直接到零和走穿返回的模式*/
#define VER_CH50						0x50			/*20210310,之前三角电机计算零位宽度的时候，有一个参数减小了，后面影响了传感器误差*/
#define VER_CH52						0x52			/*202100408 LX生克电机9-4报警，检查条件不对。*/
#define VER_CH53						0x53			/*20210708 支持硬件删除711器件,本来是0x54版本，
													由于谢工提前发布了版本号，
													只能编译成0x53版本，
													以后还是连兴用偶数模式。
												*/
//#define VER_CH						0x54			/*by hlc 20220119连兴度目在信号内的时候走0，需要先走出再回来*/									
#define VER_CH56						0x56			/*by hlc 20220315连兴度目动态的时候不考虑负数位置的信号检查*/									
#define VER_CH						0x58			/*by hlc 20220929 连兴电机纱嘴类似于推针电机*/									

#else
//#define VER_CH	    					0x33      /*20190618 by hlc 紧吊电机用的生克接口，之前生克的检查范围比较大，现在调成50*/
//#define VER_CH	    					0x36      /*20190704 by  hlc 机头模式8下面生克默认编码器0的bug处理*/
//#define VER_CH37	    					0x37      /*201900801 by  hlc 简双模式下面，回零步数走太多，改成走到零位就停*/
//#define VER_CH						0x39		/*20190810 6.3命令来的急的情况下之前会选针出错*/
//#define VER_CH						0x3A		/*20190819 6.2命令扩展成0x16.X2命令，选针器命令带序号下来（3bits），做检查是否重复*/

//#define VER_CH						0x3B		/*20190826 普通动作电机默认检查0位，工作位带下来检查*/
//#define VER_CH						0x3C			/*20190925 紧编电机支持零位在中间*/
//#define VER_CH						0x3D			/*201901012纱嘴异常命令报警关闭*/
//#define VER_CH						0x3E			/*201901028 慈星编码器款二段度目支持*/
//#define VER_CH						0x40			/*201901111 6.3 状态和6.2命令保持一致,支持压脚重定向*/
//#define VER_CH						0x41			/*20191217 硬件版本不检查*/

//#define VER_CH						0x42			/*20191227 编码器重修正的时候如果不在线就触发报警*/

//#define VER_CH						0x43			/*20200522   电磁铁过流报警*/
//#define VER_CH						0x44			/*20200720   必沃电机加减速放缓，启动频率减低*/

//#define VER_CH						0x45			/*20200807 新龙9004报警关闭*/
//#define VER_CH						0x46			/*20200825 电源关闭的时候电机释放使能*/
//#define VER_CH						0x47			/*20200911 电机配置的情况下undefined 模式会导致全局数据被改的BUG修复*/
//#define VER_CH						0x48			/*20201010 编码器的电机编码器方向由电机零位方向指定。。。。*/
//#define VER_CH						0x49			/*20201123 中间一个点的生克电机，过零的时候加检查。*/
//#define VER_CH						0x4A			/*20210114 屏蔽711报警*/
//#define VER_CH						0x4B			/*20210118强隆版本*/


//#define VER_CH						0x4C			/*20210119编码器降成本模式*/
//#define VER_CH						0x4E			/*20210301,选针器刀头未清报警*/
//#define VER_CH						0x4F			/*20210310,之前三角电机计算零位宽度的时候，有一个参数减小了，后面影响了传感器误差*/
//#define VER_CH						0x50			/*20210514,推针电机零位方向在正数的时候，归零命令和动作命令接续的时候，有问题*/
//#define VER_CH						0x51			/*20210603  关闭711报警检查*/

//#define VER_CH						0x53			/*20210708 支持硬件删除711器件*/
#ifdef NO_CHECKINPUT_FOR_LXACT_REALTIME
#define VER_CH						0x55			/*20210713 LX act 不做实时检查--特殊版本*/
#else
//#define VER_CH						0x53			/*20210708 支持硬件删除711器件*/
//#define VER_CH						0x57			/*20211018 支持动作电机负数位置到0的时候，如果出零了就归零一下*/
//#define VER_CH						0x59			/*20211108 编码器成本降低之后的版本，开机第一次编码器转位置值没有实现*/
//#define VER_CH						0x5B		/*20211230 点感应的三角电机，自动检测范围由350加到400*/
//#define VER_CH						0x5D		/*20220209 之前0x44版本(20200720)的功能通过宏定义区分FOR_BW_DM_SLOWLYRUN，避免其它厂家动态度目差异*/
//#define VER_CH						0x5F			/*20220330 编码器成本降低之后的版本(PW模式),失步报警的时候多检查一次*/

//#define VER_CH						0x60			/*20220401 报警的时候机头停止比较慢，每条命令清报警改成16ms自动重报一次，直到07-01 清除报警*/
//#define VER_CH						0x61			/*20220405 支持普通动作电机零位补偿的时候不报警*/
//#define VER_CH						0x63			/*20220429 编码器值读取加了很多过滤条件*/
//#define VER_CH						0x64			/*20220712 99.100版本，非慈星版本，取消动作电机实时检测功能*/

#ifdef SHIELD_LIFT_ALARM
#define VER_CH						0xFE		/*20221125 特殊版本，屏蔽推针电机报警*/ 
#else
#define VER_CH						0x65		/*20220810 99.101版本，解决电机调头多次执行的问题*/ 

#endif

#endif
#endif


#ifdef DEBUG_VER
	#ifdef VER_CH
		#undef 	VER_CH	
	#endif
	#define 	VER_CH 								0xFF /*测试版本*/

#define VER_LAST 						0xffff

#else

#define VER_LAST 						VER|((VER_CH-1)<<8)

#endif


//#define VER_LAST 						VER|((VER_CH-1)<<8)


/*获取特殊属性
*返回四个字节,第一个字节表示版本号-子号(01-99)--BCD码
*第二个字节表示是否为特定厂家(0x00-0xFF):关于特定厂家:0x00表示普通厂家(非指定厂家)、0x01表示慈星，0x02表示连兴，0x03表示强隆，0x04表示....
*第三，第四个字节备用，默认为0000.
*/
unsigned int Get_Special_attr(void)
{
	extern volatile unsigned char Step_use_exBoard;
	unsigned int ret=0;
	ret |=VER_CH;
	#ifdef QL_STEPMOTOR_START_LOW
	ret |= (0x03 <<8);
	#else
	ret |= (0x00<<8);
	#endif
	ret |= ((Step_use_exBoard?1:0)<<16);	
	return ret;

}

unsigned short lastAPP_ver=VER_LAST;

const unsigned int BootVer_ex   __attribute__((at(0x08020200))) = 0xaaddccee;
const unsigned int BootVer_app __attribute__((at(0x08020204))) = VER | (VER_CH<<8);


