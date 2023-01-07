/***********************************************************************
电机加减速参数生命
			STart/Stop
			脉冲/脉冲数量
			加减速
浙江恒强科技
软件平台部:F11500
2022/12/1
************************************************************************/

#ifndef __MOTORPWM__H
#define	__MOTORPWM__H

#define TIM1COUNTCYCLE    0.000005    //计数周期

/*
电机操作编号
*/
enum MOTORNUM
{
  DMMOTOR1=0,
	DMMOTOR2=1,
  DMMOTOR3=2,
	DMMOTOR4=3,	
};
/*
加减速状态标志
*/
enum PulseSpeedFlag
{
	CompletionAcceleration=1, //加速完成（到达目标速度）
	StartUpSpeed=3,           //开始减速
	CompleteDeceleration=2,   //减速完场（电机停）
	StartSlowDown=4,          //开始减速 
};

/*
电机加减速
属性参数
*/
typedef struct
{
	unsigned char StateFlag;             //过程状态标志参考 枚举PulseSpeedFlag	
	unsigned int  PulseCount[4];         //脉冲计数
	unsigned int  PulseTargetCount[4];   //目标脉冲数
	unsigned int  TargetSpeed[4];        //目标速度
	unsigned int  CurrentSpeed[4];       //当前速度（实时变化）		
}AccelerationAndDeceleration;
	


signed char TIM1ComparisonInterrupt(void);

#endif




