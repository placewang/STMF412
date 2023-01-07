/***********************************************************************
关于电磁阀的IO定义
浙江恒强科技
软件平台部:F11500
2022/11/17
************************************************************************/

#ifndef __SOLENIDVALVELO__H
#define	__SOLENIDVALVELO__H

#define SOLENOIDVAVELONUMBER  13

#define SV1_Pin 	GPIO_PIN_0
#define SV2_Pin 	GPIO_PIN_1
#define SV3_Pin 	GPIO_PIN_2

#define SV4_Pin 	GPIO_PIN_3
#define SV5_Pin 	GPIO_PIN_4
#define SV6_Pin 	GPIO_PIN_5
#define SV7_Pin 	GPIO_PIN_6
#define SV8_Pin 	GPIO_PIN_7

#define SV9_Pin 	GPIO_PIN_12
#define SV10_Pin 	GPIO_PIN_13
#define SV11_Pin 	GPIO_PIN_14
#define SV12_Pin 	GPIO_PIN_15

#define SHAZUI_POWERSWITCH_Pin 	GPIO_PIN_10

#define SV1_Port  GPIOB
#define SV2_Port  GPIOB
#define SV3_Port  GPIOB 
#define SV4_Port  GPIOB
#define SV5_Port  GPIOB
#define SV6_Port  GPIOB
#define SV7_Port  GPIOB
#define SV8_Port  GPIOB
#define SV9_Port  GPIOB
#define SV10_Port  GPIOB
#define SV11_Port  GPIOB
#define SV12_Port  GPIOB

#define SHAZUI_POWERSWITCH_Port 	GPIOB

#define SOLENIDVALVENUMBER                               6        //电磁阀数量
#define SOLENIDVALVETIMINGOPERATION                      10*2.5*1   //电磁阀打入打出时间间隔(MS)
 /*
电磁阀(沙嘴)操作编号
*/
typedef enum
{
	SHAZUI1_OUT=      0,
	SHAZUI1_IN=    1,	
	SHAZUI2_OUT=      2,
	SHAZUI2_IN=    3,		
	SHAZUI3_OUT=      4,
	SHAZUI3_IN=    5,		
	SHAZUI4_OUT=      6,
	SHAZUI4_IN=    7,
	SHAZUI5_OUT=      8,
	SHAZUI5_IN=    9,		
	SHAZUI6_OUT=      10,
	SHAZUI6_IN=    11,
	POWERSWITCH=     12                     //电源开关
}SolenoidValve;

/*
沙嘴通电时间
（电磁阀为磁保持，通电时间过长有烧毁风险）
*/
typedef struct
{
	unsigned char  OutMark[SOLENIDVALVENUMBER];                          //打出动作标记
	unsigned char  InMark[SOLENIDVALVENUMBER];                           //打入动作标记
	unsigned char  ActionSwitchFlag_OUT[SOLENIDVALVENUMBER];             //打出任务操作标记
	unsigned char  ActionSwitchFlag_IN[SOLENIDVALVENUMBER];              //打入任务操作标记
	unsigned short SV[SOLENIDVALVENUMBER];                               //电磁阀进/出通电时间记录                                                               
}SZPowerOnTime;

/*
电磁阀线圈电阻测试参数
*/
typedef struct
{
		unsigned char   OperationFlag[SOLENIDVALVENUMBER];                  //操作标志位	
		unsigned char   MeasureMarkPosition[SOLENIDVALVENUMBER];            //电阻测算标志位
		unsigned short	LastCoilResistanceValue[SOLENIDVALVENUMBER];        //前一次测算线圈电阻值
		unsigned short	CurrentCoilResistanceValue[SOLENIDVALVENUMBER];     //当前测算线圈电阻值
		unsigned short  Rtime[SOLENIDVALVENUMBER];                          //电磁阀进/出通电时间记录 	
}SZ_MeasuringResistance;

extern SZPowerOnTime SZ_PowerTime;
extern SZ_MeasuringResistance  CalculationOfResistance; 


void SolenidValveSetAllIn(void);
void SolenidValveSetAllOut(void);

signed char SolenidValveSetSwitch(SolenoidValve ,unsigned char );
void SZ_Timing_Interruption(SZPowerOnTime *szp,SZ_MeasuringResistance *cr);

signed SolenidValveSetOut(SZPowerOnTime *SZU,unsigned char Num);
signed SolenidValveSetIN(SZPowerOnTime *SIN,unsigned char pum);

signed char  CalculateCoilResistanceOfSolenoid(SZ_MeasuringResistance  *cr,unsigned char cunm);

#endif



