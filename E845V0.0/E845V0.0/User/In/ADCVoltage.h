/***********************************************************************
关于ADC采集接口定义
浙江恒强科技
软件平台部:F11500
2022/11/18
************************************************************************/
#ifndef __ADCVOLTAGE__H
#define	__ADCVOLTAGE__H


#define  POWER012V_CH                           10     //电源DC12V检测通道
#define  SHAZUIOVERCURRENT_CH                   14     //沙嘴电流检测通道


#define  SZ_CURRENTSAMPLINGCOUNT                10     //纱嘴电流连续采样次数
#define  SZ_NOLOADVOLTAGEADCVALUE               1.647  //纱嘴空载ADC输出值

#define  SZ_SAMPINGTIME                         100
/*
纱嘴电磁阀
采样过程参数
*/
typedef struct
{
	unsigned char NumberSamples;                         //连续采样次数
	unsigned char TransformationCompletionMark;          //转化完成标志 
	unsigned int  ValueCurrentEnd;                       //最终有效转化电流值单位ma
	double        ValueSampleEnd;                        //最终滤波完成电压值(平均值)	
	float         SamplesMinVal;                         //连续采样中的最小值
	float         SamplesMaxVal;                         //连续采样中的最大值
	float         SamplesValue[SZ_CURRENTSAMPLINGCOUNT]; //连续采样值存储
}CurrentSamplingFiltering;


extern CurrentSamplingFiltering CurrentSamplingClass;

unsigned short Get_Adc_Hadc1(unsigned int ch);      
signed char SZ_CurrentSamplingFiltering(CurrentSamplingFiltering *);	
#endif














