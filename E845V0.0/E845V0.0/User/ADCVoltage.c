/***********************************************************************
关于ADC采集转换操作
浙江恒强科技
软件平台部:F11500
2022/11/18
************************************************************************/
#include "ADCVoltage.h"
#include "adc.h"

CurrentSamplingFiltering CurrentSamplingClass={0};
/*
获得ADC值
ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
返回值:转换结果
*/
unsigned short Get_Adc_Hadc1(unsigned int ch)   
{
	ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ch;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  HAL_ADC_Start(&hadc1);                                //开启ADC
  HAL_ADC_PollForConversion(&hadc1,SZ_SAMPINGTIME);     //轮询转换(阻塞)
  return (unsigned short)HAL_ADC_GetValue(&hadc1);
}

/*
取连续采样中的最大和最小值
exv:电流采样集合体
*/

void SZ_CurrentSamplingGetExtremevalue(CurrentSamplingFiltering *exv)
{
		float Maxval=0.0,Minval=0.0;
		Maxval = exv->SamplesValue[0];
		Minval = exv->SamplesValue[0];
	for(int i=0;i<SZ_CURRENTSAMPLINGCOUNT;i++)
	{
		if(Maxval<exv->SamplesValue[i])
		{
			Maxval = exv->SamplesValue[i];
		}
		if(Minval>exv->SamplesValue[i])
		{
				Minval = exv->SamplesValue[i];
		}
	}
	exv->SamplesMaxVal = Maxval;
	exv->SamplesMinVal = Minval;
}
/*
求连续采样平均值
arg:电流采样集合体
*/
void SZ_ADCFindTheAverageValue(CurrentSamplingFiltering *arg)
{
		float Sum=0.0;
		//求和
		for(int a=0;a<SZ_CURRENTSAMPLINGCOUNT;a++)
		{
			Sum+=arg->SamplesValue[a];
		}
		//拿掉最大最小值
		Sum=Sum-(arg->SamplesMaxVal+arg->SamplesMinVal);
		//求平均
		arg->ValueSampleEnd =(float)(Sum/(SZ_CURRENTSAMPLINGCOUNT-2));
}
/*
纱嘴电流采样滤波:中位值平均滤波,采样N个值,去掉最大最小,计算N-2的平均值
*Csf:电流采样集合类
*/
 signed char SZ_CurrentSamplingFiltering(CurrentSamplingFiltering *Csf)
 {
			double ADC_Vol_OVERURRENTT=0.0;
			__IO  unsigned int ADC_ConvertedValue_OVERCURRENT=0;
			
			//采样
			if((Csf->NumberSamples<SZ_CURRENTSAMPLINGCOUNT)&&Csf->TransformationCompletionMark==0)
			{
					ADC_ConvertedValue_OVERCURRENT=Get_Adc_Hadc1(SHAZUIOVERCURRENT_CH);
					ADC_Vol_OVERURRENTT =(float) ADC_ConvertedValue_OVERCURRENT/4096*(float)3.27;
					if(ADC_Vol_OVERURRENTT>=SZ_NOLOADVOLTAGEADCVALUE)
					{	
						Csf->SamplesValue[Csf->NumberSamples]=ADC_Vol_OVERURRENTT;
						Csf->NumberSamples++;
					}		
					if(Csf->NumberSamples==SZ_CURRENTSAMPLINGCOUNT)
					{
						Csf->TransformationCompletionMark=1;
					}					
			}
			//拿出最大最小值，求平均值
			if((Csf->NumberSamples==SZ_CURRENTSAMPLINGCOUNT)&&Csf->TransformationCompletionMark==1)
			{
				SZ_CurrentSamplingGetExtremevalue(Csf);
				SZ_ADCFindTheAverageValue(Csf);
				Csf->TransformationCompletionMark=2;
			}
			//电流转化
			else if(Csf->TransformationCompletionMark==2)
			{
				Csf->ValueCurrentEnd=(unsigned int)(((SZ_NOLOADVOLTAGEADCVALUE-Csf->ValueSampleEnd)*1000/55)*1000);
				Csf->NumberSamples=0;
				Csf->TransformationCompletionMark=0;	
			}
			return 0;
 }




