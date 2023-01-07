#include <stdio.h>
#include <stdlib.h>
#include "stm32f2xx_adc.h"
#include "stm32f2xx_dac.h"
#include "stm32f2xx_dma.h"
#include "stm32f2xx_rcc.h"
#include "Alert.h"
#include "Platform_config.h"
#include "stm32f2xx.h"



extern void arch_Power_ctrl_ex(unsigned char whichpower,unsigned char onoff,unsigned char whichcp);
extern unsigned int arch_need_current_add(unsigned int currentdata);
extern void printf_enable(void);
extern void myprintf(const char *format, ...);

extern void GPIO_SET_711_FAULT_AN(unsigned char isN);
#if 0
#ifndef uint32_t
typedef unsigned long uint32_t;
#endif
#ifndef uint16_t
typedef unsigned short uint16_t;
#endif

#endif
/* //(28000/21) --28V                                 //(32000/21)  //32V*/
#define DC24_MAX				(1333)	
/*//(20000/21= 952 ) --20V                                 //(16000/21=762)  //16v*/
#define DC24_MIN				(952)	

/*//(15000*15/215)  //15V*/
#define DC12_MAX				(1046)
/* //(628)	//(9000*15/215)  //9v    (697)   //(10000*15/215)  //10v */
#define DC12_MIN				(697)	
/* //(28000/21) --28V                                 //(32000/21)  //32V*/
#define DC24_N_MAX				(1333)	
/*//(20000/21 = 952) --20V                                 //(16000/21=762)  //16v   */
#define DC24_N_MIN				(952)	

/*   //(16000/21=762)  //16v   */
//#define DC24_N_MIN_EX				(762)	

/*   //(14000/21=667)  //14v   */
#define DC24_N_MIN_EX				(667)	


/*   //(16000/21=762)  //16v   */
#define DC24_P_MIN_EX				(762)	



/* 3A, 1650+3*55 =1815*/
#define DC24_LONG_MAX_A_N	(1815)

/* 1.5A, 1650+1.5*55 =1733*/
#define DC24_LONG_MAX_A_N_LOW	(1733)


/* 6A, 1650+6*55 =1980*/
/* 8A, 1650+8*55 =2090*/
#define DC24_LONG_MAX_A_P	(2090)

#define DC24_LONG_MS_N		(1500)
#define DC24_LONG_MS_P		(1000)
/*小电流持续长时间*/
#define DC24_LONG_MS_N_LOW	(5*1000)

#define DC24_LONG_HLL_MS_N	(200)
#define DC24_LONG_HLL_MS_P	(200)


enum {
	POWER_LIST_INDEX_P24_H = 0,
	POWER_LIST_INDEX_P24_L,	
	POWER_LIST_INDEX_N24_H,
	POWER_LIST_INDEX_N24_L,
	POWER_LIST_INDEX_P12_H,
	POWER_LIST_INDEX_P12_L,
	POWER_LIST_INDEX_P24_L_EX,		/*正24V极低*/
	POWER_LIST_INDEX_N24_L_EX,		/*负24V极低*/
	
	POWER_LIST_INDEX_N3A_LONGTIME,	
	POWER_LIST_INDEX_N1A5_LONGTIME,
	POWER_LIST_INDEX_P8A_LONGTIME,	
	
	POWER_LIST_INDEX_P3A_LONGTIME,	
	POWER_LIST_INDEX_P1A5_LONGTIME,
	POWER_LIST_INDEX_N8A_LONGTIME,	
	
	POWER_LIST_INDEX_MAX,
};
	
//unsigned int power_isalert_last_timer[POWER_LIST_INDEX_MAX];//0--+24_H,1--+24_L,2---24_H,3---24_L,4--+12_H,5--+12_L


typedef struct {
	unsigned char index;
	char *title;
	unsigned int last_time_tick;
	unsigned int time_out_data;		/*单位是250us*/
	unsigned int compar_val;		/*比较值*/	
	unsigned char islessthan;		/*是小于?*/
	unsigned char isok_cnt;
}POWER_ALERT;

enum
{
	data_is_greater_than_max=0,
	data_is_less_than_min,
	data_compar_type_max,
};

POWER_ALERT power_isalert_last_timer[]={
	{POWER_LIST_INDEX_P24_H,"+24H",0,DC24_LONG_HLL_MS_P<<2,DC24_MAX,data_is_greater_than_max,0},/*正24V过压*/
	{POWER_LIST_INDEX_P24_L,"+24L",0,DC24_LONG_HLL_MS_P<<2,DC24_MIN,data_is_less_than_min,0},/*正24V欠压*/
	{POWER_LIST_INDEX_N24_H,"-24H",0,DC24_LONG_HLL_MS_N<<2,DC24_N_MAX,data_is_greater_than_max,0},///*负24V过压*/
	{POWER_LIST_INDEX_N24_L,"-24L",0,DC24_LONG_HLL_MS_N<<2,DC24_N_MIN,data_is_less_than_min,0},/*负24V欠压*/
	{POWER_LIST_INDEX_P12_H,"+12H",0,20<<2,DC12_MAX,data_is_greater_than_max,0},/*+12V过压*/
	{POWER_LIST_INDEX_P12_L,"+12L",0,20<<2,DC12_MIN,data_is_less_than_min,0},/*+12V欠压*/
	{POWER_LIST_INDEX_P24_L_EX,"+24LL",0,50<<2,DC24_P_MIN_EX,data_is_less_than_min,0},/*正24V电压极低*/
	{POWER_LIST_INDEX_N24_L_EX,"-24LL",0, 50<<2,DC24_N_MIN_EX,data_is_less_than_min,0},/* 负24V电压极低*/
	
	{POWER_LIST_INDEX_N3A_LONGTIME,"-3A",0,DC24_LONG_MS_N<<2,DC24_LONG_MAX_A_N,data_is_greater_than_max,0},/* 3A长时间通电*/
	{POWER_LIST_INDEX_N1A5_LONGTIME,"-1A5",0,DC24_LONG_MS_N_LOW<<2,DC24_LONG_MAX_A_N_LOW,data_is_greater_than_max,0},/* 负24V 1.5A  持续10s*/
	{POWER_LIST_INDEX_P8A_LONGTIME,"+8A",0,DC24_LONG_MS_P<<2,DC24_LONG_MAX_A_P,data_is_greater_than_max,0},/* +24V 8A长时间通电1S*/

	{POWER_LIST_INDEX_P3A_LONGTIME,"+3A",0,DC24_LONG_MS_N<<2,DC24_LONG_MAX_A_N,data_is_greater_than_max,0},/* 3A长时间通电*/
	{POWER_LIST_INDEX_P1A5_LONGTIME,"+1A5",0,DC24_LONG_MS_N_LOW<<2,DC24_LONG_MAX_A_N_LOW,data_is_greater_than_max,0},/* 负24V 1.5A  持续10s*/
	{POWER_LIST_INDEX_N8A_LONGTIME,"-8A",0,DC24_LONG_MS_P<<2,DC24_LONG_MAX_A_P,data_is_greater_than_max,0},/* +24V 8A长时间通电1S*/
	

	
};

#if 0
#define C24_MAX					
#define C24_MIN

#define C24_N_MAX
#define C24_N_MIN

#endif



#define ADC1_DR_Address    ((uint32_t)0x4001244C)
/*__IO*/ uint16_t ADCConvertedValue;


typedef struct {
	char *title;
	int	channel;
	int	sampletime;
	uint16_t	*ConvData;
}ADC_MAP;

#ifdef USE_ADC2_

ADC_MAP adcdev[] = {
	{ "DC24_P", ADC_Channel_8, ADC_SampleTime_56Cycles},   				//ADC12_IN8
	{ "DC12", ADC_Channel_9, ADC_SampleTime_56Cycles},				//ADC12_IN9
	//{ "C24_N", ADC_Channel_10, ADC_SampleTime_112Cycles}, 				 //ADC123_IN10
	//{ "C24_P", ADC_Channel_11, ADC_SampleTime_112Cycles}, 				 //ADC123_IN11
	{ "DC24_N", ADC_Channel_15, ADC_SampleTime_56Cycles}, 				 //ADC12_IN15
	{ "Vbat", ADC_Channel_Vbat, ADC_SampleTime_480Cycles},  			//ADC1_IN18 
	{ "vrf", ADC_Channel_Vrefint, ADC_SampleTime_56Cycles},				//ADC1_IN17
	{ "TempS", ADC_Channel_TempSensor, ADC_SampleTime_480Cycles}, 	 //ADC1_IN16
	
	
};

ADC_MAP adcdev_c[] = {	
	{ "C24_N", ADC_Channel_10, ADC_SampleTime_112Cycles}, 				 //ADC123_IN10
	{ "C24_P", ADC_Channel_11, ADC_SampleTime_112Cycles}, 				 //ADC123_IN11	
	{ "C24_Yarn", ADC_Channel_6, ADC_SampleTime_112Cycles},			//ADC12_IN6
};

#else
ADC_MAP adcdev[] = {
	{ "DC24_P", ADC_Channel_8, ADC_SampleTime_56Cycles},   				//ADC12_IN8
	{ "DC12", ADC_Channel_9, ADC_SampleTime_56Cycles},				//ADC12_IN9
	{ "C24_N", ADC_Channel_10, ADC_SampleTime_112Cycles}, 				 //ADC123_IN10
	{ "C24_P", ADC_Channel_11, ADC_SampleTime_112Cycles}, 				 //ADC123_IN11
	{ "DC24_N", ADC_Channel_15, ADC_SampleTime_56Cycles}, 				 //ADC12_IN15
	{ "Vbat", ADC_Channel_Vbat, ADC_SampleTime_480Cycles},  			//ADC1_IN18 
	{ "vrf", ADC_Channel_Vrefint, ADC_SampleTime_56Cycles},				//ADC1_IN17
	{ "TempS", ADC_Channel_TempSensor, ADC_SampleTime_480Cycles}, 	 //ADC1_IN16
	{ "C24_Yarn", ADC_Channel_6, ADC_SampleTime_112Cycles},			//ADC12_IN6
	
};


#endif

#define COUNTOF(__s)	(sizeof(__s) / sizeof(__s[0]))

#define DMA_COUNT	8

#define MAXCHANNEL	COUNTOF(adcdev)
#ifdef USE_ADC2_
#define MAXCHANNEL_C	COUNTOF(adcdev_c)

unsigned short RegularConvData_Tab_c[MAXCHANNEL_C];
#endif
unsigned short RegularConvData_Tab[MAXCHANNEL*DMA_COUNT];
unsigned short Adc_Result[MAXCHANNEL];


volatile unsigned int DC24_P_CURR_A = 0;
volatile unsigned int DC24_N_CURR_A = 0;
volatile unsigned char DC24_N711_NO =0;
volatile unsigned char DC24_P711_NO =0;



volatile unsigned int DC24_P_CURR_V = 0;
volatile unsigned int DC24_N_CURR_V = 0;
volatile unsigned int DC12_P_CURR_V = 0;
volatile unsigned int Vbat_CURR_V = 0;
volatile unsigned int Vrf_CURR_V = 0;
volatile unsigned int TempS_CURR_V = 710;
volatile unsigned int DC24_N_CURR_A_Yarn = 0;


//#define MAX_DC24_COUNT		(128)
//#define MAX_DC24_DATA_COUNT (72)
volatile unsigned int DC24_PN_CURR_Arry[MAX_DC24_COUNT];
/*
volatile unsigned short DC24_PCR_Arry_TEST[MAX_DC24_COUNT];
volatile unsigned short DC_index_test=0;
volatile unsigned short DC_index_test2=0;
*/
volatile unsigned int DC24_BASE_CURR_Arry[2][MAX_DC24_A_BASE_DATA_COUNT];//0--N,,1--P
volatile unsigned int DC24_BASE_index[2]={0,0};

volatile unsigned int DC24_PN_Count=0;
volatile unsigned int DC24_PN_Count_test=0;
volatile unsigned char DC24_PN_EnableBIT=0;
volatile unsigned char DC24_Start_check=0;


volatile unsigned char DC24_PN_whichNO =0;
volatile unsigned int DC24_PN_CURR_Zero =1650;
volatile unsigned int DC24_P_CURR_Zero =1650;
volatile unsigned int DC24_N_CURR_Zero =1650;


volatile unsigned int DC24_PN_data_Arry[MAX_DC24_DATA_COUNT];


void ADC_Self_Calibration(void);


/*

void Send_debug_adc_cnt()
{
	int i=0;
	for (i=0;i<=DC_index_test2;i++)
	{
		Message_Send_4halfword(0x0709, i, DC24_PCR_Arry_TEST[i], 0);

	}
}
*/

static void ADC_ChannelsConfig(void)
{
	int i;
	for(i = 0; i < MAXCHANNEL; i++) {
		ADC_RegularChannelConfig(ADC1, adcdev[i].channel, 1 + i, adcdev[i].sampletime);
//		adcdev[i].ConvData = &RegularConvData_Tab[i];
		adcdev[i].ConvData = &Adc_Result[i];
		//RegularConvData_Tab[i] = 0x2345;
		
	}
}

#ifdef USE_ADC2_
static void ADC2_ChannelsConfig(void)
{
	int i;
	for(i = 0; i < MAXCHANNEL_C; i++) {
		ADC_RegularChannelConfig(ADC2, adcdev_c[i].channel, 1 + i, adcdev_c[i].sampletime);
		adcdev_c[i].ConvData = &RegularConvData_Tab_c[i];
		//adcdev[i].ConvData = &Adc_Result[i];
		//RegularConvData_Tab[i] = 0x2345;
		
	}
}

#endif

#if 0
uint16_t Get_ADC_data_channel(unsigned int whichch)
{
	return (*adcdev[whichch].ConvData)*3300 / 0xfff;
}

#endif

void ADC_start_main()
{


	
	//myprintf("\r\n ADC_SoftwareStartConv(ADC2)...\n\r");
	#ifdef USE_ADC2_
	ADC_SoftwareStartConv(ADC2);
	#endif
	ADC_SoftwareStartConv(ADC1);
	//myprintf("\r\n ADC_SoftwareStartConv(ADC2_ok)...\n\r");
	return ;
}


#if 0

 ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

#endif
void ADC_HardInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	 ADC_CommonInitTypeDef ADC_CommonInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure; 
	 int i;

	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	for (i=0;i<MAXCHANNEL*DMA_COUNT;i++)
		RegularConvData_Tab[i]=0;
	
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA2_Stream0);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//ADC1_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = MAXCHANNEL*DMA_COUNT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	
	//DMA_InitStructure.DMA_M2M = DMA_MemoryInc_Disable;   //????
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	/* Enable DMA1 channel 1 */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	
	//ADC_DeInit();
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);


  /* ADC Common Init **********************************************************/
  	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_StructInit(&ADC_InitStructure);


  /* ADC1 Init ****************************************************************/
 
  	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfConversion = MAXCHANNEL;
  	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/

	ADC_ChannelsConfig();
	/* ADC1 regular channel 14 configuration */ 
	//################改为 Channel 10(电位器)###### 内部温度传感器 改为 Channel 16 ###################
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);
	//内部温度传感器  添加这一句 
	/* Enable the temperature sensor and vref internal channel */
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_VBATCmd(ENABLE);

	//ADC_RESETCALIBRATION calibration

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	// ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	
	//ADC_Self_Calibration();
	
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

		/* Enable ADC1 */
	//ADC_Cmd(ADC1, ENABLE);

	//ADC_SoftwareStartConv(ADC1);
}


#ifdef USE_ADC2_
#define ADC_STREAMX			DMA2_Stream3

#define ADC_IRQn			DMA2_Stream3_IRQn

void ADC2_HardInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	// ADC_CommonInitTypeDef ADC_CommonInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure; 
	 int i;

	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	for (i=0;i<MAXCHANNEL_C;i++)
		RegularConvData_Tab_c[i]=0;
	
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(ADC_STREAMX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;//ADC1_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RegularConvData_Tab_c;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = MAXCHANNEL_C;
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	
	//DMA_InitStructure.DMA_M2M = DMA_MemoryInc_Disable;   //????
	DMA_Init(ADC_STREAMX, &DMA_InitStructure);

	/* Enable DMA1 channel 1 */
	DMA_Cmd(ADC_STREAMX, ENABLE);

	
	//ADC_DeInit();
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 , ENABLE);


  /* ADC Common Init **********************************************************/
  #if 0
  	ADC_DeInit();
  	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  	ADC_CommonInit(&ADC_CommonInitStructure);
	
	
#endif
	ADC_StructInit(&ADC_InitStructure);
  /* ADC1 Init ****************************************************************/
 
  	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfConversion = MAXCHANNEL_C;
  	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/

	ADC2_ChannelsConfig();
	/* ADC1 regular channel 14 configuration */ 
	//################改为 Channel 10(电位器)###### 内部温度传感器 改为 Channel 16 ###################
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);
	//内部温度传感器  添加这一句 
	/* Enable the temperature sensor and vref internal channel */
	//ADC_TempSensorVrefintCmd(ENABLE);
	//ADC_VBATCmd(ENABLE);

	//ADC_RESETCALIBRATION calibration

	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
	// ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC2, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC2, ENABLE);

	
	//ADC_Self_Calibration();
	
	DMA_ITConfig(ADC_STREAMX,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

		/* Enable ADC1 */
	//ADC_Cmd(ADC1, ENABLE);

	//ADC_SoftwareStartConv(ADC2);
}
#else
void ADC2_HardInit(void)
{
}
#endif

void ADC_Self_Calibration(void)
{
	  
	  ADC1->CR2|=1<<0;      //开启AD转换器     
   	 ADC1->CR2|=1<<3;       //使能复位校准  
    		while(ADC1->CR2&1<<3); //等待校准结束             
    		//该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。       
    	ADC1->CR2|=1<<2;        //开启AD校准      
   	 while(ADC1->CR2&1<<2);  //等待校准结束
    //该位由软件设置以开始校准，并在校准结束时由硬件清除  
    return;
}


void ADC_WatchdogConfig(void)
{
  ADC_AnalogWatchdogSingleChannelConfig(ADC1,ADC_Channel_10);
  ADC_AnalogWatchdogThresholdsConfig(ADC1,1500,0xFFF);
  ADC_AnalogWatchdogCmd(ADC1,ADC_AnalogWatchdog_SingleRegEnable);

  ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
}

void arch_check_Overload_longTime(unsigned char is250us)
{

return ;
#if 0
	static unsigned int DC24_N_curr_cnt=0;
	static unsigned int DC24_N_curr_cnt_low=0;
	static unsigned int DC24_P_curr_cnt=0;

	if (DC24_N_CURR_A >DC24_LONG_MAX_A_N)
	{
		DC24_N_curr_cnt++;
		if (DC24_N_curr_cnt>=(is250us?(DC24_LONG_MS_N<<2):(DC24_LONG_MS_N>>1)))
		{
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24);
			#ifdef NEW_ALARM_STYLE
			alert_push(POWER_ERR_CODE_ARG(1,(30)|(0xE8<<8)));
			#else
			alert_push(HEAD_DC24_CURR_OVERLOAD,0);

			#endif
			
		}//必须关断了
	}
	else
	{
		DC24_N_curr_cnt =0;
	}
	
	if (DC24_N_CURR_A >DC24_LONG_MAX_A_N_LOW)
	{
		DC24_N_curr_cnt_low++;
		if (DC24_N_curr_cnt_low>=(is250us?(DC24_LONG_MS_N_LOW<<2):(DC24_LONG_MS_N_LOW>>1)))
		{
			//POWER_XZ_NEGATIVE24V_OUTPUT(1);
			arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24);
			#ifdef NEW_ALARM_STYLE
			alert_push(POWER_ERR_CODE_ARG(1,(15)|(0xE8<<8)));
			#else
			alert_push(HEAD_DC24_CURR_OVERLOAD,2);
			#endif
			
		}//必须关断了
	}
	else
	{
		DC24_N_curr_cnt_low =0;
	}
	
	
	if (DC24_P_CURR_A >DC24_LONG_MAX_A_P)
	{
		DC24_P_curr_cnt++;
		if (DC24_P_curr_cnt>=(is250us?(DC24_LONG_MS_P<<2):(DC24_LONG_MS_P>>1)))
		{
			//POWER_XZ_POSITIVE24V_OUTPUT(0);  
			arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24);
			#ifdef NEW_ALARM_STYLE
			alert_push(POWER_ERR_CODE_ARG(1,(60)|(24<<8)));
			#else
			alert_push(HEAD_DC24_CURR_OVERLOAD,1);
			#endif
		}//必须关断了		
	}
	else
	{
		DC24_P_curr_cnt =0;
	}

	#endif
	
}

extern unsigned int arch_get_ticktime(void);

char check_power_alert_is_coming(unsigned char whichone,unsigned int newdata)
{
	int time;
	POWER_ALERT *pa;
	if (whichone>=POWER_LIST_INDEX_MAX) return 0;	
	pa=&power_isalert_last_timer[whichone];
	if (pa->index!=whichone)
		return 0;
	if ((pa->islessthan==data_is_less_than_min)?(newdata<pa->compar_val):(newdata>pa->compar_val))
	{
		/*if (whichone==POWER_LIST_INDEX_P1A5_LONGTIME)
		{
			//DC24_PCR_Arry_TEST[DC_index_test++]=newdata;
			//if (DC_index_test>=MAX_DC24_COUNT-1)
			DC_index_test++;
		}
		*/
	
		time= arch_get_ticktime();

		pa->isok_cnt=0;
		if (pa->last_time_tick)
		{
			time-= pa->last_time_tick;
			time = abs(time);
			if (time > 0x7FFFFFFF)
				time = 0xFFFFFFFF - time;
			if (time> pa->time_out_data)   	// 250us 转换成1ms 
			{
				//pa->last_time_tick =0;
				return 1;
			}
		}
		else
			pa->last_time_tick = time;
		
		return 0;

	}
	else
	{
		pa->isok_cnt++;
		if ((pa->last_time_tick ) &&(pa->isok_cnt>=5))
		{
			pa->last_time_tick=0;
			pa->isok_cnt =0;
		/*	if (whichone==POWER_LIST_INDEX_P1A5_LONGTIME)
			{
				DC24_PCR_Arry_TEST[DC_index_test2++]=DC_index_test;
				if (DC_index_test2>=MAX_DC24_COUNT)
				{
					DC_index_test2 =0;
				}
				DC_index_test=0;
				
			}*/
			
		}	
		return 0;
	}

}



unsigned long arch_adc_test()
{
	int i,j;
	uint32_t v, mv;
	//extern volatile unsigned int DC24;
	//extern volatile unsigned int DC12;
	//extern int alert_push(int alert_code, int alert_arg);
	unsigned long dc_error =0;
	unsigned int data;

	/* Test DMA1 TC flag */
	if((DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0)) != RESET)
	{
	/* Clear DMA TC flag */
	DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
	//arch_LED_timer();

	for(i = 0; i < MAXCHANNEL; i++)
	{
		data = 0;
		for (j =0;j<DMA_COUNT;j++)
		{
			data += RegularConvData_Tab[j * MAXCHANNEL + i];
		}		
		data >>= 3;
		Adc_Result[i] = data;
		
		mv = *adcdev[i].ConvData;
		
		v = (mv * 3300L) >>12;

		//myprintf("DC%d = %dmv, adc =%d \n\r",i==0?24:12,v,mv);

		if (i==0)
		{
			//DC24 = v;

			if (check_power_alert_is_coming(POWER_LIST_INDEX_P24_L,v)){
				dc_error |= ALARM_DC24_LOW_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P24_H,v)){
				dc_error |= ALARM_DC24_HIGH_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P24_L_EX,v)){
				dc_error |= ALARM_DC24_LOW_LOW_MASK;
			}
			
			DC24_P_CURR_V = v;
			continue;
		}
		if (i==1)
		{
			//DC12 = v;
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P12_L,v )){
				dc_error |= ALARM_DC12_LOW_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P12_H,v )){
				dc_error |= ALARM_DC12_HIGH_MASK;
			}
			DC12_P_CURR_V = v;
			continue;
		}

		#ifdef USE_ADC2_
		
		if (i==2)
		{
			//DC12 = v;
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_L,v )){
				dc_error |= ALARM_N_24_LOW_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_H,v )){
				dc_error |= ALARM_N_24_HIGH_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_L_EX,v))
			{
				dc_error |= ALARM_N_24_LOW_LOW_MASK;
			}
			DC24_N_CURR_V = v;
			continue;
		}
		if (i==3)
		{
			Vbat_CURR_V =v;
			continue;
		}
		if (i==4)
		{
			Vrf_CURR_V =v;
			continue;
		}
		if (i==5)
		{
			TempS_CURR_V= v;
			//TempS_CURR_V =250+(76000-v*100)/25;
			continue;
		}		
		
		#else
		if (i==2)
		{
			//volatile unsigned int DC24_P_CURR_A = 0;
			//volatile unsigned int DC24_N_CURR_A = 0;

			DC24_N_CURR_A = v;//(1650-v) / 55;
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N3A_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,1);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(30)|(0xE8<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,0);
				#endif
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N1A5_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,2);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(15)|(0xE8<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,2);
				#endif
			}

			if (check_power_alert_is_coming(POWER_LIST_INDEX_N8A_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,3);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(80)|(0xE8<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,3);
				#endif
			}
			//myprintf("DC24_N_CURR_A= %dmv, adc =%d \n\r",i==0?24:12,v,mv);
			if (DC24_PN_EnableBIT&0x01)
			{
				if (DC24_PN_Count>=MAX_DC24_COUNT)
					DC24_PN_Count =0;
				DC24_PN_CURR_Arry[DC24_PN_Count++]=DC24_N_CURR_A;
				//DC24_PN_Count=0;
				DC24_PN_Count_test++;
			}
			else
			{
				if (DC24_BASE_index[0]>=MAX_DC24_A_BASE_DATA_COUNT)
					DC24_BASE_index[0]=0;
				DC24_BASE_CURR_Arry[0][DC24_BASE_index[0]]=DC24_N_CURR_A;
				DC24_BASE_index[0]++;
				}
			continue;
		}
		if (i==3)
		{
			//DC12 = v;
			DC24_P_CURR_A =v; //(v-1650) / 55;

			if (check_power_alert_is_coming(POWER_LIST_INDEX_P3A_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,4);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(30)|(0xE8<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,4);
				#endif
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P1A5_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,5);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(15)|(0xE8<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,5);
				#endif
			}

			
			if (check_power_alert_is_coming(POWER_LIST_INDEX_P8A_LONGTIME,v ))
			{
				arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,6);
				#ifdef NEW_ALARM_STYLE
				alert_push(POWER_ERR_CODE_ARG(1,(60)|(24<<8)));
				#else
				alert_push(HEAD_DC24_CURR_OVERLOAD,1);
				#endif
			}
	
			
			if (DC24_PN_EnableBIT&0x02)
			{
				if (DC24_PN_Count>=MAX_DC24_COUNT)
					DC24_PN_Count =0;
				DC24_PN_CURR_Arry[DC24_PN_Count++]=DC24_P_CURR_A;
				DC24_PN_Count_test++;
			}
			else
			{
				if (DC24_BASE_index[1]>=MAX_DC24_A_BASE_DATA_COUNT)
					DC24_BASE_index[1]=0;
				DC24_BASE_CURR_Arry[1][DC24_BASE_index[1]]=DC24_P_CURR_A;
				DC24_BASE_index[1]++;
			}
			continue;
		}
		if (i==4)
		{
			//DC12 = v;
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_L,v)){
				dc_error |= ALARM_N_24_LOW_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_H,v)){
				dc_error |= ALARM_N_24_HIGH_MASK;
			}
			if (check_power_alert_is_coming(POWER_LIST_INDEX_N24_L_EX,v)){
				dc_error |= ALARM_N_24_LOW_LOW_MASK;
			}
			DC24_N_CURR_V = v;
			continue;
		}
		if (i==5)
		{
			Vbat_CURR_V =v;
			continue;
		}
		if (i==6)
		{
			Vrf_CURR_V =v;
			continue;
		}
		if (i==7)
		{
			TempS_CURR_V= v;
			//TempS_CURR_V =250+(76000-v*100)/25;
			continue;
		}
		if (i==8)
		{
			DC24_N_CURR_A_Yarn= v;
			//TempS_CURR_V =250+(76000-v*100)/25;
			continue;
		}
		#endif
		
	}

	}

	
	return dc_error;
}

#ifdef USE_ADC2_
void arch_adc_test_DMA2_string2_ch1()
{
	int i;
	unsigned int v, mv;
	//extern volatile unsigned int DC24;
	//extern volatile unsigned int DC12;
	//extern int alert_push(int alert_code, int alert_arg);
	//unsigned long dc_error =0;
	//unsigned int data;
	if((DMA_GetFlagStatus(DMA2_Stream3,DMA_FLAG_TCIF3)) != RESET)
	{
	/* Clear DMA TC flag */
		DMA_ClearFlag(DMA2_Stream3,DMA_FLAG_TCIF3);
		for(i = 0; i < MAXCHANNEL_C; i++)
		{			
		
			mv = *adcdev_c[i].ConvData;
			v = (mv * 3300L) / 0xFFF;
			if (i==0)
			{
				//volatile unsigned int DC24_P_CURR_A = 0;
				//volatile unsigned int DC24_N_CURR_A = 0;

				DC24_N_CURR_A = v;//(1650-v) / 55;
				if(DC24_N_CURR_A<10)
				{
					if(!DC24_N711_NO)
					{
						DC24_N711_NO =1;
						{
							GPIO_SET_711_FAULT_AN(1);
						}
					}
					continue;					
				}
				if (check_power_alert_is_coming(POWER_LIST_INDEX_N3A_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,7);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(30)|(0xE8<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,0);
					#endif
				}
				if (check_power_alert_is_coming(POWER_LIST_INDEX_N1A5_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,8);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(15)|(0xE8<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,2);
					#endif
				}

				if (check_power_alert_is_coming(POWER_LIST_INDEX_N8A_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_N_24,POWER_OFF_NP_24,9);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(80)|(0xE8<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,3);
					#endif
				}
				
				//myprintf("DC24_N_CURR_A= %dmv, adc =%d \n\r",i==0?24:12,v,mv);
				if (DC24_PN_EnableBIT&0x01)
				{
					if (DC24_PN_Count>=MAX_DC24_COUNT)
						DC24_PN_Count =0;
					DC24_PN_CURR_Arry[DC24_PN_Count++]=DC24_N_CURR_A;
					//DC24_PN_Count=0;
					DC24_PN_Count_test++;
				}
				else
				{
					if (DC24_BASE_index[0]>=MAX_DC24_A_BASE_DATA_COUNT)
						DC24_BASE_index[0]=0;
					DC24_BASE_CURR_Arry[0][DC24_BASE_index[0]]=DC24_N_CURR_A;
					DC24_BASE_index[0]++;
					}
				continue;
			}
			if (i==1)
			{
				//DC12 = v;
				DC24_P_CURR_A =v; //(v-1650) / 55;
				if(DC24_P_CURR_A<10)
				{
					if(!DC24_P711_NO)
					{
						DC24_P711_NO =1;
						{
							GPIO_SET_711_FAULT_AN(0);
						}
					}
					continue;					
				}
				if (check_power_alert_is_coming(POWER_LIST_INDEX_P3A_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,10);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(30)|(0xE8<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,4);
					#endif
				}
				if (check_power_alert_is_coming(POWER_LIST_INDEX_P1A5_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,11);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(15)|(0xE8<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,5);
					#endif
				}

				
				if (check_power_alert_is_coming(POWER_LIST_INDEX_P8A_LONGTIME,v ))
				{
					arch_Power_ctrl_ex(POWER_CTR_P_24,POWER_OFF_NP_24,12);
					#ifdef NEW_ALARM_STYLE
					alert_push(POWER_ERR_CODE_ARG(1,(60)|(24<<8)));
					#else
					alert_push(HEAD_DC24_CURR_OVERLOAD,1);
					#endif
				}
				
				if (DC24_PN_EnableBIT&0x02)
				{
					if (DC24_PN_Count>=MAX_DC24_COUNT)
						DC24_PN_Count =0;
					DC24_PN_CURR_Arry[DC24_PN_Count++]=DC24_P_CURR_A;
					DC24_PN_Count_test++;
				}
				else
				{
					if (DC24_BASE_index[1]>=MAX_DC24_A_BASE_DATA_COUNT)
						DC24_BASE_index[1]=0;
					DC24_BASE_CURR_Arry[1][DC24_BASE_index[1]]=DC24_P_CURR_A;
					DC24_BASE_index[1]++;
				}
				continue;
			}

			if (i==2)
			{
				DC24_N_CURR_A_Yarn= v;
				//TempS_CURR_V =250+(76000-v*100)/25;
				continue;
			}
		}
				
	}

}

#else
void arch_adc_test_DMA2_string2_ch1()
{
}
#endif

void wait_ms(unsigned int ms);
u16 ADC_Filter(void)
{
	u16 result=0;
	u32 i;

	for(i=16;i>0;i--)
	{

		wait_ms(50);
		result += ADCConvertedValue;
	}

	return result/16;
}

void test_adc()
{
	u32 Value;
	u32 temp;

	ADC_HardInit();
	printf_enable();
	while (1)
	{
		//printf("The ADC1 Converted Value is %d \r\n",ADCConvertedValue);
		//v=	ADCConvertedValue*3.30/4096;
		//printf("The Voltage is %f \r\n",v );

		//myprintf("\r\nadc test\r\n");
		Value=ADC_Filter();
		//printf("The ADC_Filter Value is %d \r\n",(int)Value);
		myprintf("adc test %d\r\n", Value);

		temp=(14200 - Value*33000/4096)*10/435 + 25;
		myprintf("Temperature %d \r\n",temp );
		//myprintf("%s%c%c%c%c%c%s","#**",(int)Value/256,(int)Value%256,'&',(int)((temp-(int)temp)*100),(int)temp,"**%");

		//wait_ms(500);
		//for(v=0;v<5000;v++);for(v=0;v<5000;v++);for(v=0;v<5000;v++);
	}


}





//dac



void DAC_HardInit(void)
{
	DAC_InitTypeDef    DAC_InitStructure;
	//GPIO_InitTypeDef   GPIO_InitStructure;



	/* Enable DAC clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* DAC1 channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init( DAC_Channel_1, &DAC_InitStructure);

	DAC_Init( DAC_Channel_2, &DAC_InitStructure);

	/* Enable DAC1 Channel1 */
	DAC_Cmd( DAC_Channel_1, ENABLE);
	DAC_Cmd( DAC_Channel_2, ENABLE);
	
}

#define DAC_VDDA	3300L
void DAC_SetVoltage_channel1(long Vout)
{
	long val;

	Vout=arch_need_current_add(Vout);

	val = (Vout * 8 * 4095L) / (DAC_VDDA*10);

	

	/* Output converted value on DAC1_OUT1 */
	DAC_SetChannel1Data( DAC_Align_12b_R, val);
	DAC_SoftwareTriggerCmd( DAC_Channel_1, ENABLE);
}

void DAC_SetVoltage_channel2(long Vout)
{
	long val;
	
	Vout=arch_need_current_add(Vout);
	val = (Vout * 8 * 4095L) / (DAC_VDDA*10);

	/* Output converted value on DAC1_OUT1 */
	DAC_SetChannel2Data( DAC_Align_12b_R, val);
	DAC_SoftwareTriggerCmd( DAC_Channel_2, ENABLE);
}


