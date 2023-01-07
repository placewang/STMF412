
#include <stdio.h>
#include "massage.h"
#include "arch.h"

#include "stm32f2xx.h"
#include "platform_config.h"

/*----------------------------------------------------------------------------*/
/* 按串行接收MSB */
enum {
	ENC_EVEN = 0,	// 前15位偶校验位
	ENC_MAGDEC,
	ENC_MAGINC,
	ENC_LIN,		//线性度报警
	ENC_COF,		//Cordic溢出报警
	ENC_OCF,		//偏差补偿完成
	ENC_D0,
	ENC_D1,
	ENC_D2,
	ENC_D3,
	ENC_D4,
	ENC_D5,
	ENC_D6,
	ENC_D7,
	ENC_D8,
	ENC_D9,
	ENC_D10,
	ENC_D11,
	ENC_BIT_MAX,
};


#ifndef ENCODER_NUM_MAX
#define ENCODER_NUM_MAX		8
#endif /* ENCODER_NUM_MAX */



#define SSI_DELAY_TIME_FIRST	(15)		// 12 -- 500KHz
#define SSI_DELAY_TIME_DATA		(6)

#if ENCODER_DMA_SUPPORT
#define SSI_STREAMX				DMA2_Stream2
#define SSI_IRQn				DMA2_Stream2_IRQn


static volatile uint8_t spi_rxbuff[4];

static volatile int coder_idx = 0;
static volatile int coder_fresh = 0;		// 编码值数据已经刷新标记

volatile int8_t coder_ret[ENCODER_NUM_MAX];

volatile char coder_stat[ENCODER_NUM_MAX];
volatile short coder_coder[ENCODER_NUM_MAX];

volatile int coder_rx[ENCODER_NUM_MAX];

volatile int coder_spi_get[ENCODER_NUM_MAX];



static uint32_t *prxbuf = (uint32_t *)&spi_rxbuff[0];

#endif /* ENCODER_DMA_SUPPORT */

volatile int  read_coder_cnt_=0;

volatile int coder_err_cntmax[ENCODER_NUM_MAX];

// by xhl 2011/04/27
//volatile int SSI_IDX2LOG[10] = {0, 1, 2, 3, 5, 6, 7 , 8, 4, 9};
// by xhl 2012/07/30
//volatile int SSI_IDX2LOG[10] = {5, 6, 7, 8, 0, 1, 2 , 3, 4, 9};
#ifdef E490_V10_BOARD
static volatile int SSI_IDX2LOG[ENCODER_NUM_MAX] = {1, 3, 0, 2, 5, 4, 7, 6};	// 强隆
#else
static volatile int SSI_IDX2LOG[ENCODER_NUM_MAX] = {0, 2, 1, 3, 4, 5, 6, 7};	// 强隆
#endif

volatile int coder_err_cnt[ENCODER_NUM_MAX];	//SSI 数据读取错误计数
volatile int coder_err_cnt_er[5][ENCODER_NUM_MAX];	//SSI 数据读取错误计数



void ssidelay15ns(volatile unsigned int delay)
{
	while(--delay);
}

#if ENCODER_DMA_SUPPORT
unsigned int spiisr_cnt=0;



void SPI_RX_ISR(void)
{
	static unsigned char spi_cnt=0;
	unsigned char eid;

	if(SPI_I2S_GetITStatus(SPI1,SPI_I2S_IT_RXNE))
	{
		
		SPI_I2S_ClearITPendingBit(SPI1,SPI_I2S_IT_RXNE);
		spiisr_cnt++;
		spi_rxbuff[spi_cnt] = SPI_I2S_ReceiveData(SPI1);
		spi_cnt++;
		if (spi_cnt>=3)
		{
			int i, ret ;
			unsigned int iencoder;
			unsigned short coder;
			unsigned char state;
			unsigned char even;
			spi_cnt =0;
			SPI_Cmd(SPI1, DISABLE);
			eid = coder_idx;
			coder_idx++;			
			if (coder_idx >= ENCODER_NUM_MAX)
			coder_idx = 0;
			SSI_Select(SSI_IDX2LOG[coder_idx]);
			coder_spi_get[eid] = ((((int)spi_rxbuff[0] & 0x7F) << 16)	| ((int)spi_rxbuff[1] << 8)	| ((int)spi_rxbuff[2])) >> 5;
			coder_fresh |= (1 << eid);

			 //= iencoder;

			#if 0

			do {
			ret = 0;
			state = iencoder & 0x3F;
			coder = iencoder >> 6;
			
			// Not Ready
			if ((state == 0x3F) || (state == 0x0))
			{
				ret = -100;
			//	coder_err_cnt_er[0][eid]++;
				break;
			}


			if (state & 0x08)
			{
				ret = -1;
				coder_err_cnt_er[0][eid]++;
				break;
			}
			// Cordic & Line
			if (state & 0x10)
			{
				ret = -1;
				coder_err_cnt_er[1][eid]++;
				break;
			}

			
			
			// MagINCn & MagDECn
			if ((state & 0x6) == 0x6)
			{
				ret = -2;
				coder_err_cnt_er[2][eid]++;
				break;
			}
			
			// Parity
			even = 0;
			for (i = 0; i < ENC_BIT_MAX; i++)
			{
				if (iencoder & (1 << i))
					even += 1;
			}
			
			if (even & 0x1)
			{
				ret = -3;
				coder_err_cnt_er[3][eid]++;
				break;
			}

			// Compensation
			if ((state & 0x20) == 0)
			{
				ret = -4;
				coder_err_cnt_er[4][eid]++;
				break;
			}
			
			coder_rx[eid] = iencoder;
			
			iencoder = coder_coder[eid];
			iencoder = abs(iencoder - coder);
			//if (iencoder > 200)
			{
			//	break;
			}
			coder_coder[eid] = coder;
			coder_stat[eid] = state;
		} while (0);
		coder_ret[eid] = ret;

		if (ret < 0)
		{
			coder_err_cnt[eid]++;
			if (coder_err_cnt[eid] > coder_err_cntmax[eid])
				coder_err_cntmax[eid] = coder_err_cnt[eid];
		}
		else
		{
			coder_err_cnt[eid] = 0;
		}
		#endif
			

			SPI_Cmd(SPI1, ENABLE);
			
		}
		//SPI1->DR = 0xff;
	}
}

void SSI_IRQx(void)
{
	int i, ret;
	unsigned int iencoder;
	unsigned short coder;
	unsigned char state;
	unsigned char even;
	unsigned char eid;
	
	//ECODE_SELECT_CODE_2(0);
	//GPIOE->BSRRH = GPIO_Pin_3;
	
	/* DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(SSI_STREAMX, DMA_IT_TCIF2))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(SSI_STREAMX, DMA_IT_TCIF2);
		SPI_Cmd(SPI1, DISABLE);
		
		eid = coder_idx;
		coder_idx++;
		if (coder_idx >= ENCODER_NUM_MAX)
			coder_idx = 0;

		/* 片选信号在发送SCLK之前留足够的时间 另:此接口函数默认关闭所有片选信号 */
		SSI_Select(SSI_IDX2LOG[coder_idx]);
		//ssidelay15ns(10);

		/* MSB: 第一个字节的最高位无效 */
		iencoder = ((((int)spi_rxbuff[0] & 0x7F) << 16)
					| ((int)spi_rxbuff[1] << 8)
					| ((int)spi_rxbuff[2])) >> 5;
	
		coder_fresh |= (1 << eid);
		
	#if 0
		coder_rx[eid] = iencoder;
	#else
		do {
			ret = 0;
			state = iencoder & 0x3F;
			coder = iencoder >> 6;
			
			// Not Ready
			if ((state == 0x3F) || (state == 0x0))
			{
				ret = -100;
				break;
			}
			
			// Cordic & Line
			if (state & 0x18)
			{
				ret = -1;
				break;
			}
			
			// MagINCn & MagDECn
			if ((state & 0x6) == 0x6)
			{
				ret = -2;
				break;
			}
			
			// Parity
			even = 0;
			for (i = 0; i < ENC_BIT_MAX; i++)
			{
				if (iencoder & (1 << i))
					even += 1;
			}
			
			if (even & 0x1)
			{
				ret = -3;
				break;
			}

			// Compensation
			if ((state & 0x20) == 0)
			{
				ret = -4;
				break;
			}
			
			coder_rx[eid] = iencoder;
			
			iencoder = coder_coder[eid];
			iencoder = abs(iencoder - coder);
			
			coder_coder[eid] = coder;
			coder_stat[eid] = state;
			//if (iencoder > 200)
			//{
			//	break;
			//}
		} while (0);
		coder_ret[eid] = ret;

		if (ret < 0)
		{
			coder_err_cnt[eid]++;
			if (coder_err_cnt[eid] > coder_err_cntmax[eid])
				coder_err_cntmax[eid] = coder_err_cnt[eid];
		}
		else
		{
			coder_err_cnt[eid] = 0;
		}
	#endif
		
		SPI_Cmd(SPI1, ENABLE);
		DMA_Cmd(SSI_STREAMX, ENABLE);
	}

	//ECODE_SELECT_CODE_2(1);
	//GPIOE->BSRRL = GPIO_Pin_3;
}

static void SSI_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = SSI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

	DMA_DeInit(SSI_STREAMX);

	while (DMA_GetCmdStatus(SSI_STREAMX) != DISABLE);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spi_rxbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SSI_STREAMX, &DMA_InitStructure);

	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(SSI_STREAMX, DMA_IT_TC, ENABLE);
	
	/* DMA Stream enable */
	DMA_Cmd(SSI_STREAMX, ENABLE);
}

void spi1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	coder_idx = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//GPIO_DeInit(GPIOB);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	
	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*----------------------------------------------------------------------------*/

	SPI_I2S_DeInit(SPI1);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//时钟极性，空闲时为
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//第1个边沿有效，上升沿为采样时刻???
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//SSI_DMA_Config();

	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable SPI1 DMA RX request */
	//SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	SSI_Select(SSI_IDX2LOG[coder_idx]);

	SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	//SPI1->DR = 0xff;
}

#endif /* ENCODER_DMA_SUPPORT */



void Send_coder_info()
{
	int i;
	//Message_Send_4halfword(0xffff,spiisr_cnt&0xffff,spiisr_cnt>>16,0);
	//Message_Send_4halfword(0xffff,spi_rxbuff[0]|(spi_rxbuff[1]<<8),spi_rxbuff[2]|(spi_rxbuff[3]<<8),coder_idx);
	for (i=0;i<8;i++)
	{
		Message_Send_4halfword(0xff|(i<<8),coder_err_cnt_er[0][i],coder_err_cnt_er[1][i],coder_err_cnt_er[2][i]);
		Message_Send_4halfword(0xff|(i<<8),coder_err_cnt_er[3][i],coder_err_cnt_er[4][i],coder_err_cntmax[i]);
		
		//coder_err_cnt_er
	}
}


int SSI_Update(int idx)
{
#if ENCODER_DMA_SUPPORT
	return (coder_fresh & (1 << idx));
#else
	return 1;
#endif /* ENCODER_DMA_SUPPORT */
}

int SSI_Error(int idx)
{
	if (idx >= ENCODER_NUM_MAX)
		return 0xFF;
	
	return coder_err_cnt[idx];
}


int Get_read_coder_cnt_()
{
	return read_coder_cnt_;
}

#define CLI()	__set_PRIMASK(1)
#define SEI()	__set_PRIMASK(0)

void SSI_Select_with_id(unsigned char id,unsigned char select)
{
	if(select)
	{
		SSI_Select(SSI_IDX2LOG[id]);
	}
	else
	{
		SSI_DeSelect(SSI_IDX2LOG[id]);
	}
	
}

#ifdef ENCODER_IC_IS_5045OR8800
	 
int SSI_Read(int idx,short *IC_flag, int *coder, int *status)
{
	int i, even,cmd;
	long encoder;
	short loc_icf=*IC_flag;
	
	if (idx >= ENCODER_NUM_MAX)
		return -101;

	read_coder_cnt_++;

#if ENCODER_DMA_SUPPORT
	coder_fresh &= ~(1 << idx);
	encoder = coder_spi_get[idx];
#else

	if(IC_flag)
	{
		loc_icf=*IC_flag;
	}
	else
	{
		loc_icf =0;
	}

#ifdef ECODE_USE_MT6813_PWM
	if(Check_is_MT6813PWM_Mode())
	{
		Get_code_data_from_mem(idx,&encoder);
	}
	else
#endif		
	{
	
	SSI_Select(SSI_IDX2LOG[idx]);
	SSI_CLK_HIGHT();
	if(loc_icf<=-2)
	{
		ssidelay15ns(SSI_DELAY_TIME_FIRST*3);
	}
	else
	{		
		ssidelay15ns(SSI_DELAY_TIME_FIRST);
		SSI_CLK_LOW();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
	}
	i = 0;
	encoder = 0;
	even = 0;
	do {
		SSI_CLK_HIGHT();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		if (i++ < 15)
		{
			even += encoder & 0x1;
		}
		encoder <<= 1;
		SSI_CLK_LOW();
		encoder |= SSI_READ_DATA();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
	} while(i < 18);

	SSI_DeSelect(SSI_IDX2LOG[idx]);
	SSI_CLK_HIGHT();

	}
	
	coder_err_cnt[idx]++;
	if((encoder & 0x3F) == 0x3F)
		return -100;	

	if (coder) *coder = (int)(encoder >> 6);

	#ifdef ECODE_USE_MT6813_PWM
	if(Check_is_MT6813PWM_Mode())
	{
		if (status) 
			*status = (int)(encoder & 0x3F);
		goto Do_status_set;
	}
		//
	#endif
	
	if(loc_icf<=-2) /*表示8800*/
	{
		
		if (status) *status = (((int)(encoder & 0x08))<<2)|(((int)(encoder & 0x07)));
		

		if((encoder & 0x6)) {				
				return -2;
		}

		if((encoder & 0x08)==0) { //not ready				
				return -100;
			}
		
	
	}
	else/*还没识别出来或5045*/
	{
		//if (coder) *coder = (int)(encoder >> 6);
		if (status) *status = (int)(encoder & 0x3F);

		if (loc_icf>=2) /*5045*/
		{
			// Not Ready
			if(((encoder & 0x3F) == 0x3F) ||
			   ((encoder & 0x3F) == 0x0)) {
			  // coder_err_cnt_er[0][idx]++;
				return -100;		
			}

			if(encoder & 0x08) {
				//coder_err_cnt_er[0][idx]++;
				return -1;
			}
			// Cordic & Line
			if(encoder & 0x10) {
				//coder_err_cnt_er[1][idx]++;
				return -1;
			}
			
			// MagINCn & MagDECn
			if((encoder & 0x6) == 0x6) {
				//coder_err_cnt_er[2][idx]++;
				return -2;
			}

			#if 1
			if((encoder & 0x20) == 0) {
				//coder_err_cnt_er[4][idx]++;
				return -4;
			}
			#endif
			
		}
		else
		{
			if (IC_flag)
			{
				if(encoder & 0x000010)	//认为是8800
				{
					(*IC_flag)--;
				}
				else   /*5045*/
				{
					(*IC_flag)++;
				}
			}
			return -101;
		}		
	}

	#if 0
	if((encoder & 0x1) != (even & 0x1)) 
	{
		if (status) *status |= (int)(0x01);
		{				
			return -3;
		}
	}
	else
	#endif	

	
	{
		if (status) 
		{
			*status = ((*status)&0x3F)>>1;
			*status = ((*status)&0x1F)<<1;			
		}
	}
	Do_status_set:
	coder_err_cnt[idx] = 0;
	return 0;	
	
#endif
}


#else

	 
int SSI_Read(int idx,short *IC_flag, int *coder, int *status)
{
	int i, even,cmd;
	long encoder;
	
	if (idx >= ENCODER_NUM_MAX)
		return -101;

	read_coder_cnt_++;

#if ENCODER_DMA_SUPPORT
	coder_fresh &= ~(1 << idx);
	encoder = coder_spi_get[idx];
#else


#ifdef ENCODER_IC_IS_MT6815CT


	SSI_Select(SSI_IDX2LOG[idx]);

	SSI_CLK_HIGHT();
	ssidelay15ns(SSI_DELAY_TIME_FIRST);
	cmd = 0x83;
	i =0;
	do{
		// if(cmd&0x80)
		SSI_WRITE_DATA(cmd&0x80);
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		SSI_CLK_LOW();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		SSI_CLK_HIGHT();
		cmd<<=1;
		ssidelay15ns(SSI_DELAY_TIME_DATA);

	}while(i<8)

	ssidelay15ns(SSI_DELAY_TIME_DATA);

	
	
	//SSI_CLK_LOW();
	//ssidelay15ns(SSI_DELAY_TIME_DATA);
	i = 0;
	encoder = 0;
	even = 0;
	do {

		encoder <<=1;
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		SSI_CLK_LOW();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		SSI_CLK_HIGHT();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
	
		encoder += SSI_READ_DATA()?1:0;
		//ssidelay15ns(SSI_DELAY_TIME_DATA);
	} while(i < 16);

	SSI_DeSelect(SSI_IDX2LOG[idx]);
	//SSI_CLK_HIGHT();

	if (coder) *coder = (int)(encoder >> 2);
	if (status) *status = (int)(0);
	coder_err_cnt[idx] = 0;
	return 0;
	

#else

#ifdef ENCODER_IC_IS_AEAT8800


//CLI();
	SSI_Select(SSI_IDX2LOG[idx]);
	SSI_CLK_HIGHT();
	ssidelay15ns(SSI_DELAY_TIME_FIRST*3);
	//SSI_CLK_LOW();
	//ssidelay15ns(SSI_DELAY_TIME_DATA);
	i = 0;
	encoder = 0;
	even = 0;
	do {
		SSI_CLK_HIGHT();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		if (i++ < 15)
		{
			even += encoder & 0x1;
		}
		encoder <<= 1;
		SSI_CLK_LOW();
		encoder |= SSI_READ_DATA();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
	} while(i < 16);

	SSI_CLK_HIGHT();
	SSI_DeSelect(SSI_IDX2LOG[idx]);
	
//SEI();

	//////////////////////////////
	{
		
	if (coder) *coder = (int)(encoder >> 4);
	if (status) *status = (int)(encoder & 0x0F);
	
	// Not Ready
	if(!(encoder & 0x08)) {
	  return -100;		
	}

	if(encoder & 0x04) {
		//强度太大
		return -1;
	}
	// Cordic & Line
	if(encoder & 0x02) {
		//强度太小
		return -2;
	}	

	// Parity
	if((encoder & 0x1) != (even & 0x1)) {
		if (status) *status |= (int)(0x01);
		{
			//coder_err_cnt_er[3][idx]++;
			return -3;
		}
	}
	else
	{
		if (status) 
		{
			*status = ((*status)&0x0F)>>1;
			*status = ((*status)&0x07)<<1;			
		}
	}
	
	coder_err_cnt[idx] = 0;
	
	return 0;
	}
	//////////////////////////////
	

#else

	SSI_Select(SSI_IDX2LOG[idx]);
	SSI_CLK_HIGHT();
	ssidelay15ns(SSI_DELAY_TIME_FIRST);
	SSI_CLK_LOW();
	ssidelay15ns(SSI_DELAY_TIME_DATA);
	i = 0;
	encoder = 0;
	even = 0;
	do {
		SSI_CLK_HIGHT();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
		if (i++ < 15)
		{
			even += encoder & 0x1;
		}
		encoder <<= 1;
		SSI_CLK_LOW();
		encoder |= SSI_READ_DATA();
		ssidelay15ns(SSI_DELAY_TIME_DATA);
	} while(i < 18);

	SSI_DeSelect(SSI_IDX2LOG[idx]);
	SSI_CLK_HIGHT();
#endif
#endif
	

	
	coder_err_cnt[idx]++;
#endif



	if (coder) *coder = (int)(encoder >> 6);
	if (status) *status = (int)(encoder & 0x3F);


	#if 0
	// Not Ready
	if(((encoder & 0x3F) == 0x3F) ||
	   ((encoder & 0x3F) == 0x0)) {
	  // coder_err_cnt_er[0][idx]++;
		return -100;		
	}

	if(encoder & 0x08) {
		//coder_err_cnt_er[0][idx]++;
		return -1;
	}
	// Cordic & Line
	if(encoder & 0x10) {
		//coder_err_cnt_er[1][idx]++;
		return -1;
	}
	
	// MagINCn & MagDECn
	if((encoder & 0x6) == 0x6) {
		//coder_err_cnt_er[2][idx]++;
		return -2;
	}
	#endif
	// Not Ready
	if(((encoder & 0x07) == 0x07)/* ||
	   ((encoder & 0x07) == 0x0)*/) {
	  // coder_err_cnt_er[0][idx]++;
		return -100;		
	}
	
	// MagINCn & MagDECn
	if((encoder & 0x6) ) {
		//coder_err_cnt_er[2][idx]++;
		return -2;
	}
	// Parity
	if((encoder & 0x1) != (even & 0x1)) {
		if (status) *status |= (int)(0x01);
		{
			//coder_err_cnt_er[3][idx]++;
			return -3;
		}
	}
	else
	{
		if (status) 
		{
			*status = ((*status)&0x3F)>>1;
			*status = ((*status)&0x1F)<<1;			
		}
	}
	
	// Compensation

	#if 0
	if((encoder & 0x20) == 0) {
		//coder_err_cnt_er[4][idx]++;
		return -4;
	}
	#endif
	coder_err_cnt[idx] = 0;
	
	return 0;
}
#endif
