#include "stdio.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"

void wait_us(unsigned int us);
unsigned int arch_Get_Mode(void);
volatile int can_receive_count;

#if 0
void CAN_Enable_isr()
{
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
}

#endif

void CAN_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* Enable CAN clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	/* CAN register init */
	CAN_DeInit(CAN1);

	
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;//CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2=CAN_BS1_4tq;  //CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler=2;          // 3
	CAN_Init(CAN1,&CAN_InitStructure);
#if 0
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
#else
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=arch_Set_CANID(0)<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

#ifdef BOOT_CODE_JOIN_TEST_CODE

	CAN_FilterInitStructure.CAN_FilterNumber=1;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=arch_Set_CANID(1)<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

#endif	
	
#endif
	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
	
	
	/* CAN */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;    //¸üÐÂÊÂ¼þ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //ÇÀÕ¼ÓÅÏÈ¼¶0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //ÏìÓ¦ÓÅÏÈ¼¶1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï 
	NVIC_Init(&NVIC_InitStructure);                             //Ð´ÈëÉèÖÃ 

	can_receive_count = 0;
}

#define ACTION_ALARM	0
#define ACTION_DATA	1
#define ACTION_LOG     2

void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len)
{
	CanTxMsg TxMessage;
	unsigned short *pData;
	unsigned int idx;
	extern unsigned int arch_Get_ID(void);

	if(len == 0) 
		return;

	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len * 2L;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.StdId = 0x00000732 + arch_Get_ID() +1; // CAN ID
	

	pData = (unsigned short *)&TxMessage.Data;
	do {
		*pData++ = *pMsg++;
	}while(--len);

again:
	if((idx = CAN_Transmit(CAN1,&TxMessage)) == CAN_NO_MB) {
		delayus(500);
		goto again;
	}

	//
	//while(CAN_TransmitStatus(idx) != CANTXOK);
}

CanRxMsg RxMessage;
u32 *CAN0_Receive(unsigned int *otherword)
{
	if(CAN_MessagePending(CAN1,CAN_FIFO0) == 0) {
		return NULL;
	}
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

	if (otherword)
	{
		if (RxMessage.DLC>4)
		{
			*otherword =  *(u32 *)(&(RxMessage.Data[4]));
		}
		else
			*otherword =0;
	}
	//if(RxMassage.StdId == 0x320){
		return (u32*)&RxMessage.Data[0];
	//}

	//return NULL;
}

#if 0
void hook_CAN_isr(void)
{
	unsigned short *pData = (unsigned short *)&RxMessage.Data;
	int i;

	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

	can_receive_count ++;

		for(i = RxMessage.DLC; i < 8; i ++) RxMessage.Data[i] = 0x0;

		//Message_Push(RxMessage.Data[0], RxMessage.Data[1], pData[1], pData[2],pData[3]);
}

#endif

void Can_error_printf()
{
	//printf("\r\nCan_error=%x ,Last error code=%x \r\n",Get_CAN_err_cnt(),Get_CAN_Last_ErrorCode());
}

uint8 Shell_CAN_error_data_get(uint8** argv,uint8 argc)
{
}

