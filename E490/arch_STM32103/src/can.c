#include <stdio.h>
#include <string.h>
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "Misc.h"
#include "stm32f2xx_can.h"
#include "stm32f2xx_rcc.h"
#include "config.h"
#ifdef CAN_SEND_ISR_ENABLE
#include "massage.h"
#endif


void wait_us(unsigned int us);
unsigned int arch_Get_Mode(void);
void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);
#ifdef CAN_SEND_ISR_ENABLE

unsigned int can_txisr_full_cnt=0;
unsigned char can_fifo_have_data=0;
//int arch_sendMessage_isr(MASSAGE_SEND_TYPE *mst);
#endif

extern unsigned int arch_Get_ID(void);
extern unsigned int arch_Set_CANID(unsigned int isbroadcastID);
extern int Message_send_Push(unsigned char whichfifo,unsigned int CANid, 
	unsigned char DL, unsigned char *data);

extern void arch_delay_us(int us);

volatile int can_receive_count;
volatile int can_send_error=0;
volatile int can_send_to_main_cnt=0;

#ifdef CHECK_CAN_CNT
volatile int can_receive_jqd_count=0;

#endif
extern volatile unsigned int CAN_error_cnt;

#ifdef CAN_SEND_ISR_ENABLE
#if 0
void CAN_SEND_first_data()
{
	MASSAGE_SEND_TYPE firstdata;
	firstdata.can_id = 0x00000732 + arch_Get_ID() +1;
	firstdata.data_len=8;
	memset(firstdata.data,0,8);
	arch_sendMessage_isr(&firstdata);
}

#endif
#endif


void Check_Can_ERR_loop()
{
	static char last_can_error=0;
	if(last_can_error!=can_send_error)
	{	
		//DWORD tc=;
		arch_LED_Setup(can_send_error?999:99);
		last_can_error = can_send_error;		
	}
}

void CAN_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);

	/* Enable CAN clocks */
	

	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=ENABLE;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;

	CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;//CAN_BS1_10tq;//CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//CAN_BS2_4tq;  //CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler= 2;          // 3


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
	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);



	#if 1
	CAN_ITConfig(CAN1,CAN_IT_EWG ,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_EPV  ,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_BOF ,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_LEC ,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_ERR ,ENABLE);

#endif
	
	
	/* CAN */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;    //¸üÐÂÊÂ¼þ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0+1;   //ÇÀÕ¼ÓÅÏÈ¼¶0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //ÏìÓ¦ÓÅÏÈ¼¶1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï
	
	NVIC_Init(&NVIC_InitStructure);                             //Ð´ÈëÉèÖÃ 

	#ifdef CAN_SEND_ISR_ENABLE

	#if  0
	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;    //¸üÐÂÊÂ¼þ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //ÇÀÕ¼ÓÅÏÈ¼¶0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //ÏìÓ¦ÓÅÏÈ¼¶1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï
	
	NVIC_Init(&NVIC_InitStructure);                             //Ð´ÈëÉèÖÃ 

	CAN_SEND_first_data();
	#endif

#endif
#if 1

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;    //¸üÐÂÊÂ¼þ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   //ÇÀÕ¼ÓÅÏÈ¼¶0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //ÏìÓ¦ÓÅÏÈ¼¶1 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï
	NVIC_Init(&NVIC_InitStructure);                             //Ð´ÈëÉèÖÃ 
#endif

	can_receive_count = 0;
}


unsigned char arch_getCanMailBoxIsEmpty(unsigned char whichcan,unsigned char boxmask)
{
	unsigned char transmit_mailbox = 0;
	CAN_TypeDef* CANx;

	if(whichcan==0)
	{
		CANx = CAN1;
	}
	else
		CANx = CAN2;
	
  /* Check the parameters */
  	assert_param(IS_CAN_ALL_PERIPH(CANx)); 

  /* Select one empty transmit mailbox */
  if ((boxmask & 0x01)&&((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0))
  {
    	transmit_mailbox = 1;
  }
  else if ((boxmask & 0x02)&&((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1))
  {
    transmit_mailbox = 2;
  }
  else if ((boxmask & 0x04)&&((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2))
  {
    transmit_mailbox = 3;
  }
 
  return transmit_mailbox;
}



#define ACTION_ALARM	0
#define ACTION_DATA	1
#define ACTION_LOG     2
#ifdef CAN_SEND_ISR_ENABLE
int arch_sendmessage_main_while(MASSAGE_SEND_TYPE *mst)
{
	unsigned int data_i_temp[4];

	memset(data_i_temp,0,16);
	data_i_temp[0]=mst->data[0]|mst->data[1]<<8;
	data_i_temp[1]=mst->data[2]|mst->data[3]<<8;
	data_i_temp[2]=mst->data[4]|mst->data[5]<<8;
	data_i_temp[3]=mst->data[6]|mst->data[7]<<8;	

	arch_SendMessage(mst->can_id,data_i_temp,mst->data_len>>1);
	return 0;

}

#if 0
int arch_sendMessage_isr(MASSAGE_SEND_TYPE *mst)
{
	CanTxMsg TxMessage;
	unsigned short *pData;
	unsigned int idx;
	unsigned int canresendcount=0;
	
	if (mst->data_len==0) return 0 ;

	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC = mst->data_len;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.StdId =mst->can_id;
	memcpy(TxMessage.Data,mst->data,mst->data_len);

	
	
	
doit_again_cantx:	
	if((idx = CAN_Transmit(CAN1,&TxMessage)) == CAN_NO_MB) 	
	{
		//CAN_error_cnt++;
		canresendcount++;
		arch_delay_us(100);

		if (canresendcount>10)
		{
			//can_send_error =1;
			return -1;	
		}
		else
		goto doit_again_cantx;
	}
	else
	{
	#ifdef CHECK_CAN_CNT
		can_send_to_main_cnt++;
	#endif
		return 0;
	}
}
#endif
#endif

#if 0
void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len)
{
	CanTxMsg TxMessage;
	unsigned short *pData;
	//unsigned int idx;
	unsigned int canresendcount=0;

	static unsigned char can_is_send=0;
	

	if(len == 0) 
		return;

	

	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len * 2L;
	TxMessage.IDE = CAN_ID_STD;
	#if 0
	if (action==3)
	TxMessage.StdId = 0x00000132 + arch_Get_ID() +1; // CAN ID
	else
	#else	
		if (action==4)
			TxMessage.StdId = 0x00000532 + arch_Get_ID() +1; // CAN ID
		else
	#endif		
			TxMessage.StdId = 0x00000732 + arch_Get_ID() +1; // CAN ID
	
	pData = (unsigned short *)&TxMessage.Data;
	do {
		*pData++ = *pMsg++;
	}while(--len);


	if (can_is_send)  //can���ڷ����У����ܴ�Ͻ�������Ҫ����fifo��������
	{	
		#ifdef CAN_SEND_ISR_ENABLE
		Message_send_Push(0,action,TxMessage.DLC,TxMessage.Data);
		can_fifo_have_data |=1;
		#endif
		return;
	}
	

	#if 0 //def CAN_SEND_ISR_ENABLE

//#if 0
	Message_send_Push(TxMessage.StdId,TxMessage.DLC,TxMessage.Data);

	//CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
	//CAN1->sTxMailBox[0].TIR |= 0x01;

	#else
//#endif		
	canresendcount=0;
	can_is_send =1;
again:
	if(CAN_Transmit(CAN1,&TxMessage) == CAN_NO_MB) {
		
		CAN_error_cnt++;
		canresendcount++;
		arch_delay_us(100);/*������������⣿����*/

		if (canresendcount>10)
		{
			can_send_error =1;
			can_is_send =0;
			#ifdef CAN_SEND_ISR_ENABLE
			Message_send_Push(1,action,TxMessage.DLC,TxMessage.Data);
			can_fifo_have_data |=2;
			#endif
			return;	
		}
		else
		goto again;
		
	}
	can_is_send =0;
	can_send_to_main_cnt++;
	can_send_error =0;

#endif
	
	return;

	//if (canresendcount)
	//{
	//	myprintf("ok we send it %d ",canresendcount+1);
	//}

	//
	//while(CAN_TransmitStatus(idx) != CANTXOK);
}

#endif

int can_SendMessage(unsigned short canid, unsigned char *pMsg, unsigned char len)
{
	CanTxMsg TxMessage;
	unsigned short *pData;
	unsigned int idx;
	unsigned int canresendcount=0;
	extern unsigned int arch_Get_ID(void);

	if(len == 0) 
		return 0;

	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len ;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.StdId = canid;

	memcpy(TxMessage.Data,pMsg,len);	

	if(CAN_Transmit(CAN1,&TxMessage)== CAN_NO_MB) 
	{
		return -1;
	}
	//printf("ok\r\n");
	return 1;	
}



CanRxMsg RxMessage;
u32 *CAN0_Receive(unsigned int *otherword)
{
	if(CAN_MessagePending(CAN1,CAN_FIFO0) == 0) {
		return NULL;
	}

	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
	//if(RxMessage.StdId == 0x320){
		return (u32*)&RxMessage.Data[0];
	//}

	//return NULL;
}
#ifdef JAQ_WORKMODE_8_16
extern volatile unsigned int jaq_work_mode_8_16;
#endif

#include "command.h"

#if 0
int Message_Push(unsigned int cmd, unsigned int arg1, unsigned int arg2);
#else
extern int Message_Push(unsigned short cmd, unsigned short arg1, unsigned short arg2, unsigned short arg3,unsigned short arg4);
#endif

extern void Get_can_error_msg(unsigned short whichone);
extern volatile unsigned int step_in_isr;
extern int arch_do_jqdcmd_incan_isr(unsigned short *pData);
extern void Get_main_loop_time_us(unsigned char whichi,unsigned int lastct);
unsigned int firsttime_cnt;
void hook_CAN_isr(void)
{
	//CanRxMsg RxMessage;
	unsigned short *pData = (unsigned short *)&RxMessage.Data;
	int i;

	//firsttime_cnt = Get_now_time_counter();
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

	can_receive_count ++;

		for(i = RxMessage.DLC; i < 8; i ++) RxMessage.Data[i] = 0x0;
		#ifdef JAQ_WORKMODE_8_16
		if (RxMessage.Data[0] == 0x0F)
		{
		
			if (jaq_work_mode_8_16)
			{
				pData = (unsigned short *)&RxMessage.Data[1];

				Message_Push(RxMessage.Data[0], pData[0], pData[1], pData[2],pData[3]);

				return;
			}
		}
		#endif

		#ifdef JQD_DO_CAN_ISR
		if((pData[0]==0x0401)||(pData[0]==0x0106)||(pData[0]==0x0206)||(pData[0]==0x0706)||(pData[0]==0x0906))
		{
			arch_do_jqdcmd_incan_isr(pData);
	
		}
		else
		#endif
		#ifdef JQD_ISR_CNT_ENABLE
		if ((pData[0]==0x0206)||((pData[0] & 0x8FFF)==0x0216))
		{
			extern unsigned int jqdisrcnt;
			jqdisrcnt++;
			#ifdef CHECK_CAN_CNT
			can_receive_jqd_count++;
			#endif
		}
		#endif
		Message_Push(RxMessage.Data[0], RxMessage.Data[1], pData[1], pData[2],pData[3]);

		//Get_main_loop_time_us(0,firsttime_cnt);
#if 0
		if ((RxMessage.Data[0]==8)&&(RxMessage.Data[1]==8) )
		{
			//Get_can_error_msg(pData[1]&0xFF);
			arch_SendMessage(4,&step_in_isr,2);
		}
		else
		arch_SendMessage(3,&can_receive_count,2);
#endif

}

#ifdef CAN_SEND_ISR_ENABLE


extern MASSAGE_SEND_TYPE *Message_send_Pop(unsigned char whichfifo);
//extern int arch_sendMessage_isr(MASSAGE_SEND_TYPE *mst);

void hook_can_tx_2ms_isr(void)
{
	MASSAGE_SEND_TYPE *mst;

	if (!can_fifo_have_data)
		return;

	if (can_fifo_have_data & 0x01)
	{
		mst = Message_send_Pop(0);
		if (mst==NULL)
		{
		//CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			can_fifo_have_data &=~(0x01<<0);
			//return;
		}
		else
		{
			if (arch_sendmessage_main_while(mst)<0)
			{
				can_txisr_full_cnt++;  
			}
		}
		
	}
	if (can_fifo_have_data & 0x02)
	{
		mst = Message_send_Pop(1);
		if (mst==NULL)
		{
			//CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			can_fifo_have_data &=~(0x01<<1);
			return;
		}
		else
		{
			if (arch_sendmessage_main_while(mst)<0)
			{
				can_txisr_full_cnt++;  
			}
		}
		

	}
	


	//CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	
	return;
}

#if 0

void hook_CAN_TX_isr(void)
{
	MASSAGE_SEND_TYPE *mst;
	

	mst = Message_send_Pop();
	if (mst==NULL)
	{
		CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
		return;
	}
	CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	if (arch_sendMessage_isr(mst)<0)
	{
		can_txisr_full_cnt++;  
	}
	return;
	
}
#endif

#endif


extern volatile unsigned int CAN_error_cnt_err[];
void hook_CAN_err_isr(void)
{

	if (SET == CAN_GetITStatus(CAN1,CAN_IT_ERR))
	{
		CAN_error_cnt_err[4]++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_ERR);
		
	}
	
	if (SET == CAN_GetITStatus(CAN1,CAN_IT_EWG))
	{
		CAN_error_cnt_err[0]++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_EWG);
		
	}
	if (SET == CAN_GetITStatus(CAN1,CAN_IT_EPV))
	{
		CAN_error_cnt_err[1]++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_EPV);
		
	}
	if (SET == CAN_GetITStatus(CAN1,CAN_IT_BOF))
	{
		CAN_error_cnt_err[2]++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_BOF);
		
	}
	if (SET == CAN_GetITStatus(CAN1,CAN_IT_LEC))
	{
		CAN_error_cnt_err[3]=CAN_GetLastErrorCode(CAN1);
		CAN_ClearITPendingBit(CAN1,CAN_IT_LEC);
		
	}

	

	//CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

	

}

unsigned int Get_CAN_err_cnt(void)
{
	return CAN_GetReceiveErrorCounter(CAN1)|(CAN_GetLSBTransmitErrorCounter(CAN1)<<16);
	

}

unsigned int Get_CAN_Last_ErrorCode()
{
	return CAN_GetLastErrorCode(CAN1);
}

void Can_error_printf()
{
	printf("\r\nCan_error=%x ,Last error code=%x \r\n",Get_CAN_err_cnt(),Get_CAN_Last_ErrorCode());
}



