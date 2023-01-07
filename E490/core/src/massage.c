#include <stdio.h>
#include <string.h>

#include "type.h"
#include "massage.h"
#include "alert.h"
#include "arch.h"	
#include "command.h"	
#include "config.h"

#define MAX_MESSAGE	80
MASSAGE_TYPE MsgBuff[MAX_MESSAGE];
volatile unsigned int msg_rptr, msg_wptr;


#define MAX_MESSAGE_BUF_TX 64
MESSAGE_DCB_CAN_TX MsgBuff_Send[MAX_MESSAGE_BUF_TX];
volatile unsigned int msg_rptr_send, msg_wptr_send;

#define MAX_MESSAGE_BUF_TX_ISR 32
MESSAGE_DCB_CAN_TX MsgBuff_Send_ISR[MAX_MESSAGE_BUF_TX_ISR];
volatile unsigned int msg_rptr_send_ISR, msg_wptr_send_ISR;


volatile unsigned char enable_log_tosend=0;

#ifdef LOG_DEBUG_FOR_LX_AT_CH
volatile unsigned char enable_log_tosend_LX=0;
volatile unsigned char enable_log_zero_change=0;

#endif
extern unsigned int arch_Get_ID(void);

#ifdef CAN_SEND_ISR_ENABLE
#define MAX_MESSAGESEND	40
MASSAGE_SEND_TYPE MsgsendBuff[2][MAX_MESSAGESEND];//0--send_isr.1--sendagain
volatile unsigned int msg_send_rptr[2], msg_send_wptr[2];//0--send_isr.1--sendagain



int Message_send_Count(unsigned char whichfifo)
{
	int count;
	if (whichfifo>1) return MAX_MESSAGESEND;
	count = msg_send_wptr[whichfifo];
	count += MAX_MESSAGESEND;
	count -= msg_send_rptr[whichfifo];
	if(count >= MAX_MESSAGESEND) {
	count -= MAX_MESSAGESEND;
	}	
	return count;
	
}

//#if 1//def PLATFORM_TMS2812
int Message_send_Push(unsigned char whichfifo,unsigned int CANid, 
	unsigned char DL, unsigned char *data)
{
	if (whichfifo>1) return -2;
	if(Message_send_Count(whichfifo) >= MAX_MESSAGESEND - 1) {
		alert_push(CAN_BUF_ERR, 1+whichfifo);
		return -1;
	}

	MsgsendBuff[whichfifo][msg_send_wptr[whichfifo]].can_id = CANid;
	MsgsendBuff[whichfifo][msg_send_wptr[whichfifo]].data_len = DL;
	memset(MsgsendBuff[whichfifo][msg_send_wptr[whichfifo]].data,0,8);
	memcpy(MsgsendBuff[whichfifo][msg_send_wptr[whichfifo]].data,data,DL);
	
	

	msg_send_wptr[whichfifo] ++;
	if(msg_send_wptr[whichfifo] >= MAX_MESSAGESEND) {
		msg_send_wptr[whichfifo] -= MAX_MESSAGESEND;
	}
	
	return 0;
}

MASSAGE_SEND_TYPE *Message_send_Pop(unsigned char whichfifo)
{
	if (whichfifo>1) return NULL;
	if(Message_send_Count(whichfifo) > 0) {
		int rptr = msg_send_rptr[whichfifo];
		msg_send_rptr[whichfifo]++;
		if(msg_send_rptr[whichfifo] >= MAX_MESSAGESEND) {
			msg_send_rptr[whichfifo] -= MAX_MESSAGESEND;
		}
		return &MsgsendBuff[whichfifo][rptr];
	}
	return NULL;
}

#endif



void Message_Init()
{
	msg_rptr = msg_wptr = 0;
	#ifdef CAN_SEND_ISR_ENABLE
	msg_send_rptr[0] =0;
	msg_send_wptr[0] =0;
	msg_send_rptr[1] =0;
	msg_send_wptr[1] =0;
	#endif
	
	msg_rptr_send =msg_wptr_send=0;
	msg_rptr_send_ISR =msg_wptr_send_ISR=0;
/*	
	memset(MsgBuff, 0, MAX_MESSAGE * sizeof(MASSAGE_TYPE));
*/
}

int Message_Count_Tx(unsigned char isr)
{
	int count;
	if(isr)
	{
		count = msg_wptr_send_ISR;
		count += MAX_MESSAGE_BUF_TX_ISR;
		count -= msg_rptr_send_ISR;
		if(count >= MAX_MESSAGE_BUF_TX_ISR) {
			count -= MAX_MESSAGE_BUF_TX_ISR;
		}

	}
	else
	{
		count = msg_wptr_send;
		count += MAX_MESSAGE_BUF_TX;
		count -= msg_rptr_send;
		if(count >= MAX_MESSAGE_BUF_TX) {
			count -= MAX_MESSAGE_BUF_TX;
		}
	}
	return count;
}


int Message_Count()
{
	int count = msg_wptr;
	count += MAX_MESSAGE;
	count -= msg_rptr;
	if(count >= MAX_MESSAGE) {
		count -= MAX_MESSAGE;
	}

	return count;
}

int Message_TX_Push(unsigned short CanID,unsigned char *data,unsigned char dataLen)
{
	static unsigned char Tx_is_Doing=0;
	int ret=0;

	if(Tx_is_Doing)
	{
		if(Message_Count_Tx(1) >= MAX_MESSAGE_BUF_TX_ISR- 1) {
			alert_push(CAN_BUF_ERR, 254);
			ret =-1;
			//goto IS_return_fun;
		}
		else
		{

			MsgBuff_Send_ISR[msg_wptr_send_ISR].CANID= CanID;
			MsgBuff_Send_ISR[msg_wptr_send_ISR].DataLen = dataLen;
			memcpy(MsgBuff_Send_ISR[msg_wptr_send_ISR].CanData,data,dataLen);
			//memcpy_hlc(char * des, const char * ssr, unsigned int strlen)
			

			msg_wptr_send_ISR ++;
			if(msg_wptr_send_ISR >= MAX_MESSAGE_BUF_TX_ISR) {
				msg_wptr_send_ISR -= MAX_MESSAGE_BUF_TX_ISR;
			}
			ret =0;
		}
	}
	else
	{	
		Tx_is_Doing =1;
		if(Message_Count_Tx(0) >= MAX_MESSAGE_BUF_TX - 1) {
			alert_push(CAN_BUF_ERR, 255);
			ret =-1;
		//	goto IS_return_fun;
		}
		else
		{

			MsgBuff_Send[msg_wptr_send].CANID= CanID;
			MsgBuff_Send[msg_wptr_send].DataLen = dataLen;
			memcpy(MsgBuff_Send[msg_wptr_send].CanData,data,dataLen);
			//memcpy_hlc(char * des, const char * ssr, unsigned int strlen)
			

			msg_wptr_send ++;
			if(msg_wptr_send >= MAX_MESSAGE_BUF_TX) {
				msg_wptr_send -= MAX_MESSAGE_BUF_TX;
			}
			ret =0;
		}	
		
		Tx_is_Doing =0;			
	}
	
	return ret;
}

MESSAGE_DCB_CAN_TX *Message_TX_Pop(unsigned char isr)
{
	if(isr)
	{
		if(Message_Count_Tx(1) > 0) 
		{
			int rptr = msg_rptr_send_ISR;
			#if 0
			msg_rptr_send++;
			if(msg_rptr_send >= MAX_MESSAGE_BUF_TX) {
				msg_rptr_send -= MAX_MESSAGE_BUF_TX;
			}
			#endif
			return &MsgBuff_Send_ISR[rptr];
		}
		return NULL;

	}
	else
	{
		if(Message_Count_Tx(0) > 0) 
		{
			int rptr = msg_rptr_send;
			#if 0
			msg_rptr_send++;
			if(msg_rptr_send >= MAX_MESSAGE_BUF_TX) {
				msg_rptr_send -= MAX_MESSAGE_BUF_TX;
			}
			#endif
			return &MsgBuff_Send[rptr];
		}
		return NULL;
	}
}

void Message_TX_rpt_next(unsigned char isr)
{
	if(isr)
	{
		msg_rptr_send_ISR++;
		if(msg_rptr_send_ISR>= MAX_MESSAGE_BUF_TX_ISR) {
			msg_rptr_send_ISR-= MAX_MESSAGE_BUF_TX_ISR;
		}
	}
	else
	{
		msg_rptr_send++;
		if(msg_rptr_send >= MAX_MESSAGE_BUF_TX) {
			msg_rptr_send -= MAX_MESSAGE_BUF_TX;
		}
	}
}


#if 1//def PLATFORM_TMS2812
int Message_Push(unsigned short cmd, 
	unsigned short arg1, unsigned short arg2, unsigned short arg3, unsigned short arg4)
{
	if(Message_Count() >= MAX_MESSAGE - 1) {
		#ifdef NEW_ALARM_STYLE
		alert_push(OTHER_ERR_CODE_ARG(0,0));
		#else
		alert_push(CAN_BUF_ERR, 0);

		#endif
		return -1;
	}

	MsgBuff[msg_wptr].cmd = cmd;
	MsgBuff[msg_wptr].arg1 = arg1;
	MsgBuff[msg_wptr].arg2 = arg2;
	MsgBuff[msg_wptr].arg3 = arg3;
	MsgBuff[msg_wptr].arg4 = arg4;
	
	msg_wptr ++;
	if(msg_wptr >= MAX_MESSAGE) {
		msg_wptr -= MAX_MESSAGE;
	}
	
	return 0;
}
#else
int Message_Push(unsigned short cmd, unsigned short arg1, unsigned short arg2)
{
	if(Message_Count() >= MAX_MESSAGE - 1) {
		alert_push(CAN_BUF_ERR, 0);
		return -1;
	}

	MsgBuff[msg_wptr].cmd = cmd;
	MsgBuff[msg_wptr].arg1 = arg1;
	MsgBuff[msg_wptr].arg2 = arg2;

	msg_wptr ++;
	if(msg_wptr >= MAX_MESSAGE) {
		msg_wptr -= MAX_MESSAGE;
	}
	
	return 0;
}
#endif
MASSAGE_TYPE *Message_Pop()
{
	if(Message_Count() > 0) {
		int rptr = msg_rptr;
		msg_rptr ++;
		if(msg_rptr >= MAX_MESSAGE) {
			msg_rptr -= MAX_MESSAGE;
		}
		return &MsgBuff[rptr];
	}
	return NULL;
}

#define ACTION_ALARM	0
#define ACTION_DATA	1
#define ACTION_LOG     	2
#define ACTION_ST     	3



void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);


void Message_Send_head_st(unsigned char *stbuff)
{
	unsigned int buff[4];
	buff[0] = ST_HEAD_CAN |  (arch_Get_ID()<<8);
	memcpy(&buff[1],stbuff,6);
	arch_SendMessage(ACTION_ST, buff, 4);

}


void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len)
{
	unsigned char Txdata[8]={0,0,0,0,0,0,0,0};
	unsigned short *pData;
	unsigned short CanID_;
	unsigned char len_;
	extern unsigned int arch_Get_ID(void);

	if(len == 0) 
		return;
	len_ = len;
	CanID_ = 0x0732 + arch_Get_ID() +1;	
	pData = (unsigned short *)&Txdata[0];
	do {
		*pData++ = *pMsg++;
	}while(--len);
	Message_TX_Push(CanID_,Txdata,(len_<<1));	
}

void arch_SendMessage_immediately(unsigned int action, unsigned int *pMsg, unsigned int len)
{
	unsigned char Txdata[8]={0,0,0,0,0,0,0,0};
	unsigned short *pData;
	unsigned short CanID_;
	unsigned char len_;
	int ret=0;
	int loop_cnt=0;
	MESSAGE_DCB_CAN_TX Massage2TX_d;
	extern unsigned int arch_Get_ID(void);

	if(len == 0) 
		return;
	len_ = len;
	CanID_ = 0x0732 + arch_Get_ID() +1;	
	pData = (unsigned short *)&Txdata[0];
	do {
		*pData++ = *pMsg++;
	}while(--len);

	loop_cnt =0;

	do
	{
		if(arch_getCanMailBoxIsEmpty(0,0x07)!=0)
		{
			ret = can_SendMessage(CanID_,Txdata,(len_<<1));
		}
		else
			ret =-1;
		loop_cnt++;
	}
	while((ret<0)&&(loop_cnt<10000));
	if(loop_cnt>=10000)
	{
		/**/
		printf("can send immediately error !\n\r");
	}	
	//Message_TX_Push(CanID_,Txdata,(len_<<1));	
}



void arch_SendMessage_with_main()
{
	
	MESSAGE_DCB_CAN_TX *Massage2TX;
	int ret=0;
	if(arch_getCanMailBoxIsEmpty(0,0x07)!=0)
	{
		
		Massage2TX = Message_TX_Pop(0);
		if(Massage2TX!=NULL)
		{
			#if 0
			printf("s_1!canid=[%x],len=[%d],data[%x,%x,%x,%x,%x,%x,%x,%x]\n\r",Massage2TX->CANID,Massage2TX->DataLen,Massage2TX->CanData[0],Massage2TX->CanData[1],Massage2TX->CanData[2],
				Massage2TX->CanData[3],Massage2TX->CanData[4],Massage2TX->CanData[5],Massage2TX->CanData[6],Massage2TX->CanData[7]);
			#endif
			ret = can_SendMessage(Massage2TX->CANID,Massage2TX->CanData,Massage2TX->DataLen);
			if(ret>=0)
			{
				Message_TX_rpt_next(0);
				
				//printf("s_2!\n\r");
			}
		}
	}
	Massage2TX = Message_TX_Pop(1);
	if(Massage2TX!=NULL)
	{
	//	printf("s_3!\n\r");
		ret = Message_TX_Push(Massage2TX->CANID,Massage2TX->CanData,Massage2TX->DataLen);
		if(ret>=0)
		{
		//	printf("s_4!\n\r");
			Message_TX_rpt_next(1);
		}
}
	}



#if 0
void Message_Send_halfword(unsigned int Msg)
{
	arch_SendMessage(ACTION_DATA, &Msg, 1);
}



void Message_Send_2halfword(unsigned int Msg1,unsigned int Msg2)
{
	unsigned int buff[2];

	buff[0] = Msg1;
	buff[1] = Msg2;
	arch_SendMessage(ACTION_DATA, buff, 2);
}

void Message_Send_3halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3)
{
	unsigned int buff[3];

	buff[0] = Msg1;
	buff[1] = Msg2;
	buff[2] = Msg3;
	arch_SendMessage(ACTION_DATA, buff, 3);
}

#endif

void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4)
{
	unsigned int buff[4];

	buff[0] = Msg1;
	buff[1] = Msg2;
	buff[2] = Msg3;
	buff[3] = Msg4;
	arch_SendMessage(ACTION_DATA, buff, 4);
	//myprintf("send data[%0x %0x ]\n\r",buff[0]&0xff,(buff[0]>>8));
}

#if 0
void Message_Send_4halfword_debug(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4)
{
	unsigned int buff[4];

	buff[0] = Msg1;
	buff[1] = Msg2;
	buff[2] = Msg3;
	buff[3] = Msg4;
	arch_SendMessage(ACTION_DATA, buff, 4);
	//myprintf("send data[%0x %0x ]\n\r",buff[0]&0xff,(buff[0]>>8));
}

#endif



void Message_Send(unsigned int Msg)
{
	arch_SendMessage(ACTION_DATA, &Msg, 1);
}


void Message_Send_immediately(unsigned int Msg)
{
	arch_SendMessage_immediately(ACTION_DATA, &Msg, 1);

}


#if 0
void Message_Send_EX(unsigned int Msg1,unsigned int Msg2)
{
	unsigned int buff[2];

	buff[0] = Msg1;
	buff[1] = Msg2;
	arch_SendMessage(ACTION_DATA, buff, 2);
}

#endif

void Message_send_log_ecode(unsigned int arg1,unsigned int arg2,unsigned int arg3)
{
	unsigned int buff[4];
	

	if (!enable_log_tosend)
		return ;

	buff[0] = LOG_HEAD_CAN |  (arch_Get_ID()<<8);	
	buff[1] = arg1;
	buff[2] = arg2;
	buff[3] = arg3; 

	arch_SendMessage(ACTION_ALARM, buff, 4);

}

#ifdef LOG_DEBUG_FOR_LX_AT_CH
#define LOG_HEAD_CAN_LX 0xF4

void Message_send_log_LX(unsigned int arg1,unsigned int arg2,unsigned int arg3)
{
	unsigned int buff[4];
	static char idx_send=0;	

	if ((!enable_log_tosend_LX)&&(!enable_log_zero_change))
		return ;

	buff[0] = LOG_HEAD_CAN_LX |  ((unsigned short)idx_send<<8);	
	buff[1] = arg1;
	buff[2] = arg2;
	buff[3] = arg3; 
	idx_send++;

	arch_SendMessage(ACTION_ALARM, buff, 4);

}

#endif

void Message_Send_Alert(unsigned int Msg, unsigned int arg1)
{
	unsigned int buff[4];
	extern unsigned int arch_Get_ID(void);

	buff[0] = ALARM_HEAD_CAN |  (arch_Get_ID()<<8);	
	buff[1] = Msg;
	buff[2] = arg1 & 0xffff;
	buff[3] = (arg1>>16) & 0xffff; 

	arch_SendMessage(ACTION_ALARM, buff, 4);
	//myprintf("alarm_code[%0x] arg[%0x]\n\r",buff[0],buff[1]);
}

#if 0
void Message_Send_Alert_DEBUG(unsigned int Msg, unsigned int arg1)
{
	unsigned int buff[4];
	extern unsigned int arch_Get_ID(void);

	buff[0] = 0xFF |  (arch_Get_ID()<<8);	
	buff[1] = Msg;
	buff[2] = arg1;
	buff[3] = 0xFFFF; 

	arch_SendMessage(ACTION_ALARM, buff, 3);
	//myprintf("alarm_code[%0x] arg[%0x]\n\r",buff[0],buff[1]);
}

#endif
#ifdef LOGOUT_ENABLE

extern volatile unsigned short logout_needle_pos;

void Message_Send_Log_Sti(unsigned int cmd,
	unsigned int curPos, unsigned int dstPos)
{	
	unsigned int buff[4];	
	buff[0] = cmd;	
	buff[1] = logout_needle_pos;	
	buff[2] = curPos;	
	buff[3] = dstPos;		
	arch_SendMessage(ACTION_LOG, buff, 4);
}

void Message_Send_Log_Dct(unsigned int cmd,
	unsigned int group, unsigned int index, unsigned int status)
{	
	unsigned int buff[4];	

	buff[0] = cmd;	
	buff[1] = logout_needle_pos;	
	if(cmd == LOG_OUT_YARN)	{		
		buff[2] = index;	
	}	
	else {		
		buff[2] = (group << 8) | index;		
	}		
	buff[3] = status;
	arch_SendMessage(ACTION_LOG, buff, 4);
}
#endif

