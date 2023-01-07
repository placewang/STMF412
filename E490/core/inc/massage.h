
#ifndef __MASSAGE_H__
#define __MASSAGE_H__




typedef struct{
	unsigned short CANID;
	unsigned char DataLen;
	unsigned char CanData[8];
	
}MESSAGE_DCB_CAN_TX;


typedef struct {
	unsigned short cmd;
	unsigned short arg1;
	unsigned short arg2;
#if 1//def PLATFORM_TMS2812
	unsigned short arg3;
	unsigned short arg4;

#endif
}MASSAGE_TYPE;

typedef struct {
	unsigned int can_id;
	unsigned char data_len;
	unsigned char data[8];

}MASSAGE_SEND_TYPE;





typedef struct {
	unsigned char cmd_type;
	unsigned char cmd_data;
	unsigned char arg1_l;
	unsigned char arg1_h;
	unsigned char arg2_l;
	unsigned char arg2_h;
	unsigned char arg3_l;
	unsigned char arg3_h;
	unsigned short arg1;
	unsigned short arg2;
	unsigned short arg3;

}NEWCMDMASSAGE;


void Message_Init(void);
int Message_Count(void);
#if 1//def PLATFORM_TMS2812
int Message_Push(unsigned short cmd, 
	unsigned short arg1, unsigned short arg2, unsigned short arg3, unsigned short arg4);
#else
int Message_Push(unsigned short cmd, unsigned short arg1, unsigned short arg2);
#endif
MASSAGE_TYPE *Message_Pop(void);
void Message_Send(unsigned int Msg);
void Message_Send_Alert(unsigned int Msg, unsigned int arg1);

void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
void Message_send_log_ecode(unsigned int arg1,unsigned int arg2,unsigned int arg3);
#endif

