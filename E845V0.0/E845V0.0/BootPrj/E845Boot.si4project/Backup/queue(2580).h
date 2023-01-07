/***********************************************************************
关于CAN数据接受队列接口定义（通用文件）
浙江恒强科技
软件平台部:F11500
2022/11/15
************************************************************************/
#ifndef __QUEUE__H
#define __QUEUE__H

#include "can.h"
#define True  1
#define False 0
#define  CAN1LEN                     200

typedef struct 
{
  uint32_t StdId;       
  uint32_t IDE;      
  uint32_t RTR;      
  uint32_t DLC;      
	//uint32_t Timestamp; 
  //uint32_t FilterMatchIndex; 
	uint8_t RxData[8];
} CAN_RecvTypeDef;


#define QUEUE_DATA_T  CAN_RecvTypeDef      //队列数据类型定义

typedef CAN_RecvTypeDef ElemType;





// 定义循环队列结构体
typedef struct Queue{
	unsigned int front;	  //队列头下标
	unsigned int rear;	  //队列尾下标
	unsigned int MAXSIZE; //队列缓存长度（初始化时赋值）
	QUEUE_DATA_T *data;
}Queue;



extern Queue Can1_revQueuebuff;
extern QUEUE_DATA_T CAN1Buff[CAN1LEN];			 //CAN1队列缓存区
extern QUEUE_DATA_T CAN1_recDataInfo;        //CAN1接收缓存 

void InitQueue(Queue* q,QUEUE_DATA_T * buffer, unsigned int len);
int DeQueue(Queue* , ElemType *);
int EnQueue(Queue* , ElemType );
int QueueLength(Queue *q);

//CAN
void CanRevDataPermutation(CAN_RxHeaderTypeDef*, CAN_RecvTypeDef *,unsigned char * );
void CAN1_Send_Msg(unsigned char * ,const unsigned int , const unsigned int );

#endif




