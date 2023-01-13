/***********************************************************************
boot升级系统指示接口实现
浙江恒强科技
软件平台部:F11500
2022/12/20
************************************************************************/

#include "Upgrade.h"
#include "main.h"
#include "queue.h"
#include "can.h"
#include "CanApp.h"
#include "stmflash.h"
//#include "cmsis_armcc.h"
#include "SenSor.h"
#include "string.h"
unsigned char UpgradeStausTask=0;                //升级任务状态标志
unsigned char NormalUpgradeTask=0;               //正常升级状态标志
unsigned char FUpgradeStausTask=0;               //强制升级任务状态标志
unsigned int  FACKTimeCount=0;                   //强制升级应答等待时间计数 



Revbuff G_upgrade_sub={0};	                      //保存的当前需要写到的FLASH中的数据
unsigned int RevDtFlash[128];                     

QUEUE_DATA_T CanD;                                //Can数据出队暂存

/*
APP跳转函数
*/
void GoApp(void) 
{ 
   volatile uint32_t JumpAddress; 
   pFunction Jump_To; 
    if(((*(unsigned int*)FLASH_APP_START_ADDRESS)&0x2FFE0000)==0x20000000)
    {   
        /* 关闭全局中断 */
        __set_PRIMASK(1); 
        /* 关闭滴答定时器，复位到默认值 */
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        /* 设置所有时钟到默认状态 */
        HAL_RCC_DeInit();
        /* 关闭所有中断，清除所有中断挂起标志 */  
        for (int i = 0; i < 8; i++)
        {
            NVIC->ICER[i]=0xFFFFFFFF;
            NVIC->ICPR[i]=0xFFFFFFFF;
        }        
        /* 使能全局中断 */ 
        __set_PRIMASK(0);
        
        /* 在RTOS工程，这条语句很重要，设置为特权级模式，使用MSP指针 */
        __set_CONTROL(0);
        
        JumpAddress = *(volatile uint32_t*) (FLASH_APP_START_ADDRESS + 4); 
        Jump_To = (pFunction) JumpAddress; 
       /* Initialize user application's Stack Pointer */ 
       __set_MSP(*(volatile uint32_t*) FLASH_APP_START_ADDRESS); 
       Jump_To(); 
    }
}


void RestStarMCU(void)
{
    __set_PRIMASK(1);   //关闭所有中端
    NVIC_SystemReset(); //复位    
}


/*
主控请求回复
bot:命令标记
*/
void ReplyVersionNumber(BootCmdFlag *bot)
{
    //版本号
    if(bot->CheckBoardVersiontMask==1)
    {
        unsigned char data[8]={0x08,0x02,0x09,0x80,0x63,0x00,0x01,0x00};
        CAN1_Send_Msg(data,8,arch_GetBoardID());
        bot->CheckBoardVersiontMask=0;
    }
    //机头重启
    else if(bot->RestartMask==1)
    {
        bot->RestartMask=0;
        RestStarMCU();
    } 

    
    
}



/*
数据接收处理
*/
signed RevData(Revbuff *buf,QUEUE_DATA_T *CanDat)
{   
   unsigned char data[2]={0};  
   if(buf->retry_timer>=1000*5)
   {
    //接收数据中断超时
     buf->retry_state=-1; 
   } 
   if(ProtocolCmd.MasterControlSendUpgradeData==1||ProtocolCmd.MasterControlSendLastPacketData ==1)
   {  
       ProtocolCmd.MasterControlSendUpgradeData=0;
       buf->retry_timer=0; 
       for(int i=0;i<CanDat->DLC-2;i++)
       {
         buf->rx_buf[buf->rx_buf_len]=CanDat->RxData[i+2];//拿掉数据头0x3b/0x3c 0x00
         buf->rx_buf_len++;
         buf->recv_all_data_len++;  
         if(buf->rx_buf_len==512)
         {//缓存512满写入FLASH备份区
            buf->rx_buf_len=0;
            memcpy((unsigned char*)RevDtFlash,buf->rx_buf,512);
            Upgrade_cope_flash_to_backup(&buf->flash_address,RevDtFlash);             
            memset((unsigned char *)buf->rx_buf, 0xFF, 512);     
         }
       }
       if(ProtocolCmd.MasterControlSendLastPacketData ==1)
       {//最后一包写入flash备份区
            memcpy((unsigned char*)RevDtFlash,buf->rx_buf,512);
            Upgrade_cope_flash_to_backup(&buf->flash_address,RevDtFlash);             
            memset((unsigned char *)buf->rx_buf, 0xFF, 512);                
            ProtocolCmd.MasterControlSendLastPacketData =0;
            buf->retry_state=2; 
       }
       data[0]= UPGRADE_DATA_RECEIVED;
       CAN1_Send_Msg(data,2,arch_GetBoardID());  //回复应答发下一包 
    }
   return 0;
}


/*
正常升级流程
    CAN出队数据
*/
signed NormalUpgradeProcess(Revbuff* buff,QUEUE_DATA_T *ndat)
{
    unsigned char data[8]={0};
    unsigned int cmd;   
	unsigned short CRC_check,CRC_16;
    switch(NormalUpgradeTask)
    {
        case NUPGRADE_START:
          //发送准备就绪命令修改升级标志
          memset(data, 0x00, 8);
          data[0]=UPGRADE_REQUEST;
          CAN1_Send_Msg(data,2,arch_GetBoardID());  
          IndividualSectorErasure(BOOT_FLASH_UPGRADE_ADDRESS);
          cmd=CMD_UPGRADE_RCVDATA;
          STMFLASH_Write(BOOT_FLASH_UPGRADE_ADDRESS,&cmd);          
          NormalUpgradeTask=NUPGRADE_RECEIVINGSTATE;
          break; 
        case NUPGRADE_RECEIVINGSTATE:
          //等待主控发数据            
            memset((unsigned char *)buff->rx_buf, 0xFF, 512);
            buff->flash_address=BOOT_FLASH_BACKUP_ADDRESS;
            buff->retry_timer=0;
            NormalUpgradeTask=NUPGRADE_WAITREC;
            break;
        case NUPGRADE_WAITREC :
            RevData(buff,ndat);
            if(buff->retry_state==2)
            {  
                buff->retry_state=0;      
                NormalUpgradeTask= NUPGRADE_LASTTOFLASH;                
            }
            else if(buff->retry_state==-1)
            { 
                memset(data, 0x00, 8);
                data[0]=0x01;
              	data[1]=0x3d;
                CAN1_Send_Msg(data,2,arch_GetBoardID()); 
                RestStarMCU();
            }
            break;    
        case NUPGRADE_LASTTOFLASH :
            // 修改升级标志
            IndividualSectorErasure(BOOT_FLASH_UPGRADE_ADDRESS);
            cmd=CMD_UPGRADE_BURN;
            STMFLASH_Write(BOOT_FLASH_UPGRADE_ADDRESS,&cmd);
            NormalUpgradeTask=NUPGRADE_CRC_CEHCK;       
            break;
        case NUPGRADE_CRC_CEHCK:
            CRC_check = Upgrade_file_crc_check(buff->recv_all_data_len,&CRC_16);
		    if (CRC_check !=CRC_16)
            { // 校验失败上报后 重启
                memset(data, 0x00, 8);
				data[0]=0x03;
                data[1]=0x3d;   
                CAN1_Send_Msg(data,2,arch_GetBoardID()); 
                HAL_Delay(500);
                RestStarMCU();
			}
			else
			{                 
              // 校验成功 拷贝数据
              Upgrade_cope_flash_to_app();
              // 设置标志位     
              IndividualSectorErasure(BOOT_FLASH_UPGRADE_ADDRESS); 
              cmd=CMD_UPGRADE_SUCCESS;
              STMFLASH_Write(BOOT_FLASH_UPGRADE_ADDRESS,&cmd);  
              // 发送0x3e
              memset(data, 0x00, 8); 
              data[0]=UPGRADE_SUCCESS;
              CAN1_Send_Msg(data,2,arch_GetBoardID()); 
              memset(data, 0x00, 8);
              data[0]=0xf0;
              data[2]=0xc0;    
              data[6]=0xab;
              data[7]=0x01;                
              CAN1_Send_Msg(data,8,arch_GetBoardID());  
              memset(data, 0x00, 8);
              data[1]=0x01;
              data[4]=0x01;                
              CAN1_Send_Msg(data,8,arch_GetBoardID());   
              HAL_Delay(500);  
              RestStarMCU();
            }           
            break;
        default:
          break;
    }
    return 0;
}

/*
强制升级或等待跳转APP
*/
signed  ForcedUpgradeTask()
{
    switch(FUpgradeStausTask)
    {   
        case FUPGRADE_BOOTSTART:

            FUpgradeStausTask=FUPGRADE_ACK;
            FACKTimeCount=0;
            break;            
        case FUPGRADE_ACK:
            //收到应答跳转APP
            if(ProtocolCmd.MasterSendACk ==1)
            {
                ProtocolCmd.MasterSendACk =0;   
               //应答表示强制升级进入正常流程
               UpgradeStausTask= UPGRADE_STATUS;
            }
            //超时无应答进入强制升级
            else if(FACKTimeCount>=1000*2)
            {
                //无应答跳转APP
                 GoApp(); 
            }            
           break;  
        default:
          break;           
    }
    return 0;
}


/*
判断boot是否有升级标志
ADDR:标志地址
return 0:升级
       -1:错误状态(进入强制升级)
*/
signed UpgradeFlagJudgment(unsigned int ADDR)
{
    unsigned int FlagVal=0;
    FlagVal=STMFLASH_ReadWord(ADDR);

    if(FlagVal==CMD_UPGRADE_START)
    {
       return 0;
    }
    else if(FlagVal==CMD_UPGRADE_SUCCESS)   
    {
        return 2;
    }
    else if(FlagVal==CMD_UPGRADE_BURN)
    {
        return 1;
    }
    return -1;
}

/*
升级任务
*/
void UpgradeTask(void)
{
    signed char    cmd=0;
    unsigned int   ucmd=0;
    unsigned char data[2]={UPGRADE_FORCE_REQ};
    if(DeQueue(&Can1_revQueuebuff,&CanD))
    {
        CanDataOutTeamElectricMagnet(&CanD);
        ReplyVersionNumber(&ProtocolCmd);
    }
    switch(UpgradeStausTask)
    {
        case UPGRADE_BOOTSTART:     //开机状态
            cmd=UpgradeFlagJudgment(BOOT_FLASH_UPGRADE_ADDRESS);
            if(cmd==-1)
            {
                 CAN1_Send_Msg(data,2,arch_GetBoardID());
                 UpgradeStausTask= UPGRADE_FORCES;
            }
            else if(cmd==2)
            {
                CAN1_Send_Msg(data,2,arch_GetBoardID());
                GoApp(); 
            }
            else if(cmd==0)
            {
                 UpgradeStausTask= UPGRADE_STATUS;
            }
            else if(cmd==1)
            {
               //数据搬移阶段外中断，重新拷贝数据直接跳转APP
                data[0]=0xff;
                data[1]=0xf5;
                CAN1_Send_Msg(data,2,arch_GetBoardID());
                Upgrade_cope_flash_to_app();
                IndividualSectorErasure(BOOT_FLASH_UPGRADE_ADDRESS);
                ucmd=CMD_UPGRADE_SUCCESS;
                STMFLASH_Write(BOOT_FLASH_UPGRADE_ADDRESS,&ucmd); 
                //重启
                RestStarMCU();
            }
            break;       
        case UPGRADE_FORCES:        //进入强制升级状态           
            ForcedUpgradeTask();
            break;        
        case UPGRADE_STATUS:        //进入正常升级状态
            NormalUpgradeProcess(&G_upgrade_sub,&CanD);
            break;
    }
 }
