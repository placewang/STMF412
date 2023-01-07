#include <stdio.h>
//#include "type.h"
#include "command.h"
#include "alert.h"
#include "upgrade.h"


void arch_init(void);

void Test_code_arch_init(void);

DWORD *CAN0_Receive(unsigned int *otherword);
void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);
void wait_ms(unsigned int ms);

void arch_LED_On(void);
void arch_LED_Off(void);
void arch_LED_Setup(DWORD time);

int myprintf();
void GoApp(void);

#define BUFF_SIZE	0x1000
WORD Buff[BUFF_SIZE];




unsigned int BootVer[5] __attribute__((at(0x2001F000)));

DWORD Data_Count;


volatile unsigned char Yarn_use_Step=0;
volatile unsigned char TZ_use_Step=0;
volatile unsigned char SK_use_Step=1;
volatile unsigned int ms01_loop_cnt;

//extern volatile  int Check_APP_isOK;
extern volatile unsigned short last_app_ver;


#define dprintf myprintf 



void delay(DWORD delay)
{
	DWORD i = 4000L * delay;
	while(--i);
}
void Message_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4)
{
	unsigned int buff[4];

	buff[0] = Msg1;
	buff[1] = Msg2;
	buff[2] = Msg3;
	buff[3] = Msg4;
	arch_SendMessage(1, buff, 4);
	//myprintf("send data[%0x %0x ]\n\r",buff[0]&0xff,(buff[0]>>8));
}


void SendMassage(unsigned int Cmd)
{
	arch_SendMessage(1, &Cmd, 1);
}

void Alert_Send(unsigned int Cmd,unsigned short arg)
{
	unsigned int buff[3];
	extern unsigned int arch_Get_ID(void);

	buff[0] = ALARM_HEAD_CAN |  (arch_Get_ID()<<8);	
	buff[1] = Cmd;	
	buff[2] = arg;
	buff[3] = 0;
	arch_SendMessage(0, buff, 4);
}
#ifdef BOOT_UPGRADE_CHECK_CRC
extern unsigned char * arch_Uprade_get_Data_Start_add();
extern unsigned char * arch_Uprade_get_App_Start_add();

extern void arch_Upgrade_Get_Buf_Type(unsigned short *CRCdata);
extern void arch_Upgrade_Set_Buf_Type(unsigned short isapp);
extern void arch_Upgrade_Get_CRC_Data(unsigned short *CRCdata);
extern void arch_Upgrade_Set_CRC_Data(unsigned short CRCdata);

extern unsigned short arch_have_crc_flag(unsigned int *arg);
extern unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, int CRCinit);
extern unsigned short arch_crc_ok(unsigned char *buff,unsigned short CRCdata,unsigned long datelen,unsigned short *crcd);

#define CRC_AND_DATALENGTH_COUNT 	6

#endif

int main()
{
	unsigned int cmd_len;
	unsigned int cmd_Rcv;
	WORD *pData;
	DWORD Count;
	DWORD addr; 
	unsigned short needcrc =0;
	unsigned short appver[4]={0,0,0,0};
	#ifdef BOOT_UPGRADE_CHECK_CRC
	unsigned short lastdata[3]={0xffff,0xffff,0xffff};
	#endif
	int i,j;

	unsigned char recopyit=0;
	
	for (i = 0;i<4;i++)
	{
		BootVer[i] = i;
	}
	BootVer[4] = BOOTVER;  

	delay(3000);//100ms
	
	arch_init();
	delay(50);

	//dprintf("app_flag_ %d \r\n",Check_APP_isOK);

	if(arch_Upgrade_is_Start()) {
		dprintf("Upgrade Running\r\n");
Upgrade:

		fmc_init(fmc_start_addr(1));

		SendMassage(UPGRADE_REQUEST);

		arch_Upgrade_Set_Receive();
		Count = 0;
		Data_Count = 0;

		arch_LED_On();
		while(1) {
			pData = (WORD*)CAN0_Receive(NULL);
			if(pData == NULL) {
				continue;
			}
			SendMassage(UPGRADE_DATA_RECEIVED);

			
			cmd_Rcv = *pData & 0xFF;
			cmd_len = 3;
			if(cmd_Rcv == UPGRADE_ENDDATA) {
				cmd_len = (*pData >> 8) & 0xFF;
			}

			do {
				pData ++;
				Buff[Count++] = *pData;	

			 if(BOOTVER>=BOOTVER_SUPPORT_BINDING)
			 {
				if((Data_Count>=0x100)&&(Data_Count<=0x103)&&(BOOTVER>=BOOTVER_SUPPORT_BINDING))
				{
					appver[Data_Count-0x100] = *pData;
#if 1
					if(Data_Count==0x103)//特定值接收完毕
					{
						//dprintf("Get Ver data (ver=0x%X_0x%X_0x%X_0x%X)\r\n",appver[0],appver[1],appver[2],appver[3]);
							
						if(!((appver[0]==0xCCEE)&&(appver[1]==0xAADD)))
						{
							dprintf("Upgrade UNsuccessful(ver=0x%X_0x%X_0x%X_0x%X)\r\n",appver[0],appver[1],appver[2],appver[3]);
							SendMassage(UPGRADE_DATA_RECEIVEDERR);
							goto boot_upg_end;
						}
						else
						{
							if(last_app_ver!=0xFFFF)
							{
								if(appver[2]<last_app_ver)
								{
									dprintf("Upgrade UNsuccessful(last_app_ver[0x%X]>new_app_ver[0x%X])\r\n",last_app_ver,appver[2]);
									SendMassage(UPGRADE_DATA_RECEIVEDAPPVER_LOW);
									goto boot_upg_end;
								}
							}
						}
					}
#endif					
					
				}
			 }
				Data_Count ++;

				if(Count == BUFF_SIZE) {
					arch_LED_On();
					#ifdef BOOT_UPGRADE_CHECK_CRC
					{
						int i;
						for (i=0;i<3;i++)
						{
							lastdata[i] = Buff[Count-3+i];
						}
					}
					#endif
					fmc_program((BYTE*)&Buff, Count << 1);
					arch_LED_Off();
					Count = 0;
					
				}
			}while(--cmd_len);

			//SendMassage(UPGRADE_DATA_RECEIVED);

			if(cmd_Rcv == UPGRADE_ENDDATA) {
				if(Count) {
					arch_LED_On();
					#ifdef BOOT_UPGRADE_CHECK_CRC
					{
						int i;
						if(Count>=3)
						{
							for (i=0;i<3;i++)
							{
								lastdata[i] = Buff[Count-3+i];
							}
						}
						else
						{
							for (i=0;i<3;i++)
							{
								if (i<(3-Count))
								{
									lastdata[i] = lastdata[i+1];
								}	
								else
								{
									lastdata[i] = Buff[i-(3-Count)];
								}
							}							
						}
					}
					#endif
					fmc_program((BYTE*)&Buff, Count << 1);
					arch_LED_Off();
				}
				Data_Count <<= 1;
				//fmc_Verify(SECTOR_ADDR_0, SECTOR_ADDR_5, Data_Count);

				//dprintf("Total %d\r\n", Data_Count);
				#if 0
				dprintf("appver[](%x.%x.%x.%x)\r\n",appver[0],appver[1],appver[2],appver[3]);

					
				if(!((appver[0]==0xCCEE)&&(appver[1]==0xAADD)))
				{
					dprintf("Upgrade unsuccessful(ver=%x.%d)\r\n",appver[2]&0xff,appver[2]>>8);
					goto boot_upg_end;
				}
				else
					dprintf("Upgrade successful(ver=%x.%d)\r\n",appver[2]&0xff,appver[2]>>8);
					
				#endif

				#ifdef BOOT_UPGRADE_CHECK_CRC
				//dprintf("Upgrade Get_CRC_Flag(CRC_Flag:[0x%X,0x%X,0x%X])\r\n",lastdata[0],lastdata[1],lastdata[2]);
						
				if(arch_have_crc_flag((unsigned int *)&lastdata[0]))
				{
					unsigned short CRC_data_com;
					if(!arch_crc_ok(arch_Uprade_get_Data_Start_add(),lastdata[2],Data_Count-6,&CRC_data_com))
					{

						dprintf("Upgrade UNsuccessful(CRC_ERROR:[0x%X--0x%X])\r\n",lastdata[2],CRC_data_com);
						SendMassage(UPGRADE_DATA_RECEIVEDAPPCRC_ERROR);
						goto boot_upg_end;
						
					}
					//dprintf("Upgrade Successful(CRC_PASS:[0x%X--0x%X])\r\n",lastdata[2],CRC_data_com);
					needcrc =1;	
					
				}
				
				#endif
				
				
				arch_Upgrade_Set_Burn_Size(Data_Count);
				arch_Upgrade_Set_Burn();
				if(needcrc)
				{
				#ifdef BOOT_UPGRADE_CHECK_CRC
				arch_Upgrade_Set_CRC_Data(lastdata[2]);
				arch_Upgrade_Set_Buf_Type(1);				
				#endif
				}

				//arch_Upgrade_APP_Flag_init();

				copy_it_to_app:
				
				fmc_init(fmc_start_addr(0));
				arch_LED_On();
				addr = fmc_start_addr(1);
				fmc_program((BYTE *)addr, Data_Count);
				arch_LED_Off();
				if(recopyit)
					goto yes_can_gotoapp_2;
				break;
			}
		}

		SendMassage(UPGRADE_SUCCESS);

		arch_Upgrade_Finish();
		arch_LED_Off();

		dprintf("Upgrade Successful\r\n");

boot_upg_end:		
		arch_LED_Setup(1000);
		while(1) {
			unsigned short delay_t=4000;
			Alert_Send(DSP_FATAL_ERR,0);
			while(CAN0_Receive(NULL) == NULL)
			{
				delay_t--;
				if (delay_t==0)
					break;
			}
			goto yes_can_gotoapp;
		}
	}
	else if(arch_Upgrade_is_Burn()) {

		dprintf("Burnning\r\n");

		//arch_Upgrade_APP_Flag_init();
		fmc_init(fmc_start_addr(0));

		Data_Count = arch_Upgrade_Get_Burn_Size();

		addr = fmc_start_addr(1);
		fmc_program((BYTE *)addr, Data_Count);

		arch_Upgrade_Finish();
		goto yes_can_gotoapp;
	}
	else {
		DWORD delay = 4000;

		SendMassage(UPGRADE_FORCE_REQ);
		arch_LED_On();
		while(1) {

			unsigned int  main_send_board_type = 0;
			pData = (WORD*)CAN0_Receive(&main_send_board_type);
			
			if(pData == NULL) {
				if(--delay == 0) {
					break;
				}
				continue;
			}

			cmd_Rcv = *pData & 0xFF;
			if(cmd_Rcv == UPGRADE_FORCE_REQ) {

				if ((main_send_board_type!=arch_Get_Board_Type())&&(main_send_board_type))	
				{
					break;
				}
				
				dprintf("Force Upgrade Start\r\n");
				goto Upgrade;
			}
			#ifdef BOOT_CODE_JOIN_TEST_CODE
			if (cmd_Rcv == TEST_CODE_CMD_START)
			{
				goto do_test_code;
			}
			#endif
			
		}
		arch_LED_Off();
	}

	if (0)			/*升级了老程序*/
	{
	
		while(1)
		{
			Alert_Send(HEAD_BOOT_VER_ERR,1);

			get_can_data_again:
			pData = (WORD*)CAN0_Receive(NULL);
			if(pData == NULL) {
				goto get_can_data_again;
			}
			else
			{
				arch_LED_On();
				switch(*pData)
				{
					case BOOT_ACTIVE_CMD_0701://清错
							goto get_can_data_again;
						break;
					case BOOT_ACTIVE_CMD_0802://查板号
						Message_Send_4halfword(BOOT_ACTIVE_CMD_0802,0x8009,0,0);	
						goto get_can_data_again;
						break;
					case BOOT_ACTIVE_CMD_0806:   //升级
						if(*(pData+2)==0x8009)
						{
							goto Upgrade;
						}						
						break;
					default:
						break;
				}
			}
			arch_LED_Off();
		}	
	}		

yes_can_gotoapp:

#ifdef BOOT_UPGRADE_CHECK_CRC		/*CRC校验*/
{
	unsigned short crcdata,crcdata2,crcdata3;
	unsigned short bufferisapp;
	DWORD applen;
	arch_Upgrade_Get_CRC_Data(&crcdata);
	dprintf("Get_CRC_Data(%X) \r\n",crcdata);
	if(crcdata)
	{
		applen=arch_Upgrade_Get_Burn_Size();
		Data_Count = applen;
		dprintf("Get_Burn_Size(%d) \r\n",applen);
		applen -=CRC_AND_DATALENGTH_COUNT;
		arch_Upgrade_Get_Buf_Type(&bufferisapp);
		dprintf("Get_Buf_Type(%d) \r\n",bufferisapp);
		if(bufferisapp==1)
		{
			if(!arch_crc_ok(arch_Uprade_get_App_Start_add(),crcdata,applen,&crcdata2))
			{
				dprintf("app_crc_fail (%x!=%x) \r\n",crcdata,crcdata2);
				//APP IS DESTRUCT
				//	check_data_is ok
				if(!arch_crc_ok(arch_Uprade_get_Data_Start_add(),crcdata,applen,&crcdata3))
				{
					dprintf("buff_crc_fail (%x!=%x) \r\n",crcdata,crcdata3);
					if(crcdata2==crcdata3)
					{
						//isok
					}
					else
					{
						//
						dprintf("goto get_can_data_again\r\n");
						goto get_can_data_again;
					}
				}
				else  //OK
				{
					recopyit=1;
					
					dprintf("goto copy_buff_to_app \r\n");
					goto copy_it_to_app;
				}
			}
		}
	}			
}
#endif
	
	#if 0
	{
		unsigned int firstint=0;
		firstint = *(unsigned int *)0x08000000;
		Message_Send_4halfword(0xBBBB,firstint & 0xffff,(firstint>>16)&0xffff,0);
	}
	#endif
yes_can_gotoapp_2:	
dprintf("goto app() \r\n");
	GoApp();

dprintf("app is empty() \r\n");
goto get_can_data_again;	//说明APP是空的，那就跳出来等着升级

#ifdef BOOT_CODE_JOIN_TEST_CODE

do_test_code:

//做一系列的初始化，尤其是CAN的中断模式打开	

Test_code_arch_init();


while (1)
{
	

}	


#endif
	return 0;
}

