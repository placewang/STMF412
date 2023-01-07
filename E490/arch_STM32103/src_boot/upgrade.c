#include <stdio.h>
//#include "type.h"
#include "command.h"
#include "eeprom.h"
#include "stm32f2xx.h"
#include "alert.h"
//#include "fmc.h"

#ifndef BYTE
typedef unsigned char  BYTE;
#endif
#ifndef WORD
typedef unsigned short WORD;
#endif
#ifndef DWORD
typedef unsigned long  DWORD;
#endif

#define CMD_UPGRADE_START	0xA050AA00
#define CMD_UPGRADE_RCVDATA	0xA050AA55
#define CMD_UPGRADE_BURN	0xA0505A5A
#define CMD_UPGRADE_SUCCESS	0xA05055AA
#define CMD_UPGRADE_UNSUCCESS	0xA0505500



#define MAIN_BOARD_ID_FLASH_ADDR		90
#define CMD_OFFSET		0x00	// 4 BYTE
#define CMD_OFFSET_LENG		0x02	// 4 BYTE



#define SECTOR_ADDR_BOOT		((u32)0x08000000)
#define SECTOR_ADDR_APP			((u32)0x08020000)
#define SECTOR_ADDR_DATA			((u32)0x08060000)

#define SECTOR_NO_BOOT			FLASH_Sector_0
#define SECTOR_NO_APP			FLASH_Sector_5
#define SECTOR_NO_DATA			FLASH_Sector_7

#define SECTOR_FIRST_NO_BOOT		0
#define SECTOR_FIRST_NO_APP			5
#define SECTOR_FIRST_NO_DATA		7



#define FMC_BOOT_SECTOR_COUNT		3


#define FMC_PAGE_SIZE_BOOT 		0x4000
#define FMC_PAGE_SIZE 			0x20000

#define dprintf myprintf 

DWORD fmc_start_address = SECTOR_ADDR_APP;
DWORD fmc_start_sector = SECTOR_NO_APP;
DWORD fmc_start_sector_no = SECTOR_FIRST_NO_APP;
DWORD fmc_offset;
DWORD fmc_sector_size;
DWORD fmc_err_count;

#ifdef DO_OPERATE_FLASH_CHECK
extern unsigned int Flash_next_do;
/*
Flash_next_do =0 ¡À¨ª¨º?2??¡¥¡Á¡Â
Flash_next_do =1 ¡À¨ª¨º?2¨¢3y
Flash_next_do =2 D¡ä2¨´¡Á¡Â
*/
#endif
#ifdef BOOT_UPGRADE_CHECK_CRC
unsigned char * arch_Uprade_get_Data_Start_add()
{
	return (unsigned char *)SECTOR_ADDR_DATA;
}

unsigned char * arch_Uprade_get_App_Start_add()
{
	return (unsigned char *)SECTOR_ADDR_APP;
}


//extern unsigned char * arch_Uprade_get_Data_Start_add();
#define CRC_CHAR_FLAG 	(('C' << 0) | ('R' << 8) | ('C' << 16) | (0x10 << 24))
unsigned short arch_have_crc_flag(unsigned int *arg)
{
	if(*arg == CRC_CHAR_FLAG)                                                                                                                                                                                                               
	{
		return 1;
	}
	else
		return 0;
}


#define POLY16 0x1021
unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, int CRCinit)
{
	unsigned char ch;
	int i;
	unsigned short CRC_loc = (unsigned short)CRCinit;
	
	while (dlen--)
	{
		ch = *buf++;
		CRC_loc ^= (((unsigned short) ch) << 8);
		for (i = 0; i < 8; i++)
		{
			if (CRC_loc & 0x8000)
				CRC_loc = (CRC_loc << 1) ^ poly;
			else
				CRC_loc <<= 1;
		}

	}
	return CRC_loc;
}


unsigned short arch_crc_ok(unsigned char *buff,unsigned short CRCdata,unsigned long datelen,unsigned short *crcd)
{
	unsigned short crc_temp;
	crc_temp = CRC16((unsigned char *)buff,datelen,POLY16,0);
	if(crcd)
		*crcd = crc_temp;
	if (crc_temp==CRCdata)
	{
		return 1;
	}
	else
		return 0;
}

#define CMD_OFFSET_CRC_DATA		0x04		// 2 BYTE
#define CMD_OFFSET_BUF_TYPE		0x05		// 2 BYTE
#define CMD_BUF_TYPE_APP			0xCBAA	// 2 BYTE
#define CMD_BUF_TYPE_BOOT			0xCBBB	// 2 BYTE





void arch_Upgrade_Set_CRC_Data(unsigned short CRCdata)
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = CRCdata;	
	EE_Write(CMD_OFFSET_CRC_DATA, (u16*)&Upgrade_Cmd, 1);
}

void arch_Upgrade_Get_CRC_Data(unsigned short *CRCdata)
{
	//DWORD Upgrade_Cmd;
	int ret;
	ret = EE_Read(CMD_OFFSET_CRC_DATA, (u16*)CRCdata, 1);
	//dprintf("read-crc-data ret=%d\r\n",ret);
	if(ret!=0)
	{
		*CRCdata = 0;
	}
}


void arch_Upgrade_Set_Buf_Type(unsigned short isapp)
{
	DWORD Upgrade_Cmd;

	if(isapp)
	{
		Upgrade_Cmd = CMD_BUF_TYPE_APP;	
	}
	else
	{
		Upgrade_Cmd = CMD_BUF_TYPE_BOOT;	
	}
	
	EE_Write(CMD_OFFSET_BUF_TYPE, (u16*)&Upgrade_Cmd, 1);
}

void arch_Upgrade_Get_Buf_Type(unsigned short *CRCdata)
{
	unsigned short Upgrade_Cmd;
	int ret;
	*CRCdata = 0xFFFF;
	ret = EE_Read(CMD_OFFSET_BUF_TYPE, (u16*)&Upgrade_Cmd, 1);
	if(ret==0)
	{
		if(Upgrade_Cmd==CMD_BUF_TYPE_APP)
		{
			*CRCdata =1;
		}
		if(Upgrade_Cmd==CMD_BUF_TYPE_BOOT)
		{
			*CRCdata =0;
		}		
	}
}


#endif

void arch_Upgrade_Set_Start()
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = CMD_UPGRADE_START;
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+7;
	}
	#endif
	EE_Write(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);
}

int arch_Upgrade_is_Start()
{
	DWORD Upgrade_Cmd;

	EE_Read(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);

	return (Upgrade_Cmd == CMD_UPGRADE_START);
}

void arch_Upgrade_Set_Receive()
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = CMD_UPGRADE_RCVDATA;
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+8;
	}
	#endif
	EE_Write(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);
}

void arch_Upgrade_Set_Burn_Size(DWORD Data_Count)
{
#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+9;
	}
#endif
	EE_Write(CMD_OFFSET_LENG, (u16*)&Data_Count, 2);
}

DWORD arch_Upgrade_Get_Burn_Size()
{
	DWORD Data_Count;
	EE_Read(CMD_OFFSET_LENG, (u16*)&Data_Count, 2);

	return Data_Count;
}

void arch_Upgrade_Set_Burn()
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = CMD_UPGRADE_BURN;
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+10;
	}
	#endif
	EE_Write(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);
}


void arch_Upgrade_APP_Flag_init()
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = 0;
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+11;
	}
	#endif
	EE_Write(MAIN_BOARD_ID_FLASH_ADDR+7, (u16*)&Upgrade_Cmd, 1);
}



int arch_Upgrade_is_Burn()
{
	DWORD Upgrade_Cmd;

	EE_Read(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);

	return (Upgrade_Cmd == CMD_UPGRADE_BURN);
}

void arch_Upgrade_Finish()
{
	DWORD Upgrade_Cmd;

	Upgrade_Cmd = CMD_UPGRADE_SUCCESS;
	#if 0
	{
		extern volatile unsigned short writeflash_index;
		writeflash_index=WRTEFLASH_BASE+12;
	}
	#endif
	EE_Write(CMD_OFFSET, (u16*)&Upgrade_Cmd, 2);
}

void fmc_init(DWORD start_addr)
{
	fmc_start_address = start_addr;
	fmc_offset = 0;
	fmc_err_count = 0;
	fmc_sector_size = FMC_PAGE_SIZE;
	if (fmc_start_address == SECTOR_ADDR_DATA)
	{
		fmc_start_sector = SECTOR_NO_DATA;
		fmc_start_sector_no = SECTOR_FIRST_NO_DATA;
	}
	else if(fmc_start_address ==SECTOR_ADDR_APP)
	{
		fmc_start_sector = SECTOR_NO_APP;
		fmc_start_sector_no = SECTOR_FIRST_NO_APP;
	}
	else
	{
		fmc_start_sector = SECTOR_NO_BOOT;
		fmc_start_sector_no = SECTOR_FIRST_NO_BOOT;
		fmc_sector_size = FMC_PAGE_SIZE_BOOT;
	}
	

	
}

DWORD fmc_start_addr(int sel)
{
	return (sel ?((sel==2)?SECTOR_ADDR_BOOT: SECTOR_ADDR_DATA ): SECTOR_ADDR_APP);
}

unsigned int Get_Sector_address_with_SecNo(unsigned char SecNO)
{
	unsigned int sc_;
	switch(SecNO)
	{		
			case 0:
				sc_ = FLASH_Sector_0;
			break;
			case 1:
				sc_ = FLASH_Sector_1;
			break;
			case 2:
				sc_ = FLASH_Sector_2;
			break;
			case 3:
				sc_ = FLASH_Sector_3;
			break;
			case 4:
				sc_ = FLASH_Sector_4;
			break;
			case 5:
				sc_ = FLASH_Sector_5;
			break;
			case 6:
				sc_ = FLASH_Sector_6;
			break;
			case 7:
				sc_ = FLASH_Sector_7;
			break;
			case 8:
				sc_ = FLASH_Sector_8;
			break;
			case 9:
				sc_ = FLASH_Sector_9;
			break;
			case 10:
				sc_ = FLASH_Sector_10;
			break;
			default :
				sc_ = FLASH_Sector_11;
				break;		
	}
	return sc_;
}

void fmc_EraseSector_boot(void)
{
	int i;
	DWORD sc_; 

	FLASH_Unlock();
	for (i=0;i<FMC_BOOT_SECTOR_COUNT;i++)
	{
		sc_ = Get_Sector_address_with_SecNo(i);		
		#ifdef DO_OPERATE_FLASH_CHECK
		Flash_next_do = FLASH_DO_ERASE;	
		#endif			
		FLASH_EraseSector(sc_,VoltageRange_3);
	}		

	FLASH_Lock();
}

void fmc_program(BYTE *Buff, DWORD len)
{
	int i;
	DWORD data_offset = 0;
	DWORD left = len;
	DWORD wrlen;
	DWORD max_write = FMC_PAGE_SIZE;

	FLASH_Unlock();

	do {
		if(left > max_write) {
			wrlen = max_write;
		}
		else {
			wrlen = left;
		}
		if(fmc_start_sector!=SECTOR_NO_BOOT)
		{
			if(fmc_offset == 0) 
			{
				unsigned int secadd_2_erase;
				#ifdef DO_OPERATE_FLASH_CHECK
				Flash_next_do = FLASH_DO_ERASE;	
				#endif
				
				secadd_2_erase = Get_Sector_address_with_SecNo(fmc_start_sector_no);
				FLASH_EraseSector(secadd_2_erase,VoltageRange_3);
			}
		}

		//dprintf("wlen %d\r\n", wrlen);
		for(i = 0; i < wrlen; i += 2) {
			#ifdef DO_OPERATE_FLASH_CHECK
			Flash_next_do = FLASH_DO_PROGRAM;	
			#endif
			//FLASH_ProgramOptionByteData(fmc_start_address + fmc_offset + i, i/*Buff[data_offset + i]*/);
			FLASH_ProgramHalfWord(fmc_start_address + fmc_offset + i, *(u16*)&Buff[data_offset + i]);
		}

		data_offset += wrlen;
		fmc_offset += wrlen;
		if(fmc_offset >= fmc_sector_size) {
			fmc_offset -= fmc_sector_size;
			fmc_start_address += fmc_sector_size;
			fmc_start_sector_no++;
		}

		left -= wrlen;
		max_write = FMC_PAGE_SIZE;

	} while(left);

	FLASH_Lock();
	//dprintf("program end\r\n");
}

void fmc_Verify(DWORD addr0, DWORD addr1, DWORD len)
{
	DWORD i;
	volatile DWORD *pData0 = (volatile DWORD *)addr0;
	volatile DWORD *pData1 = (volatile DWORD *)addr1;

	for(i = 0; i < len; i ++) {
		dprintf("[%lx]: [%lX] <=> [%lX]\r\n", i, *(pData0 + i), *(pData1 + i));
		if(*(pData0 + i) != *(pData1 + i)) {
			dprintf("Verify Fail\r\n");
			fmc_err_count ++;
			if(fmc_err_count > 10) break;
		}
	}

	//if(fmc_err_count)
	//	dprintf("Verify Fail %ld\r\n", fmc_err_count);
}

void fmc_program_stop()
{
}

