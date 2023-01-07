/***********************************************************************
boot升级数搬移过程接口实现
浙江恒强科技
软件平台部:F11500
2022/12/20
************************************************************************/

#include "Upgrade.h"
#include "main.h"
#include "stmflash.h"


unsigned short CRC16Check(unsigned char* buf, unsigned int len)
{
	unsigned int i, j;
	unsigned short usCRCReg;
	unsigned char temp;

	usCRCReg = FLASH_read(buf) << 8;
	usCRCReg |= FLASH_read(buf + 1);

	for (i = 2; i < len; i++)
	{
		temp = FLASH_read(buf + i);
		for (j = 8; j > 0; j--)
		{
			if ((usCRCReg & 0x8000) != 0)
			{
				usCRCReg <<= 1;
				if (((temp >> (j - 1)) & 0x01) != 0)
					usCRCReg |= 0x0001;
					usCRCReg ^= 0x1021;
			}
			else
			{
				usCRCReg <<= 1;
				if (((temp >> (j - 1)) & 0x01) != 0)
					usCRCReg |= 0x0001;
			}
		}
	}
	
	for (j = 16; j > 0; j--)
	{
		if ((usCRCReg & 0x8000) != 0)
		{
			usCRCReg <<= 1;
			usCRCReg ^= 0x1021;
		}
		else
			usCRCReg <<= 1;
	}
	return usCRCReg;
}
/*

*/
unsigned short Upgrade_file_crc_check(unsigned short len,unsigned short *CRC16)
{
	unsigned char *backup_address_p;
	unsigned char  *Packend_CRCaddress;
	Packend_CRCaddress=(unsigned char *)(BOOT_FLASH_BACKUP_ADDRESS +len-6);
	if(Packend_CRCaddress[0]=='C'&&Packend_CRCaddress[1]=='R'&&Packend_CRCaddress[2]=='C'&&Packend_CRCaddress[3]==0x10)
	{
        //取下发的CRC值
        *CRC16=Packend_CRCaddress[4];
        *CRC16|=Packend_CRCaddress[5]<<8;        
		len=len-6;
	}
	backup_address_p = (unsigned char *)BOOT_FLASH_BACKUP_ADDRESS ;
	return CRC16Check(backup_address_p, len);
}
/*
	满512字节先写入内部FLASH备份扇区写操作
	address：写的地址
	data_p：写的数据，512个字节
*/
void Upgrade_cope_flash_to_backup(unsigned int* address, unsigned int *data_p)
{
    // 扇区开始，先擦除
    IndividualSectorErasure(*address);    
    for(int i=0;i<128;i++)
    {
        STMFLASH_Write(*address, data_p);
        *address += 4;
		data_p++;
    }
 
}


/*
从备份区拷贝到应用区
*/

void Upgrade_cope_flash_to_app(void)
{
	unsigned int app_address;
	unsigned int backup_address;
    //APP 扇区开始，先擦除
    IndividualSectorErasure(ADDR_FLASH_SECTOR_5);
    
	backup_address = BOOT_FLASH_BACKUP_ADDRESS;

	for (app_address = FLASH_APP_START_ADDRESS; app_address < FLASH_APP_STOP_ADDRESS; app_address+=4)
	{
        STMFLASH_Write(app_address,(unsigned int *)backup_address);
		backup_address += 4;       
	}
}



























