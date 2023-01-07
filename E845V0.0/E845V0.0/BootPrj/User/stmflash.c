#include "stmflash.h"
#include "main.h"

/*读取指定地址的字(32位数据) 
faddr:读地址 
返回值:对应数据.
*/
unsigned int STMFLASH_ReadWord(unsigned int faddr)
{
	return *(volatile unsigned int*)faddr; 
}

unsigned char FLASH_read(unsigned char *u32Addr)
{
	 return *((volatile uint8_t*)u32Addr);
}
/*获取某个地址所在的flash扇区
addr:flash地址
返回值:0~11,即addr所在的扇区
*/
unsigned char STMFLASH_GetFlashSector(unsigned int addr)
{
//    if(addr<ADDR_FLASH_SECTOR_0)return FLASH_SECTOR_0;
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6; 

 
	return FLASH_SECTOR_11;	
}
/*
单独擦除扇区
SectorAddr:要擦除扇区首地址
*/

signed IndividualSectorErasure(unsigned int SectorAddr)
{
	FLASH_EraseInitTypeDef FlashEraseInit;
    unsigned int SectorError=0;
    
	if(SectorAddr<STM32_FLASH_BASE||SectorAddr%4)                       //非法地址  
    {
        return -1;
    }   
	if(SectorAddr==ADDR_FLASH_SECTOR_1||SectorAddr==ADDR_FLASH_SECTOR_2 \
        ||SectorAddr==ADDR_FLASH_SECTOR_3||SectorAddr==ADDR_FLASH_SECTOR_4\
        ||SectorAddr==ADDR_FLASH_SECTOR_5||SectorAddr==ADDR_FLASH_SECTOR_6\
        ||SectorAddr==ADDR_FLASH_SECTOR_11
        )
    {
        HAL_FLASH_Unlock();                                                //解锁	    
        FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;                  //擦除类型，扇区擦除 
        FlashEraseInit.Sector=STMFLASH_GetFlashSector(SectorAddr);         //要擦除的扇区
        FlashEraseInit.NbSectors=1;                                        //一次只擦除一个扇区
        FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;                 //电压范围，VCC=2.7~3.6V之间!!
        if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
        {
            return -2;                                                     //发生错误了	
        }
        FLASH_WaitForLastOperation(FLASH_WAITETIME);     
        HAL_FLASH_Lock();                                                  //上锁   
    } 

    return 0;
}
/*

 特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
WriteAddr:地址
pBuffer:数据指针
*/
void STMFLASH_Write(unsigned int WriteAddr,unsigned int *pBuffer)	
{ 

	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	                                         //非法地址
    
	HAL_FLASH_Unlock();                                                               //解锁	
   
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)          //写入数据
	{ 
		 Error_Handler();	                                                          //写入异常
	}
     
	HAL_FLASH_Lock();                                                                 //上锁
} 
/*
从指定地址开始读出指定长度的数据
ReadAddr:起始地址
pBuffer:数据指针
NumToRead:字(32位)数
*/
void STMFLASH_Read(unsigned int ReadAddr,unsigned int *pBuffer,unsigned int NumToRead)   	
{
	unsigned int i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;                           //偏移4个字节.	
	}
}

