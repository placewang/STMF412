/***********************************************************************
boot升级系统指示接口定义
浙江恒强科技
软件平台部:F11500
2022/12/20
************************************************************************/

#ifndef  _UPGRADE_H
#define  _UPGRADE_H


typedef void (*pFunction)(void);

// FLASH 地址定义
#define FLASH_APP_START_ADDRESS     0x08020000     // APP跳转地址
#define FLASH_APP_STOP_ADDRESS	    0x0803FFFF	  // APP的结束地址

#define BOOT_FLASH_UPGRADE_ADDRESS  0x08004000     //boot升级标志存放地址

#define BOOT_FLASH_BACKUP_ADDRESS   0x08040000     //程序数据备份区地址

// FLASH 升级标志
#define CMD_UPGRADE_START		0xA050AA00    //启动升级的标志
#define CMD_UPGRADE_RCVDATA  	0xA050AA55    //接收升级数据标志
#define CMD_UPGRADE_BURN		0xA0505A5A    //搬运数据标志
#define CMD_UPGRADE_SUCCESS		0xA05055AA    //升级完成标志
#define CMD_UPGRADE_UNSUCCESS	0xA0505500    //升级失败标志

//升级交互相关命令说明：
#define UPGRADE_REQUEST		   0x3A	          //机头升级就绪
#define UPGRADE_DATA		   0x3B	          //主控下发升级数据
#define UPGRADE_ENDDATA		   0x3C			  //主控下发最后一包数据
#define UPGRADE_DATA_RECEIVED  0x3D	          //机头接收数据应答
#define UPGRADE_SUCCESS		   0x3E			  //机头升级成功
#define UPGRADE_FORCE_REQ	   0x3F			  //机头启动成功

#define UPGRADE_DATA_RECEIVEDERR	      0x013D	//升级命令--接收数据出错 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR 0x033D	//升级命令--CRC校验出错 





//升级流程状态
typedef enum
{
	UPGRADE_BOOTSTART=   0, //boot启动
	UPGRADE_STATUS=      1, //进入升级
    UPGRADE_FORCES=      2, //强制升级
}UpgradeProcessStatus;

//强制升级流程状态
typedef enum
{
	FUPGRADE_BOOTSTART=   0, //进入强制升级发送0x3F
	FUPGRADE_ACK=         1, //等待主控应答
    
}FUpgradeProcessStatus;
//升级流程状态
enum
{
	NUPGRADE_START=   0,         //进入正常升级流程
    NUPGRADE_RECEIVINGSTATE= 1,  //进入接收状态
    NUPGRADE_WAITREC= 2,         //等待数据下发状态 
    NUPGRADE_LASTTOFLASH= 3,     //最后一包写道FLASH备份区 
    NUPGRADE_CRC_CEHCK =4,       //CRC校验           
};

typedef struct UPGRADE_SUB
{
	signed char    retry_state;			   // 接收状态 1：满512字节  2:最后一包 收到一包 3 -1：接收超时   
	unsigned short retry_timer;			   // 接收计时超时 退出升级跳转APP
	unsigned short rx_buf_len;			   // 当前的接收长度	
	unsigned char  rx_buf[512];			   // 接收到的包（满存一次备份区FLASH）	
	unsigned int   recv_all_data_len;	   // 收到的数据总长度
    unsigned int   flash_address;		   // 当前操作的flash地址
    
}Revbuff;

typedef struct CONFIG_BAUD
{
	unsigned int Msenddata_len;			//发来的数据长度
    unsigned int CountData_len;         //本地累计接收数长度
	unsigned short crc_check;			// CRC16校验
}LastRebuff;


signed UpgradeFlagJudgment(unsigned int );
void UpgradeTask(void);
unsigned short Upgrade_file_crc_check(unsigned short len,unsigned short *CRC16);
void Upgrade_cope_flash_to_backup(unsigned int* address, unsigned int *data_p);
void Upgrade_cope_flash_to_app(void);
void GoApp(void); 
#endif






























