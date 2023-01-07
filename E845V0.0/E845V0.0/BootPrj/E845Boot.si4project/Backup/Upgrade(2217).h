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
#define FLASH_APP_START_ADDRESS 0x8004000     // APP跳转地址

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
#define UPGRADE_DATA_RECEIVEDAPPVER_LOW	  0x023D	//升级命令--接收数据出错 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR 0x033D	//升级命令--CRC校验出错 

#define BOOT_ACTIVE_CMD_0701	0x0107         //
#define BOOT_ACTIVE_CMD_0802	0x0208         //获取板子类型
#define BOOT_ACTIVE_CMD_0806	0x0608         //机头重启


#define POLY16 0x1021


#endif






























