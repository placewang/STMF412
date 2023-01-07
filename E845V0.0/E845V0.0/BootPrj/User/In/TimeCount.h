#ifndef __TIMECOUNT__H
#define __TIMECOUNT__H
#include "SolenoidValve.h"
#include "Upgrade.h"


extern unsigned int 		ledTimeCount;
extern unsigned int         FACKTimeCount;                   //强制升级应答等待时间计数 
extern Revbuff              G_upgrade_sub;	                        //保存的当前需要写到的FLASH中的数据
#endif

