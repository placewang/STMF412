/***********************************************************************
boot升级系统指示接口实现
浙江恒强科技
软件平台部:F11500
2022/12/20
************************************************************************/

#include "Upgrade.h"
#include "main.h"

/*
APP跳转函数
*/
void GoApp(void) 
{ 
   volatile uint32_t JumpAddress; 
   pFunction Jump_To; 

   JumpAddress = *(volatile uint32_t*) (FLASH_APP_START_ADDRESS + 4); 
   Jump_To = (pFunction) JumpAddress; 
   /* Initialize user application's Stack Pointer */ 
   __set_MSP(*(volatile uint32_t*) FLASH_APP_START_ADDRESS); 
   Jump_To(); 
}















