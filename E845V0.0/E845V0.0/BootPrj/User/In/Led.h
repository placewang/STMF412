/***********************************************************************
关于LED系统指示接口定义
浙江恒强科技
软件平台部:F11500
2022/11/15
************************************************************************/
#ifndef __LED__H
#define __LED__H

#define LED1_Pin 				GPIO_PIN_13
#define LED1_GPIO_Port    	    GPIOC
#define LED1_ON()       		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF()      		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE()   		((LED1_GPIO_Port->IDR & LED1_Pin)?(LED1_GPIO_Port->BSRR |= (uint32_t)LED1_Pin << 16U):(LED1_GPIO_Port->BSRR |= LED1_Pin))




void LedTask(unsigned short );
#endif



