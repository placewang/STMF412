/***********************************************************************
关于步进电机的IO端口定义（方向，使能，复位）
浙江恒强科技
软件平台部:F11500
2022/11/15
************************************************************************/

#ifndef __MOTORIO__H
#define	__MOTORIO__H

#define MOTOR1DIR_Pin 	GPIO_PIN_6
#define MOTOR2DIR_Pin 	GPIO_PIN_7
#define MOTOR3DIR_Pin 	GPIO_PIN_8
#define MOTOR4DIR_Pin 	GPIO_PIN_9

#define MOTORRST_Pin  	GPIO_PIN_12

#define MOTOREN_Pin  	  GPIO_PIN_15

#define MOTOR1DIR_Port  GPIOC
#define MOTOR2DIR_Port  GPIOC
#define MOTOR3DIR_Port  GPIOC
#define MOTOR4DIR_Port  GPIOC

#define MOTORRST_Port   GPIOA

#define MOTOREN_Port    GPIOA

#define MOTOR1DIR_Positive()       		HAL_GPIO_WritePin(MOTOR1DIR_Port, MOTOR1DIR_Pin, GPIO_PIN_RESET)
#define MOTOR2DIR_Positive()       		HAL_GPIO_WritePin(MOTOR2DIR_Port, MOTOR2DIR_Pin, GPIO_PIN_RESET)
#define MOTOR3DIR_Positive()       		HAL_GPIO_WritePin(MOTOR3DIR_Port, MOTOR3DIR_Pin, GPIO_PIN_RESET)
#define MOTOR4DIR_Positive()       		HAL_GPIO_WritePin(MOTOR4DIR_Port, MOTOR4DIR_Pin, GPIO_PIN_RESET)

#define MOTOR1DIR_Negative()      		HAL_GPIO_WritePin(MOTOR1DIR_Port, MOTOR1DIR_Pin, GPIO_PIN_SET)
#define MOTOR2DIR_Negative()      		HAL_GPIO_WritePin(MOTOR2DIR_Port, MOTOR2DIR_Pin, GPIO_PIN_SET)
#define MOTOR3DIR_Negative()      		HAL_GPIO_WritePin(MOTOR3DIR_Port, MOTOR3DIR_Pin, GPIO_PIN_SET)
#define MOTOR4DIR_Negative()      		HAL_GPIO_WritePin(MOTOR4DIR_Port, MOTOR4DIR_Pin, GPIO_PIN_SET)

#define MOTORRST_NO()       		      HAL_GPIO_WritePin(MOTORRST_Port, MOTORRST_Pin, GPIO_PIN_RESET)
#define MOTORRST_OFF()       		      HAL_GPIO_WritePin(MOTORRST_Port, MOTORRST_Pin, GPIO_PIN_SET)

#define MOTOREN_NO()       		        HAL_GPIO_WritePin(MOTOREN_Port, MOTOREN_Pin, GPIO_PIN_SET)
#define MOTOREN_OFF()       		      HAL_GPIO_WritePin(MOTOREN_Port, MOTOREN_Pin, GPIO_PIN_RESET)



signed char MotorDirSet(unsigned char Munm,char Dr);
void MotorSetPowerSwitch(unsigned char En);
void MotorSetRST(unsigned char Rs);
#endif


