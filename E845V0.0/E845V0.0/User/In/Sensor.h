/***********************************************************************
传感器输入定义
浙江恒强科技
软件平台部:F11500
2022/11/17
************************************************************************/

#ifndef __SENSOR__H
#define __SENSOR__H
#include "CanApp.h"
/*
System One(A)
*/

#define MOTOR_A_QSJ_PIN  GPIO_PIN_12  //前床三角电机零点信号管脚
#define MOTOR_A_QDM_PIN  GPIO_PIN_10  //前床度目电机零点信号管脚

#define MOTOR_A_HSJ_PIN  GPIO_PIN_2   //后床三角电机零点信号管脚
#define MOTOR_A_HDM_PIN  GPIO_PIN_11  //后床度目电机零点信号管脚

#define MOTOR_A_QSJ_PORT  GPIOC       //前床三角电机零点信号端口
#define MOTOR_A_QDM_PORT  GPIOC       //前床度目电机零点信号端口

#define MOTOR_A_HSJ_PORT  GPIOD       //后床三角电机零点信号端口
#define MOTOR_A_HDM_PORT  GPIOC       //后床度目电机零点信号端口

#define PROBE_A_PIN       GPIO_PIN_2  //探针输入管脚
#define PROBE_A_PORT      GPIOC       //探针输入端口

#define MOTOR_A_QSJ_ZEROSTATE()   (!HAL_GPIO_ReadPin(MOTOR_A_QSJ_PORT,MOTOR_A_QSJ_PIN))
#define MOTOR_A_QDM_ZEROSTATE() 	(!HAL_GPIO_ReadPin(MOTOR_A_QDM_PORT,MOTOR_A_QDM_PIN))
#define MOTOR_A_HSJ_ZEROSTATE()   (!HAL_GPIO_ReadPin(MOTOR_A_HSJ_PORT,MOTOR_A_HSJ_PIN))
#define MOTOR_A_HDM_ZEROSTATE()		(!HAL_GPIO_ReadPin(MOTOR_A_HDM_PORT,MOTOR_A_HDM_PIN))

#define PROBEINPUT_A_STATE()      (!HAL_GPIO_ReadPin(PROBE_A_PORT,PROBE_A_PIN))

/*
预留暂不使用输入信号
*/

#define MOTOR_B_QSJ_PIN  GPIO_PIN_6  //前床三角电机零点信号管脚
#define MOTOR_B_QDM_PIN  GPIO_PIN_4  //前床度目电机零点信号管脚

#define MOTOR_B_HSJ_PIN  GPIO_PIN_7  //后床三角电机零点信号管脚
#define MOTOR_B_HDM_PIN  GPIO_PIN_5  //后床度目电机零点信号管脚

#define MOTOR_B_QSJ_PORT  GPIOA       //前床三角电机零点信号端口
#define MOTOR_B_QDM_PORT  GPIOA       //前床度目电机零点信号端口

#define MOTOR_B_HSJ_PORT  GPIOA       //后床三角电机零点信号端口
#define MOTOR_B_HDM_PORT  GPIOA       //后床度目电机零点信号端口

#define PROBE_B_PIN       GPIO_PIN_3  //探针输入管脚
#define PROBE_B_PORT      GPIOC       //探针输入端口

#define MOTOR_B_QSJ_ZEROSTATE()   HAL_GPIO_ReadPin(MOTOR_B_QSJ_PORT,MOTOR_B_QSJ_PIN)
#define MOTOR_B_QDM_ZEROSTATE()   HAL_GPIO_ReadPin(MOTOR_B_QDM_PORT,MOTOR_B_QDM_PIN)
#define MOTOR_B_HSJ_ZEROSTATE()   HAL_GPIO_ReadPin(MOTOR_B_HSJ_PORT,MOTOR_B_HSJ_PIN)
#define MOTOR_B_HDM_ZEROSTATE()	  HAL_GPIO_ReadPin(MOTOR_B_HDM_PORT,MOTOR_B_HDM_PIN)

#define PROBEINPUT_B_STATE()      HAL_GPIO_ReadPin(PROBE_B_PORT,PROBE_B_PIN)

/*
系统识别
*/
#define SYSTEMID0_PIN   GPIO_PIN_14
#define SYSTEMID1_PIN		GPIO_PIN_15

#define SYSTEMID0_PROT  GPIOC
#define SYSTEMID1_PROT  GPIOC

#define SYSTEMID0_STATE()  HAL_GPIO_ReadPin(SYSTEMID0_PROT,SYSTEMID0_PIN)
#define SYSTEMID1_STATE()  HAL_GPIO_ReadPin(SYSTEMID1_PROT,SYSTEMID1_PIN)

/*
沙嘴过流报警
*/ 
#define  OVERCURRENT_PIN  GPIO_PIN_5
#define  OVERCURRENT_PROT GPIOC

#define  SZ_OVERCURRENT_STATE() HAL_GPIO_ReadPin(OVERCURRENT_PROT,OVERCURRENT_PIN)

typedef struct
{
 
    unsigned short DM_Sensorstate;     //度目电机零位传感器状态
    unsigned short DM_LastSensorstate; //度目电机零位传感器上一次状态
    
}MotorSersor;



extern  MotorSersor   MotorSensorState;                          //电机传感器状态


unsigned int arch_GetBoardID(void);


void MotorSensorStateLoad(MotorSersor  *Ms);
signed char MotorSensorRequestReturnOperation(MotorSersor  *Mrro,CmdFlag *Mcmd);
#endif



