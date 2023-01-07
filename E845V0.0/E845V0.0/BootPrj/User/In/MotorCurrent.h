/***********************************************************************
关于步进电机的电流端口定义
浙江恒强科技
软件平台部:F11500
2022/11/18
************************************************************************/

#ifndef __MOTORCURRENT__H
#define __MOTORCURRENT__H

#define TIM2COUNTVAL      500



signed char StarTime2PwmCH(unsigned char);
signed char  SetTime2DutyRatio(unsigned char PWMCH,float dutyratio);
signed char MotorCurrentSet(unsigned char Munm,unsigned short Cval);

#endif


