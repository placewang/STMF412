/***********************************************************************
关于步进电机的IO操作（方向，使能，复位）
浙江恒强科技
软件平台部:F11500
2022/11/15
************************************************************************/


#include "MotorIo.h"
#include "gpio.h"


/*
电机方向设置
Mnum:电机编号
Dr： 设置反向
*/
signed char MotorDirSet(unsigned char Munm,char Dr)
{
		if(Munm>4||(Dr!=1&&Dr!=0))
		{
			return -1;
		}
		if(Dr==1)
		{
				switch(Munm)
				{
					case 1:
					MOTOR1DIR_Positive();	
						break;
					case 2:
						MOTOR2DIR_Positive();		
						break;		
					case 3:
						MOTOR3DIR_Positive();	
						break;		
					case 4:
						MOTOR4DIR_Positive();		
						break;	
					default:
						break;	
				}
		}
		else if(Dr==0)
		{
				switch(Munm)
				{
					case 1:
						MOTOR1DIR_Negative(); 	
						break;
					case 2:
						MOTOR2DIR_Negative();		
						break;		
					case 3:
						MOTOR3DIR_Negative();	
						break;		
					case 4:
						MOTOR4DIR_Negative();		
						break;
					default:
						break;		
				}			
		}
	return 0;
}


/*
设置电机使能/失能
En:1 EN
	 0 dis	
*/
void MotorSetPowerSwitch(unsigned char En)
{
	if(En==0)
	{
		MOTOREN_OFF();
	}
	else
	{
		MOTOREN_NO(); 
	}
}

/*
电机复位操作
Rs: 1 复位
    0 工作
*/
void MotorSetRST(unsigned char Rs)
{
	if(Rs==0)
	{
			MOTORRST_NO();
	}
	else
	{
			MOTORRST_OFF();
	}

}






