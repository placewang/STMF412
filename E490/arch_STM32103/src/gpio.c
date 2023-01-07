/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "platform_config.h"




void GPIO_Configuration_8844RESET(void)
{

//	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(   RCC_AHB1Periph_GPIOF , ENABLE);
	



	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9  ;//| GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);



}



void GPIO_Configuration_powerctr(void)
{

//	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(   RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOF , ENABLE);
	


	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);



	#ifdef E490_V10_BOARD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1  ;//| GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	#else
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  ;//| GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	#endif

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7  ;//| GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	

}

#ifdef E490_V10_BOARD
void Gpio_cfg_yarn_(unsigned int isyarnusestep)
{
	return;
}

#else
#ifdef E480_BOARD_V10
void Gpio_cfg_yarn_(unsigned int isyarnusestep)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;
	if (isyarnusestep)
	{

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      	 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	}
		
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}
#endif
#endif

void Gpio_cfg_EXP_Board_(unsigned char boardtype)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 , ENABLE);
#ifndef E490_V10_BOARD
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);				//
#endif
	#if 0
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

#endif
  
	

	switch (boardtype)
	{
		case 0x00:
			
		break;
		case 0x01:
		{
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_3;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOE, &GPIO_InitStructure);				//

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |GPIO_Pin_8;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);				//
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_6|GPIO_Pin_15;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);				//
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOC, &GPIO_InitStructure);				//	
		}	
		break;
		case 0x02:
			{
				/*PWM_DA_1*/
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			/*PWM_DA_2*/
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
			GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);	

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			#ifdef ECODE_USE_MT6813_PWM
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			#else
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
			#endif
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);				//
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);				//


			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			//GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			//GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
			GPIO_Init(GPIOC, &GPIO_InitStructure);				//	

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_3;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOE, &GPIO_InitStructure);	

			GPIO_InitStructure.GPIO_Pin =    GPIO_Pin_10 | GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);

			}
		break;
		case 0x03:
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_3;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOE, &GPIO_InitStructure);	

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);				//
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);				//

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			//GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
			GPIO_Init(GPIOC, &GPIO_InitStructure);				//	

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  			//GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP;
  			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);				//	

			GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
			GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);			
			
		}			
		break;
		default:
			break;
	}

		#ifdef ECODE_USE_MT6813_PWM
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		#endif

}

void Gpio_PB4_config_def()
{
	GPIO_InitTypeDef GPIO_InitStructure;
		#ifdef ECODE_USE_MT6813_PWM
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, 0);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		#endif
}

#ifdef E499_BOARD_SUPPORT_
void Gpio_cfg_E499Board_()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	

}
#else
void Gpio_cfg_E499Board_()
{
return;
}
#endif

void GPIO_EX_E690Board()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(   RCC_AHB1Periph_GPIOG , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void GPIO_Configuration_default(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOF |RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_Init(GPIOG, &GPIO_InitStructure);

	
}


#ifdef E490_V10_BOARD

void GPIO_SET_711_FAULT_AN(unsigned char isN)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if (isN) /*-24*/
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		
	}
	else
	{
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOE, &GPIO_InitStructure);			
	}

}


void GPIO_Configuration_PB9ctr(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(   RCC_AHB1Periph_GPIOB , ENABLE);
	


	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void GPIO_Configuration(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOF |RCC_AHB1Periph_GPIOG, ENABLE);

  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); 
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 |GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7  | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;/*20190128  由于PB12管脚特殊，必须配置成悬空输入*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);





	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  |GPIO_Pin_4| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11 |GPIO_Pin_12| GPIO_Pin_13 |  GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =    GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//ok201606281806




	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 |GPIO_Pin_6;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_3 |  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10  | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	
	

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_15  ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	#ifdef E693_TEN_BLADE_NEEDLE_SELECTOR
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	#else
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	#endif
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_7   | GPIO_Pin_8| GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	#ifdef V24_CURRENT_711_NO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

	#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	#endif
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	

	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4| GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_9  | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);


GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource2,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource3,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource4,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource5,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource13,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource14,GPIO_AF_FSMC);



	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13  | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	#ifdef V24_CURRENT_711_NO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

	#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	#endif
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3|GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11  | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

#if 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	#endif

}


#else


void GPIO_Configuration(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOF |RCC_AHB1Periph_GPIOG, ENABLE);
	/* GPIO_PinRemap */
	//GPIO_PinRemapConfig(GPIO_Remap2_CAN, ENABLE);
	//AFIO->MAPR &= 0xFFFF9FFF;

//GPIO_PinSource0
	//GPIO_PinAFConfig(GPIOA,GPIO_Pin_12|GPIO_Pin_11,GPIO_AF_CAN1);

  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); 
	

	/* Configure CAN Tx (PA.12) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);



	  

  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	#if 0

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



	/* for ssi clk*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* for ssi dat*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif



	

  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7  | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_12| GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10| GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 |GPIO_Pin_6 |  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10  | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	#ifdef E480_BOARD_V10
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15 ;	
	#else
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;	
	#endif	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_15  ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12   ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	#ifdef E480_BOARD_V10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;	
	#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;	
	#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);


GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource2,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource3,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource4,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource5,GPIO_AF_FSMC);
GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);


	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11 | GPIO_Pin_13  | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_15  ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11  | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	

}

#endif

