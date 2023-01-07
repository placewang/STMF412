/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"

void IWDG_Init(void)
{

	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
		/* IWDGRST flag set */

		/* Clear reset flags */
		RCC_ClearFlag();
	}
	//��Ϊ�������Ź�ʹ�õ���LSI��������ó���������ʱ��ʹʱ��Դ�ȶ�:

	/* LSI������*/
	 RCC_LSICmd(ENABLE);//��LSI
	 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);//�ȴ�ֱ��LSI�ȶ�
 

	/* IWDG timeout equal to 280 ms (the timeout may varies due to LSI frequency
	     dispersion) */
	  /* Enable write access to IWDG_PR and IWDG_RLR registers */
	  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	  /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
	  IWDG_SetPrescaler(IWDG_Prescaler_32);

	  /* Set counter reload value to 349 */
	  IWDG_SetReload(349);

	  /* Reload IWDG counter */
	  IWDG_ReloadCounter();

	  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	  //IWDG_Enable();
}
  
