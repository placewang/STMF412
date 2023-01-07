/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f2xx.h"

typedef unsigned char uint8_t;

void shell_rcv_char(char ch);

void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_Cmd(USART1, DISABLE);
	USART_DeInit(USART1);

	/* Enable USART1 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

	/* 115200_8N1 */
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);
}

void USART_Out(u8 data)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,data);
}

u8 USART_In(u32 dly)
{
	dly *= 100000;

	//while((USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET) && dly)dly--;
	while((USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET) && dly) {
		dly--;
	}
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) != RESET)
		return USART_ReceiveData(USART1);
	else
		return 0;
}

char uart_kbhit()
{
	return 1;
}

char uart_getc(void)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USART1);
}

void uart_putc(int ch)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, ch);
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void uart_puts(char * str)
{
	int i;
	int len = strlen(str);

	for(i = 0; i < len; i ++) {
		uart_putc(str[i]);
	}
}


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return ch;
}

void uart_rxhandler(char ch);

void hook_UART1_isr(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		//shell_rcv_char(USART_ReceiveData(USART1)); 
		uart_rxhandler(USART_ReceiveData(USART1));
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
		//USART_ClearITPendingBit(USART1, USART_IT_TXE);
}

/*----------------------------------------------------------------------------*/

#define UART_BUFF_SIZE				16

typedef struct {
	uint8_t head;
	uint8_t tail;
	uint8_t dat[UART_BUFF_SIZE];
}uart_pool_t;
static uart_pool_t uart_rx;

void uart_rxhandler(char ch)
{
	uint8_t head, tail;
	uart_pool_t *prx = &uart_rx;
	
	head = prx->head;
	tail = prx->tail;
	prx->dat[head++] = ch;
	
	if (head == UART_BUFF_SIZE)
	{
		head = 0;
	}

	if (head == tail)
	{
		/* overflow */
		if (++tail == UART_BUFF_SIZE)
		{
			tail = 0;
		}
	}
	prx->head = head;
	prx->tail = tail;
}

int arch_uart_getc(char *ch)
{
	uint8_t tail;
	uart_pool_t *prx = &uart_rx;
	
	tail = prx->tail;
	if (prx->head == tail)
		return 0;

	if (ch)
		*ch = prx->dat[tail];
	
	tail++;
	if (tail == UART_BUFF_SIZE)
		tail = 0;

	prx->tail = tail;
	
	return 1;
}



unsigned char can_printf_it;
short uart_get_printf_key(char ch)
{
	static char x=0;
	short xx=0;
	//extern ;
	
	printf("\r\n Get_char:%c \r\n",ch);
	switch(ch) 
	{
		case 'D': // Printf KEY
		{
			//x=!x;
			can_printf_it =1;
			//if(x)
			{
				printf_enable();
			}
			xx=1;
		}
		break;
		case 'E': // 
		{
			if(can_printf_it)
			{
				Shell_CAN_error_data_get(NULL,0);
			}
			xx=2;
		}
		break;
		case 'R':
		{
			if(can_printf_it)
			{
				Can_error_printf();
			}
			xx=3;
		}
		break;			
					
	}

	return xx;
	
}


void arch_get_char_loop()
{	char x;
	short xxx;
	if(arch_uart_getc(&x))
	{
		xxx = uart_get_printf_key(x);
		Message_Send_4halfword(0x1111,0x2222,x,xxx);
	}
}