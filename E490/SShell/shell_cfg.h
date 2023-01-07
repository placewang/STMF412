/***************************************************************************************
Product: S-Shell
Product Info: A Shell interface for embeded systems,talorable,configurable,user-definable
Current Version: 1.0beta
License: see license.txt
CopyRight by SimonQian(Ç®Ïþ³¿)

File: shell_cfg.h
File Info: configuration file for S-Shell
File Data: 2007.11.14
****************************************************************************************
Change Log:

***************************************************************************************/
// Type definations,need to be changed for different Compiler
#include "stm32f2xx.h"
// End of type definations

// Common inclulde files
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// End of common include files

// Add other include files here to define interface functions & system functions
// End of user-defined include files

// If SShell should be on serval interfaces, for example: USB(CDC) and COMM, firmware will sellect which interface to use
// redirect these funcionts to the correspongding interface-operation functions:
// void (*Shell_WriteChar)(uint8);
// int16 (*Shell_ReadChar)(uint8);
// void (*Shell_Interface_Init)(void);
// void (*Shell_Interface_Fini)(void);
// void (*Shell_WriteStr)(uint8*);
#define SHELL_MULTIINTERFACE		0


// by xhl 2010/12/10
#define SEG_NUM		50



// enable this if your compiler support malloc() to save RAM
#define HAS_MALLOC					0

// 	Press:	'Esc'(\e) and '[' as ESC
//			'3' + COLOR as FGColor(COLOR is from '0' to '7')
//			'4' + COLOR ad BGColor
//			'0' or '1' as Font
//			';' as spacer
//			'm' as end
//	Exgample: 'Esc'+'['+'3'+'1'+';'+'4'+'0'+';'+'1'+'m'("\e[31;40;1m")
#define SHELL_COLOR_EN				0

// Define USER_NAME and USR_PASSWORD, input when login
#define SHELL_LOGIN_EN				1

// when receive data from interface, will SShell echo?
#define SHELL_CMD_ECHO				1

#if SHELL_LOGIN_EN == 1
#define USR_PSW_LEN				20
extern uint8 IsUsrPswValid(uint8*,uint8*);

#define USR_NAME				"simon"
#define USR_PASSWORD				"ilovethisgame"
#endif		// #if SHELL_LOGIN_EN == 1

// Max. buffer size of a single line
#define SHELL_LINE_BUFFER_SIZE			255

#if HAS_MALLOC == 0
// U must define the max. parameter number for that some embeded system compilers dont support malloc
#define SHELL_PARA_NUM				(SHELL_LINE_BUFFER_SIZE / 2)
#endif		// #if !(HAS_MALLOC == 1)

// how to access flash?
// 0:call function to access flash, override Shell_WriteStr_P and Shell_WriteLine_P
// 1:access directly
#define FLASH_ACCESS				1

#if FLASH_ACCESS == 0
	#define STR_DEF					char __flash *//uint8*//__flash uint8*
	#define STR_TYP					(STR_DEF)
	#define STR_T(s)				(s)
#else
	#define STR_DEF					uint8*
	#define STR_TYP					(STR_DEF)
	#define STR_T(s)				(uint8*)(s)
#endif

// Shell interface functions definations
extern void USART_Out(u8 data);
extern u8 USART_In(u32 dly);
#if SHELL_MULTIINTERFACE == 0
	#define Shell_WriteChar(CH)		USART_Out(CH)
	#define Shell_ReadChar(dly)		USART_In(dly)
	#define Shell_Interface_Init()	
	#define Shell_Interface_Fini()	
#endif


// Watchdog Reset
#define WD_Reset()



#if FLASH_ACCESS == 1
	#define Shell_WriteStr(p)		Shell_WriteStr_Int(p)
	#define Shell_WriteStr_P(p)		Shell_WriteStr((uint8*)p)
	#define Shell_WriteLine_P(p)		Shell_WriteLine((uint8*)p)
extern u8 myStrCmp(char *s0, char *s1);
	#define strcmp_P(s0,s1)			myStrCmp(s0,s1)
#else
	// s0 is in RAM while S1 in Flash
	#define strcmp_P(s0,s1)
	// Shell_GetFlashChar will be called in Shell_WriteStr_P
	#define Shell_GetFlashChar(c,p)		*(uint8*)memcpy_P(c,(p),1)
	#define Shell_WriteStr(p)		Shell_WriteStr_P((STR_DEF)p)
#endif					

