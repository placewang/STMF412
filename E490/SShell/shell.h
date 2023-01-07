/***************************************************************************************
Product: S-Shell
Product Info: A Shell interface for embeded systems,talorable,configurable,user-definable
Current Version: 1.0beta
License: see license.txt
CopyRight by SimonQian(Ç®Ïþ³¿)

File: shell_cfg.h
File Info: main include file for S-Shell
File Data: 2006.7.2
****************************************************************************************
Change Log:

***************************************************************************************/
#include "shell_cfg.h"

#define Shell_Str					STR_T("#")
#define Shell_StrVer					STR_T("1.0Beta")
//#define Shell_StrInfo					STR_T("S-Shell1.0Beta by SimonQian(www.SimonQian.com)")
#define Shell_StrInfo					STR_T("S-Shell V1.1Beta by Hailong Xu")
#define Shell_ByeStr					STR_T("S-Shell:Bye!")
#define Shell_LineStr					STR_T("\r\n")
#define Shell_Height					30

#if SHELL_COLOR_EN == 1
#define Shell_DefFGColor				Shell_Color_Blue
#define Shell_DefBGColor				Shell_Color_White
#define Shell_DefAttr					(Shell_Attrs_HighLight)

#define Shell_Attrs_None				(0)
#define Shell_Attrs_HighLight				(1 << 0)
#define Shell_Attrs_UnderLine				(1 << 3)
#define Shell_Attrs_Blink				(1 << 4)
#define Shell_Attrs_Inverse				(1 << 6)
#define Shell_Attrs_Disappear				(1 << 7)

#define Shell_Attr_None					'0'
#define Shell_Attr_HighLight				'1'
#define Shell_Attr_UnderLine				'4'
#define Shell_Attr_Blink				'5'
#define Shell_Attr_Inverse				'7'
#define Shell_Attr_Disappear				'8'

#define Shell_ESC					0x1B
#define Shell_Spacer					';'
#define Shell_Color_FG					'3'
#define Shell_Color_BG					'4'
#define Shell_Color_Black				'0'
#define Shell_Color_Red					'1'
#define Shell_Color_Green				'2'
#define Shell_Color_Yellow				'3'
#define Shell_Color_Blue				'4'
#define Shell_Color_Magenta				'5'
#define Shell_Color_Cyan				'6'
#define Shell_Color_White				'7'
#define Shell_WriteESC()				(Shell_WriteChar(Shell_ESC),Shell_WriteChar('['))		// Shell Ð´Èë×ªÒå×Ö·û
#define Shell_WriteFG(COLOR)				(Shell_WriteChar(Shell_Color_FG),Shell_WriteChar((COLOR)))
#define Shell_WriteSpacer()				(Shell_WriteChar(Shell_Spacer))
#define Shell_WriteBG(COLOR)				(Shell_WriteChar(Shell_Color_BG),Shell_WriteChar((COLOR)))
#define Shell_WriteAttr(n)				(Shell_WriteChar((n)))
#define Shell_WriteESCEnd()				(Shell_WriteChar('m'))

void Shell_SetTxtAttr(int8,int8,int8);
#endif		// #if SHELL_CFG_COLOR_EN == 1

// Shell_State definations
#define Shell_State_Running				(1 << 0)
#define Shell_State_Busy				(1 << 1)
#define Shell_State_Paused				(1 << 4)
#define Shell_State_Stopped				(1 << 7)

#define SHELLERR_CMD_NOT_FOUND				1
#define SHELLERR_SUCCESS				0

void Shell_Init(void);
void Shell_Fini(void);
void Shell_Run(void);
void Shell_Login(void);

void Shell_WriteLine(uint8*);

#if FLASH_ACCESS == 0
void Shell_WriteStr_P(STR_DEF);
void Shell_WriteLine_P(STR_DEF);
#endif

void Shell_ReadLine(uint8*,uint8);
#define Shell_Echo		(1 << 8)
#define Shell_NoEcho	0
#define Shell_EchoChar	0xFF
#define Shell_AutoEnter	(1 << 9)

#if SHELL_CMD_ECHO == 1
#define Shell_RL_Attr	(Shell_Echo | Shell_AutoEnter)
#else
#define Shell_RL_Attr	Shell_NoEcho
#endif

#if SHELL_COLOR_EN == 1
void Shell_SetTextAttr(int8 FGColor,int8 BGColor,int8 Font);
//uint8 Shell_color()
#endif		// #if SHELL_CFG_COLOR_EN == 1

// Internel command functions
uint8 Shell_Clear(uint8**,uint8);
uint8 Shell_Exit(uint8**,uint8);
uint8 Shell_Help(uint8**,uint8);
uint8 Shell_Ver(uint8**,uint8);

#if SHELL_MULTIINTERFACE == 1
extern void (*Shell_WriteChar)(uint8);
extern int16 (*Shell_ReadChar)(uint8);
extern void (*Shell_Interface_Init)(void);
extern void (*Shell_Interface_Fini)(void);

extern void (*Shell_WriteStr)(uint8*);
#endif
