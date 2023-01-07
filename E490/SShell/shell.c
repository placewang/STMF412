#include "shell_cfg.h"
#include "shell.h"
#include "shell_cmd.h"


volatile uint8 Shell_State = Shell_State_Stopped;
uint8 Shell_LineBuff[SHELL_LINE_BUFFER_SIZE];
volatile uint8 Shell_Total_cmd = 0;

void shell_rcv_char(char ch);
int Shell_GetLine(char *buff, int size);
void Shell_Set_Attr(int attr);
void shell_command_init(void);
int shell_Return(void);
int shell_BackSpace(void);
int shell_find(int mode);

typedef struct {
	unsigned int sidx;	// start
	unsigned int eidx;	// end
}IDX_TYPE;
IDX_TYPE shell_rcv_idx[SEG_NUM];
volatile unsigned int shell_rcv_rptr;
volatile unsigned int shell_rcv_wptr;
volatile unsigned int shell_rcv_attr;
volatile unsigned int shell_find_idx;
volatile unsigned int shell_time;

int myprintf(const char *format, ...);

#if SHELL_LOGIN_EN == 1
volatile unsigned char Shell_Login_Flag;
#endif

#if SHELL_MULTIINTERFACE == 1
void (*Shell_WriteChar)(uint8);
int16 (*Shell_ReadChar)(uint8);
void (*Shell_Interface_Init)(void);
void (*Shell_Interface_Fini)(void);

void (*Shell_WriteStr)(uint8*);
#endif

static uint8 Shell_GetCmdCount(void);

void Shell_WriteStr_Int(uint8 *str)
{
	while(*str)
		Shell_WriteChar(*str++);
}

void Shell_WriteLine(uint8 *str)
{
	Shell_WriteStr(str);
	Shell_WriteStr_P(Shell_LineStr);
}

#if FLASH_ACCESS == 0
void Shell_WriteStr_P(STR_DEF str)
{
	uint8 ch;

	while(Shell_GetFlashChar(&ch,str++))
	{
		Shell_WriteChar(ch);
	}
}

void Shell_WriteLine_P(STR_DEF str)
{
	Shell_WriteStr_P(str);
	Shell_WriteStr_P(Shell_LineStr);
}
#endif

void Shell_ReadLine(uint8 *buff, uint8 size)
{
	while(1) {
		if(Shell_GetLine((char*)buff, size) > 0) {
			break;
		}
	}
}

int Shell_ReadLine_Poll(uint8 *buff, uint8 size)
{
	return Shell_GetLine((char*)buff, size);
}

void ShellBuff_init()
{
	int i;

	shell_rcv_rptr = 0;
	shell_rcv_wptr = 0;
	shell_find_idx = 0;
	for(i = 0; i < SEG_NUM; i ++) {
		shell_rcv_idx[i].sidx = 0;
		shell_rcv_idx[i].eidx = 0;
	}
}

void Shell_Init(void)
{
#if SHELL_MULTIINTERFACE == 1
	if(!Shell_WriteStr)
		Shell_WriteStr = Shell_WriteStr_Int;

	if(Shell_Interface_Init)
#endif
		Shell_Interface_Init();

#if SHELL_COLOR_EN == 1
	Shell_SetTxtAttr(-1,-1,Shell_Attrs_None);
	Shell_SetTxtAttr(Shell_DefFGColor,Shell_DefBGColor,Shell_DefAttr);
#endif		// #if SHELL_COLOR_EN == 1

#if 0
	Shell_Clear((uint8**)0,0);
	Shell_Clear((uint8**)0,0);
	Shell_Clear((uint8**)0,0);
#endif
	Shell_Total_cmd = Shell_GetCmdCount();
	myprintf("total %d\n\r", Shell_Total_cmd);

	Shell_WriteLine_P(Shell_StrInfo);

	shell_command_init();

	ShellBuff_init();

	shell_time = 0;

	Shell_Set_Attr(Shell_RL_Attr);
#if 1
#if SHELL_LOGIN_EN == 1
	Shell_Login_Flag = 0;
	Shell_Set_Attr(Shell_NoEcho | Shell_AutoEnter);
#endif
#else
	Shell_Login_Flag = 1;
#endif

	Shell_Total_cmd = Shell_GetCmdCount();
	myprintf("total %d\n\r", Shell_Total_cmd);

	Shell_WriteStr_P(Shell_Str);
}

void Shell_Fini(void)
{
	Shell_State = Shell_State_Stopped;

#if SHELL_MULTIINTERFACE == 1
	if(Shell_Interface_Fini)
#endif
		Shell_Interface_Fini();
}

static uint8 Shell_ParsePara(uint8 **para,uint8 *cmd)
{
	uint8 c = 0,foundpara = 0,paranum = 0;

	do {
		if(cmd[c] == '\0') {
			if(foundpara) {
				if(para != (uint8**)0)
					para[paranum] = &cmd[foundpara - 1];
				paranum++;
			}
			break;
		}
		else if(cmd[c] == ' ') {
			if(foundpara) {
				if(para != (uint8**)0) {
					cmd[c] = '\0';
					para[paranum] = &cmd[foundpara - 1];
				}
				paranum++;
				foundpara = 0;
			}
		}
		else {
			if(!foundpara)
				foundpara = c + 1;
		}
		c++;
	} while(1);

	return paranum;
}

#if HAS_MALLOC == 0
uint8 *para[SHELL_PARA_NUM];
#endif

static uint8 Shell_DoCMD(uint8 *CMD)
{
	int8 i = -1;
#if HAS_MALLOC == 1
	uint8 **para = (uint8**)0, para_num = 0;
#else
	uint8 para_num = 0;
#endif		// #if HAS_MALLOC == 1

#if HAS_MALLOC == 1
	para_num = Shell_ParsePara((uint8**)0,CMD);
	para = (uint8**)malloc(para_num * sizeof(uint8*));
#endif		// #if HAS_MALLOC == 1
	para_num = Shell_ParsePara(para,CMD);

	if(para_num == 0)
		return SHELLERR_CMD_NOT_FOUND;

#if SHELL_LOGIN_EN == 1
	if(Shell_Login_Flag == 0) {
		if(!strcmp_P((char*)para[0], (char*)STR_T(USR_PASSWORD))) {
			Shell_Login_Flag = 1;
			Shell_Set_Attr(Shell_RL_Attr);
			ShellBuff_init();
			return SHELLERR_SUCCESS;
		}
		return SHELLERR_CMD_NOT_FOUND;
	}
#endif
	while((Shell_CMDs[++i].cmd_str != 0) && (i < Shell_Total_cmd /*SHELL_CMD_HIDEIDX*/)) {
		if(!strcmp_P((char*)para[0], (char*)Shell_CMDs[i].cmd_str)) {
			return Shell_CMDs[i].cmd_func(para, para_num);
		}
	}

	Shell_WriteStr_P(STR_T("COMMAND:\""));
	Shell_WriteStr(para[0]);
	Shell_WriteLine_P(STR_T("\" not found"));

#if HAS_MALLOC == 1
	free(para);
	para = (uint8**)0;
#endif		// #if HAS_MALLOC == 1

	return SHELLERR_CMD_NOT_FOUND;
}

static uint8 Shell_GetCmdCount(void)
{
	int8 i;

	for(i = 0; i < SHELL_CMD_HIDEIDX; i ++) {
		if(Shell_CMDs[i].cmd_str == 0) break;
	}

	return i;
}

uint8 Shell_AddCMD(char *cmd, char *help, uint8 (*func)(uint8** ,uint8 ))
{
	int8 i = 0;

	if(Shell_Total_cmd < SHELL_CMD_HIDEIDX) {
		while((Shell_CMDs[i].cmd_str != 0) && (i <= Shell_Total_cmd /*SHELL_CMD_HIDEIDX*/)) {
			if(!strcmp_P(cmd, (char*)Shell_CMDs[i].cmd_str)) {
				return SHELLERR_SUCCESS;
			}
			i ++;
		}
		
		Shell_CMDs[i+1].cmd_str = Shell_CMDs[i].cmd_str;
		Shell_CMDs[i+1].cmd_info = Shell_CMDs[i].cmd_info;
		Shell_CMDs[i+1].cmd_func = Shell_CMDs[i].cmd_func;

		Shell_CMDs[i].cmd_str = (STR_DEF)cmd;
		Shell_CMDs[i].cmd_info = (STR_DEF)help;
		Shell_CMDs[i].cmd_func = func;
		Shell_Total_cmd ++;
	}

	return SHELLERR_SUCCESS;
}

void shell_command_init(void);
void Shell_Run(void)
{
	Shell_State = Shell_State_Running;

	while(Shell_State == Shell_State_Running)
	{
		Shell_WriteStr_P(Shell_Str);
		Shell_ReadLine(Shell_LineBuff, SHELL_LINE_BUFFER_SIZE);

		//myprintf("%s\n\r", Shell_LineBuff);
		if(strlen((const char*)Shell_LineBuff) != 0)
			Shell_DoCMD(Shell_LineBuff);
	}

	Shell_Fini();
}

void Shell_Poll(void)
{
	if(Shell_ReadLine_Poll(Shell_LineBuff, SHELL_LINE_BUFFER_SIZE)) {

		//myprintf("%s\n\r", Shell_LineBuff);
		if(strlen((const char*)Shell_LineBuff) != 0)
			Shell_DoCMD(Shell_LineBuff);
		Shell_WriteStr_P(Shell_Str);
	}
}

uint8 Shell_Exit(uint8** argv,uint8 argc)
{
	Shell_State = Shell_State_Stopped;
	Shell_WriteStr_P(Shell_ByeStr);
#if SHELL_LOGIN_EN == 1
	Shell_Login_Flag = 0;
	Shell_Set_Attr(Shell_NoEcho | Shell_AutoEnter);
#endif

	return SHELLERR_SUCCESS;
}

uint8 Shell_Clear(uint8** argv,uint8 argc)
{
	uint8 i = Shell_Height;

	while(i--)
	{
		Shell_WriteLine_P(STR_T(""));
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Help(uint8 **argv,uint8 argc)
{
	int16 i = -1;

	Shell_WriteLine_P(Shell_StrInfo);
	Shell_WriteLine_P(STR_T("commands:"));
	while((Shell_CMDs[++i].cmd_str != 0) && (i <= Shell_Total_cmd /*SHELL_CMD_HIDEIDX*/)) {
		Shell_WriteChar('\t');
		Shell_WriteStr_P(Shell_CMDs[i].cmd_str);
		Shell_WriteStr_P(STR_T(" -- "));
		Shell_WriteLine_P(Shell_CMDs[i].cmd_info);
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Ver(uint8** argv,uint8 argc)
{
	Shell_WriteLine_P(Shell_StrVer);

	return SHELLERR_SUCCESS;
}

#if SHELL_COLOR_EN == 1
void Shell_SetTxtAttr(int8 FGColor,int8 BGColor,int8 Attrs)
{
	Shell_WriteESC();
	if(FGColor != -1)
	{
		Shell_WriteFG(FGColor);
	}
	if(BGColor != -1)
	{
		Shell_WriteSpacer();
		Shell_WriteBG(BGColor);
	}
	if(Attrs == Shell_Attrs_None)
	{
		Shell_WriteSpacer();
		Shell_WriteAttr(Shell_Attr_None);
	}
	else
	{
		if(Attrs & Shell_Attrs_HighLight)
		{
			Shell_WriteSpacer();
			Shell_WriteAttr(Shell_Attr_HighLight);
		}
		if(Attrs & Shell_Attrs_UnderLine)
		{
			Shell_WriteSpacer();
			Shell_WriteAttr(Shell_Attr_UnderLine);
		}
		if(Attrs & Shell_Attrs_Blink)
		{
			Shell_WriteSpacer();
			Shell_WriteAttr(Shell_Attr_Blink);
		}
		if(Attrs & Shell_Attrs_Inverse)
		{
			Shell_WriteSpacer();
			Shell_WriteAttr(Shell_Attr_Inverse);
		}
		if(Attrs & Shell_Attrs_Disappear)
		{
			Shell_WriteSpacer();
			Shell_WriteAttr(Shell_Attr_Disappear);
		}
	}

	Shell_WriteESCEnd();
}

/*			Usage
	Press:	'Esc'(\e) and '[' as ESC
			'3' + COLOR as FGColor(COLOR is from '0' to '7')
			'4' + COLOR ad BGColor
			'0' or '1' as Font
			';' as spacer
			'm' as end
	Exg:'Esc'+'['+'3'+'1'+';'+'4'+'0'+';'+'1'+'m'("\e[31;40;1m")
*/
/*
uint8 Shell_Color(uint8** argv,uint8 argc)
{
	uint8 ch = 0;

	Shell_WriteLine_P(STR_T("God bless U know how 2 use...(\'q\' to quit),Input:"));
	while((ch = Shell_ReadChar()) != 'q')
	{
		Shell_WriteChar(ch);
	}

	Shell_Clear();
	Shell_Clear();

	return SHELLERR_SUCCESS;
}
*/
#endif		// #if SHELL_COLOR_EN == 1

// by xhl 2010/12/10
volatile char Shell_Buff[SHELL_LINE_BUFFER_SIZE];
void Shell_Set_Attr(int attr)
{
	shell_rcv_attr = attr;
}

int Shell_GetLine(char *buff, int size)
{
	int rptr;
	int i;

	if(shell_rcv_rptr == shell_rcv_wptr) {
		return 0;
	}

	rptr = shell_rcv_idx[shell_rcv_rptr].sidx;
	i = 0;
	do {
		buff[i] = Shell_Buff[rptr];
		if(rptr++ >= SHELL_LINE_BUFFER_SIZE)
			rptr -= SHELL_LINE_BUFFER_SIZE;
		if(i++ >= size) break;
	} while(rptr != shell_rcv_idx[shell_rcv_rptr].eidx);
	rptr = shell_rcv_rptr + 1;
	if(rptr >= SEG_NUM) {
		rptr -= SEG_NUM;
	}
	shell_rcv_rptr = rptr;
	// by xhl 2010/12/16
	shell_find_idx = shell_rcv_rptr;

	return i;
}

int shell_write_to_Buff(char ch);
volatile int ctrl = 0;
void shell_rcv_char(char ch)
{
	//int idx;

	switch(ch)
	{
		case '\n':		// NewLine
			break;
		case '\r':		// Return
			shell_Return();
			break;
		case '\b':		// Backspace
			shell_BackSpace();
			break;
		default:
			if((ch < ' ') || (ch > 127)) {
				break;
			}
		#if 1
			if(ctrl && shell_time < 2) {
				ctrl = 0;
				
				switch(ch) {
				case 'A': // UP KEY
					shell_find(1);
					goto quit;
				case 'B': // DOWN KEY
					shell_find(0);
					goto quit;
				default:
					if(shell_rcv_attr & Shell_Echo)
					{
						if(shell_rcv_attr & Shell_EchoChar)
							Shell_WriteChar((uint8)shell_rcv_attr & Shell_EchoChar);
						else
							Shell_WriteChar('[');
					}
					shell_write_to_Buff('[');
					break;
				}
			}

			if(ch == '[') {
				ctrl = 1;
				break;
			}
		#endif
			if(shell_rcv_attr & Shell_Echo)
			{
				if(shell_rcv_attr & Shell_EchoChar)
					Shell_WriteChar((uint8)shell_rcv_attr & Shell_EchoChar);
				else
					Shell_WriteChar(ch);
			}

			shell_write_to_Buff(ch);
			break;
	}
quit:
    return ;
}

// by xhl 2010/12/16
int shell_write_to_Buff(char ch)
{
	int idx;

	idx = shell_rcv_idx[shell_rcv_wptr].eidx;
	Shell_Buff[idx++] = ch;
	if(idx >= SHELL_LINE_BUFFER_SIZE)
		idx -= SHELL_LINE_BUFFER_SIZE;

	shell_rcv_idx[shell_rcv_wptr].eidx = idx;

	return idx;
}

int shell_BackSpace()
{
	if(shell_rcv_idx[shell_rcv_wptr].sidx != shell_rcv_idx[shell_rcv_wptr].eidx)
	{
		if(shell_rcv_idx[shell_rcv_wptr].eidx > 0)
			shell_rcv_idx[shell_rcv_wptr].eidx --;
		else
			shell_rcv_idx[shell_rcv_wptr].eidx = SHELL_LINE_BUFFER_SIZE - 1;
		Shell_Buff[shell_rcv_idx[shell_rcv_wptr].eidx] = '\0';
		if(shell_rcv_attr & Shell_Echo)
			Shell_WriteStr_P(STR_T("\b \b"));
	}

	return 0;
}

int shell_Return()
{
	int idx;
	int wptr;

	if(shell_rcv_attr & Shell_AutoEnter);
	Shell_WriteStr_P(Shell_LineStr);

	idx = shell_write_to_Buff('\0');

	wptr = shell_rcv_wptr + 1;
	if(wptr >= SEG_NUM) {
		wptr -= SEG_NUM;
	}
	if(wptr == shell_rcv_rptr) {
		// is full ???
		shell_rcv_idx[shell_rcv_wptr].eidx = shell_rcv_idx[shell_rcv_wptr].sidx;
	}
	else {
		shell_rcv_wptr = wptr;
		shell_find_idx = wptr;
		shell_rcv_idx[wptr].sidx = idx;
		shell_rcv_idx[wptr].eidx = idx;
	}

	return 0;
}

int shell_find(int mode)
{
	int idx = shell_find_idx;
	int cnt;

find_next:
	if(mode == 0) {
		idx ++;
		if(idx >= SEG_NUM)
			idx -= SEG_NUM;

		if(idx == shell_rcv_wptr)
			return 0;
	}
	else if(mode == 1) {
		if(idx > 0) {
			idx --;
		}
		else {
			idx = SEG_NUM - 1;
		}
		if(idx == shell_rcv_wptr)
			return 0;
	}

	cnt = shell_rcv_idx[idx].eidx - shell_rcv_idx[idx].sidx;
	if(cnt < 0) cnt += SHELL_LINE_BUFFER_SIZE;
	if(cnt <= 1) goto find_next;

	shell_find_idx = idx;

	cnt = shell_rcv_idx[shell_rcv_wptr].eidx - shell_rcv_idx[shell_rcv_wptr].sidx;
	if(cnt < 0) cnt += SHELL_LINE_BUFFER_SIZE;
	while(cnt--) {
		Shell_WriteStr_P(STR_T("\b \b"));
	}
	//Shell_WriteStr_P(Shell_Str);
	shell_rcv_idx[shell_rcv_wptr].eidx = shell_rcv_idx[shell_rcv_wptr].sidx;
	// copy it
	idx = shell_rcv_idx[shell_find_idx].sidx;
	do {
		char ch = Shell_Buff[idx];
		if(ch == 0) break;
		shell_write_to_Buff(ch);
		Shell_WriteChar(ch);
		idx ++;
		if(idx >= SHELL_LINE_BUFFER_SIZE)
			idx -= SHELL_LINE_BUFFER_SIZE;
	}while(idx != shell_rcv_idx[shell_find_idx].eidx);

	//myprintf("find quit %d\r\n", idx);
	return 0;
}

void shell_hook_time()
{
	if(ctrl) {
		shell_time ++;
		if(shell_time > 2) {
			ctrl = 0;
			if(shell_rcv_attr & Shell_Echo)
			{
				if(shell_rcv_attr & Shell_EchoChar)
					Shell_WriteChar((uint8)shell_rcv_attr & Shell_EchoChar);
				else
					Shell_WriteChar('[');
			}
			shell_write_to_Buff('[');
		}
	}
	else shell_time = 0;
}

