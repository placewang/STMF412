/***************************************************************************************
Product: S-Shell
Product Info: A Shell interface for embeded systems,talorable,configurable,user-definable
Current Version: 1.0beta
License: see license.txt
CopyRight by SimonQian(Ç®Ïþ³¿)

File: shell_cmd.h
File Info: command type defination file for S-Shell
File Data: 2006.7.2
****************************************************************************************
Change Log:

***************************************************************************************/
typedef struct _cmd
{
	STR_DEF cmd_str;		// command string
	uint8 (*cmd_func)(uint8 **argv,uint8 argc);
	STR_DEF cmd_info;
}CMD,*PCMD;

// extern userdefined command functions
#define SHELL_CMD_HIDEIDX	30
/*const*/static CMD Shell_CMDs[SHELL_CMD_HIDEIDX] = {
	{STR_TYP("help"),Shell_Help,STR_TYP("Shell Help.")},
	{STR_TYP("cls"),Shell_Clear,STR_TYP("Clear the Shell Output.")},
	{STR_TYP("quit"),Shell_Exit,STR_TYP("Quit S-Shell.")},
	{STR_TYP("version"),Shell_Ver,STR_TYP("Show Shell version.")},
#if SHELL_COLOR_EN == 1
	//{STR_TYP("color"),Shell_color,STR_TYP("Shell Set Attr.")},
#endif
	{STR_TYP(0),(void *)0,STR_TYP(0)},
};

#define COUNTS(x)	(sizeof(x) / sizeof(x[0]))

//#define SHELL_CMD_HIDEIDX	COUNTS(Shell_CMDs)

