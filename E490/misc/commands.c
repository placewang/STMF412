#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f2xx.h"
//#include "stm32f10x_lib.h"
#include "config.h"
#include "encoder.h"
//#include "step.h"

#define SHELLERR_CMD_NOT_FOUND			1
#define SHELLERR_SUCCESS			0

#if 0
u8 CMD_Parse(u8** argv,u8 argc)
{
	return 0;
}
#endif

void wait_us(int); 
extern int myprintf(const char *format, ...);
int myatoi(uint8 *s);
u8 myStrCmp(char *s0, char *s1);
#define strcmp_P(s0,s1)			myStrCmp(s0,s1)

void TimeStart(void);
unsigned long TimeGet(void);

void arch_LED_On(void);
void arch_LED_Off(void);
void arch_Power_On(void);
void arch_Power_Off(void);
unsigned int Get_ErrorStatus(void);
unsigned int arch_Get_DirStatus(void);
void arch_StepMotor_Enable(void);
void arch_StepMotor_Disable(void);
unsigned int arch_StepMotor_Zero(unsigned int stepno);
void StepMotor_exec(unsigned int stepno, short pos, int mode,unsigned short otherarg,unsigned char ws);
int Board_Check(void);
void arch_StepMotor_Half(int stepno, int onoff);
void arch_StepMotor_Dir(int stepno, int onoff);
void arch_StepMotor_Pulse(int stepno, int onoff);
void Exec_EMF(unsigned int emfno, unsigned int onoff,unsigned char istest,unsigned short times_ms);
void ReBoot(void);
void GoApp(void);
void Exec_Jacquard(unsigned int jqdno, unsigned int sts);
void StepMotor_Reset(int stepno,unsigned short otherarg);
void StepMotor_exec(unsigned int stepno, short pos, int mode,unsigned short otherarg,unsigned char workspeed);
short StepMotor_Get_Position(unsigned int stepno);
int EE_Read(int addr, u16 *Data, int len);
int EE_Write(int addr, u16 *Data, int len);
int get_cpld_ver(char * data);

volatile int enable = 0;
void hook_test(int flag)
{
}

uint8 Shell_LED(uint8** argv,uint8 argc)
{
	if(argc > 1) {
		if(!strcmp_P((char*)argv[1], "on")) {
			arch_LED_On();
		}
		else if(!strcmp_P((char*)argv[1], "off")) {
			arch_LED_Off();
		}
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Power(uint8** argv,uint8 argc)
{
	if(argc > 1) {
		if(!strcmp_P((char*)argv[1], "on")) {
			arch_Power_On();
		}
		else if(!strcmp_P((char*)argv[1], "off")) {
			arch_Power_Off();
		}
	}

	return SHELLERR_SUCCESS;
}


uint8 Shell_Step(uint8** argv,uint8 argc)
{
	int i;
	
	if(argc == 2) {
		if(!strcmp_P((char*)argv[1], "on")) {
			arch_StepMotor_Enable();
			enable = 3;
		}
		else if(!strcmp_P((char*)argv[1], "off")) {
			arch_StepMotor_Disable();
			enable = 0;
		}
	}
	else if(argc > 2) {
		int no, pos;
		if(!strcmp_P((char*)argv[1], "all")) {
			if(!strcmp_P((char*)argv[2], "r")) {
				//StepMotor_All_Reset();
				for(i = 0; i < 8; i++)
					StepMotor_Reset(i,0);
			}
			else {
				pos = myatoi(argv[2]);
				for(i = 0; i < 8; i++)
					StepMotor_exec(i, pos, 1,0,0);
			}
		}
		else {
			no = myatoi(argv[1]);
			if(!strcmp_P((char*)argv[2], "r")) {
				StepMotor_Reset(no,0);
			}
			else {
				pos = myatoi(argv[2]);
				StepMotor_exec(no, pos, 1,0,0);
			}
		}
	}

	return SHELLERR_SUCCESS;
}

extern unsigned int emf_on_keep_time_setup;
extern unsigned int emf_off_keep_time_setup;
uint8 Shell_EMF(uint8** argv,uint8 argc)
{
	extern unsigned long emf_work_mode;
	void arch_YARNEMF_Setup(int yno, int onoff,int check);
	void arch_YARNEMF_Clear(int yno);
	//emf_work_mode = 0xFFFFFFFF;
	if(argc > 2) {
		int no;
		no = myatoi(argv[1]);

		if(!strcmp_P((char*)argv[2], "on")) {
			Exec_EMF(no, 0,0,0);
			//arch_YARNEMF_Setup(no, 0);
			//wait_us(3000); 
			//arch_YARNEMF_Clear(no);
		}
		else if(!strcmp_P((char*)argv[2], "off")) {
			Exec_EMF(no, 1,0,0);
			//arch_YARNEMF_Setup(no, 1);
			//wait_us(3000); 
			//arch_YARNEMF_Clear(no);
		}
		else if(!strcmp_P((char*)argv[2], "auto")) {
			int i;
			int repeat = myatoi(argv[3]);
			int delay = myatoi(argv[4]) * 1000;
			if(argc > 5) {
				int keep_time = myatoi(argv[5]);
				emf_on_keep_time_setup = keep_time;
				emf_off_keep_time_setup = keep_time;
			}
			for(i = 0; i < repeat; i ++) {
				Exec_EMF(no, 1,0,0);
				wait_us(delay);
				Exec_EMF(no, 0,0,0);
				wait_us(delay);
			}
		}
	}

	return SHELLERR_SUCCESS;
}


uint8 Shell_GetStatus(uint8** argv,uint8 argc)
{
	int status = 0;
	int i;
	if(argc > 1) {
		if(!strcmp_P((char*)argv[1], "err")) {
			myprintf("err status = 0x%x \n\r", Get_ErrorStatus());
		}
		else if(!strcmp_P((char*)argv[1], "zero")) {
			for(i = 0; i < 10; i ++) {
				if(arch_StepMotor_Zero(i))
					status |= 0x1 << i;
			}
			myprintf("zero status = 0x%x \n\r", status);
		}
		else if(!strcmp_P((char*)argv[1], "board")) {
			myprintf("board status = %d \n\r", Board_Check());
		}
		else if(!strcmp_P((char*)argv[1], "dir")) {
			myprintf("board status = %d \n\r", arch_Get_DirStatus());
		}
		else if(!strcmp_P((char*)argv[1], "cpld")) {
			char data[4];
			get_cpld_ver(data);
			myprintf("get cpld ver\n\r");
			myprintf("cpld ver = %x%x-V%x%x \n\r", 
			data[1],data[0],data[3],data[2]);
		}
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Jacquard(uint8** argv,uint8 argc)
{
	if(argc >= 2) {
		int no, sts;
		no = myatoi(argv[1]);
		sts = myatoi(argv[2]);
		Exec_Jacquard(no, sts);
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Reboot(uint8** argv,uint8 argc)
{
#if 0
	if(argc >= 1) {
		if(!strcmp_P((char*)argv[1], "-a")) {
			ReBoot();
		}
	}
	GoApp();
#endif
	ReBoot();

	return SHELLERR_SUCCESS;
}

#if 0
uint8 Shell_EETest(uint8** argv,uint8 argc)
{
	int i;
	u32 wData;
	u32 rData;
	u16 ret;

	myprintf("EEPROM Test \n\r");

	for(i = 0; i < 32; i += 2) {
		wData = i + 0xAA55A055;
		rData = 0;
		if((ret = EE_Write(i, (u16*)&wData, 2)) != 0) {
			myprintf("%d Write Fail %x\r\n", i, ret);
			break;
		}
		if((ret = EE_Read(i, (u16*)&rData, 2)) != 0) {
			myprintf("%d Read Fail %x\r\n", i, ret);
			break;
		}
		myprintf("%x %x\r\n", rData, wData);
		if(rData != wData) {
			myprintf("%d Check Fail\r\n", i);
			break;
		}
	}

	myprintf("\r\n ---- End ----\r\n");

	return SHELLERR_SUCCESS;
}

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define StartAddr  ((u32)0x08028000)
#define EndAddr    ((u32)0x08030000)
#define FLASH_PAGE_SIZE	((u32)0x00000800)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static vu32 TimingDelay = 0;  
u32 EraseCounter = 0x00, Address = 0x00;
u32 Data;
vu32 NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;
ErrorStatus HSEStartUpStatus;

uint8 Shell_FlashTest(uint8** argv,uint8 argc)
{
	u16 Data;
	FLASHStatus = FLASH_COMPLETE;
	MemoryProgramStatus = PASSED;
	Data = 0x1234;

	myprintf("Flash Size %d KB\r\n", (*(volatile u16*)0x1FFFF7E0));
	myprintf("SRAM Size %d KB\r\n", (*(volatile u16*)0x1FFFF7E2));

	myprintf("FLASH Test \n\r");

	/* Define the number of page to be erased */
	NbrOfPage = (EndAddr - StartAddr) / FLASH_PAGE_SIZE;

	myprintf("Erase %ld Page\r\n", NbrOfPage);
	/* Clear All pending flags */
	//FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

	/* Erase the FLASH pages */
	for(EraseCounter = 0; EraseCounter < NbrOfPage; EraseCounter++)
	{
		myprintf(".");
		FLASHStatus = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * EraseCounter));
		if(FLASHStatus != FLASH_COMPLETE) {
			myprintf("\r\nErase %ld Page Fail\r\n", EraseCounter);
			break;
		}
	}

	myprintf("\r\nProgram\r\n");
#if 1
	/*  FLASH Word program of data 0x15041979 at addresses defined by StartAddr and EndAddr*/
	Address = StartAddr;

	while(Address < EndAddr)
	{
		myprintf(".");
		FLASHStatus = FLASH_ProgramWord(Address, Data);
		Address = Address + 4;
		if(FLASHStatus != FLASH_COMPLETE) {
			myprintf("\r\nProgram %lx Fail\r\n", Address);
			break;
		}
	}

	/* Check the corectness of written data */
	Address = StartAddr;

	while((Address < EndAddr) && (MemoryProgramStatus != FAILED))
	{
		if((*(vu32*) Address) != Data)
		{
			MemoryProgramStatus = FAILED;
		}
		Address += 4;
	}

	if(MemoryProgramStatus == FAILED)
	{
		myprintf("\r\nFlash Program error!\r\n");
	}
	else
	{
		myprintf("\r\nFlash Program OK!\r\n");
	}
#endif

	myprintf("\r\n====  end  ====\r\n");

	return SHELLERR_SUCCESS;
}
#endif

uint8 Shell_Read(uint8** argv,uint8 argc)
{
	volatile u8* addr;
	if(argc >= 2) {
		addr = (u8*)myatoi(argv[1]);
		myprintf("read *(0x%x) = %d[0x%x]\r\n", addr, *(addr), *(addr));
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_Write(uint8** argv,uint8 argc)
{
	volatile u8* addr;
	u16 data;
	if(argc >= 3) {
		addr = (u8*)myatoi(argv[1]);
		data = myatoi(argv[2]);
		myprintf("write *(0x%x) = %d[0x%x]\r\n", addr, data, data);
		*(addr) = data;
	}

	return SHELLERR_SUCCESS;
}

#ifdef ENCODER_SUPPORT

uint8 Shell_SSI(uint8** argv,uint8 argc)
{
	int SSI_Read(int idx,short *IC_flag, int *coder, int *status);
	int i;
	int encoder;
	int preencoder;
	int status;
	int ret;
	unsigned long time;
	short pos = 0;
//	int idx;
	int step;
	int max = 0;


	//StepMotor_Encoder_Save();
	Encoder_Init(8);

	if(argc == 5) {
		int j;
		i = myatoi(argv[1]);
		for(j = 0; j < 100; j ++) {
			ret = SSI_Read(i,NULL,&encoder, &status);
			myprintf("step no[%d] cnt[%d] enc[%d]\r\n", i, j, encoder);
		}
		return 0;
	}
	if(argc == 2) {
		i = myatoi(argv[1]);
		preencoder = Encoder_getZero(i);
		for(pos = 0; pos < 700; pos ++) {
			StepMotor_exec(i, pos, 1,0,0);
			wait_us(5000); 
			ret = SSI_Read(i, NULL,&encoder, &status);
			if ((i < 4 && ((i & 0x1) == 0)) ||
			    (i >= 4 && ((i & 0x1) == 1))) {
				step = 4096 + preencoder - encoder;
			}
			else {
				step = 4096 + encoder - preencoder;
			}
			step &= 0xFFF;
			if(step > max && step < 100) {
				max = step;
			}
			myprintf("encoder[%d] step[%d] max[%d]", encoder, step, max);
			myprintf("status[0x%x] \r\n", status);
			preencoder = encoder;

		}
	}
	if(argc == 3) {
		i = myatoi(argv[1]);
		for(step = 0; step < 700; step ++) {
			StepMotor_exec(i, step, 1,0,0);
			wait_us(5000); 
			ret = SSI_Read(i,  NULL,&encoder, &status);
			pos = StepMotor_Get_Position(i);
			pos = (long)pos * 128L / 25L;
			if ((i < 4 && ((i & 0x1) == 0)) ||
			    (i >= 4 && ((i & 0x1) == 1))) {
				myprintf("1pos[%d] ", pos);
				pos = 4096 + Encoder_getZero(i) - pos;
			}
			else {
				myprintf("2pos[%d] ", pos);
				pos = Encoder_getZero(i) + pos;
			}
			pos &= 0xFFF;
			myprintf("pos[%d] [%d]", pos, abs(encoder - pos));
			myprintf("status[0x%x] \r\n", status);
		}
	}
#if 1
	//printf("SSI TEST\n");
	for(i = 0; i < 8; i ++) {
		myprintf("SSI[%d] ", i);

		TimeStart();
		ret = SSI_Read(i,  NULL,&encoder, &status);
		time = TimeGet();
		myprintf("%ldus ", time);
		if(ret >= 0) {
		#if 0
			if ((i < 4 && ((i & 0x1) == 0)) ||
			    (i >= 4 && ((i & 0x1) == 1))) {
				encoder = 4096 + encoder_zero[i] - encoder + 2;
			}
			else {
				encoder = 4096 + encoder - encoder_zero[i] + 2;
			}
			encoder &= 0xFFF;
			myprintf("pos[%d] ", ((long)encoder * 25L) >> 7);
			myprintf("status[0x%x] \r\n", status);
		#endif
		#if 1
			pos = StepMotor_Get_Position(i);
			//StepMotor_SSI_ckeck_encoder(i, pos, encoder);
			myprintf("encoder[%d]<=>[%d] ", encoder, Encoder_getZero(i));
		#if 0
			pos = (long)pos * 128L / 25L;
			if ((i < 4 && ((i & 0x1) == 1)) ||
			    (i >= 4 && ((i & 0x1) == 0))) {
				myprintf("1pos[%d] ", pos);
				pos = 4096 + encoder_zero[i] - pos;
			}
			else {
				myprintf("2pos[%d] ", pos);
				pos = encoder_zero[i] + pos;
			}
			pos &= 0xFFF;
		#else
			pos = Encoder_step2coder(i, pos);
		#endif
			if(abs(encoder - pos) > (4096 >> 1)) {
				myprintf("pos[%d] [%d]", pos, 4096 - abs(encoder - pos));
			}
			else 
			myprintf("pos[%d] [%d]", pos, abs(encoder - pos));
			myprintf("status[0x%x] \r\n", status);
		#endif
		#if 0
			myprintf("[%d],[%d],[%d]\r\n",
				(((long)encoder_zero[i] * 25L) >> 7),
				(((long)encoder * 25L) >> 7),
				StepMotor_Get_Position(i));
		#endif
		}
		else {
			switch(ret) {
			case -100:
				myprintf("Dis Connected\r\n");
				break;
			case -1:
				myprintf("Cordic & Line\r\n");
				break;
			case -2:
				myprintf("Magnet waring\r\n");
				break;
			case -3:
				myprintf("Parity fail\r\n");
				break;
			default:
				myprintf("ret [%d] ", ret);
				break;
			}
		}
	}
#endif

	return SHELLERR_SUCCESS;
}
#endif

uint8 Shell_Setup(uint8** argv,uint8 argc)
{
	void printf_enable(void);
	unsigned char i;
	for(i = 1; i < argc; i ++) {
		if(!strcmp_P((char*)argv[i], "-d")) {
			printf_enable();
		}
	}

	return SHELLERR_SUCCESS;
}

void Needle_isr(unsigned int Needle_dat, unsigned int jqd_mask);
uint8 Shell_Needle(uint8** argv,uint8 argc)
{
	if(argc > 2) {
		unsigned int dat = myatoi(argv[1]);
		unsigned int mask = myatoi(argv[2]);
		
		Needle_isr(dat, mask);
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_CAN_error_data_get(uint8** argv,uint8 argc)
{
	char i;
	extern unsigned int CAN_error_cnt_err[];
	for(i=0;i<5;i++)
	{
		myprintf("\r\nCan_error_data[%d]=[%d]\r\n",i,CAN_error_cnt_err[i]);
	}
}

int alert_push(int alert_code, int alert_arg);
uint8 Shell_DebugAlert(uint8** argv,uint8 argc)
{
	int alert_code = 0;
       	int alert_arg = 0;
	if(argc > 1) {
		alert_code = myatoi(argv[1]);
		if(argc > 2)
			alert_arg = myatoi(argv[2]);
		alert_push(alert_code, alert_arg);
	}

	return SHELLERR_SUCCESS;
}

uint8 Shell_AddCMD(char *cmd, char *help, uint8 (*func)(uint8** ,uint8 ));
void shell_command_init(void)
{
	Shell_AddCMD("setup",  "-d: Enable Debug", &Shell_Setup);
	Shell_AddCMD("reboot",  "ReBoot", &Shell_Reboot);
	Shell_AddCMD("get",  "Get Input Status[err | zero | board | dir | cpld]", &Shell_GetStatus);
	//Shell_AddCMD("led",  "Run Led control [on | off].", &Shell_LED);
	Shell_AddCMD("pwr",  "Get Input Status[on | off]", &Shell_Power);
	Shell_AddCMD("step", "Step Motor [on | off | {(all | no.) (r | pos)}]", &Shell_Step);
	Shell_AddCMD("emf",  "EMF no. {on | off | auto}]", &Shell_EMF);
	Shell_AddCMD("jqd",  "Jacquard no. {on | off}]", &Shell_Jacquard);
	//Shell_AddCMD("eeprom",  "EEPROM Test", &Shell_EETest);
	//Shell_AddCMD("flash",  "FLASH Test", &Shell_FlashTest);
	Shell_AddCMD("read",  "Read", &Shell_Read);
	Shell_AddCMD("write",  "Write", &Shell_Write);
#ifdef ENCODER_SUPPORT
	Shell_AddCMD("SSI",  "SSI encoder test", &Shell_SSI);
#endif
	Shell_AddCMD("Alert",  "Alert", &Shell_DebugAlert);
	Shell_AddCMD("ndlisr",  "Needle Isr", &Shell_Needle);
	Shell_AddCMD("Can",  "Can Error data", &Shell_CAN_error_data_get);
	
}

