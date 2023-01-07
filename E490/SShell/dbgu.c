/**
  *****************************************************************************
  * @file		dbgu.c
  * @author	ZhuQW
  * @version	V1.0.0
  * @date    	2015-10-01
  * @brief   
  *          
  *****************************************************************************
  * @note
  *****************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "dbgu.h"
#include "platform_config.h"


/* Private typedef -----------------------------------------------------------*/

typedef enum {
	DBGU_MODE_KEY,
	DBGU_MODE_CMD,
}DBGU_MODE_t;

typedef enum {
	DBGU_LOGIN_NULL = 0,
	DBGU_LOGIN_VERIFY,	
	DBGU_LOGIN_USER,
	DBGU_LOGIN_ROOT,
}DBGU_LOGIN_t;

#define INPUT_BIT_SIZE			4


#define DBGU_MOTOR_DEBUG		0


/* Private define ------------------------------------------------------------*/

#define DEV_NAME		"hdr"

#define USER_NAME		"user"
#define USER_PASSWORD	"user"

#define SUDO_NAME		"root"
#define SUDO_PASSWORD	"root"

#define DBGU_STRBUF_SIZE			0x3F						/* 指令缓冲最大长度 */

/* Private macro -------------------------------------------------------------*/

#define DBG_GETCHAR(ch)				arch_uart_getc(ch)			/* 读取一个字符 */
#define DBG_PUTCHAR(ch)				USART_Out(ch)			/* 输出一个字符 */
#define DBG_PRINTF(...)				myprintf(__VA_ARGS__)
#define SUDO_PRINTF(...)			myprintf(__VA_ARGS__)

#define DBG_ATOI(a)					atoi(a)

/* Private variables ---------------------------------------------------------*/

static DBGU_LOGIN_t dbgu_login = DBGU_LOGIN_NULL;
static unsigned char dbgu_ch_cnt = 0;
static unsigned char dbgu_str_buf[DBGU_STRBUF_SIZE+1];
static unsigned char dbgu_str_prev[DBGU_STRBUF_SIZE+1];
static unsigned short dbgu_idx, dbgu_val;
static unsigned char dbgu_key_enable = 0;

static uint8_t dbgu_in_check = 0;
static uint8_t dbgu_mask_bit[INPUT_BIT_SIZE];
static uint8_t dbgu_in_min = 0, dbgu_in_max = 16;



#if DBGU_MOTOR_DEBUG
#define DBGU_MOTOR_MAX			16
int motor_bit_mask = 0;
uint8_t motor_idx = 0;
int16_t motor_step_min[DBGU_MOTOR_MAX] = {0, 0, 0, 0, -200, -250, -40, -40, 0,0,0,0,0,0,0,0};
int16_t motor_step_max[DBGU_MOTOR_MAX] = {400, 400, 400, 400, 200, 250, 40, 40, 400,400,400,400,400,400,400,400};
uint8_t motor_delay[DBGU_MOTOR_MAX];
#endif /* DBGU_MOTOR_DEBUG */


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static int dbgu_para_count(char *str)
{
	int cnt = 0;
	char *ptr[6], *buf = str;
	
	while (ptr[cnt] = strtok(buf, " "))/* 提取每个参数 */
	{
		if (++cnt > 4)
			return 0;
		
		buf = NULL;
	}
	DBG_PRINTF("%d ", cnt);
	return cnt;
}

/* Public functions ----------------------------------------------------------*/

void dbgu_encoder_cmd(unsigned char **argv, unsigned char argc)
{
	int findno = 0;
	unsigned int para_d[3];
	int params = 0;
	char **argv_i = argv;
	char *type;
	
	if (argc == 0)//help
	{
		DBG_PRINTF("encoder\t [zero/setzero]\r\n");
		return;
	}
	argc--;
	type = argv_i[0];
	
	if (strcmp(type, "zero") == 0)
	{
		params = 1;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			Encoder_maskZero(para_d[0]);
			DBG_PRINTF("%s(%d) done\r\n", type, para_d[0]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int enable) \r\n",
						type, params);
		}
	}
	else if (strcmp(type, "setzero") == 0)
	{
		params = 2;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			para_d[1] = DBG_ATOI(argv_i[2]);
			Encoder_setZero(para_d[0], para_d[1]);
			DBG_PRINTF("%s(%d, %d)\r\n", type, para_d[0], para_d[1]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int stepno, int encoder)\r\n\t\tstepno encoder[]\r\n",
						type, params);
		}
	}
	else
	{
		findno = 1;
	}

	if (findno)
	{
		DBG_PRINTF("This cmd not find!\r\n");
	}
}

void dbgu_step_cmd(unsigned char **argv, unsigned char argc)
{
	int findno = 0;
	unsigned int para_d[3];
	int params = 0;
	char **argv_i = argv;
	char *type;
	
	if (argc == 0)//help
	{
		DBG_PRINTF("step\t [reset/son/setpos/cfg/run]\r\n");
		return;
	}
	argc--;
	type = argv_i[0];
	
	if (strcmp(type, "reset") == 0)
	{
		params = 1;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			StepMotor_Reset(para_d[0],0);
			DBG_PRINTF("%s(%d) done\r\n", type, para_d[0]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int enable) \r\n",
						type, params);
		}
	}
	else if (strcmp(type, "son") == 0)
	{
		params = 1;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			StepMotor_Reset(para_d[0],0);
			DBG_PRINTF("%s(%d) done\r\n", type, para_d[0]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int enable) \r\n",
						type, params);
		}
	}
	else if (strcmp(type, "setpos") == 0)
	{
		params = 2;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			para_d[1] = DBG_ATOI(argv_i[2]);
			StepMotor_Modfiy_Position(para_d[0], para_d[1], 0);
			DBG_PRINTF("%s(%d, %d)\r\n", type, para_d[0], para_d[1]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int stepno, int pos)\r\n\t\tstepno[0-5] pos[]\r\n",
						type, params);
		}
	}
	else if (strcmp(type, "run") == 0)
	{
		params = 2;
		if (argc == params)
		{
			para_d[0] = DBG_ATOI(argv_i[1]);
			para_d[1] = DBG_ATOI(argv_i[2]);
			StepMotor_exec(para_d[0], para_d[1], 1,0,0);
			DBG_PRINTF("%s(%d, %d)\r\n", type, para_d[0], para_d[1]);
		}
		else
		{
			DBG_PRINTF("%s have %d parameters(int stepno, int pos)\r\n\t\tstepno[0-5] pos[]\r\n",
						type, params);
		}
	}
	else if (strcmp(type, "get") == 0)
	{
		if (argc == 0)//help
		{
			DBG_PRINTF("step get\t [zero/pos/busy]\r\n");
			return;
		}
		argc--;
		type = argv_i[1];
		
		if (strcmp(type, "pos") == 0)
		{
			params = 1;
			if (argc == params)
			{
				unsigned int val;
				para_d[0] = DBG_ATOI(argv_i[2]);
				val = StepMotor_Get_Position(para_d[0]);
				DBG_PRINTF("step %s(%d) = 0x%x\r\n", type, para_d[0], val);
			}
			else
			{
				DBG_PRINTF("%s have %d parameters(unsigned int stepno) \r\n\t\tstepno[0-5]\r\n",
							type, params);
			}
		}
		else
		{
			findno = 1;
		}
	}
/*----------------------------------------------------------------------------*/
	else if (strcmp(type, "cfg") == 0)//cfg
	{
		if (argc == 0)//help
		{
			DBG_PRINTF("step cfg\t [reset_speed/max_speed/max_speed_sk/active]\r\n");
			DBG_PRINTF("\t\t [runmode/dir/rlt/fastmode/rstdelay/param]\r\n");
			return;
		}
		argc--;
		type = argv_i[1];
		if (strcmp(type, "max_speed") == 0)
		{
			
		}
		else if (strcmp(type, "dir") == 0)
		{
			params = 1;
			if (argc == params)
			{
				para_d[0] = DBG_ATOI(argv_i[2]);
				StepMotor_Setup_Direction(para_d[0]);
				DBG_PRINTF("%s(%d)\r\n", type, para_d[0]);
			}
			else
			{
				DBG_PRINTF("%s have %d parameters(int dir)\r\n \t\t dir[0xFF] bit set\r\n",
							type, params);
			}
		}
		else
		{
			findno = 1;
		}
	}
	else
	{
		findno = 1;
	}

	if (findno)
	{
		DBG_PRINTF("This cmd not find!\r\n");
	}
}

/*----------------------------------------------------------------------------*/

static void dbgu_parse_cmd(unsigned char **argv, unsigned char argc)
{
	int para_d[4];
	char **argv_i = argv;
	char *pfname = argv[0];

	argc--;
	if (strcmp(pfname, "key") == 0)
	{
		dbgu_key_enable = DBG_ATOI(argv[1]);
		if (dbgu_key_enable)
			DBG_PRINTF("key mode ON\r\n");
		else
			DBG_PRINTF("key mode OFF\r\n");
	}
	else if (strcmp(pfname, "step") == 0)
	{
		dbgu_step_cmd(&argv_i[1], argc);
	}
	else if (strcmp(pfname, "encoder") == 0)
	{
		dbgu_encoder_cmd(&argv_i[1], argc);
	}
	else if (strcmp(pfname, "poweron") == 0)
	{
		arch_Power_On();
	}
	else if (strcmp(pfname, "poweroff") == 0)
	{
		arch_Power_Off();
	}
	else if (strcmp(pfname, "alert_Clear") == 0)
	{
		Alert_Clear();
		DBG_PRINTF("Alert_Clear\r\n");
	}
}

static void dbgu_parse_sudo(unsigned char **argv, unsigned char argc)
{
	char *pfname = argv[0];
	
	if (strcmp(pfname, "version") == 0
		|| strcmp(pfname, "ver") == 0)
	{
		SUDO_PRINTF("%x\r\n", Get_Version());
	}
	else if (strcmp(pfname, "bulid") == 0)
	{
		SUDO_PRINTF("bulid time: %s %s\r\n", __DATE__, __TIME__);
	}
	else if (strcmp(pfname, "reset") == 0)	
	{
		SUDO_PRINTF("Reset Now\r\n");
	}
}

/*----------------------------------------------------------------------------*/


static void dbgu_key_deal(char ch)
{
	static uint16_t val = 0;
	int idx;

	if (dbgu_key_enable == 0)
		return;
	
	switch (ch)
    {
		case ']':
			dbgu_idx += 2;
		case '[':
			dbgu_idx--;
			DBG_PRINTF("[%d]",dbgu_idx);
			break;
		case '\\':
			dbgu_idx = 0;
			DBG_PRINTF("[%d]",dbgu_idx);
			break;
		case '}':
			dbgu_idx += 100;
		case '{':
			dbgu_idx -= 50;
			DBG_PRINTF("[%d]",dbgu_idx);
			break;
		//
		case '.':
			dbgu_val += 2;
		case ',':
			dbgu_val--;
			DBG_PRINTF("(%d)",dbgu_val);
			break;
		case '/':
			dbgu_val = 0;
			DBG_PRINTF("(%d)",dbgu_val);
			break;
		case '>':
			dbgu_val += 100;
		case '<':
			dbgu_val -= 50;
			DBG_PRINTF("(%d)",dbgu_val);
			break;

		case 'A':
			val = val ? 0 : 0xFFFF;
		case 'B':
			Exec_Jacquard(dbgu_idx, val);
			DBG_PRINTF("xzq(%d, 0x%X)\r\n", dbgu_idx, val);
			break;
			
		case 'E':
			val = val ? 0 : 0xFFFF;
		case 'F':
			Exec_EMF(dbgu_idx, val, 0,0);
			DBG_PRINTF("emf(%d, %d)\r\n", dbgu_idx, val);
			break;
			
		case 'S':
			#if DBGU_MOTOR_DEBUG
			if (dbgu_val == 1)
			{
				motor_bit_mask |= (1 << dbgu_idx);
				DBG_PRINTF("motor_debug(%d) ON\r\n", dbgu_idx);
			}
			else
			{
				motor_bit_mask &= ~(1 << dbgu_idx);
				DBG_PRINTF("motor_debug(%d) OFF\r\n", dbgu_idx);
			}
			#endif /* DBGU_MOTOR_DEBUG */
			break;
		case 'D':
			motor_bit_mask = 0;
			DBG_PRINTF("motor_debug all OFF\r\n");
			break;
		case 'T':
			motor_bit_mask = 0xFF;
			DBG_PRINTF("motor_debug On 0xFF\r\n");
			break;
		case 'M':
			{
				unsigned int x;
				x=Get_adc_data(dbgu_idx);
				DBG_PRINTF("ADC_data[%d]=<%d>\r\n", dbgu_idx,x);
				
			}
			break;
		////////////////////////////////////////////////////////
		case 'a':
			//Encoder_setRunCheck(dbgu_idx, dbgu_val);
			StepMotor_Set_with_Head_Mode(6);
			DBG_PRINTF("StepMotor_Set_with_Head_Mode(%d)\r\n", 6);
			break;
		case 'b':break;
		case 'c':
			Encoder_maskZero(dbgu_idx);
			DBG_PRINTF("Enc_maskZero(%d)\r\n", dbgu_idx);
			break;
		case 'd':
			Encoder_setZero(dbgu_idx, dbgu_val);
			DBG_PRINTF("Enc_setZero(%d %d)\r\n", dbgu_idx, dbgu_val);
			break;
		case 'e':{
			int ret, coder, state;
			if (dbgu_val != 0xFFFF)
			{
				ret = Encoder_getCoder(dbgu_idx, &coder, &state);
				DBG_PRINTF("[%d]Coder[ret %d..%d..0x%x]\r\n", dbgu_idx, ret, coder, state);
			}
			else
			{
				Encoder_Probe(0xFF);
			}
			}break;
		case 'f':
			//Encoder_autoZero(dbgu_idx, dbgu_val);
			StepMotor_Set_work_steps(4,dbgu_val,0);
			DBG_PRINTF("StepMotor_Set_work_steps[2](%d)\r\n", dbgu_val);
			break;
		case 'g':
			DBG_PRINTF("step_get_feet_steps()=%d \r\n", step_get_feet_steps());
			break;
		case 'h':
			//DBG_PRINTF("Enc_getZero(%d) = %d\r\n", dbgu_idx, Encoder_getZero(dbgu_idx));
			DBG_PRINTF("Get_SIG_PH_Status() = %X \r\n",  Get_SIG_PH_Status());
			break;
		case 'i':break;
		case 'j':
			{
				int ret, pos;
				ret = Encoder_getPos(dbgu_idx, 1, &pos);
				DBG_PRINTF("Enc_getPos(%d, CCW) = %d(%d)\r\n", dbgu_idx, pos, 800-pos);
			}
			break;
		case 'k':
			Encoder_rPos(dbgu_idx);
			DBG_PRINTF("Enc_rPos(%d)\r\n", dbgu_idx);
			break;
		case 'l':
			Encoder_adjPos(dbgu_idx);
			DBG_PRINTF("Enc_adj(%d)\r\n", dbgu_idx);
			break;
			
		case 'm':{
#if ENCODER_DMA_SUPPORT
				extern int coder_err_cntmax[];
				int i;
				DBG_PRINTF("\r\ncoder_err_cntmax:");
				for (i = 0; i < 8; i++)
				{
					DBG_PRINTF("[%d]", coder_err_cntmax[i]);
				}
				DBG_PRINTF("\r\n");
#endif /* ENCODER_DMA_SUPPORT */
			#if 0
				{
				LIFT_step_PWMDA_Set_val_test(dbgu_val);
				DBG_PRINTF("LIFT_PWMDA_SET>>(%d)\r\n", dbgu_val);
			}
			#endif
			}break;
		
		case 'n':
			StepMotor_exec(dbgu_idx, 200, 1,0,0);
			DBG_PRINTF("StepMotor_exec(%d, %d)\r\n", dbgu_idx, 200);
			break;
		case 'o':
			StepMotor_exec(dbgu_idx, 0, 1,0,0);
			DBG_PRINTF("StepMotor_exec(%d, %d)\r\n", dbgu_idx, 0);
			break;
		
		case 'p':
			StepMotor_Modfiy_Position(dbgu_idx, dbgu_val, 0);
			DBG_PRINTF("Step_Set_Position(%d, %d, 0)\r\n", dbgu_idx, dbgu_val);
			break;
			
		case 'r':
			StepMotor_Reset(dbgu_idx,0);
			DBG_PRINTF("Step_reset(%d)\r\n", dbgu_idx);
			break;
			
		case 's':
			StepMotor_exec(dbgu_idx, (short)dbgu_val, 1,0,0);
			//StepMotor_Type_exec(dbgu_idx, (short)dbgu_val, 4);
			//DBG_PRINTF("StepMotor_Type_exec_feet(%d, %d)\r\n", dbgu_idx, (short)dbgu_val);
			break;
			
		case 't':
			Encoder_setType(dbgu_idx, dbgu_val);
			DBG_PRINTF("Enc_setType(%d, %d)\r\n", dbgu_idx, dbgu_val);
			break;
			
		case 'u'://reset speed
			StepMotor_Set_Speed_EX(dbgu_idx, 0, dbgu_val * 100);
			DBG_PRINTF("Set Reset Speed(%d, %dHz)\r\n", dbgu_idx, dbgu_val*100);
			break;
			
		case 'v':
			StepMotor_Set_Speed_EX(dbgu_idx, 1, dbgu_val * 100);
			DBG_PRINTF("Set Exec Speed(%d, %dHz)\r\n", dbgu_idx, dbgu_val*100);
			break;
			
		case 'w':
			Encoder_Veiw(dbgu_idx, dbgu_val);
			break;
		case 'x':
			StepMotor_Setup_Direction(0x30b9);
			break;
		case 'y':
			Encoder_clr(dbgu_idx);
			DBG_PRINTF("Encoder_clr(%d)\r\n", dbgu_idx);
			break;
		case 'z':
			if (dbgu_val)
				arch_Power_On();
			else
				arch_Power_Off();
			DBG_PRINTF("Power(%d)\r\n", dbgu_val);
			
			if (dbgu_val)
				arch_StepMotor_Enable();
			else
				arch_StepMotor_Disable();
			DBG_PRINTF("Son(%d)\r\n", dbgu_val);
			Encoder_Enable(dbgu_val);
			DBG_PRINTF("Eon(%d)\r\n", dbgu_val);
			break;
		//case '':break;
		default:
			break;
	}
}

static void dbgu_cmd_deal(char ch)
{
	static unsigned char prev_cnt = 0;
	unsigned char *ptr[10], *buf;
	unsigned char **argv, argc = 0;
	
	if (ch == '\r')
	{
		DBG_PUTCHAR('\r');
		DBG_PUTCHAR('\n');
		dbgu_str_buf[dbgu_ch_cnt] = '\0';
		buf = dbgu_str_buf;
		
		memcpy((void *)dbgu_str_prev, (void *)dbgu_str_buf, dbgu_ch_cnt);
		prev_cnt = dbgu_ch_cnt;
		
		while (ptr[argc] = strtok(buf, " "))/* 提取每个参数 */
		{
			if (++argc > 6)
			{
				argc = 0;
				break;
			}
			buf = NULL;
		}
		
		if (argc)
		{
			argv = &ptr[0];
			if (dbgu_login == DBGU_LOGIN_USER)
			{
				dbgu_parse_cmd(argv, argc);
				DBG_PRINTF("%s@%s: ", USER_NAME, DEV_NAME);
			}
			else if (dbgu_login == DBGU_LOGIN_ROOT)
			{
				dbgu_parse_sudo(argv, argc);
				DBG_PRINTF("%s@%s: ", SUDO_NAME, DEV_NAME);
			}
		}
		dbgu_ch_cnt = 0;
	}
	else if(ch == '\b')
	{
	  	if (dbgu_ch_cnt)
		{
			DBG_PUTCHAR('\b');
			DBG_PUTCHAR(' ');
			DBG_PUTCHAR('\b');
			dbgu_ch_cnt--;
		}
	}
	else if (ch == '`' && dbgu_ch_cnt == 0)	//上一次输入命令还原
	{
		if (prev_cnt)
		{
			dbgu_ch_cnt = prev_cnt;
			memcpy((void *)dbgu_str_buf, (void *)dbgu_str_prev, dbgu_ch_cnt);
			DBG_PRINTF("%s", dbgu_str_buf);
		}
	}
	else if (ch >= ' ')	//可见字符
	{
	  	DBG_PUTCHAR(ch);
		dbgu_str_buf[dbgu_ch_cnt] = ch;
		dbgu_ch_cnt++;
		dbgu_ch_cnt &= DBGU_STRBUF_SIZE;
	}
}

static void dbgu_login_deal(char ch)
{
	if (ch == '\r')
	{
		if (dbgu_login == DBGU_LOGIN_NULL)
		{
			dbgu_str_buf[dbgu_ch_cnt] = ' ';
			dbgu_login = DBGU_LOGIN_VERIFY;
			DBG_PRINTF("\r\nPassword: ");
			dbgu_ch_cnt++;
			dbgu_ch_cnt &= DBGU_STRBUF_SIZE;
		}
		else
		{
			char name[16], pass[16];
			
			dbgu_str_buf[dbgu_ch_cnt] = '\0';
			sscanf(dbgu_str_buf, "%s %s", name, pass);
			
			if (strcmp(name, USER_NAME) == 0 
				&& strcmp(pass, USER_PASSWORD) == 0)
			{
				dbgu_login = DBGU_LOGIN_USER;
				DBG_PRINTF("\r\n================================");
				DBG_PRINTF("\r\n==\tHeader Control: %x\t", Get_Version());
				DBG_PRINTF("\r\n================================");
				DBG_PRINTF("\r\n\r\n");
				DBG_PRINTF("\r\n%s@%s: ", USER_NAME, DEV_NAME);
			}
			else if (strcmp(name, SUDO_NAME) == 0 
				&& strcmp(pass, SUDO_PASSWORD) == 0)
			{
				dbgu_login = DBGU_LOGIN_ROOT;
				DBG_PRINTF("\r\n================================");
				DBG_PRINTF("\r\n==\tHeader Control: %x\t", Get_Version());
				DBG_PRINTF("\r\n================================");
				DBG_PRINTF("\r\n\r\n");
				DBG_PRINTF("\r\n%s@%s: ", SUDO_NAME, DEV_NAME);
			}
			else
			{
				dbgu_login = DBGU_LOGIN_NULL;
				DBG_PRINTF("\r\nLogin Error!\r\nLogin name: ");
			}
			dbgu_ch_cnt = 0;
		}
	}
	else if (ch == '\b')
	{
		if (dbgu_ch_cnt)
		{
			dbgu_ch_cnt--;
			DBG_PUTCHAR('\b');
			DBG_PUTCHAR(' ');
			DBG_PUTCHAR('\b');
		}
	}
	else if (ch >= ' ')	//可见字符
	{
		if (dbgu_login == DBGU_LOGIN_VERIFY)
			DBG_PUTCHAR('*');
		else
			DBG_PUTCHAR(ch);
		
		dbgu_str_buf[dbgu_ch_cnt] = ch;
		dbgu_ch_cnt++;
		dbgu_ch_cnt &= DBGU_STRBUF_SIZE;
	}
}

/*----------------------------------------------------------------------------*/

#if DBGU_MOTOR_DEBUG
void dbgu_motor_clr(int idx)
{
	if (motor_bit_mask & (1 << idx))
		motor_bit_mask &= ~(1 << idx);
}

void dbgu_motor_poll(void)
{
	short pos;
	
	if (motor_bit_mask == 0)
		return;
	
	motor_idx++;
	if (motor_idx >= DBGU_MOTOR_MAX)
		motor_idx = 0;

	if (motor_bit_mask & (1 << motor_idx))
	{
		if (StepMotor_Get_Running(motor_idx))
		{
			return;
		}
		
		motor_delay[motor_idx]--;
		if (motor_delay[motor_idx] == 0)
		{
			pos = StepMotor_Get_Position(motor_idx);
			if (pos != motor_step_min[motor_idx])
			{
				StepMotor_exec(motor_idx, motor_step_min[motor_idx], 1,0,0);
			}
			else
			{
				StepMotor_exec(motor_idx, motor_step_max[motor_idx], 1,0,0);
			}
		}
	}
}
#endif /* DBGU_MOTOR_DEBUG */

void dbgu_poll(void)
{
	static DBGU_MODE_t dbgu_mode = DBGU_MODE_KEY;
  	char ch;

#if 1
	if (!DBG_GETCHAR(&ch))
	{
	#if DBGU_MOTOR_DEBUG
		//dbgu_motor_poll();
	#endif /* DBGU_MOTOR_DEBUG */
		return;
	}

#endif	
	if (dbgu_mode == DBGU_MODE_KEY)
	{
	  	if (ch == '`')
		{
		  	dbgu_mode = DBGU_MODE_CMD;
			DBG_PRINTF("Login name: ");
			return;
		}
		dbgu_key_deal(ch);
	}
	else
	{
		if (ch == 0x1B)		//ESC
		{
			DBG_PRINTF("Login out!\r\n");
			DBG_PUTCHAR(0x0C);
			DBG_PUTCHAR(0x0C);
			
			dbgu_login = DBGU_LOGIN_NULL;
			dbgu_mode = DBGU_MODE_KEY;
			dbgu_ch_cnt = 0;
			return;
		}
		
		switch (dbgu_login)
		{
			case DBGU_LOGIN_NULL:
			case DBGU_LOGIN_VERIFY:
				dbgu_login_deal(ch);
				break;
				
			case DBGU_LOGIN_USER:
			case DBGU_LOGIN_ROOT:
				dbgu_cmd_deal(ch);
				break;
				
			default:
				dbgu_login = DBGU_LOGIN_NULL;
				break;
		}
	}
}

/*----------------------------------------------------------------------------*/

