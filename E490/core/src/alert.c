#include <string.h>
#include "massage.h"
#include "alert.h"
#include "config.h"

typedef struct {
	int alert_code;
	int alert_arg;
}ALERT_TYPE;

#define ALERT_MAX	30

ALERT_TYPE alertbuff[ALERT_MAX];

#define ALERT_MAX_BACK (2)
ALERT_TYPE alertbuff_back[ALERT_MAX_BACK];




volatile unsigned char alertpushisdoing;	
volatile unsigned int alert_rptr, alert_wptr;
volatile unsigned int alertcode;
volatile unsigned int alertarg;
// by xhl 2010/06/13
volatile unsigned int alert_delay;
volatile unsigned int alert_disable;

extern void arch_SendMessage(unsigned int action, unsigned int *pMsg, unsigned int len);

extern unsigned int Scan_error(void);
extern void Message_Send_Alert(unsigned int Msg, unsigned int arg1);
unsigned long test_alert_poolcount =0;
unsigned long test_alert_poolcount1 =0;



static int strlen_hlc(const char *str)
{
	int ret=0;
	while(*str++ !='\0')
	{
		ret ++;
	}
	return ret;
	
}

#if 0
static int memcpy_hlc(char *des,const char *ssr,unsigned int strlen)
{
	int ret;
	while(strlen--)
	{
		*des++=*ssr++;
		ret++;
	}
	return 0;
}
#endif

static void alert_buf_back_init()
{
	int i;
	for (i=0;i<ALERT_MAX_BACK;i++)
	{
		alertbuff_back[i].alert_arg=0;
		alertbuff_back[i].alert_code=0;
		
	}
}

#define ACTION_ALARM	0
#define ACTION_DATA	1
#define ACTION_LOG     2

#define SEND_ALERT_END	0xFF

static int alert_send_content_(char *alertcontent,unsigned int conlen,unsigned char a_code_start,unsigned char a_code_end)
{
#if 0
	int i,k;
	unsigned int buf[4];
	int maxc;
	int sendlen=conlen;
	unsigned short *x = (unsigned short *)alertcontent;
	static unsigned short sendindex=0;

	if ((a_code_start == 0)||(conlen==0)||(a_code_end<a_code_start)||(alertcontent==0))
	{
		return 0;
	}

	//if (conlen==0) 
	{
		buf[0] = (0x0007 | 0x0008<<8);			
		buf[1] =(unsigned short )a_code_start | ((unsigned short)a_code_end<<8);
		buf[2] =conlen;
		buf[3] =sendindex++;
		arch_SendMessage(ACTION_DATA,buf,4);
	}
	if ((conlen==SEND_ALERT_END)&&(a_code_start==SEND_ALERT_END)&&(a_code_end==SEND_ALERT_END))
	{
		sendindex=0;
		return 0;
	}
	else
	{
		maxc=((conlen-1)/8)+1;

		//myprintf("yes alert is sending.....\r\n");
	
		for (i=0;i<maxc;i++)
		{
		//	myprintf("yes alert is sending.....[%d]\r\n",i);
			buf[0] =0;			
			buf[1] =0;
			buf[2] =0;
			buf[3] =0;
			
			for (k=0;k<4;k++)
			{
				buf[k] =*x++;
				
				sendlen-=2;
				if (sendlen<=0)
				{
					k++;
					break;
				}		
				
			}				
	
			arch_SendMessage(ACTION_DATA,buf,k);
		}
	}

#endif	
	return 0;

}

extern volatile unsigned int DC24N_ALARM_set;	//1320 mv=1320/55=24A  
extern volatile unsigned int DC24P_ALARM_set;	//1320 mv=1320/55=24A  


int alert_Set_overloaddata(int p_d,int n_d)
{
	DC24P_ALARM_set = p_d*55/10.0;
	DC24N_ALARM_set = n_d*55/10.0;

	return 0;
}

#define MAX_CHAR_COUNT_ALERT	128

int alert_str_with_AlertCodeArg(unsigned short a_code,unsigned char language)
{

#if 0
	unsigned char alert_content_[MAX_CHAR_COUNT_ALERT];
	unsigned char ac_len = MAX_CHAR_COUNT_ALERT;
	unsigned char start_code;
	unsigned char is_c=(language==1)?1:0;		/*1表示中文，其它表示英文*/
	
	memset(alert_content_,0,MAX_CHAR_COUNT_ALERT);
	start_code = a_code;	
	switch(a_code)
	{
		#if 0
		case STEP_ERR:		
		case STEP_ERR+1:
		case STEP_ERR+2:
			{
				return 0;
			}
			break;
		#endif	
		case STEP_ERR+3:			
			{
				start_code = 	STEP_ERR;
				if (is_c)
				{
					memcpy(alert_content_,"%D(sys)系统%D(a1:0-3)号度目电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%D(sys)系统%D(a1:0-3)号度目电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
				}
			}	
			break;
		#if 0	
		case DIR1_ERR:
			{
				return 0;
			}
			break;
		#endif		
		case DIR2_ERR:
			{
				start_code = 	DIR1_ERR;	
				if (is_c)
				{
					memcpy(alert_content_,"%t(a1=>184=\"左\";185=\"右\";\"\")方向传感器异常",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%t(a1=>184=\"左\";185=\"右\";\"\")方向传感器异常",MAX_CHAR_COUNT_ALERT);
				}
			}
			break;
		case CAN_BUF_ERR:
			{
				//start_code = 	CAN_BUF_ERR;	
				if (is_c)
				{
					memcpy(alert_content_,"%D(sys)系统CAN缓冲区溢出",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%D(sys)系统CAN缓冲区溢出",MAX_CHAR_COUNT_ALERT);
				}
			}	
			break;
		#if 0	
		case JQD_UNClEAR1:
		case JQD_UNClEAR2:
		case JQD_UNClEAR3:
		#endif	
		case JQD_UNClEAR4:
		#if 0	
		case JQD_UNClEAR5:
		case JQD_UNClEAR6:
		case JQD_UNClEAR7:
		case JQD_UNClEAR8:
		#endif
		{
			start_code = 	JQD_UNClEAR1;
			if (is_c)
			{
				memcpy(alert_content_,"%t(a2=>49=\"1\";50=\"2\";51=\"3\";52=\"4\";\"0\")号选针器未清刀",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>49=\"1\";50=\"2\";51=\"3\";52=\"4\";\"0\")号选针器未清刀",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case DC24_P_PERR:	/*+24V*/
		case DC24_N_PERR:	/*-24V*/		
		{
			start_code = 	DC24_P_PERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%i(a2:0-7)V电源异常(%x(a2:8-15))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%i(a2:0-7)V电源异常(%x(a2:8-15))",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case DC12_P_PERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统12V电源异常",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统12V电源异常",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;	
		case SHOCK_ALARM:
		{
			if (is_c)
			{	
				memcpy(alert_content_,"%t(a2=>0=\"后床\";1=\"前床\";\"机头\")撞针报警",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>0=\"后床\";1=\"前床\";\"机头\")撞针报警",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case DSP_FATAL_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头板失效",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头板失效",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case TZ_L_ERR:
		case TZ_R_ERR:	
		{
			start_code = 	TZ_L_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%t(a2=>0=\"左\";1=\"右\";\"\")探针报警",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>0=\"左\";1=\"右\";\"\")探针报警",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case SINKER1_ERR:
		case SINKER2_ERR:
		{
			start_code = 	SINKER1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:2-3)号生克电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:2-3)号生克电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
	//	case SINKER3_ERR:
		case SINKER4_ERR:	
		{
			start_code = 	SINKER3_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%t(a1:0-0=>0=\"3\";\"4\")号生克电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%t(a1:0-0=>0=\"3\";\"4\")号生克电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case FEET1_ERR:
		case FEET2_ERR:			
		{
			start_code = 	FEET1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号压脚电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号压脚电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}	
		break;
		//case TRIANGLE1_ERR:
		case TRIANGLE2_ERR:			
		{
			start_code = 	TRIANGLE1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号动作电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号动作电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		/*case YARN_STEP1_ERR:
		case YARN_STEP2_ERR:
		case YARN_STEP3_ERR:
			*/
		case YARN_STEP4_ERR:			
		{
			start_code = 	YARN_STEP1_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号动作电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:1-1)号动作电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
		//case LIFT_STEP1_ERR:
		case LIFT_STEP2_ERR:
		{
			start_code = 	LIFT_STEP1_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:2-2)号推针电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:2-2)号推针电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case JDM_STEP1_ERR:
		case JDM_STEP2_ERR:	
		{
			start_code =  JDM_STEP1_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:0-2)号紧吊电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%D(a1:0-2)号紧吊电机%t(a2:12-15=>2=\"失步+\";3=\"失步-\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);	
			}
		}
		break;
		case HEAD_BINDING_MAIN_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头绑定主控板失败",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头绑定主控板失败",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case HEAD_DC24_CURR_OVERLOAD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头%t(a2=>0=\"-\";2=\"-\";3=\"-\";\"+\")24V(%t(a2=>0=\"3\";2=\"1.5\";3=\"8\";4=\"3\";5=\"1.5\";1=\"8\")A)通电时间过长",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头%t(a2=>0=\"-\";2=\"-\";3=\"-\";\"+\")24V(%t(a2=>0=\"3\";2=\"1.5\";3=\"8\";4=\"3\";5=\"1.5\";1=\"8\")A)通电时间过长",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
		case HEAD_POWER_ISOFF:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头[%t(a2:8-8=>0=\"-\";1=\"+\";\"\")24V]电源关闭",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头[%t(a2:8-8=>0=\"-\";1=\"+\";\"\")24V]电源关闭",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case OVERLOAD_HEAD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头板过流(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头板过流(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case OVERLOAD_24V:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头[%t(a2=>0=\"-\";\"+\")24V]过流",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头[%t(a2=>0=\"-\";\"+\")24V]过流",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case OVERLOAD_POWEROFF:
		{
			if (is_c)
			{
				memcpy(alert_content_,"机头板过流导致电源关闭(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"机头板过流导致电源关闭(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case OVERLOAD_YARN:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%d(a2)号纱嘴电磁铁过流",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%d(a2)号纱嘴电磁铁过流",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case OVERLOAD_JQD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%d(a2:8-15)号选针器第%d(a2:0-7)刀过流",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%d(a2:8-15)号选针器第%d(a2:0-7)刀过流",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case OVERLOAD_ACT:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统%t(a2:8-15=>0=\"后\";\"前\")床%d(a2:0-7)号电磁铁过流",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统%t(a2:8-15=>0=\"后\";\"前\")床%d(a2:0-7)号电磁铁过流",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case ACT_STATE_NOT_CLEAR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统电磁铁状态未清零(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统电磁铁状态未清零(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case DIR_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"换向传感器状态异常(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"换向传感器状态异常(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case LIFT_STEP_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(a2:12-15)号推针电机未完成(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(a2:12-15)号推针电机未完成(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case SINKER_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(a2:12-15)号生克电机未完成(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(a2:12-15)号生克电机未完成(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case STI_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统度目电机未完成(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统度目电机未完成(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case DEVICE_CONFIG_NUM_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"未知电机类型报警",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"未知电机类型报警",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case BOARD_APP_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"系统%D(sys)程序与硬件版本不匹配",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"系统%D(sys)程序与硬件版本不匹配",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case YARN_CMD_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"纱嘴电机命令错 ",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"纱嘴电机命令错 ",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case TEMP_OVER_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)系统CPU超温报警",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)系统CPU超温报警",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case ALERT_CODE_ENCODE:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%d(a1:8-15)号电机编码器%t(a2:12-15=>2=\"失步\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%d(a1:8-15)号电机编码器%t(a2:12-15=>2=\"失步\";\"报警\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}	
						
		}
		break;
		case JQD_OPERATE_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%t(a2:4-7=>15=\"纱嘴电磁铁\";14=\"动作电磁铁\";\"选针器\")导通[%d(a2:4-7)][%d(a2:0-3)]",MAX_CHAR_COUNT_ALERT);		
			}
			else
			{
				memcpy(alert_content_,"%t(a2:4-7=>15=\"纱嘴电磁铁\";14=\"动作电磁铁\";\"选针器\")导通[%d(a2:4-7)][%d(a2:0-3)]",MAX_CHAR_COUNT_ALERT);		
			}
			
		}
		break;	
		case SEND_ALERT_END:
		{
			ac_len =SEND_ALERT_END;
			goto Send_it;
		}
		default:
		{
			return 0;
		}
		//break;	
		
	}
	ac_len = strlen_hlc((char *)alert_content_);

	if (ac_len==0)
	{
		return 0;
	}
Send_it:
	
	alert_send_content_((char *)alert_content_,ac_len,start_code,a_code);

	#endif
	return 0;
	
}


int Send_alert_content_loop(unsigned char language)
{
	int i;
	for (i=0;i<=0xff;i++)
	{
		alert_str_with_AlertCodeArg(i,language);
	}

	return 0;
}


int alert_Get_Content_withindex(int code,int arg,int iseng)
{
	//int i;
	//ALERT_TYPE *Alert;
	//char undefstr[100];
	extern int sprintf_ex(char *out, const char *format, ...);

	//myprintf("yes alert code[%x],arg[%x],index[%d]\r\n",code,arg,index);

#if 0
	switch(code)
	{
		case YARN_ACT_FAULT:			
			sprintf_ex(undefstr,iseng ? "Yarn EMF [%d]error,status[%x]":"纱嘴电磁铁[%d]故障,状态[%x]",arg >>8,arg & 0xff);
			break;
		case OVERLOAD_HEAD:
			
			sprintf_ex(undefstr,iseng ? "Head board Overload[%d]":"机头板过流[%d]",arg);
			break;

		case OVERLOAD_JQD:
			if (arg==0)
			{
				sprintf_ex(undefstr,iseng ? "Needle selector Overload[%d]":"选针器过流[%d]",arg);
			}
			else
			{
			
				sprintf_ex(undefstr,iseng ? "[%d]Needle selector [%d] blade Overload[%d]":"[%d]号选针器第[%d]刀过流",arg>>8,arg & 0xff);

			}
			break;
		case OVERLOAD_ACT:
			if (arg==0)
			{
				
				sprintf_ex(undefstr,iseng ? "ACT EMF Overload[%d]":"动作电磁铁过流[%d]",arg);
			}
			else
			{
				
				sprintf_ex(undefstr,iseng ? "[%d]ACT [%d]EMF Overload":"[%d]号三角第[%d]电磁铁过流",arg>>8,arg &0xff);
			}			
			break;

		case OVERLOAD_YARN:
			if (arg==0)
			{
				
				sprintf_ex(undefstr,iseng ? "Yarn EMF Overload[%d]":"纱嘴电磁铁过流[%d]",arg);
			}
			else
			{
				
				sprintf_ex(undefstr,iseng ? "Yarn EMF NO.[%d] Overload":"[%d]号纱嘴电磁铁过流",arg);
			}
			break;
		case SHOCK_ALARM:
			{
				sprintf_ex(undefstr,iseng ? "Alert of firing pin":"撞针报警");
			}

			break;
		case DC24_N_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of -24V power":"-24V电源报警");
			}
			break;
		case DC24_P_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of +24V power":"+24V电源报警");
			}
			break;	

		case DC12_P_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of +12V power":"+12V电源报警");
			}
			break;	

	#if 0
		case YARN_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for main board 24V invalid":"主板24V保险丝失效");
			break;
		case XZQ1_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 1 -24V invalid":"机头板1 -24V保险丝失效");
			break;
		case XZQ2_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 2 +24V invalid":"机头板2 +24V保险丝失效");
			break;
		case ACT1_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 1 +24V invalid":"机头板1 +24V保险丝失效");
			break;
		case ACT2_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 2 -24V invalid":"机头板2 -24V保险丝失效");
			break;
#endif
		
		case TZ_L_ERR://
			if (arg==1)
			{
				sprintf_ex(undefstr,iseng ? "Left probe alert":"左探针报警");
			}
			else
			{
				goto code_arg_undef;				
			}
			
			
			break;
		case TZ_R_ERR:
			if (arg==1)
			{
				sprintf_ex(undefstr,iseng ? "Right probe alert":"右探针报警");
			}
			else
			{
				goto code_arg_undef;
			}
			break;	
		
		case DIR1_ERR:

			
			sprintf_ex(undefstr,iseng ? "Left DIR sensor alert":"左方向传感器报警");
			break;
		case DIR2_ERR:
			
			
			sprintf_ex(undefstr,iseng ? "Right DIR sensor alert":"右方向传感器报警");
			break;
		
		case JQD_UNClEAR1:
		case JQD_UNClEAR2:
		case JQD_UNClEAR3:
		case JQD_UNClEAR4:
		case JQD_UNClEAR5:
		case JQD_UNClEAR6:
		case JQD_UNClEAR7:
		case JQD_UNClEAR8:
			
			//alert_format_content("[%d]号选针器未清刀","[%d]Needle selector not clear", i+1,0);
			
			sprintf_ex(undefstr,iseng ? "[%d]Needle selector not clear[%x]":"[%d]号选针器未清刀[%x]",code-JQD_UNClEAR1+1,arg);				
			break;
		case DIR_ERR:
				
				sprintf_ex(undefstr,iseng ? "Status of DIR Sensor Error [%x]":"方向传感器状态错误[%x]",arg);	
			break;


		case YARN_STATE_NOT_CLEAR :
				//alert_format_content("纱嘴状态未清零[%x]","Status of yarn EMF not clear [%x]", yarn_status,0);
				
				sprintf_ex(undefstr,iseng ? "Status of yarn EMF not clear [%x]":"纱嘴状态未清零[%x]",arg);	
					
			break;
		case CAN_BUF_ERR:
			
			sprintf_ex(undefstr,iseng ? "CAN Buffer Overflow":"CAN缓冲区溢出");	
			break;
		case STEP_ERR:
		case STEP_ERR+1:
		case STEP_ERR+2:
		case STEP_ERR+3:
		case STEP_ERR+4:
		case STEP_ERR+5:
		
				if (arg==0)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor drive chip alarm ":"%d号电机芯片报警",code-STEP_ERR+1);	
				}
				else if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor pos is not 0 ":"%d号度目编码值不为0",code-STEP_ERR+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor ZERO sensor is not 0 ":"%d号度目传感器不在0位",code-STEP_ERR+1,arg);	
				}
				else if (arg==0x9222)
				{
				//alert_format_content_isr("%d号度目工作位传感器未检测到","%d Motor work sensor invalid", i+1,0);
						
					sprintf_ex(undefstr,iseng ? "%d Motor work sensor invalid ":"%d号度目工作位传感器未检测到",code-STEP_ERR+1,arg);	
				}
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Motor Error[%d]":"%d号度目电机故障[%d]",code-STEP_ERR+1,arg);	
					}
			break;
		case SINKER1_ERR:
		case SINKER2_ERR:
				
		case SINKER3_ERR:
		case SINKER4_ERR:	
				if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Sinker Motor pos is not 0 ":"%d号申克编码值不为0",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Sinker  Motor ZERO sensor is not 0 ":"%d号申克传感器不在0位",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
				}				
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Sinker Motor Error[%d]":"%d号申克电机故障[%d]",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
					}
			
			break;
		case FEET1_ERR:
		case FEET2_ERR:
		case FEET3_ERR:
		case FEET4_ERR:	
				if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Feet Motor pos is not 0 ":"%d号压脚编码值不为0",code-(FEET1_ERR)+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Feet  Motor ZERO sensor is not 0 ":"%d号压脚传感器不在0位",code-(FEET1_ERR)+1,arg);	
				}				
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Feet Motor Error[%d]":"%d号压脚电机故障[%d]",code-(FEET1_ERR)+1,arg);	
					}
			
			break;
		case SINKER_DONE_ERR:

				//alert_format_content("%d号生克动作未完成[%d]","%d Sinker Motor not finish [%d]", i+1,step_count);
				
				sprintf_ex(undefstr,iseng ? "%d Sinker Motor not finish [%d]":"%d号生克动作未完成[%d]",(arg>>12)+1,arg & 0xfff);
			break;
		case STI_DONE_ERR:
				//alert_format_content("%d号度目动作未完成[%d]","%d Motor not finish [%d]", i+1,steps & 0x0FFF);
						
				sprintf_ex(undefstr,iseng ? "%d Motor not finish [%d]":"%d号度目动作未完成[%d]",(arg>>12)+1,arg & 0xfff);			
			break;
		
			
		default:

		code_arg_undef:

			sprintf_ex(undefstr,iseng ? "alert code [%d][%d] undefined":"报警ID[%d][%d] 未定义",code,arg);
			break;

	}

	alert_send_content_(undefstr,strlen_hlc(undefstr));

#endif

	return 0;	


}






void alert_init()
{
	alert_rptr = 0;
	alert_wptr = 0;
	alertcode = 0;
	alertpushisdoing=0;
	// by xhl 2010/06/13
	//alert_delay = 500 * 30;
	// by xhl 2011/01/13
	alert_delay = 500;
	alert_disable = 0;
	alert_buf_back_init();
}

int alert_count()
{
	int count;
	count = ALERT_MAX + alert_wptr - alert_rptr;
	if(count >= ALERT_MAX)
		count -= ALERT_MAX;

	return count;
}

int alert_find(int alert_code, int alert_arg);
int alert_push(int alert_code, int alert_arg)
{
	ALERT_TYPE *Alert;
	int ret;

	
 
	if (alertpushisdoing)				//确保重入的时候，先做临时备份
	{
		int i;
		for (i=0;i<ALERT_MAX_BACK;i++)
		{
			if ((alertbuff_back[i].alert_code==0)||(i==ALERT_MAX_BACK-1))   
			{
				alertbuff_back[i].alert_code = alert_code;
				alertbuff_back[i].alert_arg= alert_arg;
				
				break;
			}
		}
		

		return -3;
	}
	else
	{
		alertpushisdoing =1;
	}

#ifdef GUOLV_SANJIAO_ALARM
	if ((alert_code == TRIANGLE1_ERR)||(alert_code == TRIANGLE2_ERR))
	{
		goto return_exit;
	}
#endif
	if((alert_code == alertcode) &&
	   (alert_arg == alertarg)) {
		ret= 0;
		goto return_exit;
	}
	if(alert_count() == ALERT_MAX - 1) {
		ret= -1;
		goto return_exit;
	}


	{

		Alert = &alertbuff[alert_wptr];

		Alert->alert_code = alert_code;
		Alert->alert_arg = alert_arg;
		
		
		alert_wptr ++;
		if(alert_wptr >= ALERT_MAX) {
			alert_wptr -= ALERT_MAX;
		}
	}

	ret= 1;
	

	return_exit:
		alertpushisdoing = 0;
		return ret;
}

int alert_pop(int *alert_code, int *alert_arg)
{
	ALERT_TYPE *Alert;

	*alert_code = 0;
find_next:
	if(alert_count() == 0) {
		return 0;
	}

	Alert = &alertbuff[alert_rptr];

	*alert_code = Alert->alert_code;
	*alert_arg = Alert->alert_arg;

	alert_rptr ++;
	if(alert_rptr >= ALERT_MAX) {
		alert_rptr -= ALERT_MAX;
	}

	if(Alert->alert_code == 0) {
		goto find_next;
	}

	return 1;
}

int alert_find(int alert_code, int alert_arg)
{
	int i, idx;
	int count = alert_count();

	idx = alert_rptr;
	for(i = 0, idx = alert_rptr; i < count; i ++, idx ++) {
		if(idx >= ALERT_MAX) {
			idx -= ALERT_MAX;
		}
		if((alertbuff[idx].alert_code == alert_code) &&
		   (alertbuff[idx].alert_arg == alert_arg)) {
			return 1;
		}
	}
	return 0;
}

int alert_delete(int alert_code, int alert_arg)
{
	int i;

	for(i = 0; i < ALERT_MAX; i ++) {
		if((alertbuff[i].alert_code == alert_code) &&
		   (alertbuff[i].alert_arg == alert_arg)) {
			alertbuff[i].alert_code = 0;
		}
	}

	return 0;
}

unsigned int AlertDelaySendCnt=0;
void Alert_Clear()
{
	if(alertcode) {
		//myprintf("ok,delete it\r\n");
		alert_delete(alertcode, alertarg);
		alertcode = 0;
		AlertDelaySendCnt =0;
	}
}

 // by xhl 2010/06/13


void Alert_Code_Send_Delay_ISR() /*2ms isr*/
 {
	if(AlertDelaySendCnt)
		AlertDelaySendCnt--;
 }

/*d =d个2ms*/
void Alert_Code_Send_Delay_Setcnt(unsigned int  d)
{
	AlertDelaySendCnt = d;
}


void Alert_Poll()
{
	unsigned int ret;
	extern volatile unsigned char SYS_is_PowerOn;

		if(alert_disable)					
			return ;		

		if(alertcode)
		{		
			#ifndef LX_ACT_SPECIAL
			if(!AlertDelaySendCnt)			
			{
				Message_Send_Alert(alertcode, alertarg);
				Alert_Code_Send_Delay_Setcnt(8);
			}
			#endif
			return ;
		}
		if (!SYS_is_PowerOn)
			return;
		
		
	ret = alert_pop((int *)&alertcode, (int *)&alertarg);
	if(ret) {
		Message_Send_Alert(alertcode, alertarg);
		#ifndef LX_ACT_SPECIAL
		Alert_Code_Send_Delay_Setcnt(5);
		#endif
		//alert_delete(alertcode, alertarg);
		//alertcode =0;  //Debug_hlc
	}
	
	ret = Scan_error();
	if(ret) {
		if(alert_find(ret, 0) == 0) {
			alert_push(ret, 0);
		}
	}
	return ;
}

 

void alert_time_poll()
{
	if(alert_delay) {
		alert_delay --;
		if(alert_delay == 0) {
			alert_disable = 0;
		}
	}
}

void alert_cpu_fatal(int x)
{
	Message_Send_Alert(DSP_FATAL_ERR, (unsigned int)x);
}



int alert_push_again()
{
		int i;
		int doit =0;

		//return doit;
		
		for (i=0;i<ALERT_MAX_BACK;i++)
		{
			if (alertbuff_back[i].alert_code)   
			{
				alert_push(alertbuff_back[i].alert_code,alertbuff_back[i].alert_arg);

				alertbuff_back[i].alert_code =0;
				
				doit++;
			}
		}
		return doit;

}



