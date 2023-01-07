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
	unsigned char is_c=(language==1)?1:0;		/*1��ʾ���ģ�������ʾӢ��*/
	
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
					memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:0-3)�Ŷ�Ŀ���%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:0-3)�Ŷ�Ŀ���%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
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
					memcpy(alert_content_,"%t(a1=>184=\"��\";185=\"��\";\"\")���򴫸����쳣",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%t(a1=>184=\"��\";185=\"��\";\"\")���򴫸����쳣",MAX_CHAR_COUNT_ALERT);
				}
			}
			break;
		case CAN_BUF_ERR:
			{
				//start_code = 	CAN_BUF_ERR;	
				if (is_c)
				{
					memcpy(alert_content_,"%D(sys)ϵͳCAN���������",MAX_CHAR_COUNT_ALERT);
				}
				else
				{
					memcpy(alert_content_,"%D(sys)ϵͳCAN���������",MAX_CHAR_COUNT_ALERT);
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
				memcpy(alert_content_,"%t(a2=>49=\"1\";50=\"2\";51=\"3\";52=\"4\";\"0\")��ѡ����δ�嵶",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>49=\"1\";50=\"2\";51=\"3\";52=\"4\";\"0\")��ѡ����δ�嵶",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case DC24_P_PERR:	/*+24V*/
		case DC24_N_PERR:	/*-24V*/		
		{
			start_code = 	DC24_P_PERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%i(a2:0-7)V��Դ�쳣(%x(a2:8-15))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%i(a2:0-7)V��Դ�쳣(%x(a2:8-15))",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case DC12_P_PERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ12V��Դ�쳣",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ12V��Դ�쳣",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;	
		case SHOCK_ALARM:
		{
			if (is_c)
			{	
				memcpy(alert_content_,"%t(a2=>0=\"��\";1=\"ǰ��\";\"��ͷ\")ײ�뱨��",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>0=\"��\";1=\"ǰ��\";\"��ͷ\")ײ�뱨��",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case DSP_FATAL_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ��ʧЧ",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ��ʧЧ",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case TZ_L_ERR:
		case TZ_R_ERR:	
		{
			start_code = 	TZ_L_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%t(a2=>0=\"��\";1=\"��\";\"\")̽�뱨��",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%t(a2=>0=\"��\";1=\"��\";\"\")̽�뱨��",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case SINKER1_ERR:
		case SINKER2_ERR:
		{
			start_code = 	SINKER1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:2-3)�����˵��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:2-3)�����˵��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
	//	case SINKER3_ERR:
		case SINKER4_ERR:	
		{
			start_code = 	SINKER3_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%t(a1:0-0=>0=\"3\";\"4\")�����˵��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%t(a1:0-0=>0=\"3\";\"4\")�����˵��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case FEET1_ERR:
		case FEET2_ERR:			
		{
			start_code = 	FEET1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)��ѹ�ŵ��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)��ѹ�ŵ��%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}	
		break;
		//case TRIANGLE1_ERR:
		case TRIANGLE2_ERR:			
		{
			start_code = 	TRIANGLE1_ERR;	
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)�Ŷ������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)�Ŷ������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
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
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)�Ŷ������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:1-1)�Ŷ������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
		//case LIFT_STEP1_ERR:
		case LIFT_STEP2_ERR:
		{
			start_code = 	LIFT_STEP1_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:2-2)��������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:2-2)��������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		//case JDM_STEP1_ERR:
		case JDM_STEP2_ERR:	
		{
			start_code =  JDM_STEP1_ERR;
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:0-2)�Ž������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%D(a1:0-2)�Ž������%t(a2:12-15=>2=\"ʧ��+\";3=\"ʧ��-\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);	
			}
		}
		break;
		case HEAD_BINDING_MAIN_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ�����ذ�ʧ��",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ�����ذ�ʧ��",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case HEAD_DC24_CURR_OVERLOAD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ%t(a2=>0=\"-\";2=\"-\";3=\"-\";\"+\")24V(%t(a2=>0=\"3\";2=\"1.5\";3=\"8\";4=\"3\";5=\"1.5\";1=\"8\")A)ͨ��ʱ�����",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ%t(a2=>0=\"-\";2=\"-\";3=\"-\";\"+\")24V(%t(a2=>0=\"3\";2=\"1.5\";3=\"8\";4=\"3\";5=\"1.5\";1=\"8\")A)ͨ��ʱ�����",MAX_CHAR_COUNT_ALERT);		
			}
		}
		break;
		case HEAD_POWER_ISOFF:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ[%t(a2:8-8=>0=\"-\";1=\"+\";\"\")24V]��Դ�ر�",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ[%t(a2:8-8=>0=\"-\";1=\"+\";\"\")24V]��Դ�ر�",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case OVERLOAD_HEAD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ�����(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ�����(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
		}
		break;
		case OVERLOAD_24V:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ[%t(a2=>0=\"-\";\"+\")24V]����",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ[%t(a2=>0=\"-\";\"+\")24V]����",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case OVERLOAD_POWEROFF:
		{
			if (is_c)
			{
				memcpy(alert_content_,"��ͷ��������µ�Դ�ر�(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"��ͷ��������µ�Դ�ر�(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case OVERLOAD_YARN:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%d(a2)��ɴ����������",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%d(a2)��ɴ����������",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case OVERLOAD_JQD:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%d(a2:8-15)��ѡ������%d(a2:0-7)������",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%d(a2:8-15)��ѡ������%d(a2:0-7)������",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case OVERLOAD_ACT:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%t(a2:8-15=>0=\"��\";\"ǰ\")��%d(a2:0-7)�ŵ��������",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ%t(a2:8-15=>0=\"��\";\"ǰ\")��%d(a2:0-7)�ŵ��������",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case ACT_STATE_NOT_CLEAR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ�����״̬δ����(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ�����״̬δ����(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case DIR_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"���򴫸���״̬�쳣(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"���򴫸���״̬�쳣(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case LIFT_STEP_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(a2:12-15)��������δ���(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(a2:12-15)��������δ���(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case SINKER_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(a2:12-15)�����˵��δ���(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(a2:12-15)�����˵��δ���(%d(a2:0-11))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case STI_DONE_ERR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳ��Ŀ���δ���(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳ��Ŀ���δ���(%d(a2))",MAX_CHAR_COUNT_ALERT);
			}	
			
		}
		break;
		case DEVICE_CONFIG_NUM_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"δ֪������ͱ���",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"δ֪������ͱ���",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case BOARD_APP_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"ϵͳ%D(sys)������Ӳ���汾��ƥ��",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"ϵͳ%D(sys)������Ӳ���汾��ƥ��",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case YARN_CMD_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"ɴ��������� ",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"ɴ��������� ",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case TEMP_OVER_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%D(sys)ϵͳCPU���±���",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%D(sys)ϵͳCPU���±���",MAX_CHAR_COUNT_ALERT);
			}
			
		}
		break;
		case ALERT_CODE_ENCODE:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%d(a1:8-15)�ŵ��������%t(a2:12-15=>2=\"ʧ��\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}
			else
			{
				memcpy(alert_content_,"%d(a1:8-15)�ŵ��������%t(a2:12-15=>2=\"ʧ��\";\"����\")[%d(a2:0-11)]",MAX_CHAR_COUNT_ALERT);
			}	
						
		}
		break;
		case JQD_OPERATE_ERROR:
		{
			if (is_c)
			{
				memcpy(alert_content_,"%t(a2:4-7=>15=\"ɴ������\";14=\"���������\";\"ѡ����\")��ͨ[%d(a2:4-7)][%d(a2:0-3)]",MAX_CHAR_COUNT_ALERT);		
			}
			else
			{
				memcpy(alert_content_,"%t(a2:4-7=>15=\"ɴ������\";14=\"���������\";\"ѡ����\")��ͨ[%d(a2:4-7)][%d(a2:0-3)]",MAX_CHAR_COUNT_ALERT);		
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
			sprintf_ex(undefstr,iseng ? "Yarn EMF [%d]error,status[%x]":"ɴ������[%d]����,״̬[%x]",arg >>8,arg & 0xff);
			break;
		case OVERLOAD_HEAD:
			
			sprintf_ex(undefstr,iseng ? "Head board Overload[%d]":"��ͷ�����[%d]",arg);
			break;

		case OVERLOAD_JQD:
			if (arg==0)
			{
				sprintf_ex(undefstr,iseng ? "Needle selector Overload[%d]":"ѡ��������[%d]",arg);
			}
			else
			{
			
				sprintf_ex(undefstr,iseng ? "[%d]Needle selector [%d] blade Overload[%d]":"[%d]��ѡ������[%d]������",arg>>8,arg & 0xff);

			}
			break;
		case OVERLOAD_ACT:
			if (arg==0)
			{
				
				sprintf_ex(undefstr,iseng ? "ACT EMF Overload[%d]":"�������������[%d]",arg);
			}
			else
			{
				
				sprintf_ex(undefstr,iseng ? "[%d]ACT [%d]EMF Overload":"[%d]�����ǵ�[%d]���������",arg>>8,arg &0xff);
			}			
			break;

		case OVERLOAD_YARN:
			if (arg==0)
			{
				
				sprintf_ex(undefstr,iseng ? "Yarn EMF Overload[%d]":"ɴ����������[%d]",arg);
			}
			else
			{
				
				sprintf_ex(undefstr,iseng ? "Yarn EMF NO.[%d] Overload":"[%d]��ɴ����������",arg);
			}
			break;
		case SHOCK_ALARM:
			{
				sprintf_ex(undefstr,iseng ? "Alert of firing pin":"ײ�뱨��");
			}

			break;
		case DC24_N_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of -24V power":"-24V��Դ����");
			}
			break;
		case DC24_P_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of +24V power":"+24V��Դ����");
			}
			break;	

		case DC12_P_PERR:
			{
				sprintf_ex(undefstr,iseng ? "Alert of +12V power":"+12V��Դ����");
			}
			break;	

	#if 0
		case YARN_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for main board 24V invalid":"����24V����˿ʧЧ");
			break;
		case XZQ1_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 1 -24V invalid":"��ͷ��1 -24V����˿ʧЧ");
			break;
		case XZQ2_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 2 +24V invalid":"��ͷ��2 +24V����˿ʧЧ");
			break;
		case ACT1_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 1 +24V invalid":"��ͷ��1 +24V����˿ʧЧ");
			break;
		case ACT2_PERR:
			
			sprintf(undefstr,iseng ? "Fuse for head board 2 -24V invalid":"��ͷ��2 -24V����˿ʧЧ");
			break;
#endif
		
		case TZ_L_ERR://
			if (arg==1)
			{
				sprintf_ex(undefstr,iseng ? "Left probe alert":"��̽�뱨��");
			}
			else
			{
				goto code_arg_undef;				
			}
			
			
			break;
		case TZ_R_ERR:
			if (arg==1)
			{
				sprintf_ex(undefstr,iseng ? "Right probe alert":"��̽�뱨��");
			}
			else
			{
				goto code_arg_undef;
			}
			break;	
		
		case DIR1_ERR:

			
			sprintf_ex(undefstr,iseng ? "Left DIR sensor alert":"���򴫸�������");
			break;
		case DIR2_ERR:
			
			
			sprintf_ex(undefstr,iseng ? "Right DIR sensor alert":"�ҷ��򴫸�������");
			break;
		
		case JQD_UNClEAR1:
		case JQD_UNClEAR2:
		case JQD_UNClEAR3:
		case JQD_UNClEAR4:
		case JQD_UNClEAR5:
		case JQD_UNClEAR6:
		case JQD_UNClEAR7:
		case JQD_UNClEAR8:
			
			//alert_format_content("[%d]��ѡ����δ�嵶","[%d]Needle selector not clear", i+1,0);
			
			sprintf_ex(undefstr,iseng ? "[%d]Needle selector not clear[%x]":"[%d]��ѡ����δ�嵶[%x]",code-JQD_UNClEAR1+1,arg);				
			break;
		case DIR_ERR:
				
				sprintf_ex(undefstr,iseng ? "Status of DIR Sensor Error [%x]":"���򴫸���״̬����[%x]",arg);	
			break;


		case YARN_STATE_NOT_CLEAR :
				//alert_format_content("ɴ��״̬δ����[%x]","Status of yarn EMF not clear [%x]", yarn_status,0);
				
				sprintf_ex(undefstr,iseng ? "Status of yarn EMF not clear [%x]":"ɴ��״̬δ����[%x]",arg);	
					
			break;
		case CAN_BUF_ERR:
			
			sprintf_ex(undefstr,iseng ? "CAN Buffer Overflow":"CAN���������");	
			break;
		case STEP_ERR:
		case STEP_ERR+1:
		case STEP_ERR+2:
		case STEP_ERR+3:
		case STEP_ERR+4:
		case STEP_ERR+5:
		
				if (arg==0)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor drive chip alarm ":"%d�ŵ��оƬ����",code-STEP_ERR+1);	
				}
				else if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor pos is not 0 ":"%d�Ŷ�Ŀ����ֵ��Ϊ0",code-STEP_ERR+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Motor ZERO sensor is not 0 ":"%d�Ŷ�Ŀ����������0λ",code-STEP_ERR+1,arg);	
				}
				else if (arg==0x9222)
				{
				//alert_format_content_isr("%d�Ŷ�Ŀ����λ������δ��⵽","%d Motor work sensor invalid", i+1,0);
						
					sprintf_ex(undefstr,iseng ? "%d Motor work sensor invalid ":"%d�Ŷ�Ŀ����λ������δ��⵽",code-STEP_ERR+1,arg);	
				}
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Motor Error[%d]":"%d�Ŷ�Ŀ�������[%d]",code-STEP_ERR+1,arg);	
					}
			break;
		case SINKER1_ERR:
		case SINKER2_ERR:
				
		case SINKER3_ERR:
		case SINKER4_ERR:	
				if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Sinker Motor pos is not 0 ":"%d����˱���ֵ��Ϊ0",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Sinker  Motor ZERO sensor is not 0 ":"%d����˴���������0λ",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
				}				
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Sinker Motor Error[%d]":"%d����˵������[%d]",code-((code>=SINKER3_ERR )?SINKER3_ERR:SINKER1_ERR)+1,arg);	
					}
			
			break;
		case FEET1_ERR:
		case FEET2_ERR:
		case FEET3_ERR:
		case FEET4_ERR:	
				if (arg==1)
				{
					sprintf_ex(undefstr,iseng ? "%d Feet Motor pos is not 0 ":"%d��ѹ�ű���ֵ��Ϊ0",code-(FEET1_ERR)+1,arg);	
				}
				else if (arg==2)
				{
					sprintf_ex(undefstr,iseng ? "%d Feet  Motor ZERO sensor is not 0 ":"%d��ѹ�Ŵ���������0λ",code-(FEET1_ERR)+1,arg);	
				}				
				else
					{
						sprintf_ex(undefstr,iseng ? "%d Feet Motor Error[%d]":"%d��ѹ�ŵ������[%d]",code-(FEET1_ERR)+1,arg);	
					}
			
			break;
		case SINKER_DONE_ERR:

				//alert_format_content("%d�����˶���δ���[%d]","%d Sinker Motor not finish [%d]", i+1,step_count);
				
				sprintf_ex(undefstr,iseng ? "%d Sinker Motor not finish [%d]":"%d�����˶���δ���[%d]",(arg>>12)+1,arg & 0xfff);
			break;
		case STI_DONE_ERR:
				//alert_format_content("%d�Ŷ�Ŀ����δ���[%d]","%d Motor not finish [%d]", i+1,steps & 0x0FFF);
						
				sprintf_ex(undefstr,iseng ? "%d Motor not finish [%d]":"%d�Ŷ�Ŀ����δ���[%d]",(arg>>12)+1,arg & 0xfff);			
			break;
		
			
		default:

		code_arg_undef:

			sprintf_ex(undefstr,iseng ? "alert code [%d][%d] undefined":"����ID[%d][%d] δ����",code,arg);
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

	
 
	if (alertpushisdoing)				//ȷ�������ʱ��������ʱ����
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

/*d =d��2ms*/
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



