#ifndef __STEP_H__
#define __STEP_H__

#include "config.h"



//#define IS_CX_FEET_3_INPUT_LIKE_SAMPLE   /*´ÈĞÇĞÂµÄÑ¹½ÅÏµÍ³£¬ÀàËÆ¼òÒ×Èı½Ç£¬Á½¸öĞÅºÅ£¬Ò»¸öÓÃÓÚÁãÎ»£¬Ò»¸öÓÃÓÚ¹¤×÷Î»¼ÆÊı*/

//#define IS_CX_ACT
#ifdef IS_CX_ACT
#define _CX_DOT_ACT_ZERO_ADJ	8

#endif


//#define DDM_MOTOR_FULLCURR    /*¶¨Òå¶ÈÄ¿µç»úÈ«Á÷Ëø¶¨*/

#define STEPMOTOR_HALF_ENABLE

#ifdef STEPMOTOR_HALF_ENABLE
/*°ëÁ÷Ê¹ÄÜ*/
#define MOTOR_HALF_OUTPUT	1		
#else
/*°ëÁ÷½ûÖ¹(µç»úËø¶¨Ò²ÊÇÈ«Á÷Ä£Ê½)*/
#define MOTOR_HALF_OUTPUT	0

#endif



//#define MOTOR_FULL_CURR_OUTPUT	0

/*µç»ú±ä»¯Æ½»ºÄ£Ê½*/
#define SSTI_PULS_SMOOTH

#ifdef SSTI_PULS_SMOOTH

//#define V_FLOAT float 
#define V_FLOAT int 
typedef struct {
//	int mot_id;
	//int mot_cpos;
//	int mot_mpos;		// moving destination?
//	int mot_dest;
	//int mot_dir;
	//int mot_status;
	//int mot_flags;
//	int mot_sno;
	int mot_steps;
	//
	int mot_phase;		// 0: boot
				// 1: running
				// 2: decelerating..
	int mot_phase_dec_ex;		 //0--Õı³£
							// 1--¼ÓËÙÒ»´Î
	//
	// 
	V_FLOAT mot_speed;
	V_FLOAT mot_estspd;	// speed wanted..
	V_FLOAT mot_acc;		// accelerator
	//
	//long mot_period;	// 0.001ms..
	//
	// runtime stuff..
	//float mot_inispd;	// initial speed (before each step)
	V_FLOAT mot_thyspd;	// theory speed? (= estspd)?
	//...
} STEP_MOTOR_SMOOTH;


#endif




typedef struct {
	unsigned char DD_type;		/*¶¯Ì¬ÀàĞÍ:0--¸ù¾İ¸ø¶¨Ê±¼äºÍÄ¿±êÎ»ÖÃ£¬ĞŞÕıµç»úËÙ¶È
										      1--¸ù¾İ¶ÎÊı£¬Ã¿´ÎµÄÄ¿±êÖµ£¬×îÖÕÄ¿±êÖµ£¬À´¸úËæ*/
	unsigned char DD_cmd_CNT;/*¶¯Ì¬ÃüÁîÌõÊı>0,0±íÊ¾È«ËÙÅÜ*/
	unsigned char DD_cnt_cur; 	/*µ±Ç°ÊÇµÚ¼¸Ìõ*/

	unsigned char DD_cnt_cur_nextidx;/*¼ÇÂ¼ÏÂÒ»ÌõÕıÈ·µÄIDĞòºÅ*/
	
	unsigned char DD_err_cnt;  /*¼ÇÂ¼³ö´í´ÎÊı*/

	unsigned char DD_dont_act;/*²»ĞèÒª×öµ±Ç°¶¯×÷*/

	unsigned char DD_dec_speed;/*0--Õı³£´¦Àí£¬1--»ºÂı¼Ó¼õËÙ*/

	unsigned char DD_cmd_err_cnt;	/*¼ÇÂ¼ÃüÁî´íÎóµÄÇé¿ö*/
	unsigned char DD_Step_overflow_flag;/*²½ÊıÒç³ö±êÖ¾*/
	unsigned char DD_Speed_dir;/*ËÙ¶È¼Ó¼õËÙ·½Ïò,0--¼ÓËÙ£¬1--¼õËÙ*/
	unsigned char DD_IS_stope;/*ÅĞ¶ÏÊÇ²»ÊÇÖØĞÂÆğÀ´*/
	unsigned char DD_IS_High_speed;/*ÅĞ¶ÏÊÇ²»ÊÇ¸ßËÙÄ£Ê½*/
	unsigned char DD_IS_dir_first;/*ÅĞ¶ÏÊÇ²»ÊÇµ÷Í·Ö®ºóµÚÒ»´Î*/
	unsigned char DD_IS_setting;/*ÕıÔÚÉèÖÃ*/
	unsigned char DD_Needle_idx; /*ÕëÎ»ÖÃ*/

	unsigned char DD_Dont_check_idx;/*¶ÏµçĞøÖ¯ÆğÀ´µÚÒ»´Î²»×ö¼ì²é*/

	unsigned char DD_Stop_immed;	/*Á¢¿ÌÍ£Ö¹*/
	unsigned char DD_is_goto_turnaway;/*¼õËÙµ÷Í·¹ı³Ì0--ÆÕÍ¨£¬1--ÕıÔÚ¼õËÙ£¬2--¼õËÙÍê³É*/
	unsigned char DD_is_overlap_area;/*ÖØµşÇøÓò*/
	

	unsigned char DD_Next_con_enable;/*ÏÂÒ»´ÎµÄÅäÖÃÃüÁîÀ´ÁË£¬ÏÈ±£´æ*/
	unsigned char DD_Next_cmd_CNT;/* ¶ÔÓ¦DD_cmd_CNT*/
	unsigned char DD_Next_Needle_idx; /*ÕëÎ»ÖÃDD_Needle_idx*/
	unsigned short DD_Next_Target_POS;/* ¶ÔÓ¦DD_Target_POS*/
	unsigned short DD_Next_last_cmd_time_100us;/*¶ÔÓ¦DD_last_cmd_time_100us*/

	short DD_New_Target_POS;	  /*ĞÂµÄ0x28 ÃüÁîµÄ*/
	
	
	short DD_cur_cnt_target_pos;/*µ±Ç°ÕâÌõÃüÁî¶ÔÓ¦µÄÄ¿±êÎ»ÖÃ*/
	short DD_Target_POS;		  /*µ±Ç°µÄ¶¯Ì¬¶ÈÄ¿µÄ×îÖÕÄ¿±êÎ»ÖÃ*/

	short DD_Target_arr[3];/*0--µ±Ç°Ä¿±êÖµ£¬1--ÉÏ´ÎÄ¿±êÖµ£¬2--ÉÏÉÏ´ÎÄ¿±êÖµ*/
	
	short DD_last_POS;		/*ÉÏÒ»´ÎµÄÄ¿±êÎ»ÖÃ*/
	short DD_cur_steps;		/*µ±Ç°²½Êı*/
	unsigned short DD_last_cmd_time_100us;		/*Ê±¼äµã*/
	unsigned short DD_last_interval_time_100us;/*¼ä¸ôÊ±¼ä(Ê±³¤)*/
	unsigned short DD_last_interval_time_100us_for_dir;/*µ÷Í·Ç°µÄÄÇ´ÎÊ±¼ä*/

	unsigned short DD_last_i_t_buffer[4];
	unsigned short DD_last_Buf_P;
	
	unsigned short DD_speed_hz_max;/*µ±Ç°×î´óËÙ*/
	unsigned short DD_speed_max_pos;/*µ±Ç°×î´óËÙ¶ÔÓ¦Î»ÖÃ*/
	unsigned short DD_last_speed[16];	/*×îºóĞ´½øµÄËÙ¶ÈÖµ*/
	unsigned short DD_last_wp;			/*×îºóĞ´½øµÄËÙ¶È¶ÔÓ¦ĞòºÅ*/

	/*time = n*b+(0+1+2+3...n-1)d*/
	unsigned int DD_speed_last_part_B;	/*×îºóÒ»¶ÎµÄËã·¨ÖĞµÄBASE  (us)*/
	unsigned int DD_speed_last_part_D;	/*×îºóÒ»¶ÎµÄËã·¨ÖĞµÄd   (us)*/
	unsigned int DD_last_step_idx;
	unsigned int DD_last_steps;
	#ifdef SSTI_PULS_SMOOTH
	unsigned int DD_last_steps_cal_HZ;		/*ÉÏÒ»¸öÖÜÆÚµÄ²½Êı*/
	unsigned int DD_last_time_cal_HZ;	/*ÉÏÒ»¸öÖÜÆÚµÄÊ±¼ä*/
	STEP_MOTOR_SMOOTH DD_Smooth;
	#endif
	

	
	
}Step_DD_CFG;



#ifdef NEW_ALARM_STYLE
#define STEP_ALARM_ZERO_INVALID		(0x0002)
#define STEP_ALARM_ZERO_VALID			(0x0001)
#define STEP_ALARM_RESET_ERROR		(0x0003)
#define STEP_ALARM_WORK_INVALID		(0x1002)
#define STEP_ALARM_WORK_VALID			(0x1001)
#define STEP_ALARM_LOST_NOW			(0x2000)
#define STEP_ALARM_LOST_LAST			(0x3000)
#define STEP_ALARM_OTHER_ERROR		(0x4000)
#endif


struct MOTOR_ATTR_BITS {
   unsigned int is_verystep_enable		:1;		// 0-ÆÕÍ¨µç»ú£¬1--Íâ¹ÒÊ½µç»ú£¬ÄÇÃ´ÕâÀï¾Í²»ĞèÒª¼ÌĞøÖ´ĞĞ¶¯×÷
   unsigned int is_activestep_enable	:1;		// 0--Î´Ê¹ÄÜ¡£1--Ê¹ÄÜ
   unsigned int is_fast_mode			:1;		//0-ÆÕÍ¨Ä£Ê½£¬1--×î¿ìÄ£Ê½ 
   unsigned int check_type				:2;		//0--1--±íÊ¾ÆÕÍ¨´«¸ĞÆ÷Ä£Ê½£¬2--±àÂëÆ÷Ä£Ê½£¬3--´«¸ĞÆ÷+±àÂëÆ÷Ä£Ê½
   unsigned int moto_ecode_index		:4;		//20190926Ö®Ç°5£¬¸Ä³É4// //µç»ú¶ÔÓ¦µÄ±àÂëÆ÷ĞòºÅ
   unsigned int moto_type_config		:4;		////0--Î´¶¨Òå£¬1--¶ÈÄ¿£¬2--Éú¿Ë£¬3--¶¯×÷£¬4--Ñ¹½Å, 5--Ì§Õë
   unsigned int	moto_work_input_index	:5;		//
   unsigned int	moto_zero_input_index	:5;   
   unsigned int moto_type_exe			:3;	//20190926Ö®Ç°2£¬¸Ä³É3	/*Õë¶Ô²»Í¬µç»úÀàĞÍ£¬¸Ã²ÎÊıº¬Òå²»Í¬0--¼òÒ×Èı½Çµç»ú£¬1--HP¿îÈı½Çµç»ú,2--LXÈı½Ç*/
   unsigned int zero_is_positive_dir		:1;	/*±íÊ¾ÁãÎ»ÊÇ·ñÔÚÕı·½Ïò(0-Ä¬ÈÏÔÚ¸º·½Ïò£¬1-Õı·½Ïò)*/
   unsigned int zero_dir_isset			:1;	
   unsigned int zeroPos_Work_ST		:1;				/*ÁãÎ»Î»ÖÃµÄÊ±ºò£¬¹¤×÷Î»×´Ì¬Öµ*/
   unsigned int workPos_Work_ST		:1;				/*¹¤×÷Î»ÖÃµÄÊ±ºò£¬¹¤×÷Î»×´Ì¬Öµ*/
   unsigned int zero_input_NC_cfg	:1;	  /*ÁãÎ»´«¸ĞÆ÷ĞÅºÅ³£¿ª-0£¬³£±Õ-1*/
   unsigned int work_input_NC_cfg	:1;	  /*¹¤×÷Î»´«¸ĞÆ÷ĞÅºÅ³£¿ª-0£¬³£±Õ-1*/
   
   
};

union MOTOR_ATTR_REG {
   unsigned int	all;
   struct MOTOR_ATTR_BITS bit;
};




typedef struct {
/*
*	ËµÃ÷£¬»úÍ·ÏäÉÏÃæµÄµç»úºÅÊÇstepno(0-5)ÎïÀí¶¨ÒåµÄ£¬0xffÎŞĞ§
*			 Ö÷¿Ø¶ÔÓÚ¶ÈÄ¿µç»úµÄÃüÁîÖĞ¶ÔÓ¦µÄµç»úºÅÎª ID_all (0-15)¡£ÒòÎªÓĞĞ©Éú¿Ë»òÑ¹½ÅµÄÃüÁîĞèÒª×ß¶ÈÄ¿µÄÃüÁî
*			Ö÷¿Ø¶ÔÓÚÆäËüµç»úµÄÃüÁî(ÓĞ¶ÀÁ¢ÃüÁîµÄµç»ú£¬±ÈÈçÉú¿Ë£¬Ñ¹½Å£¬Èı½Ç£¬Ö®ÀàµÄ)ÖĞ¶ÔÓ¦µÄµç»úºÅÎªID_self(0-15)¡£
*/


	unsigned char moto_remap_id_all;    //Ö÷¿ØidºÅ_¶ÈÄ¿µÄ½Ó¿Ú
	unsigned char moto_remap_id_self;    //Ö÷¿ØidºÅ_ÆäËüµç»ú×Ô¼ºµÄ½Ó¿Ú
	union MOTOR_ATTR_REG moto_attr;
	#if 0
	unsigned char moto_zero_input_index; 	//
	unsigned char moto_work_input_index; 	//
	unsigned char moto_type_config;		
	unsigned char is_verystep_enable;		
	unsigned char is_activestep_enable;	
	unsigned char is_fast_mode;			
	unsigned char check_type;				
	unsigned char moto_ecode_index;		
	#endif
	
	
} STEP_REMAP_DEF;

typedef struct{
	short		position;
	short		difpos;
	unsigned short	speed;
	unsigned short	step_max;

}STEP_DDM_DEBUG_MSG;


/*´«¸ĞÆ÷×´Ì¬¹ıÂË´ÎÊı(1-7)*/
#define MOTOR_ZERO_ST_FILTER_BITS		4
#define ZERO_ST_BIT_MASK			(~(0xFF<<(MOTOR_ZERO_ST_FILTER_BITS+1)))
#define ZERO_ST_1_BIT_DATA			(~(0xFF<<MOTOR_ZERO_ST_FILTER_BITS))
#define ZERO_ST_0_BIT_DATA			(0x01<<MOTOR_ZERO_ST_FILTER_BITS)

#if 0
#define MOTOR_ZERO_IS_EXCHANGE(iszero_,stbit_) (iszero_?((stbit_ & ZERO_ST_BIT_MASK)==ZERO_ST_1_BIT_DATA)\
													      :((stbit_ &  ZERO_ST_BIT_MASK)==ZERO_ST_0_BIT_DATA))  
#endif
#define MOTOR_ZERO_IS_EXCHANGE(iszero_,stbit_) (iszero_?((stbit_ & 0x1F)==0x0F)\
													      :((stbit_ &  0x1F)==0x10))  

struct STEP_CHK_BITS {
	unsigned int last_zero_bit			:8;	/*8Î»ÓÃÓÚ´æ·Å×îºó8´ÎµÄÁãÎ»×´Ì¬*/
	unsigned int last_zero_st			:1; 	/*ÉÏÒ»´ÎµÄÁãÎ»×´Ì¬*/
	unsigned int chk_gotozero			:1; 	/*½øÁãÎ»*/
	unsigned int chk_leavezero		:1;		/*Àë¿ªÁãÎ»*/ 
	unsigned int chk_st_change		:1;		/*×´Ì¬±ä»¯*/
	unsigned int is_never_leavezero	:1;		/*×ßÍêÁË»¹Ã»Àë¿ªÁãÎ»*/
	//unsigned int check_zero_work		:4;		/*×´Ì¬¼ì²é±êÖ¾*/
	unsigned int check_sts_enable		:1;
	unsigned int check_is_first			:1;			/*¶ÏµçĞøÖ¯ÆğÀ´µÚÒ»´ÎÖ´ĞĞ*/
	unsigned int HP_check_st			:2;			/*ĞèÒª²éÑ¯µÄ×´Ì¬*/
	//unsigned int HP_input_st			:2;			/*HP¿îµç»úµÄÊäÈë´«¸ĞÆ÷×´Ì¬*/
	//unsigned int HP_input_st_last		:2;			/*HP¿îµç»ú¶¯×÷Ö®Ç°µÄ×´Ì¬*/
	unsigned int last_work_bit			:8;	/*8Î»ÓÃÓÚ´æ·Å×îºó8´ÎµÄ¹¤×÷Î»×´Ì¬*/
	unsigned int HP_check_isover		:1;		/*ÅĞ¶ÏÊÇ·ñÉÏÒ»´Î¼ì²éÁËÃ»µ½Î»*/
	unsigned int HP_check_isok		:1;		/*ÅĞ¶ÏÊÇ·ñOK*/
	unsigned int HP_auto_adj			:1;		/*ÊÇ·ñĞèÒª×Ô¶¯½ÓĞøÏÂÒ»¸öÃüÁî*/
	unsigned int LX_ACT_st_ok			:1;		/**/
	unsigned int SK_ADJ_ok				:1;		/*Éú¿ËĞŞÕıÍê³É*/
	//unsigned int is_adj_need_check		:1;/*¶¯×÷µç»ú¼ÓÁË200µÄĞŞÕıÖ®ºó£¬»¹ÊÇĞèÒª¹ıÁãĞŞÕı*/
	unsigned int check_Right_Now		:1;		
	unsigned int Zero_Out_ADJ_OK		:1;		/*Àë¿ªÁãÎ»ĞŞÕı*/ 
	
};

union STEP_CHK_REG {
   unsigned int all;
   struct STEP_CHK_BITS bit;
};




struct STEP_CHK_NOT_CLEAR_BITS {
	unsigned int check_is_first			:1;			/*¶ÏµçĞøÖ¯ÆğÀ´µÚÒ»´ÎÖ´ĞĞ*/
	unsigned int check_zero_work		:4;		/*×´Ì¬¼ì²é±êÖ¾*/
	unsigned int report_after_run		:1;		/*µç»úÔËĞĞÍêÖ®ºóÊÇ·ñ×Ô¶¯ÉÏ±¨*/
	unsigned int report_cnt				:8;		/*µç»úÔËĞĞÍêÖ®ºó×Ô¶¯ÉÏ±¨¼ÆÊı*/
	unsigned int check_zero_work_tmp	:5;		/*×´Ì¬¼ì²é±êÖ¾*/
	unsigned int return_byte			:8;		/*20190130 ±¨¾¯µÄÊ±ºò·µ»Ø¸øÖ÷¿ØµÄÊı¾İ*/
	unsigned int checkisleavezero		:1;		/*¶¯×÷µç»ú¸ºÊıµ½0µÄÊ±ºò£¬ĞèÒª¼ì²éÊÇ·ñÀë¿ªÁãÎ»£¬×öĞŞÕıÓÃ
												±ÜÃâ³¤ÆÚ¸ºÊıµ½Áã×ß¶àÁË¾ÍÎó²îÀÛ»ı*/
	unsigned int YarnStep_can_check		:1;		/*±íÊ¾µç»úÉ´×ì¿ÉÒÔÖ´ĞĞ¼ì²éĞÅºÅÃüÁî*/
	//unsigned int YarnStep_check_runtime	:1;		
	unsigned int res					:2;		/*Ô¤Áô*/ 
};

union STEP_CHK_NO_CLEAR_REG {
   unsigned int all;
   struct STEP_CHK_NOT_CLEAR_BITS bit;
};


struct STEP_ALERT_ST_BITS {
	unsigned char zero_input_st			:1;		/*ÁãÎ»ĞÅºÅÒì³£*/
	unsigned char work_input_st			:1;		/*¹¤×÷Î»ĞÅºÅÒì³£*/
	unsigned char ecord_input_st			:1;		/*±àÂëÆ÷ÊäÈë×´Ì¬Òì³£*/
	unsigned char other_alarm_st			:1;		/*µç»ú²úÉúÆäËü±¨¾¯*/
	unsigned char res						:4;		/*Ô¤Áô*/ 
};

union STEP_ALERT_ST_REG {
   unsigned char all;
   struct STEP_ALERT_ST_BITS bit;
};


struct STEP_ALERT_DELAY_BITS {
	unsigned char new_needcheck_zero			:1;		/*µ±Ç°ĞèÒª¼ì²éÁãÎ»ĞÅºÅ*/
	unsigned char last_needcheck_zero			:1;		/*ÉÏÒ»´ÎĞèÒª¼ì²éÁãÎ»ĞÅºÅ*/
	unsigned char res						:6;		/*Ô¤Áô*/ 
};


union STEP_ALERT_DELAY_REG {
   unsigned char all;
   struct STEP_ALERT_DELAY_BITS bit;
};



	
struct STEP_ST_BITS {
	unsigned int is_poweron			:1; 	/*ÊÇ·ñÔËĞĞ¹ı*/
	unsigned int dir					:2; /*·½Ïò*/
	unsigned int dir_High				:1; /*·½Ïò¶ÔÓ¦µÄµç»úÕı·´×ª*/
	unsigned int step_flags			:1; 	/*¶¯Ì¬¶ÈÄ¿Ó¦´ğ*/
	unsigned int level 					:1;   /*¸ßµÍµçÆ½*/
	unsigned int	running				:3;  /*ÔËĞĞ±êÖ¾*/
	unsigned int	phase				:2;  /*¼Ó¼õËÙ×´Ì¬*/
	unsigned int	zero				:1; /* ÉÏÒ»´ÎµÄÁãÎ»×´Ì¬*/
	unsigned int	zero2_mode			:1;
	unsigned int	zero2_count			:5;/*×î´ó31*/
	unsigned int	zero2				:1;/*¹¤×÷Î»×´Ì¬*/
	unsigned int	zero_cnt			:2; /*×î´ó15*/ 
	unsigned int	check_delay_count	:8;/* ×î´óÖ§³Ö255*///18
	unsigned int IS_Reset_ex			:1;/*¸´Î»×´Ì¬*/
	unsigned int last_dir				:2; /**/
	//unsigned int res					:1;		/*Ô¤Áô*/ 
};

union STEP_ST_REG {
   unsigned int all;
   struct STEP_ST_BITS bit;
};



	
struct STEP_Debug_Test_BITS {
	unsigned char justrun_test			:2; /**/
	//unsigned int res					:1;		/*Ô¤Áô*/ 
};

union STEP_Debug_Test_REG {
   unsigned char all;
   struct STEP_Debug_Test_BITS bit;
};

	

	



typedef struct {
	short		position;
	short		pos_2;
	short 		pos_2_main;
	unsigned char check_pos_index;
	unsigned char alert_delay_max;/*±¨¾¯ÑÓÊ±´ÎÊı£¬Ä¬ÈÏÎª0£¬±íÊ¾Á¢¼´±¨¾¯*/
	unsigned char alert_delay_cnt;/*±¨¾¯ÑÓÊ±´ÎÊı£¬¸ÃÖµ³¬¹ıÉÏÃæÕâ¸ömaxÔò±¨¾¯*/
	unsigned char alert_delay_zerolost_cnt;/*±¨¾¯ÑÓÊ±´ÎÊı£¬¸ÃÖµ³¬¹ıÉÏÃæÕâ¸ömaxÔò±¨¾¯*/

	unsigned char LX_act_just_move_zero;/*by hlc 20190701 Á¬ĞË¶¯×÷µç»ú×ß0 ²»×öĞŞÕı*/

	unsigned char act_is_justcheck_zero;/*¼òÒ×»ú²ÅÓÃµ½£¬Ö»²éÁãÎ»ĞÅºÅ*/

	unsigned char check_signal_edge;/*20191119 */

	unsigned char is_stop_wait;
	unsigned char change_speed;/**/

	unsigned char isdoing_with_bind_cmd;/*20220927 ÖÃ³É1±íÊ¾ÕıÔÚÖ´ĞĞ°ó¶¨ÃüÁî£¬ÆäËüÃüÁîÆÁ±Î*/
	unsigned int	max_speed_back;   	//ÉÏÒ»´Î×î´óÆµÂÊ

	unsigned int yarn_motor_elapse_steps;
	unsigned int	steps;
	
	#ifdef TRY_ZERO_MIDD
	unsigned int	steps_last;
	#endif
	
	unsigned int	step_max;
	unsigned int	speed;
#ifdef STEP_DEBUG_HZ
	unsigned int	speed_HZ_idx;
#endif	
	//unsigned int	speed_div;
	//unsigned int	speed_base;
	unsigned int	max_speed;   	//×î´óÆµÂÊ


	

#ifdef STEP_MOTOR_DDDM_SUPPORT
	 int speed_acc_dec;				//²½³¤ÆµÂÊ	(Ã¿´Î¼Ó¼õ¶àÉÙÆµÂÊ)
	unsigned int low_speed;			//Æô¶¯ÆµÂÊ
	//unsigned int max_speed;			//×î´óÆµÂÊ
	//unsigned int step_flags;
	unsigned int step_is_cnt;
	unsigned int step_is_cnt_old;
#endif

	
	unsigned int	alarm_step;
	
	//unsigned int	dir;
	
	
	unsigned int	state_par;	// 0 => NORMAL; 1 => Go Zero; 2 => Leave Zero

#ifdef DEBUG_ALERT_65
	unsigned int  which_gotozero;
#endif
	int last_stop_systick;
	unsigned int	acc_steps;
	unsigned int	step_wait_time;
	unsigned int 	dir_is_change_;
	unsigned int	step_reset_delay_time;
	//unsigned int	Triangle_sts;
	
	//unsigned int	Base_t_250us; 
	
	unsigned int	step_check_pos;
#ifdef STEP_TEST_IN_NDL
	unsigned int	needle;
#endif
	unsigned int	step_check_interval;

#ifdef ZERO2_SUPPORT
	//unsigned int	zero2_mode;
	//unsigned int	zero2;
	//unsigned int	zero2_count;
#endif
	//unsigned int step_stop_long;

	//unsigned int	work_position_count;


//#ifdef TRIANGLE_STEP_SUPPORT
	//unsigned int done_steps;
	//unsigned int steps_go_run;

	unsigned int change_dir_count;
	
	//unsigned int last_rest;
	//unsigned int dir_steps;

	#ifdef DEBUG_STEP_RESET_FAST_SPEED
	unsigned int reset_dir_change_cnt;/**/
	#endif
		
//#endif


#ifdef E480_BOARD_V10
	unsigned int moto_zero_width;          /*Ö÷¿ØÉèÖÃÏÂÀ´µÄÖµ£¬¾ÍÊÇµ½ÁËÁãÎ»Ö®ºóÔÙ×ß¶àÉÙËãÖĞ¼ä,É´×ìµç»ú±È½ÏÌØÊâ*/
	unsigned int moto_zero_width_self;		/*µç»ú¸´Î»µÄÊ±ºò×Ô¶¯¼ÆËãµÄÒ»¸öÖµ*/
	unsigned int moto_zero_width_temp;	
	unsigned int need_2_pos_after_reset;		/*ÏÈ¸´Î»£¬ÔÙ×ßµ½Õâ¸öÎ»ÖÃ*/
#endif
	//unsigned int need_2_pos_maxspeed;

	//unsigned int is_poweron;
	unsigned int error_count_check;
	int input_errorstep;		//´«¸ĞÆ÷Îó²î(½ø³ö´«¸ĞÆ÷µÄÊ±ºò£¬²½Êı²î)
	
	unsigned int steps_go_;			//×ßÁË¶àÉÙ²½ÁË
	unsigned int steps_check_zero;	//ÁãÎ»¼ì²é·¶Î§(ÈËÎª¼ÓÒ»¸ö·¶Î§Öµ£¬Ä¬ÈÏÎª0)
	//unsigned int steps_go_leftright;	//×ßÁË¶àÉÙ²½ÁËµ½×ó±ß»òÓÒ±ß
	unsigned int steps_go_temp;		//ÄÚ²¿¼ÆÊıÆ÷
	
	
	unsigned int state_chi;			//0,µÚÒ»´Î½ø0Î»£¬1-Àë¿ª0Î»£¬2£,µÚ¶ş´Î½ø0Î»
	unsigned int state_chi_last;			//ÉÏÒ»´ÎµÄ×´Ì¬
	
	unsigned int last_new_input_sts;			//´«¸ĞÆ÷£¬×îºóµÄ×´Ì¬ºÍ×îĞÂµÄ×´Ì¬£¬·Ö±ğÕ¼Á½Î»bit0-1,last,bit2,3new 0,2:zero.1-3:work,bit4 ±íÊ¾ÊÇ·ñ±ä»¯

	
	STEP_REMAP_DEF moto_remap_config;
	#ifdef STEP_DEBUG_DDM
	unsigned int step_poslistid;
	STEP_DDM_DEBUG_MSG Stepdebugmsg[4];
	#endif
	short check_work_pos;
	short step_input_nc_cfg;
	short HF_ex_cnt;/*20190507*/
	//#ifdef LX_ACTION_STEP_SPECIAL
	
	//unsigned char check_zero_work;
	//unsigned char check_zero_work_Timer_enable;
	//unsigned char adj_lost_steps;
	//unsigned char need_check_zero_sts_change;
	//unsigned char need_leave_zero;
	//unsigned char last_zero_act;
	unsigned char DD_error_cnt;/*¶ÈÄ¿µç»ú0-8±¨¾¯ÂË²¨¼ÆÊıÆ÷*/
	unsigned char Had_next_cmd;
	Step_DD_CFG moto_dd_config;
	union STEP_CHK_REG chk;
	union STEP_ST_REG  step_st;	
	union STEP_CHK_NO_CLEAR_REG st_no_cl;
	union STEP_ALERT_ST_REG step_alert_st;/*Õâ¸öÓÃÓÚÖ÷¶¯ÉÏ±¨µç»ú×´Ì¬*/
	union STEP_ALERT_DELAY_REG step_alert_delay_pos2_st;
	union STEP_Debug_Test_REG step_debug_test_st;
	//#endif
	


}STEP_TYPE;


typedef struct {
	unsigned short cmdID;	/*ÃüÁîĞòºÅ*/
	unsigned char Motor_NO1;/*µç»úºÅ1*/
	unsigned char Motor_NO2;/*µç»úºÅ2*/
	short Targer_pos_1;	/*µç»úºÅ1¶ÔÓ¦Ä¿±êÎ»ÖÃ*/	
	short Targer_pos_2;	/*µç»úºÅ2¶ÔÓ¦Ä¿±êÎ»ÖÃ*/
	unsigned short Delay_timeout_2ms;/*ÑÓÊ±Ê±¼ä*/
	unsigned short Exec_TimeOut;	/*¶¯×÷Ö´ĞĞ³¬Ê±Ê±¼ä*/
	unsigned char Cmd_step;/*µ±Ç°Ö´ĞĞµ½µÚ¼¸²½:
							0--ÎŞÃüÁî
							1--ÊÕµ½ÃüÁî
							2--ÕıÔÚ¹éÁã
							3--¹éÁãÍê³É
							4--ÕıÔÚÖ´ĞĞ
							5--Ö´ĞĞÍê³É*/ 
							
	

}STEP_BIND_cmd;




/*¶ÈÄ¿µç»úÁãÎ»¼ì²é±¨¾¯ÖÍºó¼¸´Î±¨¾¯*/
#define STEP_MOTOT_CHECK_ZERO_DELAY_ALERTCNT 1


#define SPET_MOTOR_MIN_SPEED_HZ	(500)
#define SPET_MOTOR_MAX_SPEED_HZ	(20000)


#define SPET_MOTOR_MIN_SPEED_DD_HZ	(500)
#define SPET_MOTOR_MAX_SPEED_DD_HZ	(10000)




/*
#ifdef TRIANGLE_STEP_SUPPORT
#define TRIANGLE_UNKNOW_ZERO	(1)
#define TRIANGLE_ZERO_ZERO		(2)
#define TRIANGLE_NEGATIVE_ZERO	(3)
#define TRIANGLE_PLUS_ZERO		(4)
#define TRIANGLE_ZERO_NEGATIVE	(5)
#define TRIANGLE_ZERO_PLUS		(6)
#define TRIANGLE_NEGATIVE_PLUS 	(7)
#define TRIANGLE_PLUS_NEGATIVE 	(8)


#endif
*/

// STEP reset flag 
#define JUST_RUN 				(0)
#define GOTO_ZERO				1
#define LEAVE_ZERO				2
#define LEAVE_STEP				3
#define MOVETO_ZERO			4
#define DETECT_ZERO2			5
#define GOTO_ZERO2	 			6
#define GOTO_ZERO3	 			61

#define CHECK_SIGN_WIDTH 		7
#define STEP_RESET 				8
#define ENCODER_LEAVE_ZERO	9
#define STEP_RESET_HP_ACT 		10
#define JUST_RUN_GO 			11
//#ifdef LX_ACTION_STEP_SPECIAL
#define MOVETO_ZERO_LX_ACT	12
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO1	121
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO2	122
#define MOVETO_ZERO_LX_ACT_CROSS_ZERO3	123

//#endif

#define GOTO_ZERO_TRY 			13
#define MOVETO_ZERO_LIFT 		14
#define CHECK_SIGN_DEVIATION	(15)
#define FEET_STEP_ISWORK		(16)
#define HP_STEP_CHECK_INPUT	(17)

#define GOTOZERO_LX_ACT		(18)
#define LEFT_EX_ISWORK          (161)
//#define LEFT_EX_ISWORK2          (162)



#ifdef TRY_ZERO_MIDD
#define GOTO_ZERO_TRY_ZERO_MIDD	(19)		/*À´»Ø¼ì²éµç»úÁãÎ»ÔÚÊ²Ã´Î»ÖÃ*/

#define GOTO_TRY_ZERO_STEPS	50			/*Ã¿´Î³¢ÊÔ×óÓÒÍØÕ¹Ñ°ÕÒÁãÎ»Î»ÖÃµÄ²½Êı*/

#endif

#define JUST_RUN_RESET 				(20)
#define ENCODER_JUST_RUN			21

#define JUST_RUN_AFTER_ADJ 			30

//#ifdef TRIANGLE_STEP_SUPPORT

/*ÎŞÂÛÈçºÎÒª×ßÕâÃ´Ğ©²½Êı²ÅÈÏÎªÊÇ±¨¾¯À´ÁË*/
#define STEP_GO_ALARM_STEPS	4

/*ÎŞÂÛÈçºÎÒª×ßÕâÃ´Ğ©²½Êı²ÅÈÏÎªÊÇ¿ÉÒÔµ÷Í·ÁË*/
#define STEP_GO_DIR_STEPS	80
#define STEP_GO_DIR_STEPS_MIN	10

/*µç»ú¼Ó¼õËÙ²½Êı*/
#ifdef FOR_BW_DM_SLOWLYRUN
#define STEP_DDM_STEPS		(32)  //(16)  
#else
#define STEP_DDM_STEPS		(16)  
#endif
//#ifdef LX_ACTION_STEP_SPECIAL

//#define STEPS_LEAVEZERO_FOR_LX	(100)  
/*20180723 ĞŞ¸Ä³É150 ´ÈĞÇÌØÊâ*/
#define STEPS_LEAVEZERO_FOR_LX	(200)    
/*Á¬ĞËÌØÊâµÄÖµ*/
/*´ÓÕıµ½¸ºÔÙ×ß10¸öµã£¬´Ó¸ºµ½ÕıÉÙ×ß10¸öµã*/
#ifdef TEST_CX_ACT
#define STEP_ZERO_GOON_STEPS	(-1)
#else
#define STEP_ZERO_GOON_STEPS	(3)
#endif

#define HP_STEP_MOTOR_CHECK_INPUT_AREA	(100)   /*HP¿î¶¯×÷µç»ú£¬Ã¿´Î¶¯×÷ÌáÇ°¼ì²éµ½Î»µÄ²½Êı£¬¡ÀÕâ¸öÖµ*/

//#endif

#define ALERT_DELAY_CNT_DEFAULT		5 /*Ä¬ÈÏ¼¸´Î±¨¾¯Ö®ºó²Å±¨¾¯*/

#define STEPS_LEAVEZERO	(30)//Ä¬ÈÏÀë¿ª0Î»µÄ²½Êı 

#define STEPS_LEAVEZERO_SMALL	(25)//Ä¬ÈÏÀë¿ª0Î»µÄ²½Êı 


#define STEPS_ERROR_RANGE	(5)	//ÁãÎ»Æ«ÒÆÎó²î×î´ó²½


#define STEPS_ZERO_GOBACK	(0)  /*µç»ú±àÂëÆ÷×Ô¶¯ÕÒÁãÖ®ºóÍù»Ø×ß¶àÉÙ²½,Ä¬ÈÏ0*/

#define GOTO_LEFT	8		//×ßµ½01Î»ÖÃ(-300)
#define GOTO_RIGHT	9		//×ßµ½10Î»ÖÃ(300)
#define CHECK_IS_LEFT 10     //¼ì²é01Î»ÖÃÊÇ²»ÊÇÕæµÄµ½ÁË
#define CHECK_IS_RIGHT 11   //¼ì²é11Î»ÖÃÊÇ²»ÊÇÕæµÄµ½ÁË
#define CHECK_ISNOT_LR 12     //¼ì²éÊÇ²»ÊÇÕæµÄÀë¿ªÁËÁ½±ß

#define TEST_SIGN_WIDTH  0x20   			//¼ì²â´«¸ĞÆ÷¿í¶È
#define TEST_SIGN_WIDTH_LEAVE  0x21   //¼ì²â´«¸ĞÆ÷¿í¶ÈÀë¿ªÖĞ¼ä
#define TEST_SIGN_WIDTH_IN 0x22   		//»Øµ½´«¸ĞÆ÷ÖĞ¼ä
#define TEST_SIGN_WIDTH_LEAVE_EX 0x23  



#define TRIANGLE_LEFT_POSTION 	(0-240)
#define TRIANGLE_RIGHT_POSTION 	(240)
#define TRIANGLE_ZERO_POSTION 	(0)

#define DEF_CHECK_WORK_POS  (2000)

//#endif


enum {
	HEAD_MODE_DEFAULT = 0,		/*4¶ÈÄ¿+2Èı½Ç+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/
	HEAD_MODE_SIMPLE,			/*4¶ÈÄ¿+2Èı½Ç+2¿Õ°×+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/
	HEAD_MODE_SKMODE2,			/*4¶ÈÄ¿+2Éú¿Ë+2¿Õ°×+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/
	HEAD_MODE_SKMODE4,			/*4¶ÈÄ¿+4Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/
	HEAD_MODE_LIFT2,			/*4¶ÈÄ¿+2Èı½Ç+2Ì§Õë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/	// ADD by zhuqiwei 160601
	HEAD_MODE_LIFT_EX,		/*4¶ÈÄ¿+2Èı½Ç+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)+2Ì§Õë*/ //E490_v1.0
	HEAD_MODE_FEET,			/*4¶ÈÄ¿+2Ñ¹½Å+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)    ¹ú¹â*/
	HEAD_MODE_LIFT_HP,		/*4¶ÈÄ¿+2Èı½Ç(HP)+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)+2Ì§Õë*/ //20160816
	HEAD_MODE_LX_ACT,			/*4¶ÈÄ¿+2Èı½Ç(LX)+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)+2Ì§Õë*/ // 2016 11 29 
	HEAD_MODE_FH_ACT,		/*4¶ÈÄ¿+2Èı½Ç(FH)+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)+2Ì§Õë*/ // 2016 11 29 
	//HEAD_MODE_LIFT3,			/*4¶ÈÄ¿+2Èı½Ç+2Éú¿Ë+2ÍÆÕë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)*/
	HEAD_MODE_FEET_CX,		/**4¶ÈÄ¿+2Ñ¹½Å+2Éú¿Ë+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)    +2Ì§Õë---´ÈĞÇ20171122*/
	HEAD_MODE_ALL_STEP_CX,	/**4¶ÈÄ¿+2Èı½Ç+2Ñ¹½Å+4É´×ì(Èç¹ûÓĞµç»úÉ³×ìµÄ»°)    +2Ì§Õë+À©Õ¹2Éú¿Ë---´ÈĞÇ20181129*/
	HEAD_MODE_MAX,
};

/*E480--×î¶àÖ§³Ö12¸öµç»ú£¬Ä¬ÈÏ0Ä£Ê½Ö§³Ö0,1,2,3 ËÄÖÖÄ£Ê½£¬E475--×î¶àÖ§³Ö6¸öµç»ú£¬Ä¬ÈÏ2Ä£Ê½£¬Ö§³Ö1,2Á½ÖÖÄ£Ê½*/

enum {
	MOTOR_TYPE_UNDEF = 0,	//0-Î´¶¨Òå 
	MOTOR_TYPE_DENSITY,	// 1-¶ÈÄ¿
	MOTOR_TYPE_SKINER,	// 2-Éê¿Ë
	MOTOR_TYPE_ACTION,	// 3-¶¯×÷
	MOTOR_TYPE_FEET,		// 4-Ñ¹½Å
	MOTOR_TYPE_YARN,		// 5-É´×ì
	MOTOR_TYPE_LIFT,		// 6-Ì§Õë
	MOTOR_TYPE_OTHER,		//7-ÆäËüµç»ú(ÕâÀàµç»ú¹¤×÷Ä£Ê½ÀàËÆÓëÉú¿Ëµç»ú)
	MOTOR_TYPE_UPDOWN,	//8-´ÈĞÇÉ³×ìÉÏÏÂÔË¶¯µç»ú£¬Ã¿ÏµÍ³4¸ö£¬½ÓÔ­ÏÈÉ³×ìµç»ú½Ó¿Ú
	MOTOR_TYPE_MAX,		//×î´óÀàĞÍºÅ
};
#define MOTOR_TYPE_COUNT	(MOTOR_TYPE_MAX-1)			//ËùÓĞµç»úÀàĞÍÊıÁ¿
#define MOTOR_TYPE_0XFF	(0xFF)				/* µç»úÀàĞÍÎª0XFFµÄÊ±ºòÖ±½Ó¶ÔÎïÀíID²Ù×÷*/

#define MOTOR_TYPE_EXE_PT		0

enum{
	DENSITY_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,
	DENSITY_MOTOR_TYPE_JF,
	DENSITY_MOTOR_TYPE_LX,
	DENSITY_MOTOR_TYPE_OTHER,	
};

enum{
	ACT_MOTOR_TYPE_PT =MOTOR_TYPE_EXE_PT,   /*0: ÆÕÍ¨µÄ°ë±ßÁÁ°ë±ß²»ÁÁ*/
	ACT_MOTOR_TYPE_HP,		/*1: HP¿î£¬Á½¸öĞÅºÅ×éºÏÓÃ*/
	ACT_MOTOR_TYPE_LX,		/*2: LX¿î£¬·­ÕëÎ»ÓĞÈ±¿ÚµÄ(FĞÍ)*/
	ACT_MOTOR_TYPE_FH,		/*3: FH¿î£¬ÔÚÁ¬ĞË»ù´¡ÉÏ£¬·­ÕëÎ»ÁíÍâ¼ÓÁËÒ»¸öĞÅºÅ¼ì²é*/
	ACT_MOTOR_TYPE_DOT,		/*4: ĞÅºÅÔÚÖĞ¼äÒ»¸öµãµÄÄ£Ê½*/
};

enum{
	LIFT_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	LIFT_MOTOR_TYPE_HF,                /*Ö§³ÖºÆ·áÄ£Ê½*/	
	LIFT_MOTOR_TYPE_HF_EX,	 /*ÖĞ¼äÃ»ĞÅºÅ£¬Á½±ßÓĞĞÅºÅ£¬ÕÒÁãµÄÊ±ºòĞèÒª¼ÆËãÖĞ¼ä¿Õ°×¿í¶È´óĞ¡*/
};

enum{
	SINKER_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	SINKER_MOTOR_TYPE_LX,                /*Ö§³ÖÁ¬ĞËÄ£Ê½*/	
	SINKER_MOTOR_TYPE_LX_EX,		/*Á¬ĞËÄ£Ê½»ù´¡ÉÏÓÖÔö¼ÓÒ»ÖÖ*/

};



enum{
	FEET_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	FEET_MOTOR_TYPE_CX,                /*Ö§³Ö´ÈĞÇÄ£Ê½¾ÍÊÇ¹¤×÷Î»×ßÍêÔÙ¼ì²â*/	
	FEET_MOTOR_TYPE_CX2,		  /*Ö§³Ö´ÈĞÇÄ£Ê½--°´ÕÕÎ»ÖÃ×ß²½Êı*/
	FEET_MOTOR_TYPE_CX3		/*Ö§³Ö´ÈĞÇÄ£Ê½--°´ÕÕĞÅºÅ×ß£¬¹¤Î»ĞèÒªĞŞÕı*/	
};


enum{
	YARN_MOTOR_TYPE_PT=MOTOR_TYPE_EXE_PT,
	YARN_MOTOR_TYPE_LX,                /*Ö§³ÖÁ¬ĞËÄ£Ê½µÄµç»úÉ´×ì£¬ÀàËÆÍÆÕëµç»ú£¬ÖĞ¼äÁÁ(ĞÅºÅÔÚÖĞ¼ä)£¬Á½Í·ÊÇÎ»ÖÃ(¹¤×÷Î»ºÍ»úĞµÁãÎ»)*/	
};






// STEP running flag 
#define RUNNING		1
#define RUNNING_OVER	2
#define RUNNING_GO	3
#define RUNNING_END	4

#ifdef LOG_DEBUG_FOR_LX_AT_CH
enum{
	LX_LOG_TYPE_ZERO_ADJ_GOTO=0, /*ÁãÎ»ĞŞÕı½ø 0*/
	LX_LOG_TYPE_ZERO_ADJ_GOTO_2, /*ÁãÎ»ĞŞÕı½ø1*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE, /*ÁãÎ»ĞŞÕı³ö2*/
	LX_LOG_TYPE_ZERO_ADJ_LEAVE_2, /*ÁãÎ»ĞŞÕı³ö3*/
	
	LX_LOG_TYPE_MOVETOZERO_ADD400, /*×ßÁãÎ»¼Ó×ß400²½4*/
	LX_LOG_TYPE_MOVETOZERO_ADD400_2, /*×ßÁãÎ»¼Ó×ß400²½5*/
	LX_LOG_TYPE_LEAVEZERO_ADD200, /*Àë¿ªÁãÎ»¼Ó×ß200²½6*/
	LX_LOG_TYPE_LEAVEZERO_ADD200_2, /*Àë¿ªÁãÎ»¼Ó×ß200²½7*/
	LX_LOG_TYPE_RUNNING_OVER,	/*Á½¸öÃüÁî½ÓĞøÁË8*/
	LX_LOG_TYPE_RUNNING_OVER_2,	/*Á½¸öÃüÁî½ÓĞøÁË9*/
	LX_LOG_TYPE_STOP_GO,	/*Í£Ö¹ÁËµ«ÊÇÎ»ÖÃÖµºÍÄ¿10±êÖµ²½ÏàµÈ£¬¼ÌĞø×ß*/
	LX_LOG_TYPE_STOP_GO_2,	/*Í£Ö¹ÁËµ«ÊÇÎ»ÖÃÖµºÍ11Ä¿±êÖµ²½ÏàµÈ£¬¼ÌĞø×ß*/
	LX_LOG_TYPE_ZERO_CHANGE,/*ÁãÎ»ĞÅºÅ±ä»¯12*/	
	LX_LOG_TYPE_ZERO_W,/*ÁãÎ»ĞÅºÅ¿í¶È13*/	
	LX_LOG_TYPE_ADJ_ISOK_BUT,/*ÁãÎ»À´ÁË14¬µ«ÊÇĞŞµÄÌõ¼şÓÖÀ´ÁË¡£*/
	LX_LOG_TYPE_ADJ_ISOK_BUT_2,/*ÁãÎ»À´ÁË£¬15µ«ÊÇĞŞµÄÌõ¼şÓÖÀ´ÁË¡£*/
};
#define ADJ_NEED_LOG_STEPS	(20)		/*ĞŞÕı³¬¹ıÕâ¸ö²½Êı²Å±¨¾¯*/

#endif



void StepMotor_Init(void);
void StepMotor_Set_ZeroAdj(unsigned int adj);
void StepMotor_Set_ZeroDetect(unsigned int area);
void StepMotor_Setup_Active(unsigned int active,unsigned char islog);
void StepMotor_Set_VeryMode(unsigned int verymask,unsigned char islog);
void StepMotor_Setup_Direction(unsigned int dir);
void StepMotor_Setup_Resolution(unsigned int mototype,unsigned int rlt);
void StepMotor_Set_Mode(unsigned int mode);
void StepMotor_Set_FastMode(unsigned int stepno,unsigned char islog);
void StepMotor_Set_Reset_Speed(unsigned int speed,unsigned int mototype);
void StepMotor_Set_Speed(unsigned int speed,unsigned int steptype);
unsigned int StepMotor_Set_SinkerSpeed(unsigned int speed);
void StepMotor_Reset(unsigned int stepno,unsigned short otherarg);
void StepMotor_Reset_LeftMotor(unsigned short otherarg);
void StepMotor_Reset_RightMotor(unsigned short otherarg);
void StepMotor_All_Reset(unsigned short otherarg);
void StepMotor_Start(unsigned int stepno);
void StepMotor_Set_Position(unsigned int stepno, short pos);
void StepMotor_Modfiy_Position(unsigned int stepno, short pos, int power);
short StepMotor_Get_Position_2(unsigned int stepno);
short StepMotor_Get_Position(unsigned int stepno);
unsigned int StepMotor_Get_Busyidx(unsigned int stepno);

unsigned int StepMotor_get_IDall_with_IDself(unsigned int step_IDself,unsigned char step_type);


void StepMotor_exec(unsigned int stepno, short pos, int mode,unsigned short otherarg,unsigned char workspeed);
void StepMotor_isr_exec(unsigned int stepid,short pos,int mode,unsigned short otherarg);

void StepMotor_Set_Check(unsigned int step);

void StepMotor_Feet_Setup(int cmd, unsigned int arg);
//int StepMotor_Sinker_Get_Busy();
//#ifdef LX_ACTION_STEP_SPECIAL
void StepMotor_Type_exec(int mid, short arg, unsigned char step_type,unsigned char check,unsigned char wspeed,unsigned char justcheckalert,unsigned char mz);
//#else
//void StepMotor_Type_exec(int mid, short arg, unsigned char step_type);
//#endif
void StepMotor_Type_Reset(unsigned int mid, unsigned char step_type,unsigned short otherarg);
void StepMotor_Type_ResetALL(int step_type,unsigned short otherarg);
void StepMotor_Type_Set_Position(unsigned int sk_id, short pos, unsigned char step_type);

void StepMotor_Set_Reset_Delay_Time(int stepno, unsigned int delay);

void StepMotor_Setup_Para(int cmd, int para);

// ÓÃÓÚ²âÊÔµç»úÊÇ·ñµ½Î»
void StepMotor_debug(void);

int check_sinker_done(void);
int check_sti_done(int leftOrRight);
void StepMotor_Poll(void);
void StepMotor_Isr(unsigned int stepno);
void StepMotor_yarn_Isr(unsigned int stepno);

void StepMotor_Set_with_Head_Mode(unsigned int mode);
unsigned char Stepmotor_get_Head_Mode(void);
void Exec_Set_Step_check_Type(unsigned int whichstep,unsigned int stepno,unsigned char c_type);

void StepMotor_Set_work_steps(unsigned char steptype,unsigned  int steps,unsigned char whichstepid);

void Step_Set_work_check_pos(unsigned char mid,short pos,unsigned char step_type);
void StepMotor_afterrun_toreport(unsigned int stepno,unsigned char islxsk);
void Exec_Reset_step_moto_one(int arg2,unsigned short otherarg);
void Exec_Reset_step_moto_all(int arg2,unsigned short otherarg);

#ifdef STEP_MOTOR_DDDM_SUPPORT
void StepMotor_exec_new_dd(unsigned int stepno, short pos, int time, int mode, int maxspd,char isnewcmd);
void StepMotor_isr_exec_new_dd(unsigned int stepid,short pos,int time,int mode,int maxspeed,char isnewcmd);
#endif

void StepMotor_justrun(unsigned int stepno,unsigned char needreturn, short pos);


void StepMotor_Isr_call(unsigned int stepno);

void Motor_Set_DM_Type(unsigned char steptype,unsigned char dm_type);

void StepMotor_Set_zero_dir(unsigned char whichtype,unsigned short stepno,unsigned short dirdata);

int is_Feet_Step(int stepno);

void Set_step_alert_st_ecoder(unsigned char stepno,unsigned char st);

void StepMotor_other_Get_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short *zw,short *iner);

void StepMotor_other_Set_Zerowide_inputerror(unsigned char step_type ,unsigned int ACT_id,short zw,short iner);

void Stepmotor_Set_ID_Type_with_NO(unsigned char stepno,unsigned int idself,unsigned int type);
void StepMotor_Set_Zero_go_Steps_ex(unsigned int steptype, short gos);

void check_lift_step_isstop(void);
unsigned char Get_StepMotor_isveryenable_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_zeroID_with_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_workID_with_IDself(unsigned int step_IDself,unsigned int step_type);
unsigned int StepMotor_get_no_with_IDself(unsigned int step_IDself,unsigned char step_type);
void check_step_motor_loop_(void);
void Exec_Set_Step_Parameter_with_motortype(unsigned char para_type,unsigned char motortype_main,unsigned short mask_,unsigned short data_);
void Set_Motor_steps_for_ACC_DEC(unsigned short acc_steps);
void Get_Step_data(void);
void Stepmotor_Set_inputID_with_NO(unsigned char stepno,unsigned char in_zero,unsigned int id_work,unsigned int zw_mask);
unsigned char Get_StepMotor_isveryenable_IDall(unsigned int step_idall);
void StepMotor_Set_Input_error(unsigned int stepno,unsigned int inerr);
void StepMotor_Setup_Direction_log(unsigned int dir);
void Step_Motor_Set_Check_all(unsigned char whichtype,unsigned int step);
void StepMotor_Set_Zero_go_Steps(unsigned int steptype,unsigned int gos);
void StepMotor_Set_zero_type_(unsigned int steptype,unsigned char whichstep,unsigned int zt);
void StepMotor_Set_zero_detect(unsigned char whichtype,unsigned int step);
void StepMotor_Set_movetozero_adv_steps(unsigned short steps,unsigned short motortype );

void StepMotor_Set_Speed_EX(unsigned char motortype,unsigned char whichspeed,unsigned int speed_hz);
void Exec_Set_DDM_need_Check(unsigned int ischeck);

void Motor_Set_minsteps_ecode_rang(unsigned char steptype,unsigned int data);
void Motor_Set_DM_check_area( short max_a, short min_a);
void Motor_Get_All_postion(unsigned int ispos2,unsigned short *posdata);
void StepMotor_Other_SetSpeed(unsigned char steptype,unsigned char whichspeed,unsigned int speeddata);
void StepMotor_Other_Set_isfast(unsigned char steptype,unsigned int isfast);
void StepMotor_Other_Set_Work_Enable(unsigned char steptype,unsigned int isenable);
void StepMotor_Setup_ACTION_Para(int cmd, int para);

void Exec_Triangle_step(unsigned int Tid,  int Pos,unsigned int alarmenable);
short StepMotor_Triangle_Get_Position(unsigned int ACT_id);
short StepMotor_Triangle_Get_Sign(unsigned int ACT_id);

void StepMotor_Action_Set_Position(unsigned int ACT_id,short pos);
void StepMotor_Triangle_Set_Sign(unsigned int sk_no,short pos);
void Motor_Set_maxsteps_cw(unsigned char steptype,unsigned int data);

void Set_Step_Motor_postion_ex(unsigned char steptype,unsigned char stepno,unsigned char dataindex,short data_pos );
void Set_Step_Motor_TO_Which_Postion(unsigned char steptype,unsigned short dataindex);
void StepMotor_Set_ZeroPos_WorkST(unsigned int steptype,unsigned char wst);
void StepMotor_Add_curr(unsigned int stepno);
void StepMotor_setMaxSpeed(unsigned int stepno,unsigned int maxspeed);
int StepMotor_Get_Running(unsigned int stepno);
void StepMotor_reback_curr(unsigned int stepno);
void StepMotor_setpos_2main_postion(unsigned int stepno);

void Get_Step_debug_msg(unsigned short whichstep);

void StepMotor_Setup_Yarnstep_Active(unsigned int yarnstep_active);

void arch_YARN_Step_Setup(int yno, int onoff);
int is_zero2_support(unsigned int stepno);
int is_LX_DENSITY_Step(int stepno);
int is_HF_LIFT_Step(int stepno);
int is_EX_OTHER_Step(int stepno);
int is_ACTION_Step(int stepno);
int is_HP_ACTION_Step(int stepno);
int is_PT_ACTION_Step(int stepno);
int is_LX_ACTION_Step(int stepno);
int is_LIFT_or_Other_Step(int stepno);
int Get_st_with_postion(short arg);
int is_FH_ACTION_Step(int stepno);
int is_DENSITY_Step(int stepno);
int is_LIFT_Step(int stepno);
int is_JF_DENSITY_Step(int stepno);
int is_lx_sinker(int stepno);
int is_YARN_Step(int stepno);
int is_PTorFH_ACTION_Step(int stepno);
int check_HF_Lift_input_error(STEP_TYPE *Step);
int is_lx_sinker_ex(STEP_TYPE *Step_ex);
int is_LX_DENSITY_Step_ex(STEP_TYPE *Step_ex);
int is_LIFT_all_Step(int stepno);
void Set_LX_ex_adjData(unsigned char steptype,unsigned short data);
void StepMotor_Speed_report(unsigned short retcmd,unsigned char typemain,unsigned char steptype);
void StepMotor_input_NCorNO_set(unsigned char steptype,unsigned char stepno,unsigned char ncno);
void StepMotor_Set_Ecoder_dir_withzerodir();

#endif

