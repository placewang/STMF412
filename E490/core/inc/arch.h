

#if 1
typedef struct {
	unsigned char jqd_no;
	unsigned char bladeno;
	unsigned char do_st;
}JQD_BLADE_TYPE;

#endif

void arch_StepMotor_Enable(void);
void arch_StepMotor_Disable(void);
void arch_StepMotor_Half(int stepno, int onoff);
void arch_StepMotor_Dir(int stepno, int onoff);
void arch_StepMotor_Pulse(int stepno, int onoff);
unsigned int arch_StepMotor_Zero(unsigned int stepno);

void arch_StepMotor_Set_Speed(unsigned int stepno, unsigned int speed);
void arch_StepMotor_Start(unsigned int stepno);
void arch_StepMotor_Active(unsigned int stepno,unsigned int add_speed);
void arch_StepMotor_Stop(unsigned int stepno);

void arch_YARNEMF_Setup(int yno, int onoff,int check);
void arch_YARNEMF_Clear(int yno);
void arch_YARNEMF_AllClear(void);

void arch_ACTEMF_Setup(int actno, int onoff,int check);
void arch_ACTEMF_Clear(int actno);
void arch_ACTEMF_AllClear(void);

int arch_Jacquard_Setup(int jqdno, int blade, int onoff,int check,char idx);
void arch_Jacquard_ClearBlade(int jqdno, int blade);
void arch_Jacquard_Clear(int jqdno);
void arch_Jacquard_AllClear(void);

void arch_Power_On(void);
void arch_Power_Off(void);

unsigned int arch_Get_DirStatus(void);
unsigned int arch_GetKey(void);
unsigned int arch_Get_Mode(void);

void arch_init(void);

void wait_ms(unsigned int ms);
void wait_us(unsigned int us);

unsigned int arch_Get_Board_Type(void);

unsigned int Scan_error(void);

unsigned int Get_ErrorStatus(void);
void Upgrade_Start(void);

int Jtag_Security_State(void);

void OverLoad_Setup(int onoff);


void Get_Head_config_Msg(unsigned int rtcdata,unsigned char whichCFG);

void SSI_Select_ecode(unsigned int s_id);
void SSI_DeSelect_ecode(unsigned int s_id);
unsigned int arch_get_ticktime(void);
void arch_shock_timer(int whichshock);
void arch_Jacquard_PWMBlade(int jqdno, int blade);
unsigned int arch_StepMotor_work(unsigned int stepno);
unsigned int arch_get_dir1(void);
unsigned int arch_get_dir2(void);
unsigned int arch_Get_ID(void);
void Set_check_mainbinding_timedelay(unsigned int delay_time_m);
void Exec_check_is_binding_ok(unsigned char istimercheck,unsigned char ispass);
void check_fan_ctr(void);
void arch_set_Step_SK_A(unsigned int cruu);
unsigned int arch_is_EMF_2_SK_board(void);

unsigned char arch_is_Ecode_board(void);
unsigned char arch_check_power_isok(unsigned char whichpower);
void arch_Power_ctrl_ex(unsigned char whichpower,unsigned char onoff,unsigned char which_cp);
void Exec_write_main_id_to_flash(unsigned short *mainid,unsigned char whichid);
void  Exec_Set_main_id_(unsigned short *mainid,unsigned char whichid);
void  Exec_Send_main_flash_id_(unsigned short cmdret,unsigned char whichid);
void  Exec_Get_head_id_(unsigned short cmdret,unsigned char whichid);
void Exec_Get_Binding_st_(unsigned short cmdret);
unsigned int arch_get_ad_base_cur_zero(unsigned int NorP);
void arch_shock_reset(int whichshock);
void arch_shock_activate(int onoff,int whichshock);
void arch_send_Operate_data(void);
void Overload_Init(void);
int Overload_recheck_finish(char * Ot);
int Overload_is_come(unsigned int overloadindex);
void arch_StepMotor_Set_UMS(unsigned int mode,unsigned int mototype);  //这里需要根据mode 设置???
unsigned int  Triangle_is_left_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2);
unsigned int  Triangle_is_right_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2);
unsigned int  Triangle_is_zero_Sign(unsigned int zerono,unsigned char isnc1,unsigned int workno,unsigned char isnc2);

void arch_StepMotor_Enable_onestepmoto(unsigned int whichstep);
void arch_StepMotor_Disable_onestepmoto(unsigned int whichstep);

unsigned int arch_need_current_add(unsigned int currentdata);

void arch_LED_On(void);
void arch_LED_Off(void);
void arch_LED_Setup(unsigned long);

unsigned int arch_StepMotor_Zero_mid_layer(unsigned int idx,unsigned char isnc);
unsigned int arch_StepMotor_Work_mid_layer(unsigned int idx,unsigned char isnc);
void Set_checkin_time(unsigned int delaytime_ms);
void CheckIn_Timer();
void Checkin_need_send_data(unsigned short retcmd);


