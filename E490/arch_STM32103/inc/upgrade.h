
#ifndef BYTE
typedef unsigned char  BYTE;
#endif
#ifndef WORD
typedef unsigned short WORD;
#endif
#ifndef DWORD
typedef unsigned long  DWORD;
#endif

void arch_Upgrade_Set_Start();
int arch_Upgrade_is_Start();
void arch_Upgrade_Set_Receive();
void arch_Upgrade_Set_Burn_Size(DWORD Data_Count);
DWORD arch_Upgrade_Get_Burn_Size();
void arch_Upgrade_Set_Burn();
int arch_Upgrade_is_Burn();
void arch_Upgrade_Finish();
DWORD fmc_start_addr(int sel);
void fmc_init(DWORD start_addr);
void fmc_program(BYTE *Buff, DWORD len);
void fmc_Verify(DWORD addr0, DWORD addr1, DWORD len);
void fmc_program_stop(void);

