#include "Alarm.h"
#include "SenSor.h"
#include "gpio.h"
#include "string.h"
#include "queue.h"
#include "SolenoidValve.h"
#include "ADCVoltage.h"
#include "CanApp.h"
Alarm AlarmSet={0};                 //报警处理集合

/*
纱嘴电磁铁报警监测 根据掩码是否上报
Alsz:报警操作集合
*/
signed char Alarm_SZ(Alarm *Alsz)
{
    unsigned char AlData[8]={0};   
    unsigned short time=0;
    time=(SZ_PowerTime.OUTKeepTime+SZ_PowerTime.INKeepTime)/2;
    if(CurrentSamplingClass.ValueCurrentEnd>=600&&Alsz->SZ_StateLock==0)
    {
        SZ_PowerTime.OvercurrentTiming=0;
        Alsz->SZ_StateLock=1;
    }
    //过滤过流突变    
    else if(CurrentSamplingClass.ValueCurrentEnd<=180&&Alsz->SZ_StateLock==1)
    {
            Alsz->SZ_StateLock=0;
            Alsz->SZ_OverloadTimeCount=0;
    }  
    else if(CurrentSamplingClass.ValueCurrentEnd>=600&&Alsz->SZ_StateLock==1&& \
            SZ_PowerTime.OvercurrentTiming>=100)
    {
        
        Alsz->SZ_OverloadTimeCount++;
        if(Alsz->SZ_OverloadTimeCount>=(10*3)+time)
        {
            Alsz->SZ_StateLock=2;
        }
        SZ_PowerTime.OvercurrentTiming=0;
    }
    else if(Alsz->SZ_StateLock==2)
    {
            memset(AlData,0x00,8);
            AlData[0]=0xf0;
            AlData[2]=0xA0;
            Alsz->SZ_AlarmStateBit=1;
            Alsz->SZ_StateLock=0;
            ProtocolCmd.PowerDC24VSwitchMask=2;
            Alsz->SZ_OverloadTimeCount=0;        
            CAN1_Send_Msg(AlData,8,arch_GetBoardID());        
    }
    return 0;
}

/*
DC12V电源短路报警
*/
signed char ALarm_DC12V(Alarm *pr)
 {
        unsigned char AlData[8]={0};
        volatile  unsigned short ADC_ConvertedValue12V;
        float ADC_Vol_12V;
        ADC_ConvertedValue12V=Get_Adc_Hadc1(POWER012V_CH );         
        ADC_Vol_12V=(float)ADC_ConvertedValue12V/4096*(float)3.3;   
        if(ADC_Vol_12V<=0.1f&&!pr->DC12V_StateLock)
        {
            memset(AlData,0x00,8);
            AlData[0]=0xf0;
            AlData[2]=0xBE;
            pr->DC12V_StateBit =1;
            pr->DC12V_StateLock=1;
            CAN1_Send_Msg(AlData,8,arch_GetBoardID());
        }
        else if(ADC_Vol_12V>=1.0f&&pr->DC12V_StateLock)
        {
             pr->DC12V_StateLock=0;
        }
        return 0;
}



/*
探针报警监测 根据掩码是否上报
As:报警操作集合
*/

signed char AlarmProbe(Alarm *As)
{
    unsigned char AlData[8]={0};
    if(!PROBEINPUT_B_STATE()&&!As->TZ_StateLock&&!(As->Mask&(unsigned short)ProbeMaskBitR))
    {
        memset(AlData,0x00,8);
        AlData[0]=0xf0;
        AlData[1]=0x01;
        AlData[2]=0xC1;
        As->TZ_AlarmStateBit =1;
        As->TZ_StateLock=1;
        CAN1_Send_Msg(AlData,8,arch_GetBoardID());    
    }
    else if(PROBEINPUT_B_STATE())
    {
        As->TZ_StateLock=0;
    }

  return 0;  
}

void AlarmTaskLoop(Alarm *atp)
{
    Alarm_SZ(atp);
    AlarmProbe(atp);
    ALarm_DC12V(atp);   
}






