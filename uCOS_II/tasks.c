#include "includes.h"

CtrlData initCtrlData;
int unlocked = 0;
int systemLocked = 1;
char usart_tx_buf[256];


void MotorUnlock(void* p_arg) {
    puts("[MotorUnlock] Start");
    p_arg = p_arg;
    SetCCR((PWMData) {ccr_max, ccr_max, ccr_max, ccr_max});
    OSTimeDly(1000);
    SetCCR((PWMData) {ccr_mid, ccr_mid, ccr_mid, ccr_mid});
    OSTimeDly(1000);
    SetCCR((PWMData) {ccr_min, ccr_min, ccr_min, ccr_min});
    OSTimeDly(1000);
    unlocked = 1;
    puts("[MotorUnlock] End");
    OSTaskDel(OS_PRIO_SELF);
}

/*
    @brief Get the stable initial value of all channels, except CH5
*/
void StickCease(void* p_arg) {
    CtrlData newCtrlData;
    puts("[StickCease] Start");
    p_arg = p_arg;
    while(1) {
        if(unlocked == 1){
            puts("[StickCease] End");
            sprintf(usart_tx_buf, "ipit=%d, ipwr=%d, irol=%d, iyaw=%d\r\n", initCtrlData.pit, initCtrlData.pwr, initCtrlData.rol, initCtrlData.yaw);
            USART1_SEND_BUF_U8((u8*)usart_tx_buf, strlen(usart_tx_buf));
            OSTaskDel(OS_PRIO_SELF);
        }
        OSTimeDly(100);
        GetReceiverData(&newCtrlData);
        initCtrlData.pit = (initCtrlData.pit + newCtrlData.pit) / 2;
        initCtrlData.pwr = (initCtrlData.pwr + newCtrlData.pwr) / 2;
        initCtrlData.rol = (initCtrlData.rol + newCtrlData.rol) / 2;
        initCtrlData.yaw = (initCtrlData.yaw + newCtrlData.yaw) / 2;
    }
}

void GuardTask(void* p_arg) {
    extern uint8_t TIM2_CH1_STA;
    extern uint32_t TIM2_CH1_VAL;
    int preSta = -1, curSta = 0; // locked
    while(1) {
        while(!(TIM2_CH1_STA & 0x80)) {
            //printf("[GuardTask] Waiting for Signal...\r\n");
            if(!systemLocked) OSTimeDly(50);
        }
        preSta = curSta;
        curSta = GetUnlockSignal();
        if(preSta == curSta) continue;
        if(curSta){
            printf("[GuardTask] System UnLock\r\n");
            OSSchedUnlock();
            OSTimeDly(50);
        }
        else{
            printf("[GuardTask] System Lock\r\n");
            OSSchedLock();
        }
    }
}


void GlobalInit(void* p_arg) {
    p_arg = p_arg;
    SystemCoreClockUpdate();
    LED_Init();
    SysTickInit();
    USART1_Init(115200);
    puts("\r\n------------------------");
    puts("[GlobalInit] Start");
    GY86_Init();
    TIM1_Config();
    TIM2_Config();
    MotorConfig();
    puts("[GlobalInit] Init complete");
    /*
    MyTaskCreate(GuardTask, NULL, 2, 128, "GuardTask");
    
    while(systemLocked) {
        printf("[GlobalInit] Waiting for SystemUnlock...\r\n");
        OSTimeDly(1000);
    }
    */
    MyTaskCreate(MotorUnlock, NULL, 3, 128, "MotorUnlock");
    MyTaskCreate(StickCease, NULL, 4, 128, "StickCease");
    
    while(!unlocked){
        puts("[GlobalInit] Waiting for MotorUnlock...");
        OSTimeDly(1000);
    }
    MyTaskCreate(LED, NULL, 62, 128, "LED");
    //MyTaskCreate(GY86_Output, NULL, 5, 128, "GY86Output");
    //MyTaskCreate(Receiver_Output, NULL, 6, 128, "ReceiverOutput");
    MyTaskCreate(ListenPID, NULL, 61, 128, "ListenPID");
    MyTaskCreate(MotorControl, NULL, 3, 128, "MotorControl");
    OSTaskDel(OS_PRIO_SELF);
}


void LED(void* p_arg){
    p_arg = p_arg;
    puts("[LED] Start");
    while(1){
        //puts("[LED] Running...");
        GPIOA->ODR |= (1 << 5);
        OSTimeDly(1000);
        GPIOA->ODR &= ~(1 << 5);
        OSTimeDly(1000);
    }
}
/*
void GY86_Output(void *p_arg){
    AttdData* pAttdData = (AttdData*) malloc(sizeof(AttdData));
    p_arg = p_arg;
    printf("[GY86Output] Start\r\n");
    while(1){
        GetAttdData(pAttdData);
    #ifdef AHRS
        printf("PIT=%d ROL=%d YAW=%d\r\n", (int)(pAttdData->pitch * 100.0), (int)(pAttdData->roll * 100.0), (int)(pAttdData->yaw * 100.0));
    #else
        printf("PIT=%.2f ROL=%.2f YAW=%.2f\r\n", pAttdData->pitch, pAttdData->roll, pAttdData->yaw);
    #endif
        OSTimeDly(100);
    }
    //free(pAttdData);
}


void Receiver_Output(void *p_arg){
    CtrlData ctrlData;
    p_arg = p_arg;
    while(1){
        GetReceiverData(&ctrlData);
        printf("ROL=%u PIT=%u PWR=%u YAW=%u\r\n", ctrlData.rol, ctrlData.pit, ctrlData.pwr, ctrlData.yaw);
        OSTimeDly(100);
    }
}
*/
/*      °˙     °˚
       °¸4°˝   °˝3°¸
        °˚\   /°˙
          \ /
           O
          / \
        °˚/   \°˙
       °˝2°¸   °¸1°˝
        °˙     °˚

      PITCH  --++
      ROLL   -+-+
      YAW    -++-
*/

// pIntData's errN is actually the current value of the target
PWMData PIDOutput(PIDData* pExtData, PIDParam extParam, PIDData* pIntData, PIDParam intParam, Attitude attitude) {
    float intTarget = extParam.Kp * pExtData->P + extParam.Ki * pExtData->I + extParam.Kd * pExtData->D;
    pIntData->P -= intTarget;
    pIntData->I -= intTarget;
    pIntData->D -= intTarget;
    float res = intParam.Kp * pIntData->P + intParam.Ki * pIntData->I + intParam.Kd * pIntData->D;
    //float res = intTarget;
    //OS_CPU_SR cpu_sr;
    //OS_ENTER_CRITICAL();
    sprintf(usart_tx_buf, "[PIDCalc] extP = %.2f, intP = %.2f, res = %.2f\r\n", pExtData->P, pIntData->P, res);
    USART1_SEND_BUF_U8((u8*)usart_tx_buf, strlen(usart_tx_buf));
    //printf("[PIDCalc] extP = %.2f, res = %.2f, intP = %.2f\r\n", pExtData->P, res, pIntData->P);
    //OS_EXIT_CRITICAL();
    switch(attitude) {
        case PITCH: return (PWMData) {-res, -res, +res, +res};
        case ROLL:  return (PWMData) {-res, +res, -res, +res};
        case YAW:   return (PWMData) {-res, +res, +res, -res};
        default:    return (PWMData) {0, 0, 0, 0};
    }
}

float min(float x, float y) {
    return x < y ? x : y;
}

float max(float x, float y) {
    return x > y ? x : y;
}

float PWMRestrict(float x) {
    if(x > ccr_min) return ccr_min;
    else if(x < ccr_max) return ccr_max;
    else return x;
}

PWMData Add(PWMData pwmData1, PWMData pwmData2) {
    return (PWMData) {PWMRestrict(pwmData1.ccr1 + pwmData2.ccr1)
                    , PWMRestrict(pwmData1.ccr2 + pwmData2.ccr2)
                    , PWMRestrict(pwmData1.ccr3 + pwmData2.ccr3)
                    , PWMRestrict(pwmData1.ccr4 + pwmData2.ccr4)};
}

void PIDDataRoll(PIDData *pData, float newP) {
    pData->D = newP - pData->P;
    pData->I += pData->P;
    pData->P = newP;
}

// external(angle)     Kp Ki Kd
PIDParam pitchParam = {0, 0, 0}, 
         rollParam  = {0, 0, 0}, 
         yawParam   = {0, 0, 0};
// internal(anglespeed)     Kp Ki Kd TEMPORARILY ABANDONED 
PIDParam pitchSpeedParam = {0, 0, 0}, 
         rollSpeedParam  = {0, 0, 0}, 
         yawSpeedParam   = {0, 0, 0};
       
PIDData pitchData = {0, 0, 0}, 
        rollData  = {0, 0, 0}, 
        yawData   = {0, 0, 0};
PIDData pitchSpeedData = {0, 0, 0}, 
        rollSpeedData  = {0, 0, 0}, 
        yawSpeedData   = {0, 0, 0};

int CommandParse(void);
       
void MotorControl(void* p_arg) {
    CtrlData ctrlData;
    AttdData attdData, tagAttdData;
    #define ccr_init ((ccr_min + ccr_mid) / 2)
    PWMData pwmData = {ccr_init, ccr_init, ccr_init, ccr_init};
    SpeedData speedData;
    float dh;
    puts("[MotorControl] Start");
    p_arg = p_arg;
    while(1) {
        //OS_CPU_SR cpu_sr;
        //puts("[MotorControl] Running...");
        GetAttdData(&attdData);
        GetReceiverData(&ctrlData);
        GetSpeedData(&speedData);
        attdData.pitch = -attdData.pitch;
        tagAttdData.pitch = -(ctrlData.pit - initCtrlData.pit) * 90.0 / 500.0; // ∑˚∫≈Œ Ã‚£ø
        tagAttdData.roll = (ctrlData.rol - initCtrlData.rol) * 90.0 / 500.0;
        tagAttdData.yaw = (ctrlData.yaw - initCtrlData.yaw) * 180.0 / 500.0;
        dh = (ctrlData.pwr - initCtrlData.pwr) * (ccr_min - ccr_max) / 1000.0;
        PIDDataRoll(&pitchData, attdData.pitch - tagAttdData.pitch);
        PIDDataRoll(&rollData,  attdData.roll - tagAttdData.roll);
        PIDDataRoll(&yawData,   attdData.yaw - tagAttdData.yaw);
        PIDDataRoll(&pitchSpeedData, speedData.pitchspeed);
        PIDDataRoll(&rollSpeedData,  speedData.rollspeed);
        PIDDataRoll(&yawSpeedData,   speedData.yawspeed);
        
        PWMData pitchPWMData = PIDOutput(&pitchData, pitchParam, &pitchSpeedData, pitchSpeedParam, PITCH);
//        PWMData rollPWMData = PIDOutput(&rollData, rollParam, &rollSpeedData, rollSpeedParam, ROLL);
//        PWMData yawPWMData = PIDOutput(&yawData, yawParam, &yawSpeedData, yawSpeedParam, YAW);
        PWMData pwrPWMData = (PWMData) {-dh, -dh, -dh, -dh};
//        pwmData = Add(Add(Add(pwmData, pitchPWMData), Add(rollPWMData, yawPWMData)), pwrPWMData);
        
        PWMData tmp = Add(Add(pwmData, pitchPWMData), pwrPWMData);
        //PWMData tmp = Add(pwmData, pwrPWMData);
        SetCCR(tmp);
        //printf("pitv=%d rolv=%d yawv=%d\r\r\n", speedData.pitchspeed, speedData.rollspeed, speedData.yawspeed);
        //printf("[MotorControl] (%.2f, %.2f, %.2f, %.2f)\r\n", tmp.ccr1, tmp.ccr2, tmp.ccr3, tmp.ccr4);
        //printf("pitchPID: %d %d %d\r\n", (int)roundf(pitchParam.Kp * 100.0), (int)roundf(pitchParam.Ki * 100.0), (int)roundf(pitchParam.Kd * 100.0));
        //printf("pitchPID: %.2f %.2f %.2f\r\n", 1.14, 5.14, 1.919);
        OSTimeDly(50);
    }
}

void ListenPID(void* p_arg) {
    p_arg = p_arg;
    extern char usart_buf[256];
    extern uint16_t usart_buf_sta;
    puts("[ListenPID] Start");
    while(1) {
        //OS_CPU_SR cpu_sr;
        //puts("[ListenPID] Running...");
        if(usart_buf_sta & 0x8000) {
            int ret = CommandParse();
            if(!ret) {
                //printf("[ListenPID] Command '%s' Executed\r\n", usart_buf);
                puts("[ListenPID] Command Executed");
                /*
                printf("[ListenPID] PID = [%d, %d, %d]\r\n", (int)roundf(pitchParam.Kp * 100.0), (int)roundf(pitchParam.Ki * 100.0), (int)roundf(pitchParam.Kd * 100.0));
                printf("                              [%d, %d, %d]\r\n", (int)roundf(rollParam.Kp * 100.0), (int)roundf(rollParam.Ki * 100.0), (int)roundf(rollParam.Kd * 100.0));
                printf("                              [%d, %d, %d]\r\n", (int)roundf(yawParam.Kp * 100.0), (int)roundf(yawParam.Ki * 100.0), (int)roundf(yawParam.Kd * 100.0));
                printf("                              [%d, %d, %d]\r\n", (int)roundf(pitchSpeedParam.Kp * 100.0), (int)roundf(pitchSpeedParam.Ki * 100.0), (int)roundf(pitchSpeedParam.Kd * 100.0));
                printf("                              [%d, %d, %d]\r\n", (int)roundf(rollSpeedParam.Kp * 100.0), (int)roundf(rollSpeedParam.Ki * 100.0), (int)roundf(rollSpeedParam.Kd * 100.0));
                printf("                              [%d, %d, %d]\r\n", (int)roundf(yawSpeedParam.Kp * 100.0), (int)roundf(yawSpeedParam.Ki * 100.0), (int)roundf(yawSpeedParam.Kd * 100.0));
                */
            }
            else {
                //printf("[ListenPID] Invalid Command '%s'\r\n", usart_buf);
                puts("[ListenPID] Invalid Command");
            }
            usart_buf_sta = 0;
            memset(usart_buf, 0, sizeof(usart_buf));
            pitchData = (PIDData) {0, 0, 0};
            rollData = (PIDData) {0, 0, 0};
            yawData = (PIDData) {0, 0, 0};
            pitchSpeedData = (PIDData) {0, 0, 0};
            rollSpeedData = (PIDData) {0, 0, 0};
            yawSpeedData = (PIDData) {0, 0, 0};
        }
        OSTimeDly(200);
    }
}
