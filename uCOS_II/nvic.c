#include "includes.h"

void NVIC_PriorityGroupConfig(INT8U NVIC_Group){
    INT32U temp, temp1;
    temp1 = NVIC_Group & 0x07; // Fetch the last 3 bits
    temp1 <<= 8;
    temp = SCB->AIRCR;
    temp &= 0xF8FF; // clear previous groups
    temp |= 0x5FA0000;
    temp |= temp1;
    SCB->AIRCR = temp;
}

void NVIC_Init(INT8U NVIC_PreemptionPriority, 
               INT8U NVIC_SubPriority,
               INT8U NVIC_Channel,
               INT8U NVIC_Group){
    INT32U temp;
    NVIC_PriorityGroupConfig(NVIC_Group);
    temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
    temp |= NVIC_SubPriority & (0xf >> NVIC_Group);
    temp &= 0xf;
    NVIC->ISER[NVIC_Channel / 32] |= 1 << NVIC_Channel % 32;
    NVIC->IP[NVIC_Channel] |= temp << 4;
}
