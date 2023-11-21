#include "includes.h"

#define OSSTACK_LIMIT (1 << 14)

__align(8) static OS_STK OSStack[OSSTACK_LIMIT];
int OSStackIdx = OSSTACK_LIMIT;

OS_STK *OSStackAlloc(INT16U allocSize) {
    OS_STK* allocPtr = &OSStack[OSStackIdx - 1];
    OSStackIdx -= allocSize;
    return allocPtr;
};

void MyTaskCreate(void (*task)(void *p_arg), void *p_arg, INT8U prio, INT16U stkSize, char *taskName){
    OS_TCB *ptcb;
    OS_STK* ptos = OSStackAlloc(stkSize);
    OSTaskCreate(task, p_arg, ptos, prio);
    ptcb = OSTCBPrioTbl[prio];
    ptcb->OSTCBStkBottom = ptos;
    ptcb->OSTCBStkSize = stkSize;
    ptcb->OSTCBTaskName = (INT8U *)(void *)taskName;
    OS_TRACE_TASK_CREATE(ptcb);
}

// definition of p_arg: p_arg[0] = argc, p_arg[1+] = argv
OS_STK* OSTaskStkInit(void (*task)(void* p_arg), void* p_arg, OS_STK* ptos, INT16U opt){
    OS_STK* pstk = ptos;
    opt = opt;
    /*
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    *(--pstk) = 0;             // No Name Register
    *(--pstk) = 0x1000;        // FPSCR
    *(--pstk) = 0;             // S15
    *(--pstk) = 0;             // S14
    *(--pstk) = 0;             // S13
    *(--pstk) = 0;             // S12
    *(--pstk) = 0;             // S11
    *(--pstk) = 0;             // S10
    *(--pstk) = 0;             // S9
    *(--pstk) = 0;             // S8
    *(--pstk) = 0;             // S7
    *(--pstk) = 0;             // S6
    *(--pstk) = 0;             // S5
    *(--pstk) = 0;             // S4
    *(--pstk) = 0;             // S3
    *(--pstk) = 0;             // S2
    *(--pstk) = 0;             // S1
    *(--pstk) = 0;             // S0
#endif
*/
    *(--pstk) = 0x01000000;     // PSR
    *(--pstk) = (OS_STK)task;   // PC
    *(--pstk) = (OS_STK)OS_TaskReturn;     // LR
    *(--pstk) = 0;              // R12
    *(--pstk) = 0;              // R3
    *(--pstk) = 0;              // R2
    *(--pstk) = 0;              // R1
    *(--pstk) = (OS_STK) p_arg; // R0
    /*
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    *(--pstk) = 0;             // S31
    *(--pstk) = 0;             // S30
    *(--pstk) = 0;             // S29
    *(--pstk) = 0;             // S28
    *(--pstk) = 0;             // S27
    *(--pstk) = 0;             // S26
    *(--pstk) = 0;             // S25
    *(--pstk) = 0;             // S24
    *(--pstk) = 0;             // S23
    *(--pstk) = 0;             // S22
    *(--pstk) = 0;             // S21
    *(--pstk) = 0;             // S20
    *(--pstk) = 0;             // S19
    *(--pstk) = 0;             // S18
    *(--pstk) = 0;             // S17
    *(--pstk) = 0;             // S16
#endif
*/
    *(--pstk) = 0;              // R11
    *(--pstk) = 0;              // R10
    *(--pstk) = 0;              // R9
    *(--pstk) = 0;              // R8
    *(--pstk) = 0;              // R7
    *(--pstk) = 0;              // R6
    *(--pstk) = 0;              // R5
    *(--pstk) = 0;              // R4
    return pstk;
}


void SysTickInit(void){
    *(uint32_t*)(&(SCB->SHP[8])) &= ~(15u << 28);
    SysTick->LOAD = 84000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= (1 << 2);
    SysTick->CTRL |= (1 << 1);
    SysTick->CTRL |= (1 << 0);
}



void    OSInitHookBegin(void){ return; }
void    OSInitHookEnd(void){ return; }
void    OSTaskCreateHook(OS_TCB* ptcb){ return; }
void    OSTaskDelHook(OS_TCB* ptcb){ return; }
void    OSTaskSwHook(void){ return; }
void    OSTaskStatHook(void){ return; }
void    OSTCBInitHook(OS_TCB* ptcb){ return; }
void    OSTimeTickHook(void){ return; }
void    OSTaskIdleHook(void){ return; }
void    OSTaskReturnHook(OS_TCB* ptcb){ return; }
