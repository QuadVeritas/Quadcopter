// Cheating

#ifndef OS_CPU_H
#define OS_CPU_H

//#include "stm32f401xe.h"

typedef unsigned char   BOOLEAN;
typedef unsigned char   INT8U;
typedef signed char     INT8S;
typedef unsigned short  INT16U;
typedef signed short    INT16S;
typedef unsigned int    INT32U;
typedef signed int      INT32S;
typedef float           FP32;
typedef double          FP64;
typedef unsigned int    OS_STK;    // stack stores pointers that are 32-bit
typedef unsigned int    OS_CPU_SR;

#define                 OS_CRITICAL_METHOD      (3u)
#define                 OS_STK_GROWTH           (1u)
#define                 OS_ENTER_CRITICAL()     {cpu_sr = GetProcessorPSW(); \
                                                DisableInterrupts();}
#define                 OS_EXIT_CRITICAL()      {SetProcessorPSW(cpu_sr);}
#define                 OS_TASK_SW()            OSCtxSw()
                                                
extern void             OSStartHighRdy(void);
extern void             OSCtxSw(void);
extern void             OSIntCtxSw(void);
extern void             OSTickISR(void);


extern OS_CPU_SR        GetProcessorPSW(void);
extern void             SetProcessorPSW(OS_CPU_SR cpu_sr);
extern void             DisableInterrupts(void);
extern void             EnableInterrupts(void);




#endif

