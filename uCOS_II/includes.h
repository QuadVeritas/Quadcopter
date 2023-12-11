#ifndef INCLUDES_H
#define INCLUDES_H

#pragma pack(8)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

typedef struct {
    float pitch;
    float roll;
    float yaw;
} AttdData;

typedef struct {
    float pitchspeed;
    float rollspeed;
    float yawspeed;
} SpeedData;

typedef struct {
    short pwr;
    short yaw;
    short rol;
    short pit;
} CtrlData;

typedef struct {
    float ccr1;
    float ccr2;
    float ccr3;
    float ccr4;
} PWMData;

typedef struct {
    float Kp, Ki, Kd;
} PIDParam;

typedef struct {
    float P, I, D;
} PIDData;

typedef enum {
    PITCH,
    ROLL,
    YAW
} Attitude;

#include "stm32f401xe.h"
#include "core_cm4.h"

#include "ucos_ii.h"
#include "SEGGER_SYSVIEW.h"

#include "gy86.h"
#include "mpu6050.h"
#include "iic.h"
#include "gy86time.h"
#include "ahrs.h"
#include "receiver.h"

#pragma pack(8)

#define STM32F401RET6
#define AHRS
//#define ENABLE_UPPER_COM
//#define AHRS_DEBUG
#define ARC_TO_DEG 57.295780f

void NVIC_PriorityGroupConfig(INT8U NVIC_Group);
void NVIC_Init(INT8U NVIC_PreemptionPriority, 
               INT8U NVIC_SubPriority,
               INT8U NVIC_Channel,
               INT8U NVIC_Group);

void LED_Init(void);
void USART1_Init(uint32_t);
void TIM1_Config(void);
void TIM2_Config(void);
void MotorConfig(void);
void delay_us(int nus);
void delay_ms(int nms);
void GY86_Init(void);

void SysTickInit(void);
OS_STK *OSStackAlloc(INT16U);
void MyTaskCreate(void (*task)(void *p_arg), void *p_arg, INT8U prio, INT16U stkSize, char *taskName);
void MotorUnlockAndStickStable(void);

void GlobalInit(void*);
void LED(void*);
void GY86_Output(void *);
void Receiver_Output(void*);
void MotorControl(void*);
void ListenPID(void*);
void Get_Gyro_Bias(void*);
void Get_Accel_Bias(void*);

void USART1_SEND_BUF_U8(u8* buf,u8 len);
void SEND_OULA_ANGLE(short row,short pit,short yaw);

#endif
