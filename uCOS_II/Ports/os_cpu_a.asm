                IMPORT  OSRunning
                IMPORT  OSTCBCur
                IMPORT  OSTCBHighRdy
                IMPORT  OSPrioCur
                IMPORT  OSPrioHighRdy
                IMPORT  SYSVIEW_TaskSwitchedIn
                ;IMPORT  PendSV_Handler          ; VERY IMPORTANT!
                EXPORT  OSStartHighRdy
                EXPORT  OSCtxSw
                EXPORT  OSIntCtxSw
                EXPORT  OSTickISR
                EXPORT  GetProcessorPSW
                EXPORT  SetProcessorPSW
                EXPORT  EnableInterrupts
                EXPORT  DisableInterrupts
                
;Stack_Size      EQU     0x0000400
;
;                AREA    STACK, NOINIT, READWRITE, ALIGN = 3 ; 1 << 3 = 8
;Stack_Mem       SPACE   Stack_Size
;__initial_sp
;                EXPORT  __initial_sp
;
;Heap_Size       EQU     0x0000200
;
;                AREA    HEAP, NOINIT, READWRITE, ALIGN = 3
;__heap_base
;Heap_Mem        SPACE   Heap_Size
;__heap_limit
;                EXPORT  __heap_base
;                EXPORT  __heap_limit
                
;                PRESERVE8
;                THUMB
;                AREA    RESET, DATA, READONLY
;                DCD     __initial_sp
;                DCD     Reset_Handler
;                DCD     NMI_Handler
;                DCD     HardFault_Handler
;                DCD     MemManage_Handler
;                DCD     BusFault_Handler
;                DCD     UsageFault_Handler
;                DCD     0
;                DCD     0
;                DCD     0
;                DCD     0
;                DCD     SVC_Handler
;                DCD     0
;                DCD     0
;                DCD     PendSV_Handler
;                DCD     SysTick_Handler
;
;                AREA    |.text|, CODE, READONLY
;Reset_Handler   PROC
;                IMPORT  SystemInit
;                IMPORT  __main
;                LDR     R0, =SystemInit
;                BLX     R0
;                LDR     R0, =__main
;                BX      R0
;                ENDP
;
;NMI_Handler     PROC
;                EXPORT  NMI_Handler                [WEAK]
;                B       .
;                ENDP
;HardFault_Handler
;                PROC
;                EXPORT  HardFault_Handler          [WEAK]
;                B       .
;                ENDP
;MemManage_Handler
;                PROC
;                EXPORT  MemManage_Handler          [WEAK]
;                B       .
;                ENDP
;BusFault_Handler
;                PROC
;                EXPORT  BusFault_Handler           [WEAK]
;                B       .
;                ENDP
;UsageFault_Handler
;                PROC
;                EXPORT  UsageFault_Handler         [WEAK]
;                B       .
;                ENDP
;SVC_Handler     PROC
;                EXPORT  SVC_Handler                [WEAK]
;                B       .
;                ENDP
;DebugMon_Handler
;                PROC
;                EXPORT  DebugMon_Handler           [WEAK]
;                B       .
;                ENDP
                PRESERVE8
                AREA UCOSII_PORT, CODE, READONLY

                
                
SysTick_Handler
                EXPORT  SysTick_Handler            [WEAK]
                B       .

NVIC_INT_CTRL   EQU     0xE000ED04
NVIC_SYSPRI14   EQU     0xE000ED22
NVIC_PENDSV_PRI EQU           0xFF
NVIC_PENDSVSET  EQU     0x10000000
                
; Should not be written as a function, to avoid surplus steps
OSStartHighRdy
                LDR     R4, =NVIC_SYSPRI14
                LDR     R5, =NVIC_PENDSV_PRI    
                STR     R5, [R4]                ; Set PendSV's priority to minimum
                MOV     R4, #0
                MSR     PSP, R4                 ; PSP = 0
                LDR     R4, =OSRunning
                MOV     R5, #1                  ; OSRunning = true
                STRB    R5, [R4]
                LDR     R4, =NVIC_INT_CTRL
                LDR     R5, =NVIC_PENDSVSET
                STR     R5, [R4]                ; Trigger PendSV
                CPSIE   I                       ; Enable Interrupts
                B       .
                    
OSCtxSw                    
OSIntCtxSw
				PUSH    {R4,R5}
                LDR     R4, = NVIC_INT_CTRL     ; Actually SCB->ICSR
                LDR     R5, = NVIC_PENDSVSET    ; PENDSVSET: bit28, R5 straightly = (1 << 28)
                STR     R5, [R4]                ; Trigger PendSV_Handler
                POP     {R4,R5}
                BX      LR
                    
OSTickISR
                BX      LR
                    
                    

GetProcessorPSW
                MRS     R0, PRIMASK
                BX      LR
                    
SetProcessorPSW
                MSR     PRIMASK, R0
                BX      LR

DisableInterrupts
                CPSID   I
                BX      LR

EnableInterrupts
                CPSIE   I
                BX      LR
                NOP
                END