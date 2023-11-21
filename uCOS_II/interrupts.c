#include "includes.h"

void SysTick_Handler(void){
    if(OSRunning == OS_TRUE){
        OSIntEnter();
        OSTimeTick();
        OSIntExit();
    }
}
