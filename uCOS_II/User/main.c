#include "includes.h"

int main(void){
    
    OSInit();
    
#if (OS_TRACE_EN == 1u)
    OS_TRACE_INIT();
#endif
    MyTaskCreate(GlobalInit, NULL, 1, 1024, "Init");
    
#if (OS_TRACE_EN == 1u)
    OS_TRACE_START();
#endif
    OSStart();
    return 0;
}
