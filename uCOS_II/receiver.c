#include "includes.h"

uint8_t TIM1_CH_CAPTURE_STA[5];
uint32_t TIM1_CH_CAPTURE_VAL[5];
uint8_t TIM2_CH1_STA;
uint32_t TIM2_CH1_VAL;

void TIM1_Config(void){
    // PRINCIPLE: CONFIGURE LEAST MANY REGISTERS
    
    // STEP 1: Enable TIM1 & GPIOA
    RCC->AHB1ENR    |= (1 << 0);
    
    RCC->APB2ENR    |= (1 << 0);
    
    // STEP 2: Configure GPIOA 8, 9, 10, 11
    GPIOA->MODER    |= (2 << (2 * 8))
                     | (2 << (2 * 9))
                     | (2 << (2 * 10))
                     | (2 << (2 * 11));
    
    GPIOA->OSPEEDR  |= (3 << (2 * 8))
                     | (3 << (2 * 9))
                     | (3 << (2 * 10))
                     | (3 << (2 * 11));
    
    GPIOA->PUPDR    |= (2 << (2 * 8))
                     | (2 << (2 * 9))
                     | (2 << (2 * 10))
                     | (2 << (2 * 11));
                     
    GPIOA->AFR[1]   |= (1 << (4 * 0))
                     | (1 << (4 * 1))
                     | (1 << (4 * 2))
                     | (1 << (4 * 3));
    
    // STEP 3: Configure TIM1
    TIM1->ARR = 0xFFFFFFFF;
    
    TIM1->PSC = 84 - 1;
    
    TIM1->CCMR1     |= (1 << 0)
                     | (1 << 8); // Set reflection of IC1 & 2
                     
    TIM1->CCMR2     |= (1 << 0)
                     | (1 << 8); // Set reflection of IC3 & 4
                     
    TIM1->CCER      |= (1 << 0)
                     | (1 << 4)
                     | (1 << 8)
                     | (1 << 12); // Enable capture
                     
    TIM1->EGR       |= (1 << 1)
                     | (1 << 2)
                     | (1 << 3)
                     | (1 << 4); // Enable generation
                        
    TIM1->DIER      |= (1 << 0)
                     | (1 << 1)
                     | (1 << 2)
                     | (1 << 3)
                     | (1 << 4); // Enable interrupt
                     
    TIM1->CR1       |= (1 << 0);
    // Refer to TIM1->CCR1 to see restored TIMx_CNT
    // Refer to TIM1->SR to see if interrupt occurs
    
    NVIC_Init(2, 0, 27, 2);
}

void TIM2_Config(void) {
    RCC->AHB1ENR |= (1 << 0); // GPIOA
    RCC->APB1ENR |= (1 << 0); // TIM2
    GPIOA->MODER &= ~(3 << 0); // AF
    GPIOA->MODER |=  (2 << 0);
    GPIOA->OTYPER &= ~(1 << 0); // PP
    GPIOA->OSPEEDR |= (3 << 0); // 100MHz
    GPIOA->PUPDR &= ~(3 << 0); // Pull Down
    GPIOA->PUPDR |= (2 << 0);
    GPIOA->AFR[0] &= ~(15 << 0); // AF1
    GPIOA->AFR[0] |= (1 << 0);
    
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->PSC = 84 - 1;
    TIM2->CCMR1 |= (1 << 0); // CC1 input
    TIM2->CCER |= (1 << 0); // CC1 enable
    TIM2->EGR |= (1 << 1); // generate capture event
    TIM2->DIER |= (1 << 0) | (1 << 1); // int enable
    TIM2->CR1 |= (1 << 0); // counter enable
    
    NVIC_Init(2, 0, 28, 2);
}

int GetReceiverData(CtrlData* pCtrlData) {
    uint32_t range = 0;
    int i;
    for(i = 1; i <= 4; ++i){
        if(TIM1_CH_CAPTURE_STA[i] & 0x80){
            range = TIM1_CH_CAPTURE_VAL[i];
            TIM1_CH_CAPTURE_STA[i] = 0;
            switch(i){
                case 1: pCtrlData->rol = range; break;
                case 2: pCtrlData->pit = range; break;
                case 3: pCtrlData->pwr = range; break;
                case 4: pCtrlData->yaw = range; break;
            }
        }
    }
    return 0;
}

int GetUnlockSignal(void) {
    int retval = 0;
    if(TIM2_CH1_STA & 0x80) {
        //printf("VAL=%u\r\n", TIM2_CH1_VAL);
        if(TIM2_CH1_VAL > 120000 && TIM2_CH1_VAL < 200000) retval = 1;
        TIM2_CH1_STA = 0;
    }
    return retval;
}


void TIM1_CC_IRQHandler(void){
    int i;
    uint16_t TIM1SR;
    OSIntEnter();
    TIM1SR = TIM1->SR;
    for(i = 1; i <= 4; ++i){
        if(!(TIM1_CH_CAPTURE_STA[i] & 0x80)){ // not capture yet
            if(TIM1SR & 0x01){
                if(TIM1_CH_CAPTURE_STA[i] & 0x40){
                    if((TIM1_CH_CAPTURE_STA[i] & 0x3F) == 0x3F){
                        TIM1_CH_CAPTURE_STA[i] |= 0x80;
                        TIM1_CH_CAPTURE_VAL[i] = 0xFFFFFFFF;
                    }
                    else
                        TIM1_CH_CAPTURE_STA[i]++;
                }
            }
            if(TIM1SR & (1 << i)){
                if(TIM1_CH_CAPTURE_STA[i] & 0x40){ // a rise captured
                    TIM1_CH_CAPTURE_STA[i] |= 0x80;
                    switch(i){
                        case 1: TIM1_CH_CAPTURE_VAL[i] = TIM1->CCR1; break;
                        case 2: TIM1_CH_CAPTURE_VAL[i] = TIM1->CCR2; break;
                        case 3: TIM1_CH_CAPTURE_VAL[i] = TIM1->CCR3; break;
                        case 4: TIM1_CH_CAPTURE_VAL[i] = TIM1->CCR4; break;
                        default: ;
                    }
                    TIM1->CCER &= ~(1 << (1 + 4 * (i - 1)));
                }
                else{ // capture up
                    TIM1_CH_CAPTURE_STA[i] = 0;
                    TIM1_CH_CAPTURE_VAL[i] = 0;
                    TIM1_CH_CAPTURE_STA[i] |= 0x40;
                    TIM1->CR1 &= ~(1 << 0); // disable tim
                    TIM1->CNT = 0;
                    TIM1->CCER |= (1 << (1 + 4 * (i - 1))); // set down-capture
                    TIM1->CR1 |= (1 << 0); // enable tim again
                }
            }
        }
    }
    TIM1->SR = 0;
    OSIntExit();
}

void TIM2_IRQHandler(void) { // Untested
    uint16_t TIM2_SR;
    OSIntEnter();
    TIM2_SR = TIM2->SR;
    if(!(TIM2_CH1_STA & 0x80)){ // not capture yet
        if(TIM2_SR & 1){
            if(TIM2_CH1_STA & 0x40){
                if((TIM2_CH1_STA & 0x3F) == 0x3F){
                    TIM2_CH1_STA |= 0x80;
                    TIM2_CH1_VAL = 0xFFFFFFFF;
                }
                else
                    TIM2_CH1_STA++;
            }
        }
        if(TIM2_SR & (1 << 1)){
            if(TIM2_CH1_STA & 0x40){ // a rise captured
                TIM2_CH1_STA |= 0x80;
                TIM2_CH1_VAL = TIM2->CCR1;
                TIM2->CCER &= ~(1 << 1); // set up capture
            }
            else{ // capture up
                TIM2_CH1_STA = 0;
                TIM2_CH1_VAL = 0;
                TIM2_CH1_STA |= 0x40;
                TIM2->CR1 &= ~(1 << 0); // disable tim
                TIM2->CNT = 0;
                TIM2->CCER |= (1 << 1); // set down-capture
                TIM2->CR1 |= (1 << 0); // enable tim again
            }
        }
    }
    TIM2->SR = 0;
    OSIntExit();
}

