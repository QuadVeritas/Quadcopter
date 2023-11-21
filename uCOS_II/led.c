#include "includes.h"

void LED_Init(){
    
#ifdef STM32F401CCU6
    RCC->AHB1ENR    |=  (1 << 2);
    GPIOC->MODER    &=  ~(3 << (13 * 2));
    GPIOC->MODER    |=  (1 << (13 * 2));
    GPIOC->OTYPER   &=  ~(1 << 13);
    GPIOC->OSPEEDR |=  (3 << (13 * 2));
    GPIOC->PUPDR    &=  ~(3 << (13 * 2));
    GPIOC->PUPDR    |=  (2 << (13 * 2));
#endif
    
#ifdef STM32F401RET6
    RCC->AHB1ENR    |=  (1 << 0);
    GPIOA->MODER    &=  ~(3 << (5 * 2));
    GPIOA->MODER    |=  (1 << (5 * 2));
    GPIOA->OTYPER   &=  ~(1 << 5);
    GPIOA->OSPEEDR |=  (3 << (5 * 2));
    GPIOA->PUPDR    &=  ~(3 << (5 * 2));
    GPIOA->PUPDR    |=  (2 << (5 * 2));
#endif
    
}
