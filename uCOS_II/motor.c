#include "includes.h"

void delay_ms(int nms){
    int i;
    for(i = 0; i < nms * 540; ++i);
}

void SetCCR(PWMData pwmData){
	TIM3->CCR1 &= ~(0XFFFF);
	TIM3->CCR2 &= ~(0XFFFF);
	TIM3->CCR3 &= ~(0XFFFF);
	TIM3->CCR4 &= ~(0XFFFF);
	TIM3->CCR1 |= (short) pwmData.ccr1;
	TIM3->CCR2 |= (short) pwmData.ccr2;
	TIM3->CCR3 |= (short) pwmData.ccr3;
	TIM3->CCR4 |= (short) pwmData.ccr4;
    //printf("[CCR] (%d, %d, %d, %d)\r\n", TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
}

void MotorConfig(void){
    //使能TIM3,GPIOC时钟
	RCC->APB1ENR|=1<<1; //TIM3
	RCC->AHB1ENR |= (1 << 0)
                  | (1 << 1); //PA & PB
	
	//PA6, PA7, PB0, PB1
	GPIOA->MODER&=~(3<<(2*6));
	GPIOA->MODER&=~(3<<(2*7));
	GPIOB->MODER&=~(3<<(2*0));
	GPIOB->MODER&=~(3<<(2*1));
	GPIOA->MODER|=(2<<(2*6)); //复用模式
	GPIOA->MODER|=(2<<(2*7));
	GPIOB->MODER|=(2<<(2*0));
	GPIOB->MODER|=(2<<(2*1));
	
	GPIOA->OSPEEDR|=(2<<(2*6)); //50MHz
	GPIOA->OSPEEDR|=(2<<(2*7));
	GPIOB->OSPEEDR|=(2<<(2*0));
	GPIOB->OSPEEDR|=(2<<(2*1));
	
	
	GPIOA->AFR[0] |=(2<<(4*6));
	GPIOA->AFR[0] |=(2<<(4*7));
	GPIOB->AFR[0] |=(2<<(4*0));
	GPIOB->AFR[0] |=(2<<(4*1));
	
	GPIOA->ODR |= (1 << 6) | (1 << 7);
    GPIOB->ODR |= (1 << 0) | (1 << 1);
	
	//配置psc,arr
	TIM3->ARR=arr;
	TIM3->PSC=psc;
	
	//配置PWM1
	TIM3->CCMR1 |=(6<<4);  //110,PWM模式1
	TIM3->CCMR1 |=(6<<12);
	TIM3->CCMR2 |=(6<<4);
	TIM3->CCMR2 |=(6<<12);
	
	//配置使能
	TIM3->CCMR1 |=1<<3;
	TIM3->CCMR1 |=1<<11;
	TIM3->CCMR2 |=1<<3;
	TIM3->CCMR2 |=1<<11;
	//配置低电平有效
	TIM3->CCER |=1<<1;
	TIM3->CCER |=1<<5;
	TIM3->CCER |=1<<9;
	TIM3->CCER |=1<<13;
	TIM3->CCER |=1<<0;
	TIM3->CCER |=1<<4;
	TIM3->CCER |=1<<8;
	TIM3->CCER |=1<<12;
	
	//配置CR1
	TIM3->CR1 |=1<<0;
	TIM3->CR1 |=1<<7;
}
