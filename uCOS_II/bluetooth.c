#include "includes.h"

int fputc(int ch, FILE *f){
    while((USART1->SR & 0x40) == 0);
    USART1->DR = (uint8_t) ch;
    return ch;
}

void USART1_Init(INT32U baudrate){
    
    FP32 usart_div;
    INT16U mantissa, fraction;
    extern INT32U SystemCoreClock;
    
    
    RCC->AHB1ENR     |=   (1 << 1); // GPIOB
    RCC->APB2ENR     |=   (1 << 4); // USART1
    GPIOB->MODER     &= ~((3 << 6 * 2) | (3 << 7 * 2));
    GPIOB->MODER     |=   (2 << 6 * 2) | (2 << 7 * 2); // AF Mode
    GPIOB->OTYPER    &= ~    ((1 << 6) | (1 << 7)); // PP
    GPIOB->OSPEEDR  &= ~((3 << 6 * 2) | (3 << 7 * 2));
    GPIOB->OSPEEDR  |=   (2 << 6 * 2) | (2 << 7 * 2); // 50MHz
    GPIOB->PUPDR     &= ~((3 << 6 * 2) | (3 << 7 * 2));
    GPIOB->PUPDR     |=   (1 << 6 * 2) | (1 << 7 * 2); // Pull-up
    GPIOB->AFR[0]    &=~((15u << 6 * 4) | (15u << 7 * 4));
    GPIOB->AFR[0]    |=   (7 << 6 * 4) | (7 << 7 * 4); // AF7: USART1/2
    
    usart_div = (FP32) (SystemCoreClock) / (FP32) (baudrate * 16U);
    mantissa = (INT32U) usart_div;
    fraction = (INT32U) (usart_div - (FP32) mantissa) * 16;
    USART1->BRR = (mantissa << 4) + fraction;
    USART1->CR1 &= ~(1 << 15);   // OVER8 = 0
    USART1->CR1 |= (1 << 2) | (1 << 3) | (1 << 5) | (1 << 13);
    NVIC_Init(3, 3, USART1_IRQn, 2);
}

char usart_buf[257];
uint16_t usart_buf_sta;



// use the right strlen
// 0 for normal, 1 for error
int rprefixcmp(const char* str1, const char* str2) {
    int l = 0, r = strlen(str2);
    if(strlen(str1) < r) return 1;
    for(; l < r; ++l) {
        if(str1[l] != str2[l]) return 1;
    }
    return 0;
}

extern PIDParam pitchParam, rollParam, yawParam;
extern PIDParam pitchSpeedParam, rollSpeedParam, yawSpeedParam;

// 0 for normal, 1 for error
// format: {p|r|y}{e|i} <Kp> <Ki> <Kd>#
// e => angle, i => anglespeed
int CommandParse() {
    float* pParam = 0;
    if(!rprefixcmp(usart_buf, "pe "))      pParam = (float*) &pitchParam;
    else if(!rprefixcmp(usart_buf, "pi ")) pParam = (float*) &pitchSpeedParam;
    else if(!rprefixcmp(usart_buf, "re ")) pParam = (float*) &rollParam;
    else if(!rprefixcmp(usart_buf, "ri ")) pParam = (float*) &rollSpeedParam;
    else if(!rprefixcmp(usart_buf, "ye ")) pParam = (float*) &yawParam;
    else if(!rprefixcmp(usart_buf, "yi ")) pParam = (float*) &yawSpeedParam;
    else return 1;
    uint8_t idx = 3, pre = 2, id = 0;
    uint8_t len = usart_buf_sta & 0x3fff;
    while(idx < len) {
        if(usart_buf[idx] == ' ' || usart_buf[idx] == '#') {
            uint8_t l = pre + 1, r = idx - 1;
            if(('0' <= usart_buf[l] && usart_buf[l] <= '9') || 
               (usart_buf[l] == '-' && usart_buf[l + 1] >= '0' && usart_buf[l + 1] <= '9')) {
                if(pParam == 0) return 1; // num without paramselect declaration
                char* tmp = (char*) malloc((r - l + 1) * sizeof(char));
                for(int i = l; i <= r; ++i)
                    *(tmp + i - l) = usart_buf[i];
                *(pParam + id) = atof(tmp);
                ++id;
                free(tmp);
            }
            else return 1;
            pre = idx;
        }
        ++idx;
    }
    return 0;
}

// 手机+蓝牙模块方案：由于啥b蓝牙助手单次发送长度限制，只能缩减命令长度
void USART1_IRQHandler(void) {
    OSIntEnter();
    if(USART1->SR & (1 << 5)) {
        uint8_t dr = USART1->DR;
        if(!(usart_buf_sta & 0x8000)) {
            if(dr == '#') usart_buf_sta |= 0x8000;
            usart_buf[usart_buf_sta & 0x3fff] = dr;
            usart_buf_sta++;
            if((usart_buf_sta & 0x3fff) > 256) usart_buf_sta = 0;
            //putchar(usart_buf[strlen(usart_buf) - 1]);
            // 中断里面调用printf会扰乱中断
        }
    }
    OSIntExit();
}

/* 双蓝牙模块方案
void USART1_IRQHandler(void) {
    OSIntEnter();
    if(USART1->SR & (1 << 5)) {
        uint8_t dr = USART1->DR;
        if(!(usart_buf_sta & 0x8000)) {
            if(usart_buf_sta & 0x4000) {
                if(dr != 0x0a) usart_buf_sta = 0;
                else usart_buf_sta |= 0x8000;
            }
            else {
                if(dr == 0x0d) usart_buf_sta |= 0x4000;
                else {
                    usart_buf[usart_buf_sta & 0x3fff] = dr;
                    usart_buf_sta++;
                    if(usart_buf_sta > 256) usart_buf_sta = 0;
                }
            }
            //printf("buf = %s\r\n", usart_buf);
        }
    }
    OSIntExit();
}*/

// 匿名上位机需求

void USART1_SEND_DATA_U8(u8 data)
{
    USART1->DR = (data & (uint16_t)0x01FF);
	//while(!USART_GetFlagStatus(USART1,USART_FLAG_TC));
    while(!(USART1->SR & 0x40));
}

void USART1_SEND_BUF_U8(u8* buf,u8 len)
{
	while(len)
	{
		USART1_SEND_DATA_U8(*buf);
		buf++;
		len--;
	}
}

void SEND_OULA_ANGLE(short row,short pit,short yaw)
{
	u8 i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 buf[13]={0};
	
	buf[0]=0xaa;
	buf[1]=0xff;
	buf[2]=0x03;
	buf[3]=0x07;
	
	buf[4]=(u8)row;
	buf[5]=(u8)(row>>8);
	
	buf[6]=(u8)pit;
	buf[7]=(u8)(pit>>8);

	buf[8]=(u8)yaw;
	buf[9]=(u8)(yaw>>8);
	
	buf[10]=0x00;
	
	for(i=0; i < (buf[3]+4); i++)
	{
		sumcheck += buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
		addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
	}
	buf[11]=sumcheck;
	buf[12]=addcheck;
	
	USART1_SEND_BUF_U8(buf,13);
	
}
