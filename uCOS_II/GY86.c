#include "includes.h"

void delay_us(int nus){
    int i;
    for(i = 0; i < nus; ++i);
}


#ifdef AHRS

int GetAttdData(AttdData* pAttdData) {
    extern float Pitch, Roll, Yaw;
    Get_AHRS_Data();
    AHRS_Update(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
    pAttdData->pitch = Pitch;
    pAttdData->roll = Roll;
    pAttdData->yaw = Yaw;
    return 0;
}

    extern float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
int GetSpeedData(SpeedData* pSpeedData) {
    short pittmp, roltmp, yawtmp;
    uint8_t data_write[14];
    if(!i2cread(MPU6050_Addr2, ACCEL_XOUT_H, 14, data_write)){
        pittmp = (((int16_t)data_write[8])<<8) | data_write[9];
        roltmp = (((int16_t)data_write[10])<<8) | data_write[11];
        yawtmp = (((int16_t)data_write[12])<<8) | data_write[13];
        pSpeedData->pitchspeed = -((float)pittmp - Gyro_Xout_Offset) / Gyro_500_Scale_Factor;
        pSpeedData->rollspeed = ((float)roltmp - Gyro_Yout_Offset) / Gyro_500_Scale_Factor;
        pSpeedData->yawspeed = ((float)yawtmp - Gyro_Zout_Offset) / Gyro_500_Scale_Factor;
        return 0;
    }
    return -1;
}

#else

//初始化IIC
void IIC_Init(void)
{			
  RCC->AHB1ENR |= (1 << 1);  //使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIOB->MODER &=    ~((3 << 8 * 2) | (3 << 9 * 2));
  GPIOB->MODER |=    (1 << 8 * 2) | (1 << 9 * 2);
  GPIOB->OTYPER &=   ~((1 << 8) | (1 << 9));
  GPIOB->OSPEEDR |=    (3 << 8 * 2) | (3 << 9 * 2);
  GPIOB->PUPDR &=    ~((3 << 8 * 2) | (3 << 9 * 2));
  GPIOB->PUPDR |=    (1 << 8 * 2) | (1 << 9 * 2);
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

int GetGY86Data(AttdData* pAttdData) {
    short accx,accy,accz;
    float ax,ay,az;
    float anglex,angley,anglez;
    
    MPU_Get_Accelerometer(&accx,&accy,&accz);
    ax = accx/16384.0; //量程为+-2g,16位，65536除4
    ay = accy/16384.0;
    az = accz/16384.0;
    if(accx < 0) anglex = -accx;
    else anglex = accx;
    if(accy < 0) angley = -accy;
    else angley = accy;
    if(accz < 0) anglez = -accz;
    else anglez = accz;
    if(ax > 1) ax = 1;
    else if (ax < -1) ax=1;
    if(ay > 1) ay = 1;
    else if (ax < -1) ax=1;
    if(az > 1) az = 1;
    else if (ax < -1) ax=1;
    
    anglex = acos(ax) * 57.29577; //反余弦，乘弧度rad
    angley = acos(ay) * 57.29577;
    anglez = acos(az) * 57.29577;
    
    //printf("accX=%.2fg accY=%.2fg accZ=%.2fg\r\n", ax, ay, az);
    pAttdData->pitch = angley;
    pAttdData->roll = anglex;
    pAttdData->yaw = anglez;
    return pAttdData;
}

#endif
