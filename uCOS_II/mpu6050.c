#include "includes.h"

// Factor for converting radian to degree

// Sensor data value
float init_ax, init_ay, init_az,
      init_gx, init_gy, init_gz,
      init_mx, init_my, init_mz;

// Offset of angular velocity zero drift
float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
// Offset of acceleration zero drift
float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;

// Get gyroscope bias
void Get_Gyro_Bias(void* p_arg)
{
    p_arg = p_arg;
    int16_t gyro[3];
    uint8_t data_write[6];
    extern int unlocked;
    puts("[GyroBias] Start");
    int cnt = 0;
    extern char usart_tx_buf[256];

    while(1){
        if(unlocked == 1){
            Gyro_Xout_Offset /= (float) cnt;
            Gyro_Yout_Offset /= (float) cnt;
            Gyro_Zout_Offset /= (float) cnt;
            puts("[GyroBias] End");
            /*
            sprintf(usart_tx_buf, "gx = %.2f, gy = %.2f, gz = %.2f\r\n", Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset);
            USART1_SEND_BUF_U8((u8*)usart_tx_buf, strlen(usart_tx_buf));
            */
            OSTaskDel(OS_PRIO_SELF);
        }
        OSTimeDly(50);
        if(!i2cread(MPU6050_Addr2, GYRO_XOUT_H, 6, data_write))
        {
            // Get xyz angular velocity
            gyro[0] = ((((int16_t)data_write[0])<<8) | data_write[1]);
            gyro[1] = ((((int16_t)data_write[2])<<8) | data_write[3]);
            gyro[2] = ((((int16_t)data_write[4])<<8) | data_write[5]);
            Gyro_Xout_Offset = (Gyro_Xout_Offset + gyro[0]);
            Gyro_Yout_Offset = (Gyro_Yout_Offset + gyro[1]);
            Gyro_Zout_Offset = (Gyro_Zout_Offset + gyro[2]);
            cnt++;
        }
    }
}

// Get accelerometer bias
void Get_Accel_Bias(void* p_arg) {
    p_arg = p_arg;
    int16_t accel[3]; 
    uint8_t data_write[6];
    extern int unlocked;
    puts("[AccelBias] Start");
    int cnt = 0;
    extern char usart_tx_buf[256];

    while(1){
        if(unlocked == 1){
            Accel_Xout_Offset /= (float) cnt;
            Accel_Yout_Offset /= (float) cnt;
            Accel_Zout_Offset /= (float) cnt;
            puts("[AccelBias] End");
            /*
            sprintf(usart_tx_buf, "ax = %.2f, ay = %.2f, az = %.2f\r\n", Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset);
            USART1_SEND_BUF_U8((u8*)usart_tx_buf, strlen(usart_tx_buf));
            */
            OSTaskDel(OS_PRIO_SELF);
        }
        OSTimeDly(50);
        if(!i2cread(MPU6050_Addr2, ACCEL_XOUT_H, 14, data_write))
        {
            // Get xyz acceleration
            accel[0] = (((int16_t)data_write[0])<<8) | data_write[1];
            accel[1] = (((int16_t)data_write[2])<<8) | data_write[3];
            accel[2] = (((int16_t)data_write[4])<<8) | data_write[5];
            Accel_Xout_Offset = (Accel_Xout_Offset + accel[0]);
            Accel_Yout_Offset = (Accel_Yout_Offset + accel[1]);
            Accel_Zout_Offset = (Accel_Zout_Offset + accel[2]);
            cnt++;
        }
    }
}

// MPU6050 initialization
void MPU6050_Init(void)
{
    // Release sleeping status
    I2C_WriteByte(MPU6050_Addr, PWR_MGMT_1, 0x01);

    // Set the sampling frequency to 1KHZ
    I2C_WriteByte(MPU6050_Addr, SMPLRT_DIV, 0x00);
    // Set the bandwidth of low-pass filtering to 5HZ
    I2C_WriteByte(MPU6050_Addr, CONFIG, 0x06);

    // Enable bypass I2C
    I2C_WriteByte(MPU6050_Addr, INT_PIN_CFG, 0x42);
    // Enable FIFO operation
    I2C_WriteByte(MPU6050_Addr, USER_CTRL, 0x40);

    // Set the gyroscope acquisition range to +-500бу/s
    I2C_WriteByte(MPU6050_Addr, GYRO_CONFIG, 0x0B);
    // Set the accelerometer acquisition range to +-4g
    I2C_WriteByte(MPU6050_Addr, ACCEL_CONFIG, 0x08);

    /*
    // Get the zero drift offset
    Get_Gyro_Bias();
    Get_Accel_Bias();
    */
}

// HMC5883L initialization
void HMC5883L_Init(void)
{
    // Set the standard data output rate to 75HZ
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_ConfigurationRegisterA, 0x18);
    // Set the sampling frequency to +-1.3Ga
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_ConfigurationRegisterB, 0x20);
    // Turn on continuous measurement mode
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_ModeRegister, 0x00);
}

// Get sensor data for AHRS
void Get_AHRS_Data(void)
{
    int16_t gyro[3], accel[3];
    uint8_t data_write[14];

    // Get acceleration and angular velocity
    if(!i2cread(MPU6050_Addr2, ACCEL_XOUT_H, 14, data_write))
    {
        accel[0] = (((int16_t)data_write[0])<<8) | data_write[1];
        accel[1] = (((int16_t)data_write[2])<<8) | data_write[3];
        accel[2] = (((int16_t)data_write[4])<<8) | data_write[5];
        gyro[0]  = (((int16_t)data_write[8])<<8) | data_write[9];
        gyro[1]  = (((int16_t)data_write[10])<<8) | data_write[11];
        gyro[2]  = (((int16_t)data_write[12])<<8) | data_write[13];
        
        // Zero drift processing and unit conversion
        init_ax = (float)(accel[0] - Accel_Xout_Offset) / Accel_4_Scale_Factor;
        init_ay = (float)(accel[1] - Accel_Yout_Offset) / Accel_4_Scale_Factor;
        init_az = (float)(accel[2] + (Accel_4_Scale_Factor - Accel_Zout_Offset)) / Accel_4_Scale_Factor;
        init_gx = ((float)gyro[0] - Gyro_Xout_Offset) / Gyro_500_Scale_Factor / ARC_TO_DEG;
        init_gy = ((float)gyro[1] - Gyro_Yout_Offset) / Gyro_500_Scale_Factor / ARC_TO_DEG;
        init_gz = ((float)gyro[2] - Gyro_Zout_Offset) / Gyro_500_Scale_Factor / ARC_TO_DEG;
    }
    // Get magnetic flux
    if(!i2cread(HMC5883L_Addr2, HMC5883L_XOUT_MSB, 6, data_write))
    {    
        init_mx = (data_write[0] << 8) | data_write[1];
        init_my = (data_write[4] << 8) | data_write[5];
        init_mz = (data_write[2] << 8) | data_write[3];

        // Complement processing and unit conversion
        if(init_mx > 0x7fff) init_mx-=0xffff;
        if(init_my > 0x7fff) init_my-=0xffff;
        if(init_mz > 0x7fff) init_mz-=0xffff;
        init_mx /= 1090.0f;
        init_my /= 1090.0f;
        init_mz /= 1090.0f;
    }
}
