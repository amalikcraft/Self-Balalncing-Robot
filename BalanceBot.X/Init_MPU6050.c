
#include <xc.h>
#include "I2C.h"
#include "MPU6050.h"
#include <math.h>
#include "UART.h"  // for debugging serial terminal
#include <stdio.h>



void MPU6050_Init(){
    
     //________________Initialize Wheel MPU6050  at address 0x68 
    
    
  // Power-Up Delay & I2C_Init
  __delay_ms(100);
  I2C_Master_Init();
 
  // Setting The Sample (Data) Rate
  I2C_Start(0xD0);
  I2C_Master_Write(SMPLRT_DIV);
  I2C_Master_Write(0x07);
  I2C_Master_Stop();
 
  // Setting The Clock Source
  I2C_Start(0xD0);
  I2C_Master_Write(PWR_MGMT_1);
  I2C_Master_Write(0x01);
  I2C_Master_Stop();
 
  // Configure The DLPF
  I2C_Start(0xD0);
  I2C_Master_Write(CONFIG);
  I2C_Master_Write(0x00);
  I2C_Master_Stop();
 
  // Configure The ACCEL (FSR= +-2g)
  I2C_Start(0xD0);
  I2C_Master_Write(ACCEL_CONFIG);
  I2C_Master_Write(0x00);
  I2C_Master_Stop();
 
  // Configure The GYRO (FSR= +-2000d/s)
  I2C_Start(0xD0);
  I2C_Master_Write(GYRO_CONFIG);
  I2C_Master_Write(0x18);
  I2C_Master_Stop();
 
  // Enable Data Ready Interrupts
  I2C_Start(0xD0);
  I2C_Master_Write(INT_ENABLE);
  I2C_Master_Write(0x01);
  I2C_Master_Stop();
  
  //Disable Temp,GY,GZ from FIFO;
  I2C_Start(0xD0);
  I2C_Master_Write(FIFO_EN);	
  I2C_Master_Write(0x4F);
  I2C_Master_Stop();
  
  
  
  //________________Initialize ARM MPU6050  at address 0x69
  
   
  // Setting The Sample (Data) Rate
  I2C_Start(0xD2);
  I2C_Master_Write(SMPLRT_DIV);
  I2C_Master_Write(0x07);
  I2C_Master_Stop();
 
  // Setting The Clock Source
  I2C_Start(0xD2);
  I2C_Master_Write(PWR_MGMT_1);
  I2C_Master_Write(0x01);
  I2C_Master_Stop();
 
  // Configure The DLPF
  I2C_Start(0xD2);
  I2C_Master_Write(CONFIG);
  I2C_Master_Write(0x00);
  I2C_Master_Stop();
 
  // Configure The ACCEL (FSR= +-2g)
  I2C_Start(0xD2);
  I2C_Master_Write(ACCEL_CONFIG);
  I2C_Master_Write(0x00);
  I2C_Master_Stop();
 
  // Configure The GYRO (FSR= +-2000d/s)
  I2C_Start(0xD2);
  I2C_Master_Write(GYRO_CONFIG);
  I2C_Master_Write(0x18);
  I2C_Master_Stop();
 
  // Enable Data Ready Interrupts
  I2C_Start(0xD2);
  I2C_Master_Write(INT_ENABLE);
  I2C_Master_Write(0x01);
  I2C_Master_Stop();
  
    //Disable Temp,GX,GZ, from FIFO;
  I2C_Start(0xD2);
  I2C_Master_Write(FIFO_EN);	
  I2C_Master_Write(0x2F);
  I2C_Master_Stop();
  
 
}