
#include <xc.h>
#include "I2C.h"
#include "MPU6050.h"
#include <math.h>

#include "UART.h"  // for debugging serial terminal
#include <stdio.h>



int Ax,Ay,Az,T,Gx,Gy,Gz;
double AX, AY, AZ, t, GX, GY, GZ;
double stateAngle;


double degreeVar = .1; //.1 degrees

double KalmanAngle = 0; 
double KalmanUncertaintyAngle = 2*2;
double KalmanOutput[]= {0,0};
double KalmanGain = 0;


void kalmanFilter(double KalmanState, double KalmanUncertainty, double KalmanInput, double KalmanMeasurement) {
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  KalmanGain = KalmanUncertainty * 1/(KalmanUncertainty + degreeVar * degreeVar);
  KalmanState = KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty =(1-KalmanGain) * KalmanUncertainty;
  KalmanOutput[0] = KalmanState; 
  KalmanOutput[1] = KalmanUncertainty;
}




void MPU6050_Init()
{
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
  

  
  
}

double WheelMPU6050_Read(){
    
  // Prepare For Reading, Starting From ACCEL_XOUT_H
  I2C_Start(0xD0);
  I2C_Master_Write(ACCEL_XOUT_H);
  I2C_Master_Stop();
  I2C_Start(0xD0);
  I2C_Read(0); // dummy
  Ax = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  Ay = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  Az = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  T  = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  Gx = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  Gy = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
  Gz = ((int)I2C_Read(0)<<8) | (int)I2C_Read(1);
  I2C_Master_Stop();
 
  // Convert The Readings
  AX = (double)Ax/16384.0 - 0.032;
  AY = (double)Ay/16384.0 - 0.025;
  AZ = (double)Az/16384.0 - 0.075;
  GX = (double)Gx/131.0; //+ 0.34;;
  t =  (double)T/340.00 +36.53;
  
  //Compute Angle with Kalman filter Acc+Gyro
  
  stateAngle = atan(AX/sqrt(AY*AY + AZ*AZ))*1/(3.14159/180);
  kalmanFilter(KalmanAngle, KalmanUncertaintyAngle, GX, stateAngle);
  KalmanAngle = KalmanOutput[0]; 
  KalmanUncertaintyAngle = KalmanOutput[1];

  return KalmanAngle;
  
  
}


void calibrate_gyro(){
    double x = 0;
    for(int i = 0; i< 30; i++){
        x = MPU6050_Read();
    }
}