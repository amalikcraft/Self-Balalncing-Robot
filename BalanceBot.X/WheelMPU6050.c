
#include <xc.h>
#include "I2C.h"
#include "MPU6050.h"
#include <math.h>
#include "UART.h"  // for debugging serial terminal
#include <stdio.h>
#include "WheelMPU6050.h"




int Ax,Ay,Az,Gx;
double AX, AY, AZ, GX;
double stateAngle;
double wheelOffset = 0.0;


double degreeVar = .1; //.1 degree
double KalmanAngle = 0; 
double KalmanUncertaintyAngle = 4;
double KalmanOutput[]= {0,0};
double KalmanGain = 0;


void WheelKalmanFilter(double KalmanState, double KalmanUncertainty, double KalmanInput, double KalmanMeasurement) {
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.000256;
  KalmanGain = KalmanUncertainty * 1/(KalmanUncertainty + degreeVar * degreeVar);
  KalmanState = KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty =(1-KalmanGain) * KalmanUncertainty;
  KalmanOutput[0] = KalmanState; 
  KalmanOutput[1] = KalmanUncertainty;
}


double WheelMPU6050_Read(){
    
  // Prepare For Reading, Starting From ACCEL_XOUT_H
  I2C_Start(0xD0);
  I2C_Master_Write(ACCEL_XOUT_H);
  I2C_Master_Stop();
  I2C_Start(0xD1);
  I2C_Read(0); // dummy
  Ax = (I2C_Read(0)<<8) | I2C_Read(0);
  Ay = (I2C_Read(0)<<8) | I2C_Read(0);
  Az = (I2C_Read(0)<<8) | I2C_Read(0);
  Gx = (I2C_Read(0)<<8) | I2C_Read(1);
  I2C_Master_Stop();
 
  // Convert The Readings
  AX = Ax/16384.0;// - 0.032;
  AY = Ay/16384.0;// - 0.025;
  AZ = Az/16384.0;// - 0.075;
  GX = Gx/131.0; //+ 0.34;;

  
  //Compute Angle with Kalman filter Acc+Gyro
  
  stateAngle = atan(AX/sqrt(AY*AY + AZ*AZ))*57.3;
  WheelKalmanFilter(KalmanAngle, KalmanUncertaintyAngle, GX, stateAngle);
  KalmanAngle = KalmanOutput[0]; 
  KalmanUncertaintyAngle = KalmanOutput[1];

  return KalmanAngle;
  
  
}


void calibrate_wheel_gyro(){
    double x = 0;
    for(int i = 0; i< 30; i++){
        x = WheelMPU6050_Read();
    }
}