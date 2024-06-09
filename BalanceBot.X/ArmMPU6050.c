
#include <xc.h>
#include "I2C.h"
#include "MPU6050.h"
#include <math.h>
#include "UART.h"  // for debugging serial terminal
#include <stdio.h>
#include "ArmMPU6050.h"



int armAx,armAy,armAz,armGy;
double armAX, armAY, armAZ, armt, armGX, armGY, armGZ;
double armstateAngle;
double armOffset = 8.0;

double armdegreeVar = .1; //.1 degrees
double armKalmanAngle = 0; 
double armKalmanUncertaintyAngle = 4;//2*2;
double armKalmanOutput[]= {0,0};
double armKalmanGain = 0;


void ArmKalmanFilter(double armKalmanState, double armKalmanUncertainty, double armKalmanInput, double armKalmanMeasurement) {
  armKalmanState = armKalmanState + 0.004*armKalmanInput;
  armKalmanUncertainty = armKalmanUncertainty + .000256;//0.004 * 0.004 * 4 * 4;
  armKalmanGain = armKalmanUncertainty * 1/(armKalmanUncertainty + armdegreeVar * armdegreeVar);
  armKalmanState = armKalmanState + armKalmanGain * (armKalmanMeasurement-armKalmanState);
  armKalmanUncertainty =(1-armKalmanGain) * armKalmanUncertainty;
  armKalmanOutput[0] = armKalmanState; 
  armKalmanOutput[1] = armKalmanUncertainty;
}


double ArmMPU6050_Read(){
    
  // Prepare For Reading, Starting From ACCEL_XOUT_H

   I2C_Start(0xD2);
   I2C_Master_Write(ACCEL_XOUT_H);
   I2C_Master_Stop();
   I2C_Start(0xD3);  
   I2C_Read(0); // dummy
   armAx = (I2C_Read(0)<<8) | I2C_Read(0);
   armAy = (I2C_Read(0)<<8) | I2C_Read(0);
   armAz = (I2C_Read(0)<<8) | I2C_Read(0);
   armGy = (I2C_Read(0)<<8) | I2C_Read(1);
   I2C_Master_Stop();
 
  // Convert The Readings
  armAX = armAx/16384.0;// - 0.032;
  armAY = armAy/16384.0;// - 0.025;
  armAZ = armAz/16384.0;// - 0.075;
  armGY = armGy/131.0;

  
  //Compute Angle with Kalman filter Acc+Gyro
  
  armstateAngle = armOffset + atan(armAY/sqrt(armAX*armAX + armAZ*armAZ))*57.3;//1/(3.14159/180);
  ArmKalmanFilter(armKalmanAngle, armKalmanUncertaintyAngle, armGY, armstateAngle);
  armKalmanAngle = armKalmanOutput[0]; 
  armKalmanUncertaintyAngle = armKalmanOutput[1];

  return armKalmanAngle;
  
  
}


void calibrate_arm_gyro(){
    double x = 0;
    for(int i = 0; i< 30; i++){
        x = ArmMPU6050_Read();
    }
}