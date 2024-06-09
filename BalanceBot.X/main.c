
#include <xc.h>
#include <stdio.h>
#include "config.h"
#include "I2C.h"
#include "UART.h"
#include "MPU6050.h"
#include <stdbool.h>
#include "ArmMPU6050.h"
#include "Init_MPU6050.h"
#include "WheelMPU6050.h"


//Wheel Globals
double CurrentWheelAngle = 0;
double DesiredAngle = 0;
double AngleError = 0;
double PrevAngleError = 0;
double PrevIterm = 0;
double Pterm;
double Iterm;
double Dterm;
double PID_U = 0;
double Max = 1;
double KP = .5,  KI = .3,  KD = 0.0015;

//Arm Globals
double CurrentArmAngle = 0;
double armDesiredAngle = 0;
double armAngleError = 0;
double armPrevAngleError = 0;
double armPrevIterm = 0;
double armPterm;
double armIterm;
double armDterm;
double armPID_U = 0;
double armMax = 5000;
double armKP = 25.0,  armKI = .001,  armKD = 0.15;



double ArmBalancePID(double InputAngle){
    
    
    armAngleError = InputAngle - armDesiredAngle;
    armPterm = armKP*(armAngleError);
    
    armIterm = armPrevIterm + armKI*(armAngleError + armPrevAngleError);
    if (armIterm>armMax)armIterm = armMax;
    else if(armIterm<-armMax)armIterm = -armMax; //integral wind-up
    
    armDterm = armKD * (armAngleError-armPrevAngleError);
    
    armPID_U = armPterm + armIterm - armDterm;
    //if (armPID_U>armMax)armPID_U = armMax;
    //else if(armPID_U<-armMax) armPID_U = -armMax;
    
    armPrevIterm = armIterm;
    armPrevAngleError = armAngleError;
    return armPID_U;

}

void ArmMotor(bool direction, unsigned int steps){
    
    TRISCbits.RC0 = 0; //white clk
    TRISCbits.RC1 = 0;
    LATCbits.LATC1 = direction;
    

    int frequency = 5000/steps; //10us pulses per step   700* 1/abs(steps)
    int i = 0;
    int k = 0;
    
    while(k<steps/30){ // number of steps
        LATCbits.LATC0 = 1;
        i = 0;
        while(i<frequency){  //speed control
            __delay_us(1);
            i++;
        }
        k++;        
        LATCbits.LATC0 = 0;
    } 
}


double WheelBalancePID(double InputAngle){
    
    
    AngleError = InputAngle - DesiredAngle;
    Pterm = KP*(AngleError);
    
    Iterm = PrevIterm + KI*(AngleError + PrevAngleError);
    if (Iterm>Max)Iterm = Max;
    else if(Iterm<-Max) Iterm = -Max; //integral wind-up
    
    Dterm = KD * (AngleError-PrevAngleError);
    
    PID_U = Pterm + Iterm - Dterm;
    if (PID_U>Max)PID_U = Max;
    else if(PID_U<-Max) PID_U = -Max;
    
    PrevIterm = Iterm;
    PrevAngleError = AngleError;
    return PID_U;

}


void BothWheels(bool direction, unsigned int steps){

    LATDbits.LATD1 = direction;
    LATBbits.LATB3 = ~direction;
    
    int frequency =1;// Max/steps;  //50     //10us pulses per step
        
    
    LATDbits.LATD0 = 1;
    LATBbits.LATB4 = 1;  

    for(int i = 0; i<frequency;i++){

    } 
    LATDbits.LATD0 = 0;
    LATBbits.LATB4 = 0;                  
    
    
}


void initializeCLK(){
    OSCCONbits.IRCF = 0x07; //set internal clock to 8MHz
    OSCCONbits.SCS = 0x03; //
    //while(OSCCONbits.IOFS != 1) //check if clock is stable
}

void main(void) {
    
    initializeCLK();
    UART_TX_Init();
    char buffer[40];

    MPU6050_Init();
    calibrate_arm_gyro();
    calibrate_wheel_gyro();
    
    TRISBbits.RB3 = 0;  //step right
    TRISBbits.RB4 = 0;  //white
    TRISDbits.RD1 = 0;  //step left
    TRISDbits.RD0 = 0;  //white
    
    

    unsigned int arm_u =0;
    unsigned int wheel_u =0;
    int h = 0;
    while(1){        
        
        
        CurrentArmAngle = ArmMPU6050_Read();
        armPID_U = ArmBalancePID(CurrentArmAngle);
        arm_u = abs(armPID_U);
        
        
        if(armPID_U>0){
            ArmMotor(1,arm_u);

        }
        else{
            ArmMotor(0,arm_u);
        }
        
        
        CurrentWheelAngle = WheelMPU6050_Read();
        PID_U = WheelBalancePID(CurrentWheelAngle);
        wheel_u = abs(PID_U);
        if(PID_U>0){
            BothWheels(0,wheel_u);
        }
        else{
            BothWheels(1,wheel_u);
        }
        
        sprintf(buffer, " WheelAngle =  %.1f  |  ArmAngle= %.1f  %d\r\n ", CurrentWheelAngle, CurrentArmAngle,h);
        UART_Write_String(buffer);
        

    }

}

