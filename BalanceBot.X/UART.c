
#include <xc.h>
#include "UART.h"


void UART_TX_Init(void){

    TRISCbits.RC7=1;
    TRISCbits.RC6=1;
    
    SPBRGH = (16 & 0xFF00) >> 8;
    SPBRG = 16 & 0x00FF;
    
    RCSTAbits.CREN = 1;
    RCSTAbits.SPEN = 1;
    BAUDCONbits.BRG16 = 1;
    TXSTAbits.SYNC = 0; 
    TXSTAbits.BRGH = 1;
    TXSTAbits.TXEN = 1;
    IPR1bits.RCIP=1;
    PIE1bits.RCIE=1;

}

void UART_Write(unsigned char data)
{
  while(!TRMT);
  TXREG = data;
}

void UART_Write_String(char* buf)
{
    int i=0;
    while(buf[i] != '\0')
        UART_Write(buf[i++]);
}