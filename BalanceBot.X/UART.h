/* File: UART.h */

#define _XTAL_FREQ  8000000


#define BaudRate    115200 
#define TX_D        TRISC6
#define RX_D        TRISC7

//-----------[ Functions' Prototypes ]--------------

//---[ UART Routines ]---

void UART_TX_Init(void);
void UART_Write(unsigned char);
void UART_Write_String(char*);