#include <p18lf45k80.h>
#include <stdio.h>
#include <stdlib.h>
#include <timers.h>
#include <string.h>
#include <delays.h>
#include "pragmas.h"
#include "CANFUNCTIONS.h"

unsigned char new_msg = 0;
int a = 0;
//*********************************************************************
// Interrupt Service Routine
//*********************************************************************

//#pragma interrupt service_isr
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void service_isr(void);

// This function directs execution to the actual interrupt code.

void high_ISR(void)
{
    _asm GOTO service_isr _endasm
}

#pragma code
#pragma interrupt service_isr

void service_isr()
{

    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
    }



}

void main(void) {

    char msg_inicial[] = "\n\n  PIC18LF45K80 \r\n  TS-FALSE - Control\n\n\n";
    char rpms[15];
    init_canport();
    SetupPortA();
    init_uartport();
    SendMessage(&msg_inicial[0]);

    delayms(1000);

    while(1){
        if (new_msg == 1){
            new_msg = 0;
            sprintf(rpms, "EIDH = %u\n", temp_EIDH);
            SendMessage(rpms);
            sprintf(rpms, "EIDL = %u\n", temp_EIDL);
            SendMessage(rpms);
            sprintf(rpms, "SIDH = %u\n", temp_SIDH);
            SendMessage(rpms);
            sprintf(rpms, "SIDL = %u\n", temp_SIDL);
            SendMessage(rpms);
            sprintf(rpms, "DLC = %u\n", temp_DLC);
            SendMessage(rpms);
            sprintf(rpms, "D0:CRC= %u\n", temp_D0);
            SendMessage(rpms);
            sprintf(rpms, "D1:DataL = %u\n", temp_D1);
            SendMessage(rpms);
            sprintf(rpms, "D2:DataH = %u\n", temp_D2);
            SendMessage(rpms);
            sprintf(rpms, "D3:Var = %u\n", temp_D3);
            SendMessage(rpms);
            sprintf(rpms, "D4:Seq = %u\n", temp_D4);
            SendMessage(rpms);
            sprintf(rpms, "D5:Type = %u\n", temp_D5);
            SendMessage(rpms);
            sprintf(rpms, "D6:Dest = %u\n", temp_D6);
            SendMessage(rpms);
            sprintf(rpms, "D7:Ver = %u\n", temp_D7);
            SendMessage(rpms);
            sprintf(rpms, "err = %u\n\n", temp_err);
            SendMessage(rpms);
        }
    }
}