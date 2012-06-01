/* 
 * File:   temperaturaADC.c
 * Author: albarc
 *
 * Created on 30 de mayo de 2012, 11:58
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p18lf45k80.h>
#include <adc.h>
#include <timers.h>
#include <delays.h>
#include "pragmas.h"
#include "functions.h"

// Defines
#define VDD 3360

// Variables
int refreshTime = 2;    //Cada 5 segundos por defecto

// Funciones
void init(void);
void SendMessage (char* Msg );
void SetTimer1(unsigned int time);
void SetRefreshTime(int refresh);

//*********************************************************************
// Interrupt Service Routine
//*********************************************************************

//#pragma interrupt service_isr
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void service_isr(void);

// This function directs execution to the actual interrupt code.

void high_ISR(void){
    _asm GOTO service_isr _endasm
}

#pragma code
#pragma interrupt service_isr
int cuentaInt = 0;
int cuentaSegundos = 0;

void service_isr(){
    // INTERRUPCION TIMER1 OVERFLOW
    if(PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;
        SetTimer1(62500); //250ms = 4us*25000
        cuentaInt++;
        cuentaSegundos = cuentaInt/4;
    }
}

/*
 * 
 */
//int main(int argc, char** argv) {
void main(void) {
    char msg[40];
    int lecturaADC = 0;
    long voltajeTemp = 0;
    int diff = 0;
    int tempx10 = 0;
    int tempEntero = 0;
    double temperatura = 0.0;

    init();
    init_canport();
    
    sprintf(msg, "Leer temperatura. Inicio\r\n");
    SendMessage(msg);

    while(1){
        if (cuentaSegundos >= refreshTime){
            cuentaInt = 0;
            cuentaSegundos = 0;
            
            Delay100TCYx(10);
            ConvertADC();
            while(BusyADC());
            lecturaADC = ReadADC();

            // Regla de tres para sacar los milivoltios
            // El valor máximo del ADC son 4095 (con la entrada a Vdd)
            // ¿Por qué 4095 si son 10 bits?
    //        voltajeTemp = ((long)lecturaADC*VDD)/4095;
            voltajeTemp = ((long)lecturaADC*VDD)/4150; // 4095 se pasa un poco (10-20 mV de más)

            diff = (int)(750 - voltajeTemp); // 750 mV = 25ºC; 10mV/ºC
            tempx10 = 250 - diff; // temperatura real * 10
            temperatura = ((double)tempx10)/10.0;
            tempEntero = (int)(temperatura + 0.5);  // Para redondear al entero más cercano

//            sprintf(msg, "tempx10 = %d\r\n", tempx10);
//            SendMessage(msg);
            sprintf(msg, "tempEntero = %d\r\n", tempEntero);
            SendMessage(msg);
            
            ECAN_Transmit(tempEntero);

//            sprintf(msg, "txb0d1:%d\n", TXB0D1);
//            SendMessage(msg);
//            sprintf(msg, "txb0d2:%d\n", TXB0D2);
//            SendMessage(msg);
        }
    }
}

void init(void) {
    // Deshabilita el sensor de temperatura de PICDEM Z #CS = 1
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1;

    // UART
    TXSTA1 = 0x20;  //Transmit is enabled
    RCSTA = 0x90;   //Serial port is enabled | Enables receiver
    SPBRG = 12;     //Baud rate = 9600 a MHz. Formula, SPBRG = (Fosc / (64 * Baud rate) ) - 1
    TRISCbits.TRISC6 = 0; //TX out
    TRISCbits.TRISC7 = 1; //RX in

    // ADC
    TRISBbits.TRISB0 = 1; // RB0 (Canal 10 ADC) INPUT
    ADCON0bits.ADON = 0; // Apagado al configurar
    ADCON1 = 0b00000000; // all ports to analogs
    ADCON0bits.CHS = 10; // CH AN10
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido

    // Interrupción Timer 1
    INTCON = 0xC0;
    T1CON = 0x3F; //0x3B;
    T1GCON = 0x00;
    PIE1bits.TMR1IE = 1;
    SetTimer1(62500); //250ms = (8Mhz/32)^-1*62500
}

void SendMessage (char* Msg ) {
    int len, count;

    len = strlen(Msg);
    for (count = 0; count < len; count++){
        //while( !PIR1bits.TX1IF );
        while(!TXSTA1bits.TRMT);
        TXREG = *(Msg+count);
    }
}

void SetTimer1(unsigned int time)
{
	TMR1H = 0-(time/256);
	TMR1L = 0-(time&0xff);
        PIR1bits.TMR1IF = 0;
}

void SetRefreshTime(int refresh)
{
    refreshTime = refresh;
}