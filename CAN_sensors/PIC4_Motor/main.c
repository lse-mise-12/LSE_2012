#include <p18lf45k80.h>
#include "stdio.h"
#include "stdlib.h"
#include <stdlib.h>
#include <string.h>
#include <adc.h>            // ADC library functions
#include <portb.h>          // PORTB library function
#include <timers.h>
#include <delays.h>
#include <pwm.h>
#include <capture.h>
#include "pragmas.h"
#include "functions.h"



#define MAXSERIALMSG        80
//#define NUM_CONVERTIONS     50
#define NUM_CONVERTIONS     10
#define ADC_TOP             1979

//---------------------------------------------------------------------
// Function Prototypes
//---------------------------------------------------------------------

unsigned int num_pulsos = 0, tiempo = 0, tiempo2=0, lectura_velocidad = 0, interrup=0;
unsigned long tiempo_total = 0;
unsigned int velocidad_actual = 0;
unsigned long temp_ADC_conv = 0;
unsigned long ADC_conv = 0;
unsigned char pulsadaTeclaONOFF = 0;
int enviar = 0, ledOFF=0, start=0;
int R4down = 0;
int entrapulso = 0;
int R5down = 0;
int R6down = 0;

int RD2down = 0;

unsigned char new_msg=0, arranque = 0;



void init(void);
//void pwm(int);
void delayms(int);
int velocidad(int);
void SetupEUSART(void);
void SendMessage (char* Msg );
void SetTimer1(unsigned int time);
void SetTimer0(unsigned int time);

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

unsigned long pulsos = 0;
unsigned char pulsos_total = 0x00;

char rpms[15];


void service_isr()
{
    // Can interrupt new message.
    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
    }

    
    // INTERRUPCION TIMER1 OVERFLOW
    if(PIR1bits.TMR1IF == 1){
        //if(arranque==0x01) {
        pulsos_total +=  pulsos;

        pulsos = 0;
        PIR1bits.TMR1IF = 0;
        SetTimer1(62500); //250ms = 4us*25000
        enviar++;
        if (enviar > 4){
            enviar = 0;
            pulsos_total = 0;
        }
       // }
    }


    // INTERRUPCION PULSADOR RB7-RB4
    if(INTCONbits.RBIF == 1){
        //if(arranque==0x01) {
        if(PORTBbits.RB4){
            if(entrapulso){
                entrapulso = 0;
                pulsos++;
            }
        }
        else
        {
            entrapulso = 1;
        }

        if(PORTBbits.RB7){
            if(R4down){
                R4down = 0;
                //se activa el CC
                if (pulsadaTeclaONOFF == 1){
                    pulsadaTeclaONOFF = 0; // desactivar CC
                    if(ledOFF==0) {
                    PORTCbits.RC4 = 0;
                    }
                    else{
                    PORTCbits.RC4 = 0;}
                    //sprintf(rpms, " CC OFF \n");
                    //SendMessage(rpms);
                    //SetDCPWM2(50);
                }else{
                    pulsadaTeclaONOFF = 1; // activamos el CC
                    if(ledOFF==1) {
                    PORTCbits.RC4 = 0;
                    }
                    else{
                    PORTCbits.RC4 = 1;
                    }
                    velocidad_actual = velocidad(ADC_conv);
                    //sprintf(rpms, " CC ON (v %u) \n", velocidad_actual);
                    //SendMessage(rpms);
                }
            }
        }else
            R4down = 1;

        if(pulsadaTeclaONOFF == 1){
            if(PORTBbits.RB5){ //se sube la velocidad
                 if(R5down){
                    R5down = 0;
                    if (velocidad_actual > 75)
                        velocidad_actual = 80;
                    else
                        velocidad_actual = velocidad_actual + 5;
                 }
            }else
                R5down = 1;

            if(PORTBbits.RB6){ //se baja la velocidad
                 if(R6down){
                    R6down = 0;
                    if (velocidad_actual < 10)
                        velocidad_actual = 0;
                    else
                        velocidad_actual = velocidad_actual - 5;
                 }
            }else
                R6down = 1;
            //sprintf(rpms, " V- (%u) \n", velocidad_actual);
            //SendMessage(rpms);
        }
        INTCONbits.RBIF = 0;
    //}
    }
    
}

//main
void main(void){


    char msg_inicial[] = "\n\n  PIC18LF45K80 \r\nMotor Module V8-350CV\n\n\n";
    char rpms[15];
    
    int n = 0;
    int ar = 0;

    //unsigned char arranque = 0x02;
    SetupEUSART();
    init_canport();
    init();
    SendMessage(&msg_inicial[0]);

    seqnumber = 0x00;

    mensaje.destIDH = 0x00;
    mensaje.destIDL = 0x00;
    mensaje.destino = 0x00;
    mensaje.type = 0x00;
    mensaje.seq_number = seqnumber;
    mensaje.variable = 0x00;
    mensaje.byteH = 0x00;
    mensaje.byteL = 0x00;
    mensaje.datos = 0x00;
    mensaje.CRC = DATA;

    PORTCbits.RC4 = 0;
    temp_ADC_conv = 0;
    SetDCPWM2(0);
    R4down = 1;
    //arranque=ECAN_Receive();

    //ECAN_Transmit(arranque);
    //pulsos_total = RXB0D1;
    //ECAN_Transmit(pulsos_total);

    while(1){
//        write_msg(TSIDH, TSIDL, TEST, INST_TEST, pulsos_total, pulsadaTeclaONOFF, 0);
//        ECAN_Transmit(mensaje, 0);      // Reenviar TEST
        if ( (new_msg == 1) && (RX_TYP == TEST) && (RXB0D2 == TEST_VALUE) ) {               // Comprobar tipo test
                write_msg(TSIDH, TSIDL, TEST, INST_TEST, pulsos_total, pulsadaTeclaONOFF, 0);
                ECAN_Transmit(mensaje, 0);      // Reenviar TEST
        }
        if ( ( new_msg == 1 ) && ( RX_LSB == STATE_ON ) && (RX_VAR == INST_STATE) && (RX_TYP == WRITE) ){
            start = 1;
        }
        if ( ( new_msg == 1 ) && ( RX_LSB == STATE_OFF ) && (RX_VAR == INST_STATE) && (RX_TYP == WRITE) ){
            start = 0;
        }

        while(start == 1){

        if ( (new_msg == 1) && (RX_TYP == TEST) && (RXB0D2 == TEST_VALUE) ) {               // Comprobar tipo test
                write_msg(TSIDH, TSIDL, TEST, INST_TEST, pulsos_total, pulsadaTeclaONOFF, 0);
                ECAN_Transmit(mensaje, 0);      // Reenviar TEST
        }
        if ( ( new_msg == 1 ) && ( RX_LSB == STATE_ON ) && (RX_VAR == INST_STATE) && (RX_TYP == WRITE) ){
            start = 1;
        }
        if ( ( new_msg == 1 ) && ( RX_LSB == STATE_OFF ) && (RX_VAR == INST_STATE) && (RX_TYP == WRITE) ){
            start = 0;
        }
            
        //arranque=ECAN_Receive();
        //arranque=RXB0D1;
        //ECAN_Transmit(arranque);
        /*
        if(new_msg == 1){
            arranque=RXB0D1;
            ar = 1;
        }
        */

    //    if(start == 0) {
    //        ledOFF = 1;
    //        SetDCPWM2(0);
    //        PORTCbits.RC4 = 0;
    //    }
    //    if(start == 1){
            ledOFF=0;
            //INTCONbits.RBIE =1;
        
            //arranque = 0xAA;
            //ECAN_Transmit(arranque);
            //ECAN_Transmit(TXB0SIDLbits.SID);
            //pulsos_total++;

            //sprintf(rpms, "tr0\n");
            ConvertADC();
            //sprintf(rpms, "tr1\n");
            while(BusyADC());
            //sprintf(rpms, "tr2\n");

            temp_ADC_conv += ReadADC();

            n++;
            if(n > NUM_CONVERTIONS){
                ADC_conv = temp_ADC_conv / n;
                n = 0;
                temp_ADC_conv = 0;
            }

            if(pulsadaTeclaONOFF == 0){
                velocidad_actual = velocidad(ADC_conv);
                //sprintf(rpms, "CC OFF \n");
            }
            else if (velocidad(ADC_conv) > velocidad_actual + 5){
                pulsadaTeclaONOFF = 0;
                PORTCbits.RC4 = 0;
                //sprintf(rpms, "CC auto off \n");
                //SendMessage(rpms);
            }

            SetDCPWM2(velocidad_actual);
//ECAN_Transmit(enviar);
            if(enviar == 4 ){
                enviar = 0;
                //sprintf(rpms, "P/s = %lu\n", pulsos_total);
                //SendMessage(rpms);
                //ECAN_Transmit(pulsos_total);
                write_msg(TSIDH, TSIDL, READ, INST_VELOC, 0, 0, pulsos_total);
                ECAN_Transmit(mensaje, 1);      // Reenviar TEST

                pulsos_total=0;
                //PORTCbits.RC4 =! PORTCbits.RC4;
            }
           // RXB0D1=0x00;// No se puede escribir en un registro de lectura ;)
        }


            ledOFF = 1;
            SetDCPWM2(0);
            PORTCbits.RC4 = 0;
            velocidad_actual = 0;
            pulsos_total=0;
            enviar = 0;
            pulsadaTeclaONOFF = 0;

        //else{
            //pulsos_total = 0xBC;

            //ECAN_Transmit(pulsos_total);
            //INTCONbits.RBIF = 0;
           //INTCONbits.RBIE =0;
            //SetDCPWM2(0);
         //   ledOFF=1;
           // PORTCbits.RC4 = 0;
        //}

    }
}


void init(void){


TRISAbits.TRISA0 = 1;
LATAbits.LATA0 = 0;
TRISCbits.TRISC2 = 0;


EnablePullups();
TRISBbits.TRISB5 = 1; //RB5 digital IN: boton de speed up
TRISBbits.TRISB6 = 1; //RB6 digital IN: boton de speed down
TRISBbits.TRISB7 = 1; //RB7 digital IN: boton de on/off
TRISDbits.TRISD4 = 1; //RD4 digital IN: ECCP1
TRISBbits.TRISB3 = 1;
TRISCbits.TRISC4 = 0;

TRISDbits.TRISD2 = 0;
/////////// INICIALIZACION ADC
// arriba PA.0 configurado como entrada
ANCON0 = 0x00;  // ADC OFF
ADCON1 = 0x00; // AN0 as analog input
ADCON2 = 0x81;
ADCON0 = 0x01; // ADC ON
ANCON1 = 0x00;
/////////////////////////
PORTCbits.RC4 = 0;
PORTDbits.RD2 = 0;
/////////////////////////////
// Habilitar interrupciones PORTB
INTCON = 0xE8;
IOCB = 0xF0;

//////////////////////////////////

RCONbits.IPEN=1;
PIE3 = 0X02; //ECCP1 Interrupt Enable Bit

PIE5bits.RXB0IE = 1;
PIE5bits.RXB1IE = 1;

INTCONbits.PEIE = 1; // Enable peripheral interrupts
INTCONbits.GIE = 1; // Enable global interrupts
//PIR3=0x02;
//////////////////////////////

// PWM
OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
//OpenPWM2(255, 0x00); // 100Hz
OpenPWM2(19, 0x00); // 100KHz
SetDCPWM2(0);
// Interrupcion Timer 1
T1CON = 0x3F; //0x3B;
T1GCON = 0x00;
PIE1bits.TMR1IE = 1;
SetTimer1(62500); //250ms = (8Mhz/32)^-1*62500

}

int velocidad(int ADC_conv){
    unsigned long temp;
    if(ADC_conv > ADC_TOP) ADC_conv = ADC_TOP;
    temp = ((unsigned long)(ADC_conv)*80000)/ADC_TOP;
    return temp/1000;

}


void delayms(int tiempo){                       // delays en ms a 8MHz ORIGINAL
                                            // como yo estoy a 16 MHz tiempo x2

    int tiempo_16MHz = 2* tiempo;
    int cuenta_ = 0;
    while ( cuenta_ < tiempo_16MHz )
    {
            Delay1KTCYx(2);
            cuenta_++;
    }
}


void SetupEUSART(void) {
    TXSTA1 = 0x20;  //Transmit is enabled
    RCSTA = 0x90;   //Serial port is enabled | Enables receiver
    SPBRG = 12;     //Baud rate = 9600 a MHz. Formula, SPBRG = (Fosc / (64 * Buad rate) ) - 1
    TRISCbits.TRISC6 = 0; //TX out
    TRISCbits.TRISC7 = 1; //RX in
}


void SetTimer1(unsigned int time)
{
	TMR1H = 0-(time/256);
	TMR1L = 0-(time&0xff);
        PIR1bits.TMR1IF = 0;
}
void SetTimer0(unsigned int time)
{
       // TMR0H = 0x00;
	//TMR0L = 0x00;
	TMR0H = 0-(time/256);
	TMR0L = 0-(time&0xff);
	INTCONbits.TMR0IF = 0;
}

