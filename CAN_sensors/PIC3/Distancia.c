
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p18lf45k80.h>
#include <adc.h>
#include <pwm.h>
#include <math.h>
#include <timers.h>
#include <delays.h>
#include "pragmas.h"
#include "functions.h"


/*** Instrucciones CAN ***/
// Variables
#define INST_CAR_ONOFF 0x18         // Coche (ON/OFF)
#define INST_SENSOR_ONOFF 0x19      // Sensor distancia (ON/OFF)
#define INST_UMBRAL 0x20            // Umbral del sensor para activación sonora

// Valores
#define CAR_ON 0x01       // Estado del coche - On
#define CAR_OFF 0x00      // Estado del coche - Off
#define SENSOR_ON 0x01     // Estado de las luces - On
#define SENSOR_OFF 0x00    // Estado de las luces - Off

// Variables
unsigned char new_msg=0, estado_sensor = 0,estado_coche=0;
unsigned int umbral_recibido = 0, umbral=730,aux_pwm=0,dist=0; //por defecto a 30 cm
char msg,aux_duty=0;

// Funciones
void init(void);
void SendMessage (char* Msg );
void SetUmbral(int umbral);
int read_sensor(void);
void delayms(int);

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
    // Can interrupt new message.
    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
        if ( new_msg != 1 ){
            new_msg == 0;
        }
    }

 //if(PIR1bits.TMR2IF == 1){
    //PIR1bits.TMR2IF = 0;
    //if(aux_duty<300){
        //aux_duty++;
        //aux_pwm=1022;
    //}
   // else if(aux_duty<600){
       // aux_duty++;
        //aux_pwm=1;
    //}
   // else
        //aux_duty=1;
// }
}

void main(void) {
    char msg[40];
    int aux;

    init();
    init_canport();

    sprintf(msg, "PIC3 Sensor Distancia. Inicio\r\n");
    SendMessage(msg);

    PORTDbits.RD4=0;

    while(1){
        
       //sprintf(msg, "%u--%u-%u--%u-%u--%u\n",RXB0SIDH,RXB0SIDL,RXF0SIDH,RXF0SIDL,RXB0D3,RXB0D1);
       //sprintf(msg, "hola\n");
       //sprintf(msg, "%u\n", RXB0D3);
       //endMessage(msg);
       //sprintf(msg, "%u\n", RXB0D3);
       //SendMessage(msg);
      // sprintf(msg, "%u\n", RXB0SIDL);
       //SendMessage(msg);
       delayms(1000);
//SendMessage(msg);
        if (new_msg == 1){
            new_msg = 0;
            sprintf(msg, "New MSG.\n");
            SendMessage(msg);

            if (RXB0D5==TEST){
                ECAN_Transmit_dist(TEST,16,0x4A,66); //0x4A, 0x42
            }else

            switch (RXB0D3){
            case INST_CAR_ONOFF:
                if (RXB0D1 == CAR_ON){
                    estado_coche = 1;
                    sprintf(msg, "Coche ON.\n");
                    SendMessage(msg);
                }
                else{
                    estado_coche = 0;
                    PORTBbits.RB5=1; //Vcc IR OFF
                    sprintf(msg, "Coche OFF.\n");
                    SendMessage(msg);
                }
                break;

            case INST_SENSOR_ONOFF:
                if (RXB0D1 == SENSOR_ON){
                    estado_sensor = 1;
                    PORTBbits.RB5 = 0; //Vcc IR ON
                    sprintf(msg, "Sensor ON.\n");
                    SendMessage(msg);

                }else{
                    estado_sensor = 0;
                    PORTBbits.RB5 = 1; //Vcc IR OFF
                    sprintf(msg, "Sensor OFF.\n");
                    SendMessage(msg);
                }
                break;

            case INST_UMBRAL:
                umbral_recibido  = RXB0D1;
                if (umbral_recibido<=10) umbral_recibido=10;
                if (umbral_recibido>=80) umbral_recibido=80;
                
                sprintf(msg, "Umbral dist: %u\n", umbral_recibido);
                SendMessage(msg);
                umbral=13884*pow(umbral_recibido,-0.85544);
                sprintf(msg, "Umbral bits: %u\n", umbral);
                SendMessage(msg);
                break;
                
            default:// Instruccion no reconocida
                sprintf(msg, "Error: Instruccion no reconocida.\n");
                SendMessage(msg);
                break;
            }
        }
    }
            //PORTDbits.RD0=1;
            //aux=read_sensor();
            //sprintf(msg, "%u sensor\n", aux);
            //SendMessage(msg);
            //sprintf(msg, "%u-%u estadosensorcoche\n", estado_coche,estado_sensor);
            //delayms(1000);
        if (estado_coche == 1 && estado_sensor==1){
            aux=read_sensor();       
            //delayms(1000);
            //umbral=1000;
            //OpenPWM2(255,0x00);
            dist=pow(13884/aux,1/0.85544);

            
            if (dist<=10) dist=10;
            if (dist>=80) dist=255;
            ECAN_Transmit_dist(READ,17,0,dist);
            sprintf(msg, "sensor: %u bits, %u cm\n", aux,dist);
            SendMessage(msg);

            if (aux>=umbral){
                //T2CONbits.TMR2ON=1;
                //OpenPWM2(255-((aux/4)&255),0x00);
                //OpenPWM2(255,2584-aux);
                //OpenPWM2(255,0x00);

                //PORTDbits.RD4=1;
                //SetDCPWM2(aux_pwm);
                //SetDCPWM2(80);
                //PORTBbits.RB5 = 1;
                PORTDbits.RD4=1;
                //PORTDbits.RD4=!PORTDbits.RD4;
            }
            else{
                //SetDCPWM2(1);
                PORTDbits.RD4=0;
            }
        }
    }


void init(void){

    // UART
    TXSTA1 = 0x20;  //Transmit is enabled
    RCSTA = 0x90;   //Serial port is enabled | Enables receiver
    SPBRG = 12;     //Baud rate = 9600 a MHz. Formula, SPBRG = (Fosc / (64 * Baud rate) ) - 1
    TRISCbits.TRISC6 = 0; //TX out
    TRISCbits.TRISC7 = 1; //RX in

    // ADC
    TRISBbits.TRISB1 = 1; // RB1 (Canal 8 ADC) INPUT
    ADCON0bits.ADON = 0; // Apagado al configurar
    ADCON1 = 0b00000000; // all ports to analogs
    ADCON0bits.CHS = 8; // CH AN10
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido

    //PWM
    TRISDbits.TRISD4=0;
    //OpenTimer2(TIMER_INT_ON & T2_PS_1_16 & T2_POST_1_1);
    //OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
    //INTCONbits.GIE=1;
    //OpenPWM2(250, 0x00);
    //SetDCPWM2(1);

    //Botón RD0 OUT
     TRISBbits.TRISB5 = 0 ;
}

void SendMessage (char* Msg ){
    int len, count;

    len = strlen(Msg);
    for (count = 0; count < len; count++){
        while(!TXSTA1bits.TRMT);
        TXREG = *(Msg+count);
    }
}

int read_sensor(void){

    int lecturaADC;

            Delay10TCYx(1);
            ConvertADC();
            while(BusyADC());
            lecturaADC = ReadADC();

     return lecturaADC;
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