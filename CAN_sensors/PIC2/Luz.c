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

// Luces de Poscion
#define LED_LUCES LATAbits.LATA1

// Luces de Freno
#define LED_FRENO LATAbits.LATA2

// Variables
int refreshTime = 1, estado_coche = 0, control_luces = 0;    //Cada segundo por defecto
unsigned int umbral_luces = 10000; // 10000 ohm por defecto
unsigned char new_msg=0,  estado_luces = 0, estado_anterior = 1;
unsigned int umbral_recibido = 0;
char msg;
struct CAN_mesg mensaje;
unsigned char seqnumber = 0x00;

// Funciones
void init(void);
void SendMessage (char* Msg );
void SetTimer1(unsigned int time);
void SetRefreshTime(int refresh);
void SetUmbral(int umbral);
void luces_automaticas(void);

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
int DistcuentaInt = 0,LuzcuentaInt = 0;
int DistcuentaSegundos = 0, LuzcuentaSegundos = 0;

void service_isr(){
    // Can interrupt new message.
    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
        if ( new_msg != 1 ){
            new_msg == 0;
        } //else {
            //sprintf(msg, "newmsg err\n");
            //SendMessage(msg);

        //}
    }

    // INTERRUPCION TIMER1 OVERFLOW
    if(PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;
        SetTimer1(62500); //250ms = 4us*25000
        LuzcuentaInt++;
        LuzcuentaSegundos = LuzcuentaInt/4;
        DistcuentaInt++;
        DistcuentaSegundos = DistcuentaInt/4;
    }
}

/*
 *
 */
//int main(int argc, char** argv) {
void main(void) {
    char msg[40];
    int temp_l, temp_h;
    int control_luces_auto = 0;
    
    int seqnumber = 0;
    init();
    init_canport();

    sprintf(msg, "PIC2. Inicio\r\n");
    SendMessage(msg);

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

    while(1){

        // Actuar si hay un nuevo mensaje

        // Falta LED luces automaticas ON
        // Falta sensor distancia
        if (new_msg == 1){
            new_msg = 0;
            sprintf(msg, "New MSG.\n");
            SendMessage(msg);
            switch (RXB0D3){
            case INST_STATE2:
                if (RXB0D1 == STATE_ON){
                    estado_coche = 1;
                    sprintf(msg, "Coche ON.\n");
                    SendMessage(msg);
                }
                else{
                    estado_coche = 0;
                    sprintf(msg, "Coche OFF.\n");
                    SendMessage(msg);
                }
                break;

            case INST_LIGHT_CONTROL:

                switch(RXB0D1){
                    case CONTROL_LIGHT_ON:
                        control_luces_auto = 0;      // Luces Auto off
                        estado_luces = 1;       // Encender luces

                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;

                        mensaje.destino = TSIDL;
                        mensaje.type = EVENT;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_LIGHT_CONTROL;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = LIGHT_ON;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;

                        ECAN_Transmit(mensaje, 0);
                        LED_LUCES = 1;
                        sprintf(msg, "Luces ON.\n");
                        SendMessage(msg);
                        break;

                
                    case CONTROL_LIGHT_AUTO_ON:
                       T1CONbits.TMR1ON = 1;
                    //if (control_luces_auto == 0){
                    //    control_luces_auto = 1;      // Luces modo automatico ON
                        sprintf(msg, "Luces Auto ON.\n");
                        SendMessage(msg);
                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = EVENT;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_LIGHT_CONTROL;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = CONTROL_LIGHT_AUTO_ON;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;

                        ECAN_Transmit(mensaje, 0);
                        luces_automaticas();
                        break;

                    
                    case CONTROL_LIGHT_AUTO_OFF:
                        control_luces_auto = 0;     // Luces modo automatico OFF
                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = EVENT;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_LIGHT_CONTROL;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = CONTROL_LIGHT_AUTO_OFF;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;

                        ECAN_Transmit(mensaje, 0);
                        sprintf(msg, "Luces Auto OFF.\n");
                        SendMessage(msg);
                        break;

                    case CONTROL_LIGHT_OFF:

                        control_luces_auto = 0;         // Luces modo automatico OFF
//                    if(control_luces_auto == 1){
//                        control_luces_auto = 0
//                    }else{}
                        estado_luces = 0;               // Apagar luces
                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = EVENT;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_LIGHT_CONTROL;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = LIGHT_OFF;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;

                        ECAN_Transmit(mensaje, 0);
                        LED_LUCES = 0;
                        sprintf(msg, "Luces OFF.\n");
                        SendMessage(msg);
                        break;
                    default:
                        sprintf(msg, "Error: Instruccion no reconocida.\n");
                        SendMessage(msg);
                        break;
                }

            case INST_LIGHT_UMBRAL:
                temp_l = RXB0D1;
                temp_h = RXB0D2;
                umbral_recibido  = (temp_h<<8) + temp_l;
                sprintf(msg, "Umbral: %u\n", umbral_recibido);
                SendMessage(msg);
                break;
            default:// Instruccion no reconocida
                sprintf(msg, "Error: Instruccion no reconocida.\n");
                SendMessage(msg);
                break;
            }
        }

        if (control_luces_auto == 1){
            if (LuzcuentaSegundos >= refreshTime){
                luces_automaticas();
            }
        }else{
            //stop and restart timer
            T1CONbits.TMR1ON = 0;
            LuzcuentaInt = 0;
            LuzcuentaSegundos = 0;
        }
    }
}

void init(void){
    // Deshabilita el sensor de temperatura de PICDEM Z #CS = 1
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1;

    // Led (luces)
    TRISAbits.TRISA1 = 0;         //RA1 AN1 as out LED
    LED_LUCES = 0;

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

    // Interrupción Timer 1
    INTCON = 0xC0;
    T1CON = 0x3F; //0x3B;
    T1GCON = 0x00;
    PIE1bits.TMR1IE = 1;
    SetTimer1(62500); //250ms = (8Mhz/32)^-1*62500
}

void SendMessage (char* Msg ){
    int len, count;

    len = strlen(Msg);
    for (count = 0; count < len; count++){
        //while( !PIR1bits.TX1IF );
        while(!TXSTA1bits.TRMT);
        TXREG = *(Msg+count);
    }
}

void SetTimer1(unsigned int time){
	TMR1H = 0-(time/256);
	TMR1L = 0-(time&0xff);
        PIR1bits.TMR1IF = 0;
}

void SetRefreshTime(int refresh){
    refreshTime = refresh;
}

void SetUmbral(int umbral){
    umbral_luces = umbral;
}

void luces_automaticas(void){
    int lecturaADC = 0;
    long voltajeLuz = 0;
    unsigned int R_divisor = 12000;
    unsigned long int resistencia = 0;
    char msg[40];

     // Leer ADC y sacar valor de la resistencia. Solo si est� en modo auto.

            LuzcuentaInt = 0;
            LuzcuentaSegundos = 0;

            Delay100TCYx(10);
            ConvertADC();
            while(BusyADC());
            lecturaADC = ReadADC();

            // Regla de tres para sacar los milivoltios
            // El valor máximo del ADC son 4095 (con la entrada a Vdd)
            // ¿Por qué 4095 si son 10 bits?
    //        voltajeTemp = ((long)lecturaADC*VDD)/4095;
            voltajeLuz = ((long)lecturaADC*VDD)/4150; // 4095 se pasa un poco (10-20 mV de más)

            resistencia = (int)((R_divisor/(VDD - voltajeLuz)) * voltajeLuz);

            // Encender las luces del coche o no
            if (resistencia > umbral_luces) {
                LED_LUCES = 1;
                estado_luces = 0x01;
                if(estado_anterior == 0){

                    seqnumber++;

                    mensaje.destIDH = TSIDH;
                    mensaje.destIDL = TSIDL;
                    mensaje.destino = TSIDL;
                    mensaje.type = EVENT;
                    mensaje.seq_number = seqnumber;
                    mensaje.variable = INST_LIGHT_CONTROL;
                    mensaje.byteH = 0x00;
                    mensaje.byteL = LIGHT_ON;
                    //mensaje.datos = 0x00;
                    mensaje.CRC = DATA;

                    ECAN_Transmit(mensaje, 0);

                    sprintf(msg, "Luces auto ON.\n");
                    SendMessage(msg);
                }
                estado_anterior = 1;
            }  else {
                LED_LUCES = 0;
                estado_luces = 0;
                if(estado_anterior == 1){
                    seqnumber++;

                    mensaje.destIDH = TSIDH;
                    mensaje.destIDL = TSIDL;
                    mensaje.destino = TSIDL;
                    mensaje.type = EVENT;
                    mensaje.seq_number = seqnumber;
                    mensaje.variable = INST_LIGHT_CONTROL;
                    mensaje.byteH = 0x00;
                    mensaje.byteL = LIGHT_OFF;
                    //mensaje.datos = 0x00;
                    mensaje.CRC = DATA;

                    ECAN_Transmit(mensaje, 0);

                    sprintf(msg, "Luces auto OFF.\n");
                    SendMessage(msg);
                }
                estado_anterior = 0;
            }

//            sprintf(msg, "tempx10 = %d\r\n", tempx10);
//            SendMessage(msg);
           // sprintf(msg, "r = %lu\r\n", resistencia);
           // SendMessage(msg);

//            ECAN_Transmit(tempEntero);

//            sprintf(msg, "txb0d1:%d\n", TXB0D1);
//            SendMessage(msg);
//            sprintf(msg, "txb0d2:%d\n", TXB0D2);
//            SendMessage(msg);

}
