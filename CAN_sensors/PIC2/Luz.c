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
int refreshTime = 1, control_luces = 0;    //Cada segundo por defecto
unsigned int umbral_luces = 10000; // 10000 ohm por defecto
unsigned char new_msg=0,  estado_luces = 0, estado_anterior = 1;
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
void delayms(int);

unsigned char estado_sensor = 0,estado_coche=0;
unsigned int umbral_recibido = 0, umbral=730,aux_pwm=0,dist=0; //por defecto a 30 cm
char msg,aux_duty=0;

// Funciones distancia
int read_sensor(void);


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
    int aux;
    unsigned char lsb = 0x00;
    
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

            if (RXB0D5==TEST){
                lsb = estado_luces && (control_luces_auto << 1);
                ECAN_Transmit_dist(TEST,19,dist, lsb); //0x4A, 0x42
            }else

            sprintf(msg, "New MSG.\n");
            SendMessage(msg);
            switch (RXB0D3){
            case INST_SENSOR_ONOFF:
                if (RXB0D1 == SENSOR_ON){
                    estado_sensor = 1;
                    estado_coche = 1;
                    PORTBbits.RB5 = 0; //Vcc IR ON
                    sprintf(msg, "Sensor ON.\n");
                    SendMessage(msg);

                }else{
                    estado_sensor = 0;
                    estado_coche = 0;
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

            case INST_LIGHT_CONTROL:            // Luces control manual
                switch(RXB0D1){
                    case CONTROL_LIGHT_ON:
                        control_luces_auto = 0;      // Luces modo Auto off
                        estado_luces = 1;       // Encender luces

                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;

                        mensaje.destino = TSIDL;
                        mensaje.type = READ;
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

                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_L_AUTO;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = LIGHT_OFF;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;
                        ECAN_Transmit(mensaje, 0);

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
                        mensaje.type = READ;
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

                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_L_AUTO;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = LIGHT_OFF;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;
                        ECAN_Transmit(mensaje, 0);

                        break;
                    default:
                        sprintf(msg, "Error: Instruccion no reconocida.\n");
                        SendMessage(msg);
                        break;
                }

                case INST_L_AUTO:                   // Control luces automatico
                switch(RXB0D1){
                    case CONTROL_LIGHT_ON:
                        control_luces_auto = 1;      // Luces Auto on
                        //estado_luces = 1;       // Encender luces

                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;

                        mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_L_AUTO;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = LIGHT_ON;
                        //mensaje.datos = 0x00;
                        mensaje.CRC = DATA;

                        ECAN_Transmit(mensaje, 0);
                        //LED_LUCES = 1;
                        sprintf(msg, "Luces auto ON.\n");
                        SendMessage(msg);
                        break;
                    case CONTROL_LIGHT_OFF:

                        control_luces_auto = 0;         // Luces modo automatico OFF
//                    if(control_luces_auto == 1){
//                        control_luces_auto = 0
//                    }else{}
                        //estado_luces = 0;               // Apagar luces
                        seqnumber++;

                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;
                        mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_L_AUTO;
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

        /***************  DISTANCIA * ********/
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

        /***************  LUCES * ********/
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
    ADCON0bits.CHS = 8; // CH AN8
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido


    TRISBbits.TRISB0 = 1; // RB0 (Canal 10 ADC) INPUT

    // Interrupción Timer 1
    INTCON = 0xC0;
    T1CON = 0x3F; //0x3B;
    T1GCON = 0x00;
    PIE1bits.TMR1IE = 1;
    SetTimer1(62500); //250ms = (8Mhz/32)^-1*62500
    TRISDbits.TRISD4=0;
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



            ADCON0bits.ADON = 0; // Apagado al configurar
            ADCON1 = 0b00000000; // all ports to analogs
            ADCON0bits.CHS = 10; // CH AN8
            ADCON2 = 0b10100000; // Acquisition time and fclk
            ADCON0bits.ADON = 1; // Encendido

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
                    mensaje.type = READ;
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
                    mensaje.type = READ;
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


int read_sensor(void){

    int lecturaADC;

            ADCON0bits.ADON = 0; // Apagado al configurar
            ADCON1 = 0b00000000; // all ports to analogs
            ADCON0bits.CHS = 8; // CH AN8
            ADCON2 = 0b10100000; // Acquisition time and fclk
            ADCON0bits.ADON = 1; // Encendido
            
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