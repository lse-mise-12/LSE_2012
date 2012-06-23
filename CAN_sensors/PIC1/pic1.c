/* 
 * File:   pic1.c
 * Author: lse
 *
 * Created on 8 de junio de 2012, 12:21
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p18lf45k80.h>
#include <adc.h>
#include <timers.h>
#include <delays.h>
#include <portb.h>
#include "pragmas.h"
#include "functions.h"

// Defines
#define VDD 5000    // ¿o 5V?
#define umbralHumedad 95
#define ledWipers LATAbits.LATA1
#define zumbador LATDbits.LATD4

// Variables
int temperatureRefreshTime = 2;
int humidityRefreshTime = 5;

unsigned char puertaAbierta = 0;
unsigned char estadoWipers = 0;
unsigned char modoWipers = 0;
unsigned char cambioWipersManual = 0;
unsigned char recibidoWipers = 0;

char mens[40];


unsigned char new_msg=0;
// Funciones
void init(void);
void SendMessage (char* Msg );
void SetTimer1(unsigned int time);
void SetTemperatureRefreshTime(int refresh);
//void SetHumidityRefreshTime(int refresh);
void initADCforTemperature(void);
void initADCforHumidity(void);
int ReadTemperature(void);
int ReadHumidity(void);
void initPuertaInterrupt(unsigned char abierta);
unsigned char estadoPuerta(void);

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

// Para el tiempo de refresco de la temperatura (tienen que ser diferentes
// porque pueden llevar tiempos de refresco distintos y por lo tanto
// actualizarse en diferentes momentos, aunque usan el mismo timer)
int tempCuentaInt = 0;
int tempCuentaSegundos = 0;

// Para el tiempo de refresco de la humedad
int humCuentaInt = 0;
int humCuentaSegundos = 0;
int cambioPuerta = 0;

void service_isr(){
    // Interrupción contactor
    if(INTCON3bits.INT1IF == 1) {
        cambioPuerta = 1;
//        INTCON3bits.INT1IF = 0; // Borrar el flag
        if (PORTBbits.RB1 == 1){
            INTCON2bits.INTEDG1 = 0;
            puertaAbierta = 1;
//            sprintf(mens, "La puerta se ha abierto\n");
//            SendMessage(mens);
        } else if (PORTBbits.RB1 == 0){
            INTCON2bits.INTEDG1 = 1;
            puertaAbierta = 0;
//            sprintf(mens, "La puerta se ha cerrado\n");
//            SendMessage(mens);
        }
        INTCON3bits.INT1IF = 0; // Borrar el flag
    }
    // INTERRUPCION TIMER1 OVERFLOW
    if(PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;
        SetTimer1(62500); //250ms = 4us*25000
        tempCuentaInt++;
        tempCuentaSegundos = tempCuentaInt/4;
        humCuentaInt++;
        humCuentaSegundos = humCuentaInt/4;
    }
    // Can interrupt new message.
    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
        if ( new_msg == 1 ){
            new_msg = 1;
        }
        else{new_msg = 0;}//else {
            //sprintf(msg, "newmsg err\n");
            //SendMessage(msg);

        //}
    }
//    // Interrupción contactor
//    if(INTCON3bits.INT1IF == 1) {
////        INTCON3bits.INT1IF = 0; // Borrar el flag
//        if (PORTBbits.RB1 == 1){
//            INTCON2bits.INTEDG1 = 0;
//            sprintf(mens, "La puerta se ha abierto\n");
//            SendMessage(mens);
//        } else if (PORTBbits.RB1 == 0){
//            INTCON2bits.INTEDG1 = 1;
//            sprintf(mens, "La puerta se ha cerrado\n");
//            SendMessage(mens);
//        }
////        puertaAbierta = estadoPuerta();
////        if (puertaAbierta == 1){
////            sprintf(mens, "La puerta se ha abierto\n");
////            SendMessage(mens);
////            // Cambiar el flanco de interrupción: si la puerta está abierta
////            // el pin está a 1. Y por tanto el próximo será de bajada.
////            INTCON2bits.INTEDG1 = 0;
////        } else if (puertaAbierta == 0){
////            sprintf(mens, "La puerta se ha cerrado\n");
////            SendMessage(mens);
////            INTCON2bits.INTEDG1 = 1;
////        }
////        puertaAbierta
//
//        INTCON3bits.INT1IF = 0; // Borrar el flag
//    }

}

/*
 *
 */
//int main(int argc, char** argv) {
void main(void) {
    char msg[40];

    int temperatura = 0;
    int humedad = 0;
    int humedadAnterior = 0;

//    // Variables para la temperatura
//    int lecturaADC = 0;
//    long voltajeTemp = 0;
//    int diff = 0;
//    int tempx10 = 0;
//    int tempEntero = 0;
//    double temperatura = 0.0;
    struct CAN_mesg mensaje;
    unsigned char seqnumber, aux = 0x00;
    int umbral_acc = 22;
    int estado_coche = 0, estado_acc = 0;
    int umbral_l, umbral_h, refresh_l, refresh_h, refresh_temp;

    init();

    puertaAbierta = estadoPuerta();
    if (puertaAbierta == 1){
        sprintf(msg, "La puerta empieza abierta\n");
    } else if (puertaAbierta == 0){
        sprintf(msg, "La puerta empieza cerrada\n");
    }
    SendMessage(msg);

    initPuertaInterrupt(puertaAbierta);

    cambioPuerta = 0;
    zumbador = 0;

//    puertaAbierta = estadoPuerta();
//    sprintf(msg, "Puerta = \r\n");
//    SendMessage(msg);

    init_canport();

    sprintf(msg, "PIC 1. Inicio\r\n");
    SendMessage(msg);

    seqnumber = 0x00;

    mensaje.destIDH = 0x00;
    mensaje.destIDL = 0x00;

    mensaje.destino = 0x20;
    mensaje.type = 0x00;
    mensaje.seq_number = seqnumber;
    mensaje.variable = 0x00;
    mensaje.byteH = 0x00;
    mensaje.byteL = 0x00;
    mensaje.datos = 0x00;
    mensaje.CRC = DATA;

    while(1){
//        puertaAbierta = 2;

        // Actuar si hay un nuevo mensaje
        if ( (new_msg == 1) && (RXB0D6 == 0x00) ){
            new_msg = 0;
            sprintf(msg, "New MSG.\n");
            SendMessage(msg);
             if ( (RXB0D5==TEST) && (RXB0D6 = 0x00) && (RXB0D2 == 0xA4) ) {
                
                mensaje.destIDH = TSIDH;
                mensaje.destIDL = TSIDL;
                mensaje.type = TEST;
                mensaje.seq_number = seqnumber;
                mensaje.variable = TEST;
                mensaje.byteH = temperatura;
                aux = estado_coche && (puertaAbierta << 1) && (0x00 << 2) && (estadoWipers << 3) && (modoWipers << 4);
                mensaje.byteL = aux;
                //mensaje.datos = 0x00;
                mensaje.CRC = DATA;
                ECAN_Transmit(mensaje,0); //0x4A, 0x42
            }
            else{
            switch (RXB0D3){
            case INST_STATE1:
                if(RXB0D1 == STATE_OFF){  // Coche apagado
                    estado_coche = 0;
                    estado_acc = 0;
                    sprintf(msg, "coche off\n");
                    SendMessage(msg);
                }else {
                    estado_coche = 1;
                    zumbador = 0;
                    sprintf(msg, "coche on\n");
                    SendMessage(msg);
                }

                if(RXB0D1 == STATE_ON){ // Coche encendido
                    estado_coche = 1;
                    estado_acc = 0;
                    zumbador = 0;
                    sprintf(msg, "coche on\n");
                    SendMessage(msg);
                }
                else {
                    estado_coche = 0;
                    sprintf(msg, "coche off\n");
                    SendMessage(msg);
                }
                break;

            case INST_TEMP_REFRESH:
                refresh_l = RXB0D1;
                refresh_h = RXB0D2;
                refresh_temp = (refresh_h<<8) + refresh_l;
                SetTemperatureRefreshTime(refresh_temp);
                break;

            case INST_WIPERS:
                recibidoWipers = RXB0D1;
                switch (recibidoWipers) {
                    case 0x00: // Apagado manual
                        cambioWipersManual = 1;
                        estadoWipers = 0;
                        modoWipers = 0;
                        sprintf(msg, "Inst 0x00\n");
                        SendMessage(msg);
                        break;
                    case 0x01: // Encendido manual
                        cambioWipersManual = 1;
                        estadoWipers = 1;
                        modoWipers = 0;
                        sprintf(msg, "Inst 0x01\n");
                        SendMessage(msg);
                        break;
//                    case 0x10: // Deshabilita automatico
//                        modoWipers = 0;
//                        sprintf(msg, "Inst 0x10\n");
//                        SendMessage(msg);
//                        break;
//                    case 0x11: // Habilita automatico
//                        modoWipers = 1;
//                        sprintf(msg, "Inst 0x11\n");
//                        SendMessage(msg);
//                        break;
                    default:
                        break;
                }
                break;

            case INST_W_AUTO:
                recibidoWipers = RXB0D1;
                switch (recibidoWipers) {
                    case 0x00: // Apagado manual
                        //cambioWipersManual = 1;
                        //estadoWipers = 0;
                        modoWipers = 0;
                        sprintf(msg, "Inst 0x00\n");
                        SendMessage(msg);
                        break;
                    case 0x01: // Encendido manual
                        //cambioWipersManual = 1;
                        //estadoWipers = 1;
                        modoWipers = 1;
                        sprintf(msg, "Inst 0x01\n");
                        SendMessage(msg);
                        break;
//                    case 0x10: // Deshabilita automatico
//                        modoWipers = 0;
//                        sprintf(msg, "Inst 0x10\n");
//                        SendMessage(msg);
//                        break;
//                    case 0x11: // Habilita automatico
//                        modoWipers = 1;
//                        sprintf(msg, "Inst 0x11\n");
//                        SendMessage(msg);
//                        break;
                    default:
                        break;
                }
                break;

                case TEST :
                    mensaje.destIDH = TSIDH;
                mensaje.destIDL = TSIDL;
                mensaje.type = TEST;
                mensaje.seq_number = seqnumber;
                mensaje.variable = TEST;
                mensaje.byteH = temperatura;
                aux = estado_coche && (puertaAbierta << 1) && (0x00 << 2) && (estadoWipers << 3) && (modoWipers << 4);
                mensaje.byteL = aux;
                //mensaje.datos = 0x00;
                mensaje.CRC = DATA;
                ECAN_Transmit(mensaje,0); //0x4A, 0x42
                    break;


            default:// Instruccion no reconocida
                    seqnumber++;
                    mensaje.destIDH = TSIDH;
                    mensaje.destIDL = TSIDL;

                    //mensaje.destino = TSIDL;
                    mensaje.type = ERROR;
                    mensaje.seq_number = RXB0D4;
                    mensaje.variable = RXB0D3;
                    mensaje.byteH = RXB0D2;
                    mensaje.byteL = RXB0D1;
                    //mensaje.datos = 0x00;
                    mensaje.CRC = DATA;
                    ECAN_Transmit(mensaje, 0);
                break;
            }
            }
        }

        if (cambioPuerta == 1) {    // comprobar si seguridad está activada
            cambioPuerta = 0;
            if (puertaAbierta == 1) {
                seqnumber++;
                mensaje.destIDH = TSIDH;
                mensaje.destIDL = TSIDL;

                //mensaje.destino = TSIDL;
                mensaje.type = READ;
                mensaje.seq_number = seqnumber;
                mensaje.variable = INST_PUERTA;
                mensaje.byteH = 0x00;
                mensaje.byteL = PUERTA_ABIERTA;
                //mensaje.datos = 0x00;
                mensaje.CRC = DATA;
                ECAN_Transmit(mensaje, 0);
                sprintf(msg, "La puerta se ha abierto\n");
                SendMessage(msg);

                if (estado_coche == 0){
                    zumbador = 1;
                    sprintf(msg, "zumbador on seg. on\n");
                    SendMessage(msg);
                }
                if (estado_coche == 1) {
                    zumbador = 0;
                    sprintf(msg, "zumbador off \n");
                    SendMessage(msg);
                }

            } else if (puertaAbierta == 0) {
                zumbador = 0;
                seqnumber++;
                mensaje.destIDH = TSIDH;
                mensaje.destIDL = TSIDL;

                //mensaje.destino = TSIDL;
                mensaje.type = READ;
                mensaje.seq_number = seqnumber;
                mensaje.variable = INST_PUERTA;
                mensaje.byteH = 0x00;
                mensaje.byteL = PUERTA_CERRADA;
                //mensaje.datos = 0x00;
                mensaje.CRC = DATA;
                ECAN_Transmit(mensaje, 0);
                sprintf(msg, "La puerta se ha cerrado\n");
                SendMessage(msg);
            }
        }

        if (tempCuentaSegundos >= temperatureRefreshTime){
            tempCuentaInt = 0;
            tempCuentaSegundos = 0;

            initADCforTemperature();
            Delay100TCYx(10);

            temperatura = ReadTemperature();

            // Mandar mensaje CAN
            seqnumber++;
            mensaje.destIDH = TSIDH;
            mensaje.destIDL = TSIDL;

            //mensaje.destino = TSIDL;
            mensaje.type = READ;
            mensaje.seq_number = seqnumber;
            mensaje.variable = INST_TEMP_INT;
            //mensaje.byteH = 0x00;
            //mensaje.byteL = CONTROL_OFF;
            mensaje.datos = temperatura;
            mensaje.CRC = DATA;
            ECAN_Transmit(mensaje, 1);

//            sprintf(msg, "txb0d1:%d\n", TXB0D1);
//            SendMessage(msg);
//            sprintf(msg, "txb0d2:%d\n", TXB0D2);
//            SendMessage(msg);
        }
        if (humCuentaSegundos >= humidityRefreshTime){
            humCuentaInt = 0;
            humCuentaSegundos = 0;

            initADCforHumidity();
            Delay100TCYx(10);

            humedad = ReadHumidity();

            if (modoWipers == 1) {

                if (humedad > umbralHumedad) {
                    ledWipers = 1;
                    estadoWipers = 1;
                    if (humedadAnterior <= umbralHumedad) {
                        seqnumber++;
                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;

                        //mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_WIPERS;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = estadoWipers;
                        mensaje.CRC = DATA;
                        ECAN_Transmit(mensaje, 0);
                        sprintf(msg, "wipers auto on\n");
                        SendMessage(msg);
                    }
                } else if (humedad <= umbralHumedad){
                    ledWipers = 0;
                    estadoWipers = 0;
                    if (humedadAnterior > umbralHumedad) {
                        seqnumber++;
                        mensaje.destIDH = TSIDH;
                        mensaje.destIDL = TSIDL;

                        //mensaje.destino = TSIDL;
                        mensaje.type = READ;
                        mensaje.seq_number = seqnumber;
                        mensaje.variable = INST_WIPERS;
                        mensaje.byteH = 0x00;
                        mensaje.byteL = estadoWipers;
                        mensaje.CRC = DATA;
                        ECAN_Transmit(mensaje, 0);
                        sprintf(msg, "wipers auto off\n");
                        SendMessage(msg);
                    }
                }
            }
            humedadAnterior = humedad;
        }
        if (cambioWipersManual == 1) {
            cambioWipersManual = 0;

            seqnumber++;
            mensaje.destIDH = TSIDH;
            mensaje.destIDL = TSIDL;

            //mensaje.destino = TSIDL;
            mensaje.type = READ;
            mensaje.seq_number = seqnumber;
            mensaje.variable = INST_WIPERS;
            mensaje.byteH = 0x00;

            if (estadoWipers == 0) {
                ledWipers = 0;
                mensaje.byteL = estadoWipers;
                mensaje.CRC = 0xFF;
                ECAN_Transmit(mensaje, 0);
                sprintf(msg, "wipers off\n");
                SendMessage(msg);
            } else {
                ledWipers = 1;
                mensaje.byteL = estadoWipers;
                mensaje.CRC = 0xFF;
                ECAN_Transmit(mensaje, 0);
                sprintf(msg, "wipers on\n");
                SendMessage(msg);
            }
        }
    }
}

void init(void) {
    // Deshabilita el sensor de temperatura de PICDEM Z #CS = 1
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1;

    // UART (RC6, RC7)
    TXSTA1 = 0x20;  //Transmit is enabled
    RCSTA = 0x90;   //Serial port is enabled | Enables receiver
    SPBRG = 12;     //Baud rate = 9600 a MHz. Formula, SPBRG = (Fosc / (64 * Baud rate) ) - 1
    TRISCbits.TRISC6 = 0; //TX out
    TRISCbits.TRISC7 = 1; //RX in

    // ADC para temperatura por defecto (RB0 = CH10)
    TRISBbits.TRISB0 = 1; // RB0 (Canal 10 ADC) INPUT
    ADCON0bits.ADON = 0; // Apagado al configurar
    ADCON1 = 0b00000000; // all ports to analogs
    ADCON0bits.CHS = 10; // CH AN10
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido

    // Inicialización pin contactor (RB1)
    ANCON1bits.ANSEL8 = 0;  // Entrada digital
    TRISBbits.TRISB1 = 1;   // RB1 input
    
    // Led Wipers (RA0)
    TRISAbits.TRISA0 = 0;   // output
    LATAbits.LATA0 = 0;
    TRISAbits.TRISA1 = 0;   // output
    LATAbits.LATA1 = 0;

    // Zumbador (RD4)
    TRISDbits.TRISD4=0;

    // Interrupcion Timer 1
    INTCON = 0xC0;
    T1CON = 0x3F; //0x3B;
    T1GCON = 0x00;
    PIE1bits.TMR1IE = 1;
    SetTimer1(62500); //250ms = (8Mhz/32)^-1*62500
}

void initPuertaInterrupt(unsigned char abierta) {
    // Si empieza abierta, el pin estará a uno y necesitamos que el
    // flanco de la interrupción inicialmente sea de bajada (de 1 a 0).
    if (abierta == 1){
        INTCON2bits.INTEDG1 = 0; // flanco de bajada
    } else if (abierta == 0){
        INTCON2bits.INTEDG1 = 1; // flanco de subida
    }
    INTCON3bits.INT1IF = 0; // Borrar el flag
    INTCON3bits.INT1IE = 1; // Habilitar interrupción
    INTCON = 0xF8;  // Habilitar todas las interrupciones
}

unsigned char estadoPuerta(void) {
    unsigned char abierta = 0;
    // El contactor da VDD cuando está abierto y GND cuando está cerrado
    if (PORTBbits.RB1 == 1){
        abierta = 1;
    } else if (PORTBbits.RB1 == 0){
        abierta = 0;
    }
    return abierta;
}

void initADCforTemperature(void) {
    TRISBbits.TRISB0 = 1; // RB0 (Canal 10 ADC) INPUT
    ADCON0bits.ADON = 0; // Apagado al configurar
    ADCON1 = 0b00000000; // all ports to analogs
    ADCON0bits.CHS = 10; // CH AN10
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido
}

void initADCforHumidity(void) {
    // ALE, aquí tendrías que poner tu inicialización del ADC
    // con el canal del ADC (no puede ser el 10 que es el que usa la temp)
    TRISBbits.TRISB4 = 1; // RB4 (Canal 9 ADC) INPUT
    ADCON0bits.ADON = 0; // Apagado al configurar
    ADCON1 = 0b00000000; // all ports to analogs
    ADCON0bits.CHS = 9; // CH AN8
    ADCON2 = 0b10100000; // Acquisition time and fclk
    ADCON0bits.ADON = 1; // Encendido
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

void SetTimer1(unsigned int time){
	TMR1H = 0-(time/256);
	TMR1L = 0-(time&0xff);
        PIR1bits.TMR1IF = 0;
}

void SetTemperatureRefreshTime(int refresh){
    temperatureRefreshTime = refresh;
}

int ReadTemperature(void){
    char msg[40];

    int lecturaADC = 0;
    long voltajeTemp = 0;
    int diff = 0;
    int tempx10 = 0;
    int tempEntero = 0;
    double temperatura = 0.0;

    ConvertADC();
    while(BusyADC());
    lecturaADC = ReadADC();

    // Regla de tres para sacar los milivoltios
    // El valor maximo del ADC son 4095 (con la entrada a Vdd)
//        voltajeTemp = ((long)lecturaADC*VDD)/4095;
    voltajeTemp = ((long)lecturaADC*VDD)/4150; // 4095 se pasa un poco (10-20 mV de mas)

    diff = (int)(750 - voltajeTemp); // 750 mV = 25ºC; 10mV/ºC
    tempx10 = 250 - diff; // temperatura real * 10
    temperatura = ((double)tempx10)/10.0;
    tempEntero = (int)(temperatura + 0.5);  // Para redondear al entero mas cercano

    sprintf(msg, "temperatura = %d\r\n", tempEntero);
    SendMessage(msg);

    return tempEntero;
}

/*
void SetHumidityRefreshTime(int refresh) {
    humidityRefreshTime = refresh;
}
*/

int ReadHumidity(void) {
    char msg[40];

    int humedad = 0;
    int lecturaADC = 0;
    long voltaje;
    double aux;

    ConvertADC();
    while(BusyADC());
    lecturaADC = ReadADC();

    // ALE: aquí iría tu código para leer con el ADC la humedad,
    // que la guardarías en entero en la variable humedad.

  //  voltaje = ( lecturaADC * 3500 ) / 4096;
    voltaje = ((long)lecturaADC*VDD)/4096;


  //  Explained:  humedad = (int)(( ( voltaje / VDD ) - 0.1515) / 0.00636);
    aux= voltaje/3350.0;
    aux=aux-0.1515;
    aux=aux/0.00636;
 
    humedad=(int) aux;

    if (humedad > 100) {
        humedad = 100;
    } else if (humedad < 0) {
        humedad = 0;
    }

    sprintf(msg, "humedad = %d\r\n", humedad);
    SendMessage(msg);

    return humedad;
}

