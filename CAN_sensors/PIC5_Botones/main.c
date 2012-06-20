#include <stdio.h>
#include <stdlib.h>
#include <p18lf45k80.h>
#include <timers.h>
#include <string.h>
#include <delays.h>
#include <portb.h>          // PORTB library function
#include "pragmas.h"
#include "functions.h"

/* */
void init(void);
void write_msg (unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
unsigned char new_msg = 0, estado_control_luces = 0, estado_parking = 0, estado_control_wipers = 0;
unsigned char seqnumber;
struct CAN_mesg mensaje;
int R0down = 0;
int R1down = 0;
int R2down = 0;
int R4down = 0;
int R5down = 0;
int R6down = 0;
int R7down = 0;

int estado_luces = 0, cambio = 0, estado_wipers = 0,  estado_motor = 0;
char rpms[15];


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

//unsigned long pulsos = 0;
//unsigned char pulsos_total = 0x42;
//unsigned long asdf;
//char rpms[15];

void service_isr(){
    if(PIR5bits.RXB0IF == 1){
        new_msg = ECAN_Receive();
    }

    if( INTCONbits.INT0IF == 1){   // RB0 : Int Motor
        if(PORTBbits.RB0){
            cambio = 1;
//            LED_MOTOR = !LED_MOTOR;
        }
        INTCONbits.INT0IF =0;
    }

    if( INTCON3bits.INT1IF == 1){   // RB1 : Wipers IN/OFF
        if(PORTBbits.RB1){
            cambio = 3;
//            LED_WIP = !LED_WIP;
        }
            INTCON3bits.INT1IF =0;
     }
        
     if(INTCONbits.RBIF == 1){
         if(PORTBbits.RB5){      // Boton luces ON / OFF
            if(R5down){
                R5down = 0;
                cambio = 2;
//                LED_LUZ = !LED_LUZ;
            }
        }else
            R5down = 1;

         if(PORTBbits.RB6){      // Boton luces Auto
            if(R6down){
                R6down = 0;
                cambio = 21;
//                LED_LUZ_AUTO = !LED_LUZ_AUTO;
                //estado_luces = 2;
            }
        }else
            R6down = 1;

         if(PORTBbits.RB4){      // Boton wipers auto
            if(R4down){
                R4down = 0;
//                LED_WIP_AUTO = !LED_WIP_AUTO;
                //estado_wipers = 2;
                cambio = 31;
            }
        }else
            R4down = 1;

        if(PORTBbits.RB7){      // Boton parking trasero ON/OFF
            if(R7down){
                R7down = 0;
                cambio = 4;
//                LED_PARKING = !LED_PARKING;
            }
        }else
            R7down = 1;

        INTCONbits.RBIF = 0;
    }

}

void main(void){
    //char msg_inicial[] = "\n\n  PIC18LF45K80 \r\n  PIC5 - Contro Panel\n\n\n";
    init();
    init_uartport();
    init_canport();
 
    //SendMessage(&msg_inicial[0]);
    
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

    delayms(200);
    LED_MOTOR = 0;
    LED_LUZ = 0;
    LED_LUZ_AUTO = 0;
    LED_PARKING = 0;
    LED_WIP = 0;
    LED_WIP_AUTO = 0;

    delayms(300);
    LED_MOTOR = 1;
    LED_LUZ = 1;
    LED_LUZ_AUTO = 1;
    LED_PARKING = 1;
    LED_WIP = 1;
    LED_WIP_AUTO = 1;
    delayms(500);

    LED_MOTOR = 0;
    LED_LUZ = 0;
    LED_LUZ_AUTO = 0;
    LED_PARKING = 0;
    LED_WIP = 0;
    LED_WIP_AUTO = 0;

    //sprintf(rpms, "Ready\n");
    //SendMessage(rpms);


    while(1){
        if (new_msg){
            if ((RX_TYP == TEST) && (RXB0D2 == TEST_VALUE) ) {               // Comprobar tipo test
                write_msg(TSIDH, TSIDL, TEST, INST_TEST, 0, 0);
                ECAN_Transmit(mensaje, 0);      // Reenviar TEST
            }
        }

            //else{
            //   switch(RX_VAR){     // Numero de variable
                    /*
                    case 0x13:
                        if (RX_LSB == STATE_ON){     // Llega Encender pic
                            pic5_state = 1;
                        }
                        if (RX_LSB == STATE_OFF){     // Llega Encender pic
                            pic5_state = 0;
                        }

                        break;
                    */
              /*      case 0x25:
                        if (RX_LSB == LIGHT_ON){
                            //luces encendidas
                        }
                        if (RX_LSB == LIGHT_OFF){
                            //luces apagadas
                        }
                        break;

                    case 0x26:

                        break;

                    case 0x27:

                        break;

                    case 0x28:

                        break;

                    default:
                        write_msg(TSIDH, TSIDL, ERROR, INST_ERROR, ERROR, ERROR);
                        ECAN_Transmit(mensaje, 0);      // Reenviar ERROR
                        break;

                }*/



        switch(cambio){
            case 0:
                cambio = 0;
                break;
            case 1://motor
                cambio = 0;
                if (estado_motor == 1){
                    estado_motor = 0;   // Motor apagado
                    write_msg(TSIDH, TSIDL, WRITE, INST_ARRA_BOT, 0, STATE_OFF);
                    ECAN_Transmit(mensaje, 0);
                    LED_MOTOR = 0;
                }
                else{
                    estado_motor = 1;   // Motor encendido
                    write_msg(TSIDH, TSIDL, WRITE, INST_ARRA_BOT, 0, STATE_ON);
                    ECAN_Transmit(mensaje, 0);
                    LED_MOTOR = 1;
                }
                break;

            case 2:             //luces
                cambio = 0;
                if (estado_luces == 0){
                    estado_luces = 1; // Encender luces
                    write_msg(TSIDH, TSIDL, WRITE, INST_LIGHT_BOT, 0, LIGHT_ON);
                    ECAN_Transmit(mensaje, 0);
                    LED_LUZ = 1;
                    //sprintf(rpms, "luces on\n");
                    //SendMessage(rpms);
                }
                else
                {
                    estado_luces = 0;       // Apagar luces
                    write_msg(TSIDH, TSIDL, WRITE, INST_LIGHT_BOT, 0, LIGHT_OFF);
                    ECAN_Transmit(mensaje, 0);
                    LED_LUZ = 0;
                    //sprintf(rpms, "luces off\n");
                    //SendMessage(rpms);
                }
                break;

            case 21:            //luces auto
                cambio = 0;
                if(estado_control_luces == CONTROL_LIGHT_AUTO_ON){
                    estado_control_luces = CONTROL_LIGHT_AUTO_OFF;
                    LED_LUZ_AUTO = 0;
                }
                else{
                    estado_control_luces = CONTROL_LIGHT_AUTO_ON;
                    LED_LUZ_AUTO = 1;
                }
                    write_msg(TSIDH, TSIDL, WRITE, INST_LIGHT_BOT, 0, estado_control_luces);
                    ECAN_Transmit(mensaje, 0);
                    //LED_LUZ_AUTO = !LED_LUZ_AUTO;
                    //sprintf(rpms, "luces auto\n");
                    //SendMessage(rpms);
                break;

            case 3:   // Wipers
                cambio = 0;
                if (estado_wipers == 1){
                    estado_wipers = 0;
                    write_msg(TSIDH, TSIDL, WRITE, INST_WIPERS_BOT, 0, WIPERS_OFF);
                    ECAN_Transmit(mensaje, 0);
                    LED_WIP = 0;
                    //sprintf(rpms, "wip off\n");
                    //SendMessage(rpms);
                }
                else{
                    estado_wipers = 1;
                    write_msg(TSIDH, TSIDL, WRITE, INST_WIPERS_BOT, 0, WIPERS_ON);
                    ECAN_Transmit(mensaje, 0);
                    LED_WIP = 1;
                    //sprintf(rpms, "wip on\n");
                    //SendMessage(rpms);
                }
                break;

            case 31:    // Wipers Auto mode
                cambio = 0;
                if (estado_control_wipers == CONTROL_WIPERS_AUTO_OFF){
                    estado_control_wipers = CONTROL_WIPERS_AUTO_ON;
                    LED_WIP_AUTO = 1;
                }
                else{
                    estado_control_wipers = CONTROL_WIPERS_AUTO_OFF;
                    LED_WIP_AUTO = 0;
                }
                write_msg(TSIDH, TSIDL, WRITE, INST_WIPERS_BOT, 0, estado_control_wipers);
                ECAN_Transmit(mensaje, 0);
                //sprintf(rpms, "wip auto\n");
                //SendMessage(rpms);
                break;

            case 4: //parking
                cambio = 0;
                if (estado_parking == PARKING_ON){
                    estado_parking = PARKING_OFF;
                    LED_PARKING = 0;
                }
                else{
                    estado_parking = PARKING_ON;
                    LED_PARKING = 1;
                }
                write_msg(TSIDH, TSIDL, WRITE, INST_PARKING_BOT, 0, estado_parking);
                ECAN_Transmit(mensaje, 0);
                //LED_PARKING = !LED_PARKING;
                //sprintf(rpms, "parking on off\n");
                //SendMessage(rpms);
                break;
        }
    }
}

void init(void){

    

    //INTCON2bits.INTEDG0 = 1;
    //INTCON2bits.INTEDG0 = 1;
    // Botones
    EnablePullups();
    ANCON1bits.ANSEL9 = 0;
    ANCON1bits.ANSEL8 = 0;
    ANCON1bits.ANSEL10 = 0;

    // Motor
    TRISBbits.TRISB0 = 1; //RB0 digital IN: boton de encendido motor
    
    // Wipers
    TRISBbits.TRISB1 = 1; //RB1 digital IN: boton wipers ON/OFF
    TRISBbits.TRISB4 = 1; //RB2 digital IN: boton wipers Auto
    
    // Luces
    TRISBbits.TRISB5 = 1; //RB5 digital IN: boton de Luces ON/OFF
    TRISBbits.TRISB6 = 1; //RB6 digital IN: boton de Luces Auto
   
    // Sensor parking traseo
    TRISBbits.TRISB7 = 1; //RB7 digital IN: boton parking trasero
    
    INTCONbits.INT0IE = 1;
    INTCON3bits.INT1E = 1;
    //INTCON3bits.INT2E = 1;
    //INTCON3bits.INT3E = 1;
//


    // Habilitar interrupciones PORTB
    INTCON = 0xF8;
    IOCB = 0xF0;


    // Led (luces)
    TRISAbits.TRISA0 = 0;       // A0 LED :: Motor ON/OFF
    LED_MOTOR = 0;
    TRISAbits.TRISA1 = 0;       // A1 LED :: Luces ON/OFF
    LED_LUZ = 0;
    TRISAbits.TRISA2 = 0;       // A2 LED :: Luces Auto
    LED_LUZ_AUTO =0;
    TRISAbits.TRISA3 = 0;       // A3 LED :: Parking ON/OFF
    LED_PARKING =0;
    TRISAbits.TRISA5 = 0;       // A5 LED :: Wipers ON/OFF
    LED_WIP =0;
    TRISAbits.TRISA6 = 0;       // A6 LED :: Wipers auto
    LED_WIP_AUTO =0;

    ANCON0bits.ANSEL0 = 0;      //AN0 Pin configured as a digital port
    PORTAbits.RA0 = 1;



}

void write_msg (unsigned char idh_dest, unsigned char idl_dest, unsigned char type, unsigned char variable, unsigned char byteH, unsigned char byteL){
    seqnumber++;
    mensaje.destIDH = idh_dest;
    mensaje.destIDL = idl_dest;

    mensaje.destino = idl_dest;
    mensaje.type = type;
    mensaje.seq_number = seqnumber;
    mensaje.variable = variable;
    mensaje.byteH = byteH;
    mensaje.byteL = byteL;
    //mensaje.datos = 0x00;
    mensaje.CRC = DATA;
}