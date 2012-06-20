
#define TRUE    1
#define FALSE   0


#define TIMER_VALUE 0xE17B  //1s


#define DATA 0xBB
/*** Instrucciones CAN ***/
// Variables
#define INST_STATE 0x13     // Estado del coche
#define INST_CONTROL 0x14   // Estado de las luces
#define INST_UMBRAL 0x15    // Umbral de las luces
#define INST_ERROR  0xFF    // Variable de error

// Valores
#define STATE_ON 0x01       // Estado del coche - On
#define STATE_OFF 0x00      // Estado del coche - Off
#define CONTROL_ON 0x01     // Estado de las luces - On
#define CONTROL_OFF 0x00    // Estado de las luces - Off
#define CONTROL_AUTO 0x10   // Estado de las luces - Control automatico
#define LIGHT_ON 0x01       // Estado de las luces - Write
#define LIGHT_OFF 0x00      // Estado de las luces - Write

// Tipos de comando
#define TEST  0x00
#define READ  0x01
#define WRITE 0x02
#define ERROR 0x03
#define EVENT 0x04

#define TIMER_VALUE 0xE17B  //1s


#define DATA 0xBB // Valor de CRC

//TS :: ID = 0x0000
#define TSIDH 0x00
#define TSIDL 0x00

//PIC 1: Airbag, Seguridad, Limpia, Sleep, Temp :: ID = 0x0001
#define PIC1IDH 0x00
#define PIC1IDL 0x01

//PIC 2: Distancia, Luces :: ID = 0x0002
#define PIC2IDH 0x00
#define PIC2IDL 0x02

//PIC 3: Parking :: ID = 0x0003
#define PIC3IDH 0x00
#define PIC3IDL 0x03

//PIC 4: Motor :: ID = 0x0004
#define PIC4IDH 0x00
#define PIC4IDL 0x04

//PIC 5: Control Panel :: ID = 0x0005
#define PIC5IDH 0x00
#define PIC5IDL 0x05

//Soy la TS
#define MYIDHIGH 0x00
#define MYIDLOW 0x00

/*********************************************************************
*
*                            Global Variables
*
*********************************************************************/
unsigned char temp_EIDH;
unsigned char temp_EIDL;
unsigned char temp_SIDH;
unsigned char temp_SIDL;
unsigned char temp_DLC;
unsigned char temp_D0;
unsigned char temp_D1;
unsigned char temp_D2;
unsigned char temp_D3;
unsigned char temp_D4;
unsigned char temp_D5;
unsigned char temp_D6;
unsigned char temp_D7;
unsigned char temp_err;


void SendMessage (char* );

/*
 Configura los registros del puerto UART
 */
void init_uartport(void){

    TRISCbits.TRISC6 = 0;   //TX out
    TRISCbits.TRISC7 = 1;   //RX in
    TXSTA1bits.TXEN = 1;    //Transmit is enabled
    RCSTAbits.SPEN = 1;     //Serial port is enabled
    RCSTAbits.CREN = 1;     //Enables receiver
    SPBRG = 12;             //Baud rate = 9600 a MHz. Formula, SPBRG = (Fosc / (64 * Buad rate) ) - 1

}

/*
 Configura los registros del puerto CAN
 */
void init_canport(void){

    //Configuramos los pines del can TX y Rx
    TRISBbits.TRISB2 = 0;   //hacemos output CANTX
    TRISBbits.TRISB3 = 1;   //hacemos input  CANRX


    // Enter CAN module into config mode
    CANCON = (0x80 | CANCON);       //REQOP<2:0>=100
    while(!(CANSTATbits.OPMODE == 0x04));

    // Enter CAN module into Mode 0
    ECANCON = (0x3f & ECANCON); //ponemos 00 en los bit 7-6

    //  125 Kbps @ 8MHz
    BRGCON1 = 0x03; //0000 0011     //SJW=1TQ     BRP = 3,
    BRGCON2 = 0x90; //1001 0000     //SEG2PHTS=1    sampled=once  PS1=3TQ  PropagationT=1TQ
    BRGCON3 = 0x02; //0000 0010     //PS2=3TQ

    // Initialize Receive Filters
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;

    RXF0SIDH = MYIDHIGH;
    RXF0SIDL = MYIDLOW;


    RXF1EIDH = 0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH = 0x00;
    RXF1SIDL = 0x00;

    RXF2EIDH = 0x00;
    RXF2EIDL = 0x00;
    RXF2SIDH = 0x00;
    RXF2SIDL = 0x00;

    RXF3EIDH = 0x00;
    RXF3EIDL = 0x00;
    RXF3SIDH = 0x00;
    RXF3SIDL = 0x00;

    RXF4EIDH = 0x00;
    RXF4EIDL = 0x00;
    RXF4SIDH = 0x00;
    RXF4SIDL = 0x00;

    RXF5EIDH = 0x00;
    RXF5EIDL = 0x00;
    RXF5SIDH = 0x00;
    RXF5SIDL = 0x00;

    RXM0EIDH = 0x00;      //
    RXM0EIDL = 0x00;
    RXM0SIDH = 0xFF;     // Standard and extended msg will be received
    RXM0SIDL = 0xE0;



    //  The second mask is used to ignore all SIDs and EIDs
    RXM1EIDH = 0x00;    // 0's for EID and SID
    RXM1EIDL = 0x00;
    RXM1SIDH = 0xFF;     // Standard and extended msg will be received
    RXM1SIDL = 0xE0;



    do{
        CANCONbits.REQOP = 0;
    }while(CANSTATbits.OPMODE != 0);

    RXB0CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB0CONbits.RXM1 = 0;//RXB1CON = 0x00;
    RXB1CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB1CONbits.RXM1 = 0;//RXB1CON = 0x00;

    PIE5bits.RXB0IE = 1;
    PIE5bits.RXB1IE = 1;

    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1; // Enable global interrupts


}

/*********************************************************************
*
*                      Transmit Sample Message
*
*********************************************************************/
void ECAN_Transmit(unsigned char value)
{

    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;
    TXB0SIDH = PIC5IDH; //dirección del motor
    TXB0SIDL = PIC5IDL;

    TXB0DLC = 0x08; //tamaño 1 bytes

    TXB0D7 = 0x0A;  // versión de protocolo CAN
    TXB0D6 = 0x04; // comando de destino
    TXB0D5 = 0x02; // comando write
    TXB0D4 = 0x00;
    TXB0D3 = 0x00;
    TXB0D2 = 0x00;
    TXB0D1 = value;
    TXB0D0 = 0x00;


    TXB0CON = TXB0CON | 0x08; //Set the buffer to transmit, TXREQ = 1
    PORTBbits.RB0 = !PORTBbits.RB0;  //invertimos el pin
}


/*********************************************************************
*
*                Check the buffers, if it have message
*
*********************************************************************/
unsigned char ECAN_Receive(void)
{
    if (RXB0CONbits.RXFUL){
        temp_EIDH = RXB0EIDH;
        temp_EIDL = RXB0EIDL;
        temp_SIDH = RXB0SIDH;
        temp_SIDL = RXB0SIDL;
        temp_DLC = RXB0DLC;
        temp_D0 = RXB0D0;
        temp_D1 = RXB0D1;
        temp_D2 = RXB0D2;
        temp_D3 = RXB0D3;
        temp_D4 = RXB0D4;
        temp_D5 = RXB0D5;
        temp_D6 = RXB0D6;
        temp_D7 = RXB0D7;
        temp_err = RXERRCNT;
        PORTAbits.RA0 = !PORTAbits.RA0;
        RXB0CON = RXB0CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFE;     //RXB0IF = 0;
        return 1;
    }else if (RXB1CONbits.RXFUL){
        PORTAbits.RA0 = !PORTAbits.RA0;
        RXB1CON = RXB1CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFD;     //RXB1IF = 0;
        return 1;
    }
    return 0;
}


/*
 Configura los registros del TIMER 0
 */
void init_timer0port(void){
    WriteTimer0(TIMER_VALUE);
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
}


void SendMessage (char* Msg) {
    int len, count;

    len = strlen(Msg);
    for (count = 0; count < len; count++)
    {
        while( !PIR1bits.TXIF );
        TXREG = (Msg[count]);
    }
}

//LED del programador
void SetupPortA(void){
    TRISAbits.TRISA0 = 0;         //RA0 AN0 as out LED
    ANCON0bits.ANSEL0 = 0;      //AN0 Pin configured as a digital port
    PORTAbits.RA0 = 1;

    TRISBbits.TRISB0 = 0;   //hacemos output
    ANCON1bits.ANSEL10 = 0;      //AN10 Pin configured as a digital port
    PORTBbits.RB0 = 1;      //hacemos cero el bit 0

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
