//Soy el PIC 4
#define MYIDHIGH PIC4IDH
#define MYIDLOW PIC4IDL



#define TSIDH 0x00
#define TSIDL 0x00
//#define MYIDLOW 0x60
//#define TXID 0x04


//MOTOR CONFIGURATION
#define MAXSERIALMSG        80
#define NUM_CONVERTIONS     10
#define ADC_TOP             1979

/*********************************************************************
*
*                            Global Variables
*
*********************************************************************/


void SendMessage (char* );
unsigned char seqnumber;
struct CAN_mesg mensaje;
//const char msg_inicial[] = "\r\n\n  **PIC18LF45K80**\r\nCAN module test\r\n\n\n";

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
    
    //ID del MOTOR(Yo)= MYIDHIGH||MYIDLOW
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

    // Initialize Receive Masks
    RXM0EIDH = 0x00;      //
    RXM0EIDL = 0x00;
    //m�scara todo a "1" --> deja pasar todo el ID hacia el filtro
    RXM0SIDH = 0xFF;     // Standard and extended msg will be received
    RXM0SIDL = 0xE0;


    //  The second mask is used to ignore all SIDs and EIDs
    RXM1EIDH = 0x00;    // 0's for EID and SID
    RXM1EIDL = 0x00;
    //m�scara todo a "1" --> deja pasar todo el ID hacia el filtro
    RXM1SIDH = 0xFF;     // Standard and extended msg will be received
    RXM1SIDL = 0xE0;



    // Enter CAN module into normal mode
    //CANCONbits.REQOP = 0;
    do{
        CANCONbits.REQOP = 0;
    }while(CANSTATbits.OPMODE != 0);
    //loop back mode bit 7-5 = 010
    /*
    do{
        CANCONbits.REQOP = 0x02;
    }while(CANSTATbits.OPMODE != 0x02);
    */
    // Set Receive Mode for buffers; bit 6-5
    //11 = Receive all messages (including those with errors); filter criteria is ignored
    //10 = Receive only valid messages with extended identifier; EXIDEN in RXFnSIDL must be ?1?
    //01 = Receive only valid messages with standard identifier; EXIDEN in RXFnSIDL must be ?0?
    //00 = Receive all valid messages as per the EXIDEN bit in the RXFnSIDL register
    RXB0CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB0CONbits.RXM1 = 1;//RXB1CON = 0x00;
    RXB1CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB1CONbits.RXM1 = 1;//RXB1CON = 0x00;

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
void ECAN_Transmit(struct CAN_mesg mensaje, int selec){
    char tempChar[2];
    if (selec == 1){    // Se manda el valor en int datos
        memcpy(&tempChar[0], &mensaje.datos, sizeof(mensaje.datos));
    }else{              // Se manda el valor en byteL y byteH
        tempChar[1] = mensaje.byteH;
        tempChar[0] = mensaje.byteL;
    }

    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;
    TXB0SIDH = mensaje.destIDH; //direcci�n del pic de las luces
    TXB0SIDL = mensaje.destIDL;

    TXB0DLC = 0x08; //tama�o 1 bytes

    TXB0D7 = 0x0A;  // versi�n de protocolo CAN
    TXB0D6 = mensaje.destino; // comando de destino
    TXB0D5 = mensaje.type; // Tipo de comando
    TXB0D4 = mensaje.seq_number;
    TXB0D3 = mensaje.variable;  // Numero de variable
    TXB0D2 = tempChar[1]; // MSB Valor de variable
    TXB0D1 = tempChar[0]; // LSB Valor de variable
    TXB0D0 = mensaje.CRC;

    TXB0CON = TXB0CON | 0x08; //Set the buffer to transmit, TXREQ = 1
    //PORTBbits.RB0 = !PORTBbits.RB0;  //invertimos el pin
}
/*
void ECAN_Transmit(char value)
{

    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;

    TXB0SIDH = TSIDH;
    //TXB0SIDLbits.SID = TSIDL;
    TXB0SIDL = TSIDL;

    TXB0DLC = 0x08; // Tama�o 8 bytes

    TXB0D7 = 0x0A; // Versi�n protolo (1.0)
    TXB0D6 = 0x00; // Direcci�n de destino (TS --> ID=0)
    TXB0D5 = 0x02; // Instrucci�n WRITE 
    TXB0D4 = 0x00; // numero de secuencia
    TXB0D3 = DATA; // Variable que se va a escribir
    TXB0D2 = 0x00; // MSB Valor de variable
    TXB0D1 = value; // LSB Valor de variable
    TXB0D0 = 0x00; // Valor CRC

    

    //Configuramos prioridad
    //TXB0CONbits.TXREQ = 1; //Set the buffer to transmit
    TXB0CON = TXB0CON | 0x08; //Set the buffer to transmit, TXREQ = 1
        //PORTBbits.RB0 = 0;
     //   PORTAbits.RA0 = 0;
    //while(TXB0CONbits.TXREQ == 1)
    //{
    //    PORTAbits.RA0 = 1;
        //TXB0CONbits.TXREQ = 0;
    //}
     //   PORTAbits.RA0 = 0;

    //    PORTBbits.RB0 = 1;
    //if (TXB0CONbits.TXERR == 1)
    //    PORTBbits.RB0 = 0;
                //!PORTBbits.RB0;  //invertimos el pin
    //SendMessage("123\n");
}
 */

/*********************************************************************
*
*                Check the buffers, if it have message
*
*********************************************************************/
unsigned char ECAN_Receive(void)
{

    if (RXB0CONbits.RXFUL){
        PORTAbits.RA0 = !PORTAbits.RA0;     
        RXB0CON = RXB0CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFE;     //RXB0IF = 0;
        return TRUE;    
    }
    
    else if (RXB1CONbits.RXFUL){
        PORTAbits.RA0 = !PORTAbits.RA0;
        RXB1CON = RXB1CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFD;     //RXB1IF = 0;        
        return TRUE;
    }
    return FALSE;
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


void write_msg (unsigned char idh_dest, unsigned char idl_dest, unsigned char type, unsigned char variable, unsigned char byteH, unsigned char byteL, int datos){
    seqnumber++;
    mensaje.destIDH = idh_dest;
    mensaje.destIDL = idl_dest;

    mensaje.destino = idl_dest;
    mensaje.type = type;
    mensaje.seq_number = seqnumber;
    mensaje.variable = variable;
    mensaje.byteH = byteH;
    mensaje.byteL = byteL;
    mensaje.datos = datos;
    mensaje.CRC = DATA;
}
