//CAN BUS CONFIGURATION


//Initialize the CAN port
void CONFIG_CAN(void){

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
    RXF2SIDH = 0xF0;
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
    //  The first mask is used that accepts all SIDs and no EIDs
    RXM0EIDH = 0x00;      //
    RXM0EIDL = 0x00;
    RXM0SIDH = 0xFF;     // Standard and extended msg will be received
    RXM0SIDL = 0xE0;


    //  The second mask is used to ignore all SIDs and EIDs
    RXM1EIDH = 0x00;    // 0's for EID and SID
    RXM1EIDL = 0x00;
    RXM1SIDH = 0xFF;     // Standard and extended msg will be received
    RXM1SIDL = 0xE0;


    // Enter CAN module into normal mode
    do{
        CANCONbits.REQOP = 0;
    }while(CANSTATbits.OPMODE != 0);

    // Set Receive Mode for buffers; bit 6-5
    //11 = Receive all messages (including those with errors); filter criteria is ignored
    //10 = Receive only valid messages with extended identifier; EXIDEN in RXFnSIDL must be ?1?
    //01 = Receive only valid messages with standard identifier; EXIDEN in RXFnSIDL must be ?0?
    //00 = Receive all valid messages as per the EXIDEN bit in the RXFnSIDL register
    RXB0CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB0CONbits.RXM1 = 0;//RXB1CON = 0x00;
    RXB1CONbits.RXM0 = 1;//RXB0CON = 0x00;
    RXB1CONbits.RXM1 = 0;//RXB1CON = 0x00;

}
