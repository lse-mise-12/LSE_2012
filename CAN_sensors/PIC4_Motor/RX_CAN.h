// RX CAN FUNCTION

unsigned char ECAN_Receive(void)
{

    if (RXB0CONbits.RXFUL){
        RXB0CON = RXB0CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFE;     //RXB0IF = 0;
        PORTAbits.RA0 = !PORTAbits.RA0;
        if(RXB0D0==0xFF)
        return 1;
        else
        return  0;

    }else if (RXB1CONbits.RXFUL){
        RXB1CON = RXB1CON & 0x7F; //RXFUL = 0;
        PIR5 = PIR5 & 0xFD;     //RXB1IF = 0;
        if(RXB1D0==0xFF)
        return 1;
        else
        return  0;
    }

    return 0;
}
