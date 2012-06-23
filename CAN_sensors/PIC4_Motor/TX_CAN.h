//TX CAN FUNCTION

#define DATA 0xBB
#define TXID 0x04

void ECAN_Transmit(char value)
{

    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;
    TXB0SIDH = 0x00;
    TXB0SIDL = TXID;
    TXB0DLC = 0x02; //tamaño 2 bytes
    TXB0D0 = DATA;
    TXB0D1 = value;

    //Configuramos prioridad
    //TXB0CONbits.TXREQ = 1; //Set the buffer to transmit
    TXB0CON = TXB0CON | 0x08; //Set the buffer to transmit, TXREQ = 1
    //while(TXB0CONbits.TXREQ == 1);
    PORTBbits.RB0 = !PORTBbits.RB0;  //invertimos el pin
}
