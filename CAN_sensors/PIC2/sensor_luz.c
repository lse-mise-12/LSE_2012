/*
 * File:   sensor_luz.c
 * Author: albarc
 *
 * Created on 19 de abril de 2012, 20:57
 */

#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <adc.h>            // ADC library functions
#include <portb.h>          // PORTB library function
#include <timers.h>
#include <math.h>

// Configuration Bit Values

// CONFIG1H
#pragma config OSC = HSPLL//4Mhz Xtal x4 = @16MHz//INTIO67      // Oscillator Selection bits (External RC oscillator, port function on RA6)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)
// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)
// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)
// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)
// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit

//---------------------------------------------------------------------
// Function Prototypes
//---------------------------------------------------------------------
void Setup(void);				// Configures peripherals and variables
// Writes a string from ROM to the USART
void putrsUSART(const rom char *data);
void delayms(int tiempo );
void SendMessage ( char* Msg );
void SetTimer(unsigned int time);
void lecturaADC(void);
int NSL19M51ResistanceToIlluminance( int resistance );
void lecturaADCdummy(void);

#define Wait4Timer();    while(INTCONbits.TMR0IF==0);

//int ADCvalue = 0;

//---------------------------------------------------------------------
// Variable declarations
//---------------------------------------------------------------------

const char end[] = ";\r\n";
const char start[] = " Programa para medir del sensor de luz:\r\n";
const char finLinea[] = "\r\n";

unsigned char s[80]; 			// auxiliar buffer



// Setup de todos los módulos necesarios
void Setup(void) {
int i;

//EnablePullups();           // Enable PORTB pullups
ADCON1 = 0b00000100;//0x00;      // all ports to analogs      //0x0f;// all ports to digital
					//
//DDRCbits.RC3 = 1; // Set SCL (PORTC,3) pin to input
//DDRCbits.RC4 = 1; // Set SDA (PORTC,4) pin to input

TRISAbits.TRISA2 = 0;  //OUTPUT PIN
//PORTAbits.RA2 = 1; // FER: deshabilita el sensor de temperatura de PICDEM Z #CS = 1
LATAbits.LATA2 = 1;

TRISAbits.TRISA1 = 0;         //RA1 AN1 as out LED
LATAbits.LATA1 = 0;
TRISAbits.TRISA0 = 0;         //RA0 AN0 as out LED
LATAbits.LATA0 = 0;
TRISAbits.TRISA3 = 0;         //RA3 AN3 as out LED
LATAbits.LATA3 = 0;

EnablePullups();
TRISBbits.TRISB5 = 1; //RB5 digital IN
TRISBbits.TRISB4 = 1; //RB4 digital IN


//UART
//TRISC &= ~(0x40);
//TRISC |= 0x80;
TRISCbits.TRISC6 = 0; //TX out
TRISCbits.TRISC7 = 1; //RX in

// Setup the USART for 19200 baud @ 16MHz//20MHz
SPBRG = 12;//25;//9600 baud @ 16MHz//12;//15;  // 19200 baud @ 16MHz//20MHz
TXSTA = 0x20;					// setup USART transmit
RCSTA = 0x90;					// setup USART receive

//// I2C
//OpenI2C(MASTER,SLEW_OFF);	// Setup MSSP for master I2C
////SSPSTAT = 0x80;				// slew diabled
////SSPCON1 = 0x38;				// master mode enabled
//SSPADD = 39;//49;       			// 100KHz @ 16MHz//20MHz

//@16MHz
//   SSPADD       FreqI2C_SCL
//  39 0x27         100KHz
//  9  0x09         400KHz
//  99  xx           40KHz

//TIMER

//T0CON = 0x81;				// 16bit, 4:1 prescaler
//OpenTimer0(TIMER_INT_OFF&T0_SOURCE_INT&T0_16BIT&T0_PS_1_32); //(Fosc/4)/32 => To= 8 usec
T0CON = 0b10000100;//0b100X0100
//ADC
//TRISBbits.TRISB2 = 1;         //RB2 AN8 as input
TRISBbits.TRISB1 = 1;         //RB1 AN10 as input
//TRISEbits.RE1 = 1;         //Re1 AN6 as input

OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH8 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 0);
ADCON1 = 0b00000100;
//PIE1bits.SSPIE = 1;	 // Habilita la interrupcion del modulo SSP
//INTCONbits.PEIE = 1;					// Enable peripheral interrupts
//INTCONbits.GIE = 1;					// Enable all interrupts

//control ultrasonidos caros
TRISDbits.TRISD5 = 0;         //RD5 as out
LATDbits.LATD5 = 0;         //ULTRASONIDOS 2 PARADO
TRISDbits.TRISD6 = 0;         //RD6 as out
LATDbits.LATD6 = 0;         //ULTRASONIDOS 3 PARADO
TRISDbits.TRISD7 = 0;         //RD7 as out
LATDbits.LATD7 = 0;         //ULTRASONIDOS 4 PARADO
}

void main(void) {
    
    Setup();
    SendMessage(start);
//    TRISA = 0;

    while (1) {
        if(!PORTBbits.RB5){// si es cero, es decir si esta pulsado
            LATAbits.LATA0 = !LATAbits.LATA0;
            while(!PORTBbits.RB5){}//antirrebotes
        }else{}

        if(!PORTBbits.RB4){// si es cero, es decir si esta pulsado
            lecturaADCdummy();
            lecturaADC();
            while(!PORTBbits.RB4){}//antirrebotes
        }else{}
//
//        PORTA |= 0x02;
//        Delay10KTCYx(100);
////        delay();
//
//        PORTA &= ~(0x02);
//        Delay10KTCYx(100);
//        delay();
    }
    return;
}

void lecturaADC(void) {
    int ADC_upper_value = 1024;
    int resultADC;
    int milivolts = 0;
    int illuminance;
    int resistance;
    double voltRef = 3.3;
    double volts;
    float kOhm;
    int mVoltRef = 3300;
    int prueba;

    // Leer del ADC. PIC_FER (RB0 = CANAL 12) // PIC_LSEL (RB0 = CANAL 10)
    ADCON0bits.CHS = 12;
    ConvertADC();
    while(BusyADC());
    resultADC = ReadADC();

    sprintf(s, "Valor del ADC: %i #\r\n", resultADC);
    SendMessage(s);

    // Pasar el valor del ADC a milivoltios
    milivolts = (int)floor((((voltRef * resultADC)/ADC_upper_value)*1000) + 0.5);

    sprintf(s, "Valor en milivoltios: %i #\r\n", milivolts);
    SendMessage(s);
//    sprintf(s, "Valor en voltios: %.3f #\r\n", volts);
//    SendMessage(s);

    // Sacar el valor de la fotorresistencia del divisor de voltaje
    if (milivolts < 200) {
        resistance = 100;
    }
    if (201 < milivolts < 300) {
        resistance = 500;
    }
    if (301 < milivolts < 400) {
        resistance = 1000;
    }
    if (401 < milivolts < 500) {
        resistance = 2000;
    }
    if (501 < milivolts < 600) {
        resistance = 3000;
    }
    if (601 < milivolts < 700) {
        resistance = 4000;
    }
    if (701 < milivolts < 800) {
        resistance = 5000;
    }
    if (801 < milivolts < 900) {
        resistance = 6000;
    }
    if (901 < milivolts < 1000) {
        resistance = 7000;
    }
    if (milivolts > 1001) {
        resistance = 8000;
    }
//    prueba = 3300 - milivolts;
//    sprintf(s, "prueba: %i #\r\n", prueba);
//    SendMessage(s);
//    unsigned long int num = (unsigned long int) (200000 * milivolts);
//    resistance = (long int) floor((66.6 * milivolts)+0.5);   // 200000/3000 = 66.6
//    kOhm = (double) (((double)200 * (double)milivolts)/((double)mVoltRef - (double)milivolts));
//    sprintf(s, "Valor num: %lu #\r\n", num);
//    SendMessage(s);
    sprintf(s, "Valor de la fotorresistencia en ohm: %i #\r\n", resistance);
    SendMessage(s);
//    sprintf(s, "Valor de la fotorresistencia en kohm: %.2f #\r\n", kOhm);
//    SendMessage(s);

    // Mediante la tabla de calibrado sacar el valor en luxes de la luminosidad
    illuminance = NSL19M51ResistanceToIlluminance(resistance);

    sprintf(s, "Valor de la iluminancia en luxes: %i #\r\n\r\n", illuminance);
    SendMessage(s);
}

int NSL19M51ResistanceToIlluminance( int resistance )
{
  // Convert resistance to illuminance according to the NSL_19M51 curve
  double logV = (double) (((double)resistance + (double)374)/(double)135000);
  double logValue = log10(logV);
  int illuminance = (int) pow(10, ((0.0 - logValue)/0.7));
  return illuminance;
}

void lecturaADCdummy(void) {
    int dummy = 0;
    int i = 0;
    for (i = 0; i<10; i++){
        ADCON0bits.CHS = 12;
        ConvertADC();
        while(BusyADC());
        dummy = ReadADC();
    }
}
void SendMessage ( char* Msg )
{
    int len, count;

    len = strlen(Msg);
    for (count = 0; count < len; count++)
    {
        while( !PIR1bits.TXIF );
        TXREG = (Msg[count]);
    }
}

void putrsUSART(const rom char *data)
{
	do
	{
		while(!(TXSTA & 0x02));
		TXREG = *data;
	} while( *data++ );

}
