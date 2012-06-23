
// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RC6 and RC7, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = OFF      // Master Clear Enable (MCLR Disabled, RG5 Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

// CAN definitions
#define TRUE    1
#define FALSE   0

/*** Instrucciones CAN ***/
// Variables
//PIC1
#define INST_STATE1 0x04    // Estado del coche - Seguridad

//#define INST_HUM_MODO 0x07 // Manual/Auto
//#define INST_HUM_STATE 0x08 // ON/OFF
#define INST_WIPERS 0x07    //Estado limpia
#define INST_W_AUTO 0x08   // Estado limpia auto
#define INST_AAC_STATE 0x10 // Estado del aire acondicionado - On/Off/Auto
#define INST_TEMP_INT 0x11  // Temperatura interior
#define INST_AAC_UMBRAL 0x12    // Umbral de encendido apagado del aire acondicionado
#define INST_TEMP_REFRESH 0x1A // Modificar el tiempo de refresco de la temperatura

#define INST_PUERTA 0x05
#define PUERTA_ABIERTA 0x01
#define PUERTA_CERRADA 0x00

//PIC2
#define INST_STATE2 0x13    // Estado del coche
#define INST_LIGHT_CONTROL 0x14   // Estado de las luces
#define INST_LIGHT_UMBRAL 0x15    // Umbral de las luces
#define INST_ERROR  0xFF    // Variable de error

// Valores
#define STATE_ON 0x01       // Estado del coche - On
#define STATE_OFF 0x00      // Estado del coche - Off

//PIC1
#define ACC_ON 0x01         // Aire acondicionado - On
#define ACC_OFF 0x00        // Aire acondicionado - Off
#define ACC_AUTO 0x10       // Aire acondicionado - Auto

//PIC2
#define CONTROL_LIGHT_ON 0x01     // Estado de las luces - On
#define CONTROL_LIGHT_OFF 0x00    // Estado de las luces - Off
#define CONTROL_LIGHT_AUTO 0x10   // Estado de las luces - Control automatico
#define LIGHT_ON 0x01       // Estado de las luces - Write
#define LIGHT_OFF 0x00      // Estado de las luces - Write


// Tipos de comando
#define TEST  0x00
#define READ  0x01
#define WRITE 0x02
#define ERROR 0x03
#define EVENT 0x04

#define DATA 0xBB // Valor de CRC

//TS :: ID = 0x0000
#define TSIDH 0x00
#define TSIDL 0x00

//PIC 1: Airbag, Seguridad, Limpia, Sleep, Temp :: ID = 0x0001
#define PIC1IDH 0x00
#define PIC1IDL 0x20

//PIC 2: Distancia, Luces :: ID = 0x0002
#define PIC2IDH 0x00
#define PIC2IDL 0x40

//PIC 3: Parking :: ID = 0x0003
#define PIC3IDH 0x00
#define PIC3IDL 0x60

//PIC 4: Motor :: ID = 0x0004
#define PIC4IDH 0x00
#define PIC4IDL 0x80

//PIC 5: Control Panel :: ID = 0x0005
#define PIC5IDH 0x00
#define PIC5IDL 0xA0	

struct CAN_mesg{
    unsigned char destIDH;      //TXB0SIDH  :: Destino High CAN
    unsigned char destIDL;      //TXB0SIDL  :: Desitno Low CAN
//  unsigned char ver;          //TXB0D7    :: Version del protocol
    unsigned char destino;      //TXB0D6    :: PIC destino
    unsigned char type;         //TXB0D5    :: Tipo de comando
    unsigned char seq_number;   //TXB0D4    :: Numero de secuencia
    unsigned char variable;     //TXB0D3    :: Variable
    unsigned char byteH;        //TXB0D2    :: MSB
    unsigned char byteL;        //TXB0D1    :: LSB
    int           datos;        //datos -> TXB0D1 && TXB0D2
    unsigned char CRC;          //TXB0D0    :: CRC
};
