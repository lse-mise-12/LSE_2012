
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


#define TRUE    1
#define FALSE   0

/*** Instrucciones CAN ***/
#define RX_CRC RXB0D0
#define RX_LSB RXB0D1
#define RX_MSB RXB0D2
#define RX_VAR RXB0D3
#define RX_SEQ RXB0D4
#define RX_TYP RXB0D5
#define RX_DES RXB0D6
#define RX_VER RXB0D7
// Variables
#define INST_TEST 0x00      // Test inicial

#define INST_STATE 0x13     // Estado del coche

#define INST_VELOC 0x22         // Velocidad del motor
#define INST_CC 0x23            // Instruccion Cruise control
#define INST_LIGHT_BOT 0x25
#define INST_ARRA_BOT 0x26
#define INST_WIPERS_BOT 0x27
#define INST_PARKING_BOT 0x28
#define INST_ERROR  0xFF    // Variable de error

// Valores
#define TEST_VALUE 0xA4           // Test mesg.
#define STATE_ON 0x01       // Estado del coche - On - Arranque motor
#define STATE_OFF 0x00      // Estado del coche - Off - Apagar motor

#define CONTROL_LIGHT_AUTO_ON 0x11   // Estado de las luces - Control automatico ON
#define CONTROL_LIGHT_AUTO_OFF 0x10  // Estado de las luces - Control automatico OFF
#define LIGHT_ON 0x01       // Estado de las luces - Write
#define LIGHT_OFF 0x00      // Estado de las luces - Write

#define CONTROL_WIPERS_AUTO_ON 0x11   // Estado Wipers - Control automatico ON
#define CONTROL_WIPERS_AUTO_OFF 0x10  // Estado Wipers - Control automatico OFF
#define WIPERS_ON 0x01       // Estado Wipers ON - Write
#define WIPERS_OFF 0x00      // Estado Wipers OFF - Write

#define PARKING_ON 0x01       // Parking ON - Write
#define PARKING_OFF 0x00      // Parking OFF - Write

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

#define LED_MOTOR LATAbits.LATA0
#define LED_LUZ LATAbits.LATA1
#define LED_LUZ_AUTO LATAbits.LATA2
#define LED_PARKING LATAbits.LATA3
#define LED_WIP LATAbits.LATA5
#define LED_WIP_AUTO LATAbits.LATA6

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
