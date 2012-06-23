#ifndef _SPI_H
#define _SPI_H

#define DIO_DATA16_19 0x12000001UL	// Registros de datos de DIO16 a DIO19
#define RXTX_FIFOSIZE 8 // Las FIFO de RX y TX tienen 8 entradas de memoria (tamaño max por entrada 16 bits)
#define REG_ADDR_PBDR 0x80840004
#define REG_ADDR_PBDDR 0x80840014
#define GPIOBIntEn 0x808400B8
#define GPIOBIntType1 0x808400AC
#define GPIOBIntType2 0x808400B0
#define GPIOBEOI 0x808400B4
#define GPIOBDB 0x808400C4
#define GPIOINTR_VECTOR 59 //GPIO COMBINED INTERRRUPT PORT A-B
#define PORTB4_MASK 0x10

#define CAN_DATA_LENGTH 8
#define BUFFERSIZE CAN_DATA_LENGTH*100 // Tamaño máximo del buffer de recepción

#endif
