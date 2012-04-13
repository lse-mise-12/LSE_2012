#include <stdio.h>
#include <unistd.h> //libreria del sleep
#include <fcntl.h>
#include <errno.h>

int main(void){
	
	unsigned char byte_spi = 0;
	unsigned char dato = 0x0A;
	unsigned char instruccion[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09}; //,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11}; 
	int reg_spi;
	int escritos, leidos;
	escritos=leidos=0;

	reg_spi = open("/root/spi_if",O_RDWR | O_FSYNC); // Open SPI Interface
	printf("reg_spi = %d\n", reg_spi);
	if (reg_spi > 0){
	printf("spi_if open succeeded\n");
	} else {
		printf ("Open failed.\n");
		switch (errno) {
			case EACCES:  printf ("Permission denied.\n"); break;
			case EMFILE:  printf ("No file handle available.\n"); break;
			case ENOENT:  printf ("File or path not found.\n"); break;
			default:      printf ("Unknown error.\n"); break;
		}
		printf("Numero de error %d\n",errno);
	}
	
	// Envío de un array de bytes a través del SPI
	escritos = write(reg_spi, instruccion, sizeof(instruccion));
	printf("Numero de bytes escritos = %d\n",escritos);

	// Lectura de uno en uno de los datos recibidos a través del SPI
	do {
	leidos = read(reg_spi, &byte_spi, 1);
	if(leidos == 0) printf("No hay mas datos que leer\n");
	else printf("Dato leido = 0x%02X\n", byte_spi);
	} while(leidos > 0);
	
	close(reg_spi);
	return 0;
}
