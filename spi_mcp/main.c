#include <stdio.h>
#include <unistd.h> //libreria del sleep
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include "mcp.h"
#include "mcp_registers.h"

int reg_spi;

int main(int argc, char* argv[])
{
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
		return -1;
	}

	// DEFINIR MANEJADOR PARA ATENDER SEÑALES SIGIO ENVIADAS POR EL DRIVER
	printf("Traza 1 - Antes del signal\n");
	signal(SIGIO, &tigal_handler); 
	printf("Traza 2 - Antes del 1er fcntl\n");
	fcntl(reg_spi, F_SETOWN, getpid());
	printf("Traza 3 - Antes del 2o fcntl\n");
	int oflags = fcntl(reg_spi, F_GETFL);
	printf("Traza 4 - Antes del 3er fcntl\n");
	fcntl(reg_spi, F_SETFL, oflags | FASYNC);
	printf("Traza 5 - Despues del 3er fcntl\n");

	CanMessage can;
	can.id=1;
	can.dlc=1;
	can.data[0]=0x10;
	can.data[1]=0xAA;

	if(argc==2){
		if(strncmp(argv[1], "reset", strlen(argv[1]))==0){
		mcp_init(reg_spi);
		printf("Reset enviado\n");
		return 0;
		}
	}
	
	mcp_init(reg_spi);
	printf("traza a\n");
	can_send(reg_spi, &can);
	

	char aux;
	printf("traza b. reg_spi %d\n", reg_spi);
	aux = mcp_readRegister(reg_spi, MCP_CANCTRL);
	printf("Dato leido CANCTRL %02X\n",aux);
	
	aux = mcp_readRegister(reg_spi, MCP_CANSTAT);
	printf("Dato leido CANSTAT %02X\n",aux);	

	aux = mcp_readRegister(reg_spi, MCP_CNF1);
	printf("Dato leido CNF1 %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_RXB0CTRL);
	printf("Dato leido RXB0CTRL %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_RXB1CTRL);
	printf("Dato leido RXB1CTRL %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL);
	printf("Dato leido TXB0CTRL %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL+5);
	printf("Dato leido TXB0DLC %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL+6);
	printf("Dato enviado TXB0D0 %02X\n",aux);	

	aux = mcp_readStatus(reg_spi);
	printf("Dato leido STATUS %02X\n",aux);	

	//aux = mcp_readRegister(reg_spi, MCP_RXB0CTRL+6);
	//printf("Dato recibido RXB0D0 %02X\n",aux);

	aux = mcp_readRegister(reg_spi, MCP_CANINTE);
	printf("Dato leido CANINTE %02X\n",aux);		

	aux = mcp_readRegister(reg_spi, MCP_CANINTF);
	printf("Dato leido CANINTF %02X\n",aux);		
		
	while(1){
		usleep(100000);
	}
	close(reg_spi);
	return 0;
}

char clear_interrupt(char interr)	{
	char aux;
	aux = mcp_readRegister(reg_spi, MCP_CANINTF);
	interr=~interr; // hacemos el complemento de interr que tiene a '1' el bit que nos interesa, para ponerlo a 0
	aux&=interr; // hacemos la máscara
	mcp_writeRegister(reg_spi, MCP_CANINTF, aux);	// limpiamos el flag de interrupcion 'interr' en el registro del MCP
	return 1;
}

void tigal_handler(int signum){
	char icod, aux;
	if(signum == SIGIO){
		// tratamiento de la interrupcion TIGAL
		// Enviamos un mensaje al TIGAL para comprobar la fuente de la interrupción
		icod = mcp_readRegister(reg_spi, MCP_CANSTAT);
		printf("Dato leido CANSTAT %02X\n",icod);	
		icod>>=1;
		icod&=0x07;
		switch(icod) {
			case 0x02: // WAK Interrupt
				printf("WAK Interrupt\n");
				clear_interrupt(INT_WAK);
				break;
			case 0x03: // TX0 Interrupt
				printf("TX0 Interrupt\n");
				clear_interrupt(INT_TX0);
				break;
			case 0x04: // TX1 Interrupt
				printf("TX1 Interrupt\n");
				clear_interrupt(INT_TX1);
				break;
			case 0x05: // TX2 Interrupt
				printf("TX2 Interrupt\n");
				clear_interrupt(INT_TX2);
				break;
			case 0x06: // RX0 Interrupt
				printf("RX0 Interrupt\n");
				// AQUI SE DEBE TRATAR LOS MENSAJES RECIBIDOS EN RX0
				aux = mcp_readRegister(reg_spi, MCP_RXB0CTRL+6);
				printf("Dato recibido en RXB0D0 = %02X\n",aux);	
				clear_interrupt(INT_RX0);
				break;
			case 0x07: // RX1 Interrupt
				printf("RX1 Interrupt\n");
				// AQUI SE DEBE TRATAR LOS MENSAJES RECIBIDOS EN RX1
				clear_interrupt(INT_RX1);
				break;
			case 0x00: // Ninguna interrupción activada
				printf("Atencion! Ha habido un disparo de la interrupción sin interrupcion en CAN\nICOD = %02X\n",icod);
				break;
			case 0x01: // ERR Interrupt. Asociada con CANINTE y ERRIE.
				// Leemos el Flag de Error para saber la fuente	de error			
				aux = mcp_readRegister(reg_spi, MCP_EFLG);
				printf("Dato leido EFLG %02X\n",aux);
				clear_interrupt(INT_MERR);
				clear_interrupt(INT_ERR);
				switch(aux) {
					case 0x01: //TX OR RX WARNING (TEC o REC superior a 96)
					printf("TX OR RX WARNING (TEC or REC is equal or greater than 96)\n");
					break;
					case 0x02: //RX WARNING
					printf("RX WARNING Set when REC is equal to or greater than 96\n");					
					break;
					case 0x04: //TX WARNING
					printf("TX WARNING TEC is equal to or greater than 96\n");				
					break;
					case 0x08: //RX ERROR-PASSIVE
					printf("RX WARNING REC is equal to or greater than 128\n");				
					break;
					case 0x10: //TX ERROR-PASSIVE
					printf("TX WARNING TEC is equal to or greater than 128\n");				
					break;
					case 0x20: //TX BUS-OFF
					printf("Bit set when TEC reaches 255\n");				
					break;
					case 0x40: //RX0 Overflow
					printf("Set when a valid message is received for RXB0 and CANINTF.RX0IF = 1\n");				
					break;
					case 0x80: //RX1 Overflow
					printf("Set when a valid message is received for RXB1 and CANINTF.RX1IF = 1\n");				
					break;
					default:
					printf("Error inesperado: EFLG = %02X\n", aux);
					break;
				}
				break;
			default:
				printf("Atencion! Ha habido un error en la lectura de ICOD = %02X\n",icod);
				break;
		}
	}else{
		fprintf(stderr, "Signal %d received\n", signum);
	}
	signal(SIGIO, &tigal_handler); 
}

void can_send(int desc, CanMessage* can_msg){
	//uint8_t txbuf;
	//mcp_TXBuffer(&txbuf);
	mcp_write_canMsg(desc, MCP_TXB0CTRL+1, can_msg);
	mcp_start_tx(desc, MCP_TXB0CTRL+1);
}

void can_receive(int desc, CanMessage* can_msg)
{
	uint8_t stat; 
	
	stat = mcp_readStatus(desc);
	
	if ( stat & MCP_STAT_RX0IF ) {
		mcp_read_canMsg(desc, MCP_RXB0SIDH,can_msg);
		mcp_modifyRegister(desc, MCP_CANINTF, MCP_RX0IF, 0);

	}
	else if ( stat & MCP_STAT_RX1IF ) {
		mcp_read_canMsg(desc, MCP_RXB1SIDH,can_msg);
		mcp_modifyRegister(desc, MCP_CANINTF, MCP_RX1IF, 0);
	}	
}
