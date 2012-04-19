#include <stdio.h>
#include <unistd.h> //libreria del sleep
#include <fcntl.h>
#include <errno.h>
#include "mcp.h"
#include "mcp_registers.h"

int main(void)
{
	int reg_spi, i;
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

	CanMessage can;
	can.id=1;
	can.dlc=1;
	can.data[0]=0x10;
	can.data[1]=0xAA;

	char val = MCP_RESET;
	char aux;
	
	write(reg_spi, &val , 1);
	
	mcp_init(reg_spi);
	can_send(reg_spi, &can);

	aux = mcp_readRegister(reg_spi, MCP_CANCTRL);
	printf("Dato leido CANCTRL %02X\n",aux);	
	aux = mcp_readRegister(reg_spi, MCP_CANSTAT);
	printf("Dato leido CANSTAT %02X\n",aux);	
	aux = mcp_readRegister(reg_spi, MCP_CNF1);
	printf("Dato leido CNF1 %02X\n",aux);
	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL);
	printf("Dato leido TXB0CTRL %02X\n",aux);
	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL+5);
	printf("Dato leido TXB0DLC %02X\n",aux);
	aux = mcp_readRegister(reg_spi, MCP_TXB0CTRL+6);
	printf("Dato enviado TXB0D0 %02X\n",aux);	
	aux = mcp_readStatus(reg_spi);
	printf("Dato leido STATUS %02X\n",aux);	
	aux = mcp_readRegister(reg_spi, MCP_RXB0CTRL+6);
	printf("Dato recibido RXB0D0 %02X\n",aux);	

		
	close(reg_spi);
	return 0;
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
