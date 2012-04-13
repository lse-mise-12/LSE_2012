#include <stdio.h>
#include <unistd.h> //libreria del sleep
#include <fcntl.h>
#include <errno.h>
#include "mcp.h"
#include "mcp_registers.h"
//#include "use_spi.h"

void mcp_init(int desc)
{
	char val = MCP_RESET;
	uint8_t cfg1, cfg2, cfg3;
	
	write(desc, &val , 1);

	//modo de configuración
	mcp_writeRegister(desc, MCP_CANCTRL, MODE_CONFIG);

	//configurar velocidad
	cfg1 = MCP_CFG1 ;
	cfg2 = MCP_CFG2 ;
	cfg3 = MCP_CFG3 ;	
	mcp_writeRegister(MCP_CNF1, cfg1, desc);
	mcp_writeRegister(MCP_CNF2, cfg2, desc);
	mcp_writeRegister(MCP_CNF3, cfg3, desc);

	//modo de operaión normal
	mcp_writeRegister(MCP_CANCTRL, MODE_NORMAL, desc);
}

uint8_t mcp_readStatus(void){
	uint8_t i=0;
	//*****spi_write(MCP_READ_STATUS);
	//*******i = spi_read();
	return i;
}

uint8_t mcp_readRegister(uint8_t address)
{
	uint8_t i=0;	
	//spi_write(MCP_READ);
	//spi_write(address);
	//i = spi_read();	
	return i;
}

void mcp_readRegisterData(uint8_t address, uint8_t data[], uint8_t n)
{
	//uint8_t i;
	//*******spi_write(MCP_READ);
	//*******spi_write(address);

	//for (i=0; i<n; i++) {
		//data[i] = spi_read();
	//}

}
void mcp_writeRegister(int desc, uint8_t address, uint8_t data)
{
	char val = MCP_WRITE;		
	write(desc, &val, 1);
	write(desc, &address, 1);
	write(desc, &data, 1);

}

void mcp_writeRegisterData(int desc, uint8_t address, uint8_t data[], uint8_t n){
	uint8_t i;
	char val = MCP_WRITE;	
	write(desc, &val, 1);
	write(desc, &address, 1);
	for (i=0; i<n; i++) {
	  write(desc, &data[i], 1);
	}
}

void mcp_modifyRegister(int desc, uint8_t address, uint8_t mask, uint8_t data)
{
	char val = MCP_BITMOD;	
	write(desc, &val, 1);
	write(desc, &address, 1);
	write(desc, &mask, 1);
	write(desc, &data, 1);

}
void mcp_write_id(int desc, uint8_t mcp_addr,uint16_t can_id ){
  

	uint8_t tbufdata[4];
	
        tbufdata[MCP_SIDH] = (uint8_t) (can_id / 8 );
        tbufdata[MCP_SIDL] = (uint8_t) ((can_id & 0x07 )*32);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;

	mcp_writeRegisterData(desc, mcp_addr, tbufdata, 4 );
}


void mcp_write_canMsg(int desc, uint8_t sidh, CanMessage* msg){
      // Escribimos en los registros correspondientes el dato, el identificador y la longitud
	uint8_t dlc;
	dlc = msg->dlc;	
	mcp_writeRegisterData(desc, sidh+5,&(msg->data[0]), dlc );		//DATOS     TXBnSIDH+5= TXB0D0
	mcp_write_id(desc, sidh,msg->id);   					//IDENTIFICADOR
	mcp_writeRegister(desc, sidh+4,dlc );   				//LONGITUD     TXBnSIDH+4= TXBnDLC
}

void mcp_TXBuffer(uint8_t *buf_tx){

	uint8_t i, res;
	uint8_t ctrlregs[3] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };
	
	*buf_tx= 0x00;
	
	for (i=0; i<3; i++) {
		res = mcp_readRegister(ctrlregs[i]);
		if ( (res & MCP_TXB_TXREQ_M) == 0 ) {      //== MCP_TXBnCTRL(bit3)= 1(buffer pending tx), 0 (buffer not pending)
			*buf_tx = ctrlregs[i]+1;	   //TXBnCTRL+1=TXBnSIDH
		}
	}
	
}

void mcp_start_tx(int desc, uint8_t sidh){
	// Escribimos en el bit 3 de TXBnCTRL un 1
    mcp_modifyRegister(desc, sidh-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );  // TXBnCTRL = TXBnSIDH-1
	//Al poner el bit 3 de TXBnCTRL a 1--> sets this bit to request message be transmitted
}


void mcp_read_id(uint8_t mcp_addr,uint16_t* can_id )
{
    uint8_t tbufdata[4];	

	*can_id = 0;  
	mcp_readRegisterData(mcp_addr, tbufdata, 4 );  //RXBnSIDH
    	*can_id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

}

void mcp_read_canMsg(uint8_t sidh,CanMessage* msg){
	
	mcp_read_id(sidh, &(msg->id) );   			//LEE EL IDENTIFIER RXBnSIDH
	msg->dlc = mcp_readRegister(sidh+4 );  			//RXBnDLC       
	mcp_readRegisterData(sidh+5,&(msg->data[0]), msg->dlc );    //RXBnD0
}

