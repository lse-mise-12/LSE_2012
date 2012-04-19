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
	
	//configurar velocidad
	cfg1 = MCP_CFG1 ;
	cfg2 = MCP_CFG2 ;
	cfg3 = MCP_CFG3 ;	
	mcp_writeRegister(desc, MCP_CNF1, cfg1);
	mcp_writeRegister(desc, MCP_CNF2, cfg2);
	mcp_writeRegister(desc, MCP_CNF3, cfg3);

	//modo de operacion LOOPBACK para recibir los mensajes que transmitimos
	//mcp_writeRegister(desc, MCP_CANCTRL, MODE_LOOPBACK);

	//modo de operacion normal
	mcp_writeRegister(desc, MCP_CANCTRL, MODE_NORMAL);
}

uint8_t mcp_readStatus(int desc){
	uint8_t i[2];
	i[0]=MCP_READ_STATUS;
	read(desc, i, 2);
	return i[1];
}

uint8_t mcp_readRegister(int desc, uint8_t address)
{
	//int i;
	char val[3];
	//char dat[8];
	val[0] = MCP_READ;
	val[1] = address;
//	write(desc, val, 2);
	read(desc, val, 3);
	//for(i=0;i<8;i++) printf("Dato leido %d\n",dat[i]);	
	return val[2];
}

void mcp_readRegisterData(int desc, uint8_t address, uint8_t data[], uint8_t n)
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
	char val[3]; //= MCP_WRITE;
	val[0] = MCP_WRITE;
	val[1] = address;
	val[2] = data;
	write(desc, val, 3);
	//write(desc, &address, 1);
	//write(desc, &data, 1);

}

void mcp_writeRegisterData(int desc, uint8_t address, uint8_t data[], uint8_t n){
	uint8_t i;
	char val[8];	
	val[0] = MCP_WRITE;
	val[1] = address;
	//write(desc, val, 2);
	//write(desc, &address, 1);
	for (i=0; i<n; i++) {
	val[i+2]=*data;
	data++;
	}
	write(desc, val, n+2);
}

void mcp_modifyRegister(int desc, uint8_t address, uint8_t mask, uint8_t data)
{
	char val[4];// = MCP_BITMOD;	
	val[0] = MCP_BITMOD;
	val[1] = address;
	val[2] = mask;
	val[3] = data;
	write(desc, val, 4);
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
	//mcp_writeRegisterData(desc, sidh+5,&(msg->data[0]), dlc );		//DATOS     TXBnSIDH+5= TXB0D0
	mcp_writeRegister(desc, sidh+5,msg->data[0]);  // Esta la hemos sustituido por la anterior
	mcp_write_id(desc, sidh,msg->id);   					//IDENTIFICADOR
	mcp_writeRegister(desc, sidh+4,dlc );   				//LONGITUD     TXBnSIDH+4= TXBnDLC
}

void mcp_TXBuffer(int desc, uint8_t *buf_tx){

	uint8_t i, res;
	uint8_t ctrlregs[3] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };
	
	*buf_tx= 0x00;
	
	for (i=0; i<3; i++) {
		res = mcp_readRegister(desc, ctrlregs[i]);
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


void mcp_read_id(int desc, uint8_t mcp_addr,uint16_t* can_id )
{
    uint8_t tbufdata[4];	

	*can_id = 0;  
	mcp_readRegisterData(desc, mcp_addr, tbufdata, 4 );  //RXBnSIDH
    	*can_id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

}

void mcp_read_canMsg(int desc, uint8_t sidh,CanMessage* msg){
	
	mcp_read_id(desc, sidh, &(msg->id) );   			//LEE EL IDENTIFIER RXBnSIDH
	msg->dlc = mcp_readRegister(desc, sidh+4 );  			//RXBnDLC       
	mcp_readRegisterData(desc, sidh+5,&(msg->data[0]), msg->dlc );    //RXBnD0
}

