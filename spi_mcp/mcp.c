#include <stdio.h>
#include <unistd.h> //libreria del sleep
#include <fcntl.h>
#include <errno.h>
#include "mcp.h"
#include "mcp_registers.h"
#include "spi.h"

void mcp_init(int desc)
{
	char val = MCP_RESET;
	uint8_t cfg1, cfg2, cfg3;
	
	write(desc, &val , 1);

	printf("mcp_init: desc %d\n", desc);
	//configurar velocidad
	cfg1 = MCP_CFG1 ;
	cfg2 = MCP_CFG2 ;
	cfg3 = MCP_CFG3 ;	
	mcp_writeRegister(desc, MCP_CNF1, cfg1);
	mcp_writeRegister(desc, MCP_CNF2, cfg2);
	mcp_writeRegister(desc, MCP_CNF3, cfg3);

	//Activamos las interrupciones
	printf("Activacion interrupciones: desc %d, address %x, flags %d\n", desc, MCP_CANINTE, INT_RX0|INT_RX1|INT_ERR);	
	mcp_writeRegister(desc, MCP_CANINTE, INT_RX0|INT_RX1|INT_ERR); //|INT_MERR); //INT_TX0 INT_TX1 INT_TX2 INT_WAK
	printf("Activacion interrupciones OK\n");

	//modo de operacion LOOPBACK para recibir los mensajes que transmitimos
	//mcp_writeRegister(desc, MCP_CANCTRL, MODE_LOOPBACK);
	//printf("Modo de operacion LOOPBACK ON\n");

	//modo de operacion normal
	mcp_writeRegister(desc, MCP_CANCTRL, MODE_NORMAL);
	printf("Modo de operacion NORMAL ON\n");
}

// Limpia interrupciones del MCP2515 en Tigal Board
char clear_interrupt(char interr, int desc)	{
	char aux;
	aux = mcp_readRegister(desc, MCP_CANINTF);
	interr=~interr; // hacemos el complemento de interr que tiene a '1' el bit que nos interesa, para ponerlo a 0
	aux&=interr; // hacemos la máscara
	mcp_writeRegister(desc, MCP_CANINTF, aux);	// limpiamos el flag de interrupcion 'interr' en el registro del MCP
	return 1;
}

void can_send(int desc, CanMessage* can_msg, uint8_t buffer_dest){
	//uint8_t txbuf;
	//mcp_TXBuffer(&txbuf);
	//fprintf(stderr,"#");
	if(buffer_dest==0) {
		mcp_write_canMsg(desc, MCP_TXB0SIDH, can_msg); //En buffer_dest se debe pasar como argumento TXB0SIDH, TXB1SIDH o TXB2SIDH
	} else {
		if(buffer_dest==1) {
			mcp_write_canMsg(desc, MCP_TXB1SIDH, can_msg); //En buffer_dest se debe pasar como argumento TXB0SIDH, TXB1SIDH o TXB2SIDH
		} else { //buffer_dest==2 u otro valor
			mcp_write_canMsg(desc, MCP_TXB2SIDH, can_msg); //En buffer_dest se debe pasar como argumento TXB0SIDH, TXB1SIDH o TXB2SIDH	
		}
	} 
	mcp_start_tx(desc, buffer_dest);
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

/*Cambiar*/
uint8_t mcp_readStatus(int desc)
{
	uint8_t i[2];
	i[0]=MCP_READ_STATUS;
	read(desc, i, 2);
	return i[1];
}

void mcp_writeRegister(int desc, uint8_t address, uint8_t data)
{
	char val[3]; //= MCP_WRITE;
	val[0] = MCP_WRITE;
	val[1] = address;
	val[2] = data;
	write(desc, val, 3);
}

void mcp_writeRegisterData(int desc, uint8_t address, uint8_t data[], uint8_t n) {
	uint8_t i;
	char val[CAN_DATA_LENGTH+2];	
	val[0] = MCP_WRITE;
	val[1] = address;
	for(i=0;i<n;i++) {
		val[i+2]=*(data + i);
		//fprintf(stderr,"data[%d]=%02X\n",i,data[i]);
		//data++;
	}
	i=write(desc,val,n+2);
	if(i!=(n+2)) {
		fprintf(stderr, "Se han escrito %d bytes de los %d bytes disponibles", i, n+2);
	}
	/*for (i=0; i<n; i++) {
		val[1] = address+i;
		val[2] = *data;

		write(desc, val, 3);
		data++;
	}*/
}

void mcp_write_id(int desc, uint8_t mcp_addr,uint16_t can_id ){
	uint8_t tbufdata[4];
    
    tbufdata[MCP_SIDH] = (uint8_t) (can_id / 8 );
    tbufdata[MCP_SIDL] = (uint8_t) ((can_id & 0x07 )*32);
    tbufdata[MCP_EID0] = 0;
    tbufdata[MCP_EID8] = 0;
	mcp_writeRegisterData(desc, mcp_addr, tbufdata, 4 );
}

void mcp_write_canMsg(int desc, uint8_t sidh, CanMessage* msg) // En sidh se debe pasar TXB0SIDH, TXB1SIDH o TXB2SIDH
{
    // Escribimos en los registros correspondientes el dato, el identificador y la longitud
	uint8_t dlc;
	dlc = msg->dlc;
	mcp_writeRegisterData(desc, sidh+5, msg->data, dlc);	//DATOS     TXBnSIDH+5 = TXB0D0
	//mcp_writeRegister(desc, sidh+5, msg->data[0]);  // Esta es la que habia antes
	mcp_write_id(desc, sidh, msg->id);					//IDENTIFICADOR TXBnSIDH
	mcp_writeRegister(desc, sidh+4, dlc );				//LONGITUD     	TXBnSIDH+4 = TXBnDLC
	/*for(i=0;i<7;i++) {
		printf("Dato enviado data[%d]= %02X\n",i , msg->data[i]);
	}*/
}

/*Cambiar*/uint8_t mcp_readRegister(int desc, uint8_t address)
{
	uint8_t val[3];
	val[0] = MCP_READ;
	val[1] = address;
	read(desc, val, 3);
	return val[2];
}

// Esta funcion lee, del MCP2515, n registros en direcciones consecutivas
/*Cambiar*/void mcp_readRegisterData(int desc, uint8_t address, uint8_t data[], uint8_t n)
{
	uint8_t i, val[3];
	val[0] = MCP_READ;
	for (i=0; i<n; i++) {
		val[1] = address+i;
		read(desc, val, 3);
		data[i]=val[2];
	}
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

/*Cambiar*/void mcp_TXBuffer(int desc, uint8_t *buf_tx){
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

void mcp_start_tx(int desc, uint8_t buffer_dest){
	// Escribimos en el bit 3 de TXBnCTRL un 1
	if(buffer_dest==0) {
	    mcp_modifyRegister(desc, MCP_TXB0CTRL, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );  // TXBnCTRL = TXBnSIDH-1
	} else {
		if(buffer_dest==1) {
		    mcp_modifyRegister(desc, MCP_TXB1CTRL, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );  // TXBnCTRL = TXBnSIDH-1
		} else { //buffer_dest==2 u otro valor
		    mcp_modifyRegister(desc, MCP_TXB2CTRL, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );  // TXBnCTRL = TXBnSIDH-1
		}
	} 
   //mcp_modifyRegister(desc, buffer_dest-6 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );  // TXBnCTRL = TXBnSIDH-1
	//Al poner el bit 3 de TXBnCTRL a 1--> sets this bit to request message be transmitted
}

// Se debe pasar un address con los valores siguientes: 	
// Buffer RXB0 --> MCP_RXM0SIDH
// Buffer RXB1 --> MCP_RXM1SIDH

/*Cambiar*/void mcp_read_id(int desc, uint8_t mcp_addr,uint16_t* can_id )
{
    uint8_t tbufdata[4];	
	*can_id = 0;  
	mcp_readRegisterData(desc, mcp_addr, tbufdata, 4 );  //RXBnSIDH
   	*can_id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);
}


/*Cambiar*/void mcp_read_canMsg(int desc, uint8_t sidh, CanMessage* msg) // En sidh se debe pasar RXB0SIDH o RXB1SIDH
{
	mcp_read_id(desc, sidh, &(msg->id) );	//LEE EL IDENTIFIER RXBnSIDH
	msg->dlc = mcp_readRegister(desc, sidh+4 );		//RXBnDLC       
	mcp_readRegisterData(desc, sidh+5, msg->data, msg->dlc );	//RXBnD0
}

void mcp_set_mask(int desc, uint8_t address, uint16_t mask_id)
{
	mcp_write_id(desc, address, mask_id);
}

// Se debe pasar un address con los valores siguientes: 	
// Buffer RXB0 --> MCP_RXFnSIDH con n= 0 o 1
// Buffer RXB1 --> MCP_RXFnSIDH con n= 2, 3, 4 o 5
void mcp_set_filter(int desc, uint8_t address, uint16_t filter_id)
{
	mcp_write_id(desc, address, filter_id);
}
