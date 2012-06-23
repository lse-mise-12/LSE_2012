
#ifndef _MCP_H
#define _MCP_H

#include "mcp_registers.h"
#include "spi.h"
#include <inttypes.h>  //para poder declarar los uint

typedef struct{
	uint16_t id; 
	uint8_t  dlc;
	uint8_t  data[CAN_DATA_LENGTH];
	uint8_t proto_version;
	uint8_t picID;
	uint8_t type;
	uint8_t secNum;
	uint8_t variable;
	uint8_t valM;
	uint8_t valL;
	uint8_t crc;
}CanMessage;

char clear_interrupt(char interr, int desc);
void tigal_handler(int signum);
void can_send(int desc, CanMessage* can_msg, uint8_t buffer_dest);
void can_receive(int desc, CanMessage*);
void mcp_init(int desc);
uint8_t mcp_readStatus(int desc);
void mcp_writeRegisterData(int desc, uint8_t, uint8_t[], uint8_t);
void mcp_readRegisterData(int desc, uint8_t, uint8_t[], uint8_t);
void mcp_writeRegister(int desc, uint8_t, uint8_t);
uint8_t mcp_readRegister(int desc, uint8_t address);
void mcp_write_id(int desc, uint8_t,uint16_t);
void mcp_read_id(int desc, uint8_t, uint16_t*);
void mcp_write_canMsg(int desc, uint8_t ,CanMessage*);
void mcp_read_canMsg(int desc, uint8_t ,CanMessage*);
void mcp_start_tx(int desc, uint8_t );
void mcp_TXBuffer(int desc, uint8_t*);
void mcp_modifyRegister(int desc, uint8_t address, uint8_t mask, uint8_t data);
void mcp_set_mask(int desc, uint8_t address, uint16_t mask_id);
void mcp_set_filter(int desc, uint8_t address, uint16_t filter_id);
#endif
