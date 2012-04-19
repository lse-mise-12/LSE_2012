
//#include "spi.h"
#include "mcp_registers.h"
#include <inttypes.h>  //para poder declarar los uint
#define CAN_MAX (1)


typedef struct{
	uint16_t id; 
	uint8_t  dlc;
	uint8_t  data[CAN_MAX];
} CanMessage;

void can_send(int desc, CanMessage*);

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

