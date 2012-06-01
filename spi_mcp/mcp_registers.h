


#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3

//**CNF registers

#define MCP_CFG1 (MCP_SJW | MCP_BRP)
#define MCP_CFG2 (MCP_BLTMODE | MCP_SAM | MCP_PHSEG1 | MCP_PRSEG)
#define MCP_CFG3 (MCP_SOF | MCP_WAKFIL | MCP_PHSEG)

//CNF1
#define MCP_SJW			0x00		//SJW= 1xTq
#define MCP_BRP			0X01		// Prescaler = (BRP+1)*2  --Prescaler=4 -> BRP=1
//CNF2
#define MCP_BLTMODE		0x80		//bit 7=1
#define MCP_SAM			0x00		// 1 sample, bit 6=0
#define MCP_PHSEG1		(1<<3)		//PHSEG1 = PS1 (2) - 1 =1, en el bit 3
#define MCP_PRSEG		0X01		//PRSEG=PRSEQTQ(2)-1
//CNF3
#define MCP_PHSEG		0X02		//PHSEG2 = PS2(3)-1=2
#define MCP_SOF			0x00
#define MCP_WAKFIL		0x00

//*****************************//

//**TXBnCTRL registers.

#define MCP_TXB_TXBUFE_M    0x80
#define MCP_TXB_ABTF_M      0x40
#define MCP_TXB_MLOA_M      0x20
#define MCP_TXB_TXERR_M     0x10
#define MCP_TXB_TXREQ_M     0x08
#define MCP_TXB_TXIE_M      0x04
#define MCP_TXB_TXP10_M     0x03

#define MCP_TXB_RTR_M       0x40    // In TXBnDLC
#define MCP_RXB_IDE_M       0x08    // In RXBnSIDL
#define MCP_RXB_RTR_M       0x40    // In RXBnDLC

#define MCP_STAT_RXIF_MASK   (0x03)
#define MCP_STAT_RX0IF (1<<0)
#define MCP_STAT_RX1IF (1<<1)

#define MCP_EFLG_RX1OVR (1<<7)
#define MCP_EFLG_RX0OVR (1<<6)
#define MCP_EFLG_TXBO   (1<<5)
#define MCP_EFLG_TXEP   (1<<4)
#define MCP_EFLG_RXEP   (1<<3)
#define MCP_EFLG_TXWAR  (1<<2)
#define MCP_EFLG_RXWAR  (1<<1)
#define MCP_EFLG_EWARN  (1<<0)
#define MCP_EFLG_ERRORMASK  (0xF8) /* 5 MS-Bits */



// Registros MCP2515

#define MCP_RXF0SIDH	0x00
#define MCP_RXF0SIDL	0x01
#define MCP_RXF0EID8	0x02
#define MCP_RXF0EID0	0x03
#define MCP_RXF1SIDH	0x04
#define MCP_RXF1SIDL	0x05
#define MCP_RXF1EID8	0x06
#define MCP_RXF1EID0	0x07
#define MCP_RXF2SIDH	0x08
#define MCP_RXF2SIDL	0x09
#define MCP_RXF2EID8	0x0A
#define MCP_RXF2EID0	0x0B
#define MCP_CANSTAT	0x0E
#define MCP_CANCTRL	0x0F
#define MCP_RXF3SIDH	0x10
#define MCP_RXF3SIDL	0x11
#define MCP_RXF3EID8	0x12
#define MCP_RXF3EID0	0x13
#define MCP_RXF4SIDH	0x14
#define MCP_RXF4SIDL	0x15
#define MCP_RXF4EID8	0x16
#define MCP_RXF4EID0	0x17
#define MCP_RXF5SIDH	0x18
#define MCP_RXF5SIDL	0x19
#define MCP_RXF5EID8	0x1A
#define MCP_RXF5EID0	0x1B
#define MCP_TEC		0x1C
#define MCP_REC		0x1D
#define MCP_RXM0SIDH	0x20
#define MCP_RXM0SIDL	0x21
#define MCP_RXM0EID8	0x22
#define MCP_RXM0EID0	0x23
#define MCP_RXM1SIDH	0x24
#define MCP_RXM1SIDL	0x25
#define MCP_RXM1EID8	0x26
#define MCP_RXM1EID0	0x27
#define MCP_CNF3	0x28
#define MCP_CNF2	0x29
#define MCP_CNF1	0x2A
#define MCP_CANINTE	0x2B
#define MCP_CANINTF	0x2C
#define MCP_EFLG	0x2D
#define MCP_TXB0CTRL	0x30
#define MCP_TXB1CTRL	0x40
#define MCP_TXB2CTRL	0x50
#define MCP_RXB0CTRL	0x60
#define MCP_RXB0SIDH	0x61
#define MCP_RXB1CTRL	0x70
#define MCP_RXB1SIDH	0x71

#define INT_RX0	0x01
#define INT_RX1	0x02
#define INT_TX0	0x04
#define INT_TX1	0x08
#define INT_TX2	0x10
#define INT_ERR	0x20
#define INT_WAK	0x40
#define INT_MERR	0x80
#define INT_ALL	0xFF


#define MCP_TX_INT		0x1C		// Enable all transmit interrupts
#define MCP_TX01_INT		0x0C		// Enable TXB0 and TXB1 interrupts
#define MCP_RX_INT		0x03		// Enable receive interrupts
#define MCP_RXB0_INT		0x01		// Enable RXB0 receive interrupt
#define MCP_NO_INT		0x00		// Disable all interrupts

#define MCP_TX01_MASK		0x14
#define MCP_TX_MASK		0x54

// Instrucciones SPI

#define MCP_WRITE		0x02
#define MCP_READ		0x03
#define MCP_BITMOD		0x05
#define MCP_LOAD_TX0		0x40
#define MCP_LOAD_TX1		0x42
#define MCP_LOAD_TX2		0x44
#define MCP_RTS_TX0		0x81
#define MCP_RTS_TX1		0x82
#define MCP_RTS_TX2		0x84
#define MCP_RTS_ALL		0x87
#define MCP_READ_RX0		0x90
#define MCP_READ_RX1		0x94
#define MCP_READ_STATUS		0xA0
#define MCP_RX_STATUS		0xB0
#define MCP_RESET		0xC0


// Registro CANCTRL

#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP	0xE0
#define MODE_MASK	0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT	0x08
#define CLKOUT_ENABLE	0x04
#define CLKOUT_DISABLE	0x00
#define CLKOUT_PS1	0x00
#define CLKOUT_PS2	0x01
#define CLKOUT_PS4	0x02
#define CLKOUT_PS8	0x03


// Registo CANINTF 

#define MCP_RX0IF		0x01
#define MCP_RX1IF		0x02
#define MCP_TX0IF		0x04
#define MCP_TX1IF		0x08
#define MCP_TX2IF		0x10
#define MCP_ERRIF		0x20
#define MCP_WAKIF		0x40
#define MCP_MERRF		0x80

