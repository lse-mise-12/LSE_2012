/********************************************************************************************/
/* SPI Driver for communicating TS7400 Platform with MCP2515 at Tigal board                 */
/********************************************************************************************/
/********************************************************************************************/
/* Note: SFRMOUT pin is unused for compatibility issues. Instead we are using DIO19 of GPIO */
/********************************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sched.h> //libreria de las interrupciones
#include <linux/slab.h> // gcc me dijo que malloc.h estaba obsoleta
//#include <linux/malloc.h> // gcc me dijo que malloc.h estaba obsoleta
#include <linux/mm.h> //flags de kmalloc()
#include <linux/signal.h> // ¿Necesaria?
#include <asm/io.h>
#include <asm/irq.h>  // ¿Necesaria?
#include <asm/hardware.h> //Hardware register related includes //Included in asm/io.h

#include "spi.h"
#include "mcp_registers.h"

MODULE_LICENSE("Dual BSD/GPL");

/* Functions: interface as files*/
int spi_open(struct inode *inode, struct file *filp);
int spi_release(struct inode *inode, struct file *filep);
ssize_t spi_read(struct file *filep, char *buf, size_t count, loff_t *f_pos);
ssize_t spi_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos);
unsigned int spi_poll(struct file *filp, poll_table *wait);
//static int fasync_spi(int fd, struct file *filp, int on);
void tigal_isr(int parametro1, void *parametro2, struct pt_regs *parametro3);
int read_mcp_register(unsigned char *buffer, unsigned char reg_addr);
int write_mcp_register(unsigned char *buffer, unsigned char reg_addr);
int read_mcp_rxstatus(unsigned char *buffer);
int read_mcp_rxbuffer(unsigned char *buffer, unsigned char buffer_no, unsigned char dlc);
void mcp_error_check(void);
char clear_interrupt(char interr);
//int read_mcp_data_id(unsigned char *id, unsigned char buffer_no);

/*Set functions as file interface (for OS)*/
struct file_operations spi_fops = {
	read: spi_read,
	write: spi_write,
	open: spi_open,
	release: spi_release,
	poll: spi_poll
};

typedef struct spi_pipe_ {
	wait_queue_head_t inq, outq;       
	char *buffer, *end;                /* begin of buf, end of buf */
	int buffersize;                    /* used in pointer arithmetic */
	char *rp, *wp;                     /* where to read, where to write */
	int nreaders, nwriters;            /* number of openings for r/w */
	struct fasync_struct *async_queue; /* asynchronous readers */
	struct semaphore sem;              /* mutual exclusion semaphore */
	//devfs_handle_t handle;
}spi_pipe;

/* Global variables */
volatile char *datos; // volatile keyword prevents of compiler optimizations. Use for accessing memory mapped region
unsigned char databuf[BUFFERSIZE];
unsigned int read_idx, write_idx; // Variables para la implementacion del buffer circular
spi_pipe *pdata;

int spi_major = 70;

unsigned int spi_poll(struct file *filp, poll_table *wait)
{
	spi_pipe *dev = (spi_pipe *)filp->private_data;
	printk("<1>SPI Driver: entering spi_poll\n");
	//pdata->inq = filp->private_data->inq;
//	printk("<1>SPI Driver: before init_waitqueue\n");
	init_waitqueue_head(&dev->inq);
//	printk("<1>SPI Driver: after init_waitqueue\n");
	unsigned int mask = 0;
	poll_wait(filp, &dev->inq, wait);
//	printk("<1>SPI Driver: after poll_wait\n");
	if(read_idx != write_idx){	
		printk("<1>SPI Driver: poll_wait continue\n");
		mask |= POLLIN | POLLRDNORM; // readable 
	}
//	printk("<1>SPI Driver: poll_wait mask %u\n", mask);
//	printk("<1>SPI Driver: poll_wait exit\n");
	return mask;
}
/********************/
/* SPI Driver Init */ 
/*******************/ 
int spi_init (void){
	int result;

	//Register type of device: character (or block, net)

	result = register_chrdev(spi_major, "spi_if", &spi_fops);
	if (result < 0){
		printk("<1>SPI Driver: Fail major number = %d\n", spi_major);
		return result;
	}else{
		printk("<1>SPI Driver: Good major number = %d\n", spi_major);		
	}

	//0-input  1-output
	outb(0xEF, REG_ADDR_PBDDR); //todos los bits como salida, menos el bit B4

	printk("<1>Driver SPI is up!\n");
	return result;
}

/**********************/
/* SPI Driver Release */
/**********************/
void spi_exit (void) {
	//free major number
	unregister_chrdev(spi_major, "spi_if");
	printk("<1>SPI Driver: Exit success!\n");
}
/*************************/
/* Open device (as file) */
/*************************/
int spi_open(struct inode *inode, struct file *filp) {
	
	char sspcr1_val_set = 0x10;
	char sspcr1_val_clear = 0x00;  
	short sspcr0_val = SSPCR0_DSS_8BIT;
	char sspcpsr_val = 0x004A;
	short status = 0x0000;
	volatile char *ptr;
	char cs_off = 0xFF;
	int irq_req_code;
	read_idx=write_idx=0;
	
	MOD_INC_USE_COUNT;
	printk("<1>Opening SPI interface...\n");

	// Gestion puerto de interrupciones B4 0-input  1-output
	//outb(0xEF, REG_ADDR_PBDDR); //todos los bits como salida, menos el bit B4

	// SPI Interface Init Procedure	: Set SSPCR1, SSPCR0, SSPCPSR, SSPSR, SSPIIR
	outb(sspcr1_val_set,SSPCR1); //SOD=0, MS=0, Set SSE (enable SPI), LBM=0, RORIE=0, TIE=0, RIE=0
	outw(sspcr0_val,SSPCR0); //SCR=0x00, SPH=0, SPO=0,FRF=00(Motorola SPI Mode),DSS = 0x7, 8-bit data frame
	outb(sspcpsr_val,SSPCPSR); //CPSDVSR=0x02 --> 7,4MHz CPSDVSR=0x0F --> Aprox 1MHz CPSDVSR=0x4A --> 200KHz
	outb(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	outb(sspcr1_val_set,SSPCR1); //Set SSE (enable SPI)
	
	// comprobacion de que se han escrito correctamente los valores en los registros
	status = inb(SSPCR1); //Read SPI Control Register 1
	printk("<1>SPI Control Register 1 = 0x%02X\n",status);	
	status = inw(SSPCR0); //Read SPI Control Register 0
	printk("<1>SPI Control Register 0 = 0x%04X\n",status);	
	status = inb(SSPCPSR); //Read SPI Clock Prescale Register
	printk("<1>SPI Clock Prescale Register = 0x%02X\n",status);
	status = inb(SSPSR); //Read SPI Status Register
	status &= 0x1F;
	printk("<1>SPI Status Register = 0x%02X\n",status);	
	status = inb(SSPIIR); //Read SPI Interrupt Identification Register
	status &= 0x03;
	printk("<1>SPI Interrupt Identification Register = 0x%02X\n",status);

	// Configuracion de interrupcion proveniente de placa TIGAL

	//Implement interrupt service routine(ISR)	//Register ISR
	irq_req_code=request_irq(GPIOINTR_VECTOR, tigal_isr, SA_INTERRUPT, "tigal_isr", NULL);
	if(irq_req_code==EINVAL){
		printk("SPI. The requested interrupt has a number out of range, or handler is a pointer to NULL\n");
		return irq_req_code;
	}
	else if(irq_req_code==EBUSY){
		printk("SPI. There exists a registered handler for the requested interrupt\n");
		return irq_req_code;
	}
	
	//1. Disable interrupt by writing to GPIO Interrupt Enable register.
	outb(0x00, GPIOBIntEn);
	//2. Set interrupt type by writing GPIOxINTTYPE1/2 register.
	outb(0x10, GPIOBIntType1); // 0- level sentitive 1-edge sentitive
	outb(0x00, GPIOBIntType2); //1-rising edge 0-faling edge
	//3. Clear interrupt by writing to GPIOxEOI register.
	outb(0x10, GPIOBEOI);
	//habilitamos el antirebotes del puerto B4
	//outb(0x10, GPIOBDB);
	//4. Enable interrupt by writing to GPIO Interrupt Enable register.
	outb(0x10, GPIOBIntEn);

	// Registros de DIRECCIONES
	// Comprobar disponibilidad:
	if(check_mem_region(DIO_DATA16_19, 1)) {
		printk("SPI. DUMB: espacio de memoria en uso: DIO_DIR815\n");
		return  -EBUSY;
	}

	// Tomar memoria
	request_mem_region(DIO_DATA16_19, 1, "dumb_driver"); // Del DIO16 al DIO19
	ptr = __ioremap(DIO_DATA16_19, 1, 0);
	printk("SPI: dumb: ptr remap = %p\n", ptr);

	// Definir como salida el 19. Salir:
	ptr[0] = 0xF0;
	release_mem_region(DIO_DATA16_19, 1);
	printk("SPI: Escrito 1 en reg direcciones de 16 a 19\n");

	// Registros de DATOS
	// Comprobar disponibilidad
	if(check_mem_region(DIO_DATA16_19, 1)) {
		printk("SPI: DUMB: espaco de memoria en uso: DIO_DATA16_19\n");
		return -EBUSY;
	}

	// Tomar memoria
	request_mem_region(DIO_DATA16_19, 1, "dumb_driver_16_19"); // Del DIO16 al DIO19
	datos = __ioremap(DIO_DATA16_19, 1, 0);
	
	writeb(cs_off,datos);

	pdata = kmalloc(sizeof(spi_pipe), GFP_ATOMIC); // Reservo memoria con el flag GPF_ATOMIC para su uso en interrupciones
	if(pdata == NULL) return -1;

	filp->private_data = (void *)pdata;
	
	return 0;
}


/***************************************************/
/* Interrupt service routine for PB4 pin on TS7400 */
/* connected to INT pin on tigal board (MCP 2515)  */
/***************************************************/
void tigal_isr(int parametro1, void *parametro2, struct pt_regs *parametro3) {
	unsigned char rxstatus,aux_rxstatus;
	int j;
	int awake;

	printk("<1> SPI entrada en tigal_isr\n");

	awake = 0;
	// Procesamos la interrupción
	// Hay que transferir a un buffer los datos leidos para que cuando el programa de usuario acceda pueda leerlo de forma segura
	// Comprobar que la interrupción se ha producido por recepción de datos
	if(read_mcp_rxstatus(&rxstatus)<0) {
		printk("<1>SPI: Error en funcion read_mcp_rxstatus()\n");
	} else {
		printk("<1>SPI: rxstatus = 0x%02X\n", rxstatus);		
	}
	aux_rxstatus = rxstatus >> 6;
	//printk("<1>SPI: aux_rxstatus = 0x%02X\n", aux_rxstatus);	
	switch(aux_rxstatus) {
		 case 0x00: // No RX message
			printk("<1>SPI: Interrupcion de MCP2515 sin nigun mensaje recibido. Paso a la función de tratamiento de errores\n");
			mcp_error_check();
			break;
		 case 0x01: // Message in RXB0
			if(read_mcp_rxbuffer(&databuf[write_idx], 0, CAN_DATA_LENGTH)) {
				write_idx+=CAN_DATA_LENGTH; 
				printk("<1>write_idx = %d\n", write_idx);
			}
			if(write_idx>BUFFERSIZE) {
				printk("<1>Desbordamiento de buffer circular");
				write_idx=0;
			}
			if(write_idx==BUFFERSIZE) {
				write_idx=0;
			}
			for(j=read_idx;j<read_idx+CAN_DATA_LENGTH;j++) {			
				printk("<1>SPI: Dato recibido en RXB0 = 0x%02X\n", databuf[j]);
			}
			awake = 1;
			break;
		 case 0x02: // Message in RXB1
			if(read_mcp_rxbuffer(&databuf[write_idx], 1, CAN_DATA_LENGTH)) {
				write_idx+=CAN_DATA_LENGTH; 
			}
			if(write_idx>BUFFERSIZE) {
				printk("<1>Desbordamiento de buffer circular");
				write_idx=0;
			}
			if(write_idx==BUFFERSIZE) {
				write_idx=0;
			}
			for(j=read_idx;j<read_idx+CAN_DATA_LENGTH;j++) {			
				printk("<1>SPI: Dato recibido en RXB1 = 0x%02X\n", databuf[j]);
			}
			awake = 1;
			break;
		 case 0x03: // Message in both RXB0 & RXB1
			if(read_mcp_rxbuffer(&databuf[write_idx], 0, CAN_DATA_LENGTH)) {
				write_idx+=CAN_DATA_LENGTH; 
			}
			if(write_idx>BUFFERSIZE) {
				printk("<1>Desbordamiento de buffer circular");
				write_idx=0;
			}
			if(write_idx==BUFFERSIZE) {
				write_idx=0;
			}
			for(j=read_idx;j<read_idx+CAN_DATA_LENGTH;j++) {			
				printk("<1>SPI: Dato recibido en RXB0 = 0x%02X\n", databuf[j]);
			}

			if(read_mcp_rxbuffer(&databuf[write_idx], 1, CAN_DATA_LENGTH)) {
				write_idx+=CAN_DATA_LENGTH; 
			}
			if(write_idx>BUFFERSIZE) {
				printk("<1>Desbordamiento de buffer circular");
				write_idx=0;
			}
			if(write_idx==BUFFERSIZE) {
				write_idx=0;
			}
			for(j=read_idx;j<read_idx+CAN_DATA_LENGTH;j++) {			
				printk("<1>SPI: Dato recibido en RXB1 = 0x%02X\n", databuf[j]);
			}
			awake = 1;
			break;
		default:
			printk("<1>SPI: Error valor inválido de aux_rxstatus = 0x%02X\n", aux_rxstatus);
			break;
	}
	// 3. Clear interrupt by writing to GPIOxEOI register.
	outb(0x10, GPIOBEOI);
	if(awake)
		wake_up_interruptible(&pdata->inq);
	
}

/**************************/
/* Close device (as file) */
/**************************/
int spi_release(struct inode *inode, struct file *filep) {
	int sspcr1_val_clear = 0x00000000;
	// Disable interrupt by writing to GPIO Interrupt Enable register.
	outb(0x00, GPIOBIntEn);
	// liberar la interrupcion
	//fasync_spi(-1,filep,0);
	free_irq(GPIOINTR_VECTOR, NULL);
	release_mem_region(DIO_DATA16_19, 1); // Cierro el espacio de memoria
	outl(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	
	if(pdata != NULL) kfree(pdata);
	
	MOD_DEC_USE_COUNT;
	printk("<1>SPI Driver: Closing\n");
	return 0;
}

/*****************************************/
/* Read from device (read from register) */
/*****************************************/
ssize_t spi_read(struct file *filep, char *buf, size_t count, loff_t *f_pos) {
	int i;
	char *pbuf;
	//printk("<1>Entrando en spi_read\n");
	pbuf=buf;
	//printk("<1>pbuf de spi_read() = %p\n", pbuf);
	if(count % CAN_DATA_LENGTH !=0) {
		printk("<1>Error! Se debe leer un multiplo de CAN_DATA_LENGTH bytes\n");
	}
	
	if(write_idx==read_idx)
	{
		printk("<1>No quedan datos por leer (wr_idx=rd_idx)\n");
		return 0;
	}
	
	for(i=0;i<count;i++) {
		*pbuf=databuf[read_idx];
		read_idx++;
		pbuf++;
		if(read_idx>=BUFFERSIZE) { read_idx=0; }
	}
	for(i=0;i<count;i++) {
		printk("<1>Dato[%d] spi_read = 0x%02X\n",i,*(buf+i));
	}

	return count;
}

int read_mcp_rxstatus(unsigned char *buffer)
{
	unsigned char rx_status = 0x00;
	unsigned char cs_on = 0xF0;
	unsigned char cs_off = 0xFF;

	//Activamos el Chip Select
	writeb(cs_on,datos);

	// 1. Enviamos el valor de RXSTATUS INSTRUCTION a la Tigal
	outw((short)MCP_RX_STATUS,SSPDR);
	inw(SSPDR); //Dummy Read to empty RX FIFO
	
	// Esperamos hasta que se haya enviado
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	

	// 2. Generamos los flancos de reloj para almacenar en el buffer de lectura el dato recibido
	outw(0x0000,SSPDR); // Envío dummy data para generar flancos de reloj

	// Esperamos hasta que se hayan generado los flancos
	inw(SSPDR); //Dummy Read to empty RX FIFO
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	
	// Desactivamos el Chip Select
	writeb(cs_off,datos);
	
	*buffer = inb(SSPDR); //Read SPI Data Register	
	printk("<1>SPI RXStatus data = 0x%02X\n",*buffer);

	return 1;	
}

int read_mcp_rxbuffer(unsigned char *buffer, unsigned char buffer_no, unsigned char dlc)
{
	unsigned char rx_status = 0x00;
	unsigned char cs_on = 0xF0;
	unsigned char cs_off = 0xFF;
	unsigned char address;
	int i;

	if(buffer_no == 0) {
		address=MCP_READ_RX|0x02; // Formamos la instrucción a enviar al MCP --> puntero en RXB0D0
	} else {
		if(buffer_no == 1) {
			address=MCP_READ_RX|0x06; // Formamos la instrucción a enviar al MCP --> puntero en RXB1D0
		} else {
			printk("<1>SPI Error: numero de buffer de RX incorrecto\n");
			return -1;
		}
	}

	//Activamos el Chip Select
	writeb(cs_on,datos);

	// 1. Enviamos el valor de READRXBUFFER a la Tigal
	outw((short)address,SSPDR);
	inw(SSPDR); //Dummy Read to empty RX FIFO

	// Esperamos hasta que se haya enviado
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}
	// 2. Bucle para leer los bytes de datos indicados por el registro dlc
	for(i=0;i<dlc;i++) {
		// Generamos los flancos de reloj para almacenar en el buffer de lectura el dato recibido
		outw(0x0000,SSPDR); // Envío dummy data para generar flancos de reloj

		// Esperamos hasta que se hayan generado los flancos
		inw(SSPDR); //Dummy Read to empty RX FIFO
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;
		while(rx_status&SSPSR_BSY){
			rx_status = inb(SSPSR); //Read SPI Status Register
			rx_status &= 0x1F;			
		}
		// Leemos el dato
		*buffer = inb(SSPDR); //Read SPI Data Register
		printk("<1>SPI data read = 0x%02X\n",*buffer);
		buffer++;	
	}

	// Desactivamos el Chip Select
	writeb(cs_off,datos);

	printk("<1>Lectura de datos del RXBnD[0..7] OK\n");

	return dlc;	

}

/************************************************************************/
/* Function that reads a data from a specified register at tigal board  */
/************************************************************************/
int read_mcp_register(unsigned char *buffer, unsigned char reg_addr)
{
	unsigned char rx_status = 0x00;
	unsigned char cs_on = 0xF0;
	unsigned char cs_off = 0xFF;

	//Activamos el Chip Select
	writeb(cs_on,datos);

	// 1. Enviamos el valor de READINSTRUCTION a la Tigal
	outw((short)MCP_READ,SSPDR);
	inw(SSPDR); //Dummy Read to empty RX FIFO
	
	// Esperamos hasta que se haya enviado
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	

	// 2. Enviamos la dirección del registro del que queremos leer
	outw((short)reg_addr,SSPDR); //1-byte register
	inw(SSPDR); //Dummy Read to empty RX FIFO

	// Esperamos hasta que se haya enviado
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}
	
	// 3. Generamos los flancos de reloj para almacenar en el buffer de lectura el dato recibido
	outw(0x0000,SSPDR); // Envío dummy data para generar flancos de reloj

	// Esperamos hasta que se hayan generado los flancos
	inw(SSPDR); //Dummy Read to empty RX FIFO
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	
	// Desactivamos el Chip Select
	writeb(cs_off,datos);
	
	*buffer = inb(SSPDR); //Read SPI Data Register	
	printk("<1>SPI data read = 0x%02X\n",*buffer);

	return 1;	
}


/***************************************/
/* Write to device (write to register) */
/***************************************/
ssize_t spi_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos) {

	int bytes_written = 0;
	int i = 0;
	char array[RXTX_FIFOSIZE];
	char tx_status;
	char cs_on = 0xF0;
	char cs_off = 0xFF;
	
	if(count > CAN_DATA_LENGTH+2){
		printk("<1>SPI: write function only permits %d elements\n Count contents = 0x%02X\n",CAN_DATA_LENGTH+2, count);
		return(-1);
	}

	writeb(cs_on,datos);

	for(i=0;i<count;i++){
		array[i] = inb(SSPSR); //Read SPI Status Register
		array[i] &= 0x1F;
		if((array[i]&SSPSR_TNF)==0){ // Transmit FIFO is full
			printk("<1>SPI: Transmit FIFO is full. %d bytes written\n",bytes_written);
			return(0);
		}
		else {
			outw((short)*buf,SSPDR); //1-byte register
			inw(SSPDR); //Dummy Read to empty RX FIFO
			tx_status = inb(SSPSR); //Read SPI Status Register
			tx_status &= 0x1F;
			//printk("<1>SPI: BusyBit? 0x=%02X\n", tx_status&SSPSR_BSY);			
			while(tx_status&SSPSR_BSY){
				tx_status = inb(SSPSR); //Read SPI Status Register
				tx_status &= 0x1F;			
			}
			printk("<1> SPI:[%d] %02X\n",bytes_written, *buf);
			bytes_written++;
			buf++;
		}
	}

	writeb(cs_off,datos);
	
	/*for(i=0;i<count;i++){
		printk("<1>SPI Status Register at Write no.%d = 0x%02X\n",i,array[i]);
	}*/
	
	//printk("<1>SPI: no of bytes written = %d\n",bytes_written);

	return bytes_written;
}

int write_mcp_register(unsigned char *buffer, unsigned char reg_addr) {

	int i = 0;
	char tx_status;
	char cs_on = 0xF0;
	char cs_off = 0xFF;
	char buf[3];
	buf[0]= MCP_WRITE;
	buf[1]= reg_addr;
	buf[2]= *buffer;

	writeb(cs_on,datos);

	for(i=0;i<3;i++){
		tx_status = inb(SSPSR); //Read SPI Status Register
		tx_status &= 0x1F;
		if((tx_status&SSPSR_TNF)==0){ // Transmit FIFO is full
			printk("<1>SPI: Transmit FIFO is full\n");
			return(-1);
		}
		else {
			outw((short)buf[i],SSPDR); //1-byte register
			inw(SSPDR); //Dummy Read to empty RX FIFO
			tx_status = inb(SSPSR); //Read SPI Status Register
			tx_status &= 0x1F;
			//printk("<1>SPI: BusyBit? 0x=%02X\n", tx_status&SSPSR_BSY);			
			while(tx_status&SSPSR_BSY){
				tx_status = inb(SSPSR); //Read SPI Status Register
				tx_status &= 0x1F;			
			}
		}
	}

	writeb(cs_off,datos);

	printk("<1>SPI: Data send to Tigal:\n");
	for(i=0;i<3;i++) {
		printk("<1>buf[%d] = 0x%02X\n",i, buf[i]);
	}

	return 3;
}

void mcp_error_check(void)
{
	unsigned char icod, err_buf;
	printk("<1>WARNING! mcp_error_check()\n");
	// tratamiento de la interrupcion TIGAL
	// Enviamos un mensaje al TIGAL para comprobar la fuente de la interrupción
	read_mcp_register(&icod, MCP_CANSTAT);
	printk("<1>Registro CANSTAT %02X\n",icod);	
	read_mcp_register(&err_buf, MCP_EFLG);
	printk("<1>Registro EFLG %02X\n",err_buf);
	switch(err_buf) {
		case 0x01: //TX OR RX WARNING (TEC o REC superior a 96)
		printk("<1>TX OR RX WARNING (TEC or REC is equal or greater than 96)\n");
		case 0x02: //RX WARNING
		printk("<1>RX WARNING Set when REC is equal to or greater than 96\n");					
		case 0x04: //TX WARNING
		printk("<1>TX WARNING TEC is equal to or greater than 96\n");				
		case 0x08: //RX ERROR-PASSIVE
		printk("<1>RX WARNING REC is equal to or greater than 128\n");				
		case 0x10: //TX ERROR-PASSIVE
		printk("<1>TX WARNING TEC is equal to or greater than 128\n");				
		case 0x20: //TX BUS-OFF
		printk("<1>Bit set when TEC reaches 255\n");				
		case 0x40: //RX0 Overflow
		printk("<1>RX0 Overflow: Set when a valid message is received for RXB0 and CANINTF.RX0IF = 1\n");				
		case 0x80: //RX1 Overflow
		printk("<1>RX1 Overflow: Set when a valid message is received for RXB1 and CANINTF.RX1IF = 1\n");
	}				
	clear_interrupt(INT_MERR);
	clear_interrupt(INT_ERR);
}

// Limpia interrupciones del MCP2515 en Tigal Board
char clear_interrupt(char interr)	{
	char aux;
	read_mcp_register(&aux, MCP_CANINTF);
	interr=~interr; // hacemos el complemento de interr que tiene a '1' el bit que nos interesa, para ponerlo a 0
	aux&=interr; // hacemos la máscara
	write_mcp_register(&aux, MCP_CANINTF);
	return 1;
}

/*int read_mcp_data_id(unsigned char *id, unsigned char buffer_no)
{
	unsigned char rx_status = 0x00;
	unsigned char cs_on = 0xF0;
	unsigned char cs_off = 0xFF;
	unsigned char address;
	int i;

	if(buffer_no == 0) {
		address=MCP_READ_RX; // Formamos la instrucción a enviar al MCP --> puntero en RXB0SIDH
	} else {
		if(buffer_no == 1) {
			address=MCP_READ_RX|0x04; // Formamos la instrucción a enviar al MCP --> puntero en RXB1SIDH
		} else {
			printk("<1>SPI Error: numero de buffer de RX incorrecto\n");
			return -1;
		}
	}

	//Activamos el Chip Select
	writeb(cs_on,datos);

	// 1. Enviamos el valor de READRXBUFFER a la Tigal
	outw((short)address,SSPDR);
	inw(SSPDR); //Dummy Read to empty RX FIFO

	// Esperamos hasta que se haya enviado
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}
	// 2. Bucle para leer los bytes de SIDH y SIDL del dato recibido
	for(i=0;i<2;i++) {
		// Generamos los flancos de reloj para almacenar en el buffer de lectura el dato recibido
		outw(0x0000,SSPDR); // Envío dummy data para generar flancos de reloj

		// Esperamos hasta que se hayan generado los flancos
		inw(SSPDR); //Dummy Read to empty RX FIFO
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;
		while(rx_status&SSPSR_BSY){
			rx_status = inb(SSPSR); //Read SPI Status Register
			rx_status &= 0x1F;			
		}
		// Leemos el dato
		*id = inb(SSPDR); //Read SPI Data Register
		printk("<1>SPI data read SIDX= 0x%02X\n",*id);
		id++;

	}
	// Desactivamos el Chip Select
	writeb(cs_off,datos);

	printk("<1>Lectura de datos del RXBnSID[H..L] OK\n");

	return 1;	
}
*/

module_init(spi_init);
module_exit(spi_exit);
