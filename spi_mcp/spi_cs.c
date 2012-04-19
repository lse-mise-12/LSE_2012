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
#include <asm/io.h>

//Hardware register related includes
#include <asm/hardware.h> //Included in asm/io.h

#define DIO_DATA16_19 0x12000001UL	// Registros de datos de DIO16 a DIO19
#define RXTX_FIFOSIZE 8 // Las FIFO de RX y TX tienen 8 entradas de memoria (tamaño max por entrada 16 bits)

//These define's already included at compilation time --> asm/hardware.h y asm/io.h
//#define SSPCR0 0x808A0000 	// Control Register 0
//#define SSPCR1 0x808A0004 	// Control Register 1
//#define SSPDR 0x808A0008  	// Receive FIFO / Transmit FIFO Data Register
//#define SSPSR 0x808A000C	// Status Register
//#define SSPCPSR 0x808A0010  	// Clock prescale Register
//#define SSPIIR 0x808A0014	// Interrupt identification Register/Interrupt Clear Register
//#define IRQ_SSP 	53

MODULE_LICENSE("Dual BSD/GPL");

volatile char *datos; // volatile keyword prevents of compiler optimizations. Use for accessing memory mapped region

/* Functions: interface as files*/
int spi_open(struct inode *inode, struct file *filp);
int spi_release(struct inode *inode, struct file *filep);
ssize_t spi_read(struct file *filep, char *buf, size_t count, loff_t *f_pos);
ssize_t spi_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos);

/*Set functions as file interface (for OS)*/
struct file_operations spi_fops = {
	read: spi_read,
	write: spi_write,
	open: spi_open,
	release: spi_release
};

/* Global variables */
int spi_major = 70;

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
	
	MOD_INC_USE_COUNT;
	printk("<1>Opening SPI interface...\n");

	// SPI Interface Init Procedure	: Set SSPCR1, SSPCR0, SSPCPSR, SSPSR, SSPIIR
	outb(sspcr1_val_set,SSPCR1); //SOD=0, MS=0, Set SSE (enable SPI), LBM=0, RORIE=0, TIE=0, RIE=0
	outw(sspcr0_val,SSPCR0); //SCR=0x00, SPH=0, SPO=0,FRF=00(Motorola SPI Mode),DSS = 0x7, 8-bit data frame
	outb(sspcpsr_val,SSPCPSR); //CPSDVSR=0x4A --> 200KHz
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

	return 0;
}

/**************************/
/* Close device (as file) */
/**************************/
int spi_release(struct inode *inode, struct file *filep) {
	int sspcr1_val_clear = 0x00000000;
	outl(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	
	release_mem_region(DIO_DATA16_19, 1); // Cierro el espacio de memoria

	MOD_DEC_USE_COUNT;
	printk("<1>SPI Driver: Closing\n");
	return 0;
}

/*****************************************/
/* Read from device (read from register) */
/*****************************************/
ssize_t spi_read(struct file *filep, char *buf, size_t count, loff_t *f_pos) {
	int bytes_read = count;
	short data = 0x0000;
	char rx_status = 0x00;
	char cs_on = 0xF0;
	char cs_off = 0xFF;
	
	printk("<1>SPI: Count contents = 0x%02X\n",count);
	if(count > 3){
		printk("<1>SPI: read instruction only permits 3 elements = 0x%02X\n",count);
		return -1;
	}
	writeb(cs_on,datos);

	outw((short)*buf,SSPDR); //1-byte register
	buf++;
	inw(SSPDR); //Dummy Read to empty RX FIFO
	
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	

	if(count==3){
		outw((short)*buf,SSPDR); //1-byte register
		buf++;
		inw(SSPDR); //Dummy Read to empty RX FIFO
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;
		while(rx_status&SSPSR_BSY){
			rx_status = inb(SSPSR); //Read SPI Status Register
			rx_status &= 0x1F;			
		}
	}
	
	outw(0x0000,SSPDR); // Envío dummy data para generar flancos de reloj
	inw(SSPDR); //Dummy Read to empty RX FIFO
	rx_status = inb(SSPSR); //Read SPI Status Register
	rx_status &= 0x1F;
	while(rx_status&SSPSR_BSY){
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;			
	}	

	writeb(cs_off,datos);
	
	*buf = inb(SSPDR); //Read SPI Data Register	
	printk("<1>SPI data read = 0x%02X\n",*buf);

	data = inw(SSPDR); //Read SPI Data Register	
	printk("<1>ANOTHER SPI data read for debbuging purposes = 0x%04X\n",data);

	return bytes_read;
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
	
	if(count > 8){
		printk("<1>SPI: write function only permits 8 elements\n Count contents = 0x%02X\n",count);
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
			bytes_written++;
			buf++;
		}
	}

	writeb(cs_off,datos);
	
	for(i=0;i<count;i++){
		printk("<1>SPI Status Register at Write no.%d = 0x%02X\n",i,array[i]);
	}
	
	printk("<1>SPI: no of bytes written = %d\n",bytes_written);

	return bytes_written;
}

module_init(spi_init);
module_exit(spi_exit);
