//Driver SPI

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/io.h>

//Hardware register related includes
#include <asm/hardware.h> //Included in asm/io.h

#define RXTX_FIFOSIZE 8

//Estos define's ya me los incluye al compilarlo --> asm/hardware.h y asm/io.h
//#define SSPCR0 0x808A0000 	// Control Register 0
//#define SSPCR1 0x808A0004 	// Control Register 1
//#define SSPDR 0x808A0008  	// Receive FIFO / Transmit FIFO Data Register
//#define SSPSR 0x808A000C	// Status Register
//#define SSPCPSR 0x808A0010  	// Clock prescale Register
//#define SSPIIR 0x808A0014	// Interrupt identification Register/Interrupt Clear Register
//#define IRQ_SSP 	53

MODULE_LICENSE("Dual BSD/GPL");

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


int spi_init (void){
	int result;

	//Register type of device: character (or block, net)

	result = register_chrdev(spi_major, "spi_if", &spi_fops);
	if (result < 0){
		printk("<1>Fail major number = %d\n", spi_major);
		return result;
	}else{
		printk("<1>Good major number = %d\n", spi_major);		
	}

	printk("<1>Driver SPI is up!!!!!\n");
	return result;
}

void spi_exit (void) {
	//free major number
	unregister_chrdev(spi_major, "spi_if");
	//Unlock access to register
	//release_region(REG_ADDR_PEDR, REG_NUMB);
	printk("<1>Bye bye!\n");
}

char check_reg(int reg_add, int reg_value)
{
	char p_val=0;
	p_val=inb(reg_add);
	if(p_val!=reg_value)	return(0);
	else return(1);
}


/* Open device (as file) */
int spi_open(struct inode *inode, struct file *filp) {
	
	char sspcr1_val_set = 0x10;
	char sspcr1_val_clear = 0x00;  
	short sspcr0_val = SSPCR0_DSS_8BIT;
	char sspcpsr_val = 0x004A;
	short status = 0x0000;
	MOD_INC_USE_COUNT;
	printk("<1>Opening SPI interface...\n");

	// SPI Interface Init Procedure	
	outb(sspcr1_val_set,SSPCR1); //SOD=0, MS=0, Set SSE (enable SPI), LBM=0, RORIE=0, TIE=0, RIE=0
	outw(sspcr0_val,SSPCR0); //SCR=0x00, SPH=0, SPO=0,FRF=00(Motorola SPI Mode),DSS = 0x7, 8-bit data frame
	/*if(!check_reg(SSPCR0,sspcr0_val))
	{
		printk("<1>SPI Init failed\n");
		MOD_DEC_USE_COUNT;
		return(-1);
	}*/
	outb(sspcpsr_val,SSPCPSR); //CPSDVSR=0x4A --> 100KHz --> salen 200KHz ¿?
	/*if(!check_reg(SSPCPSR,sspcpsr_val))
	{
		printk("<1>SPI Init failed\n");
		MOD_DEC_USE_COUNT;
		return(-1);
	}*/
	outb(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	outb(sspcr1_val_set,SSPCR1); //Set SSE (enable SPI)
	
	// comprobacion de que se han escrito correctamente los valores en los registros
	status = inw(SSPCR0); //Read SPI Control Register 0
	printk("<1>SPI Control Register 0 = 0x%04X\n",status);	
	status = inb(SSPCR1); //Read SPI Control Register 1
	printk("<1>SPI Control Register 1 = 0x%02X\n",status);
	status = inb(SSPCPSR); //Read SPI Clock Prescale Register
	printk("<1>SPI Clock Prescale Register = 0x%02X\n",status);
	status = inb(SSPSR); //Read SPI Status Register
	status &= 0x1F;
	printk("<1>SPI Status Register = 0x%02X\n",status);	
	status = inb(SSPIIR); //Read SPI Interrupt Identification Register
	status &= 0x03;
	printk("<1>SPI Interrupt Identification Register = 0x%02X\n",status);
	return 0;
}

/* Close device (as file) */
int spi_release(struct inode *inode, struct file *filep) {
	int sspcr1_val_clear = 0x00000000;
	outl(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	MOD_DEC_USE_COUNT;
	printk("<1>Closing\n");
	return 0;
}

/* Read from device (read from register) */
ssize_t spi_read(struct file *filep, char *buf, size_t count, loff_t *f_pos) {
	char rx_status = 0x00;
	int bytes_read = 0x00000000;
	int n_reads= 0;
	short data=0x0000;
	int i;
	//printk("<1>SPI_read\n");

	for(i=0;i<count;i++) {
		rx_status = inb(SSPSR); //Read SPI Status Register
		rx_status &= 0x1F;
		n_reads++;
		printk("<1>SPI_status_read no(%d) = 0x%02X\n",n_reads,rx_status);
		if((rx_status&0x04)==0) { // Receive FIFO is empty SSPSR_RNE
			printk("<1>Receive FIFO is empty\n");
			return 0;
			}
		else {
			data = inw(SSPDR); //Read SPI Data Register
			*buf = (char)(data&0x00FF);			
			bytes_read++;
			buf++;
			}
		}	
	return bytes_read;
}
/* Write to device (write to register) */


ssize_t spi_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos) {

	int bytes_write = 0x00000000;
	int n_reads= 0;
	int i = 0;
	char array[RXTX_FIFOSIZE];

	for(i=0;i<count;i++)
	{
		array[i] = inb(SSPSR); //Read SPI Status Register
		array[i] &= 0x1F;
		n_reads++;	
		if((array[i]&SSPSR_TNF)==0) // Transmit FIFO is full
		{
			printk("<1>Transmit FIFO is full. %d bytes written\n",bytes_write);
			return(0);
		}
		else
		{
			outw((short)*buf,SSPDR); //1-byte register
			bytes_write++;
			buf++;
		}
	}
	
	for(i=0;i<count;i++) {
	printk("<1>SPI_status write no(%d) = 0x%02X\n",i,array[i]);
	}
	
	printk("<1>no of bytes written = %d\n",bytes_write);

	return bytes_write;
}

module_init(spi_init);
module_exit(spi_exit);
