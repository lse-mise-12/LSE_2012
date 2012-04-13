//Temperature Driver SPI

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <asm/mman.h>

//Hardware register related includes
#include <asm/hardware.h> //Included in asm/io.h

#include "temperature_driver.h"

#define RXTX_FIFOSIZE 8

/*Register addresses */
#define CHIP_SELECT_PAGE 0x80840000UL
#define SSP_PAGE         0x808A0000UL

/*Offsets*/
//#define SSPCR1           0x04
//#define SSPCPSR          0x10
#define CHIP_SELECT_DATA 0x30
#define CHIP_SELECT_DDR  0x34
#define SSP_DATA         0x08

MODULE_LICENSE("Dual BSD/GPL");

/* Functions: interface as files*/
int temp_open(struct inode *inode, struct file *filp);
int temp_release(struct inode *inode, struct file *filep);
ssize_t temp_read(struct file *filep, char *buf, size_t count, loff_t *f_pos);
ssize_t temp_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos);

/*Set functions as file interface (for OS)*/
struct file_operations temp_fops = {
	read: temp_read,
	write: temp_write,
	open: temp_open,
	release: temp_release
};

/* Global variables */
int temp_major = 73;


int temp_init (void){
	int result;

	//Register type of device: character (or block, net)

	result = register_chrdev(temp_major, "temperature2", &temp_fops);
	if (result < 0){
		printk("<1>Fail major number = %d\n", temp_major);
		return result;
	}else{
		printk("<1>Good major number = %d\n", temp_major);		
	}

	printk("<1>Temperature driver initialized..\n");
	return result;
}

void temp_exit (void) {
	//free major number
	unregister_chrdev(temp_major, "spi_if");
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
int temp_open(struct inode *inode, struct file *filp) {
	
//	MOD_INC_USE_COUNT;
	printk("<1>Initializing temp driver\n");
	return 0;
}

/* Close device (as file) */
int temp_release(struct inode *inode, struct file *filep) {
	int sspcr1_val_clear = 0x00000000;
	outl(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	MOD_DEC_USE_COUNT;
	printk("<1>Closing\n");
	return 0;
}

/* Read from device (read from register) */
ssize_t temp_read(struct file *filep, char *buf, size_t count, loff_t *f_pos) {
	unsigned long val;
	double temp;
	volatile unsigned char *chip_select_page, *ssp_page;
	unsigned char isNegative = FALSE;

 	/* Lets intialize our pointers */
	/*chip_select_page = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, CHIP_SELECT_PAGE);
	assert(chip_select_page != MAP_FAILED);

	ssp_page = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SSP_PAGE);
	assert(ssp_page != MAP_FAILED);
*/
	/* 
	The EP9301 Users Manual says the following algorithm must 
	be used to configure and enable the SPI bus
	http://www-s.ti.com/sc/ds/tmp124.pdf
	*/
	printk("<1> traza 1\n");
	
	ssp_page = (unsigned char)SSP_PAGE;
	chip_select_page = (unsigned char)CHIP_SELECT_PAGE;
	/* 1.)   Set enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 );
//	printk("");
//	outw(0x10, SSP_PAGE + SSPCR1); No funciona si se hace la suma ssp_page+sspcr1, ponemos el valor final en los in/out
	outw(0x10, 0x808A0004);
	printk("<1> traza1.1\n");
	/* 2.)   Write other SSP config registers(SSPCR0 & SSPCPSR)*/
	//POKE32( (unsigned long)ssp_page, 0x0F ); 
	//POKE32( (unsigned long)(ssp_page + SSPCPSR), 0xFE ); 
	outw(0x0F, SSP_PAGE);
	printk("<1> traza1.2\n");
//	outw(0xFE, SSP_PAGE + SSPCPSR); 
	outw(0xFE, 0x808A0010);
	
	printk("<1> traza 2\n");



	/* 3.)   Clear the enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(ssp_page + SSPCR1), 0x00 ); 
	outw(0x00, 0x808A0004);
	//usleep(10000); //let the lines settle

	/* 4.)   Set the enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 ); 
	outw(0x10, 0x808A0004);
	/* Done with configuration now lets read the current temp...*/

	//enable the chip select
	//POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DDR), 0x04 );
	outw(0x04, 0x808A0034);
	//POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DATA), 0x00 );
	outw(0x00, 0x808A0030);

	//send read temp command
	//POKE32( (unsigned long)(ssp_page + SSP_DATA), 0x8000 );
	outw(0x8000, 0x808A0008);
	//usleep(1000);
	// while hasta que el flag de spi este correcto

	printk("<1> traza 3\n");

	

	//disable chip select
	//POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DDR), 0x00 );
	outw(0x00, 0x808A0034);
	
	//read the temp
	//val = PEEK32( (unsigned long)(ssp_page + SSP_DATA) );
	val = inw(0x808A0008);
	printk("<1> traza 4\n");  //It should work, almost until this point
	
	//Lets check if the value is negative
	if( val <= 0xFFFF && val >= 0xE487 )
	{
		//perform two's complement
		val = ~val + 1;
		isNegative = TRUE;
	}else if( val <= 0x4B08 && val >= 0xE486 ){
		printk("<1>FAIL, invalid register value(out of range)...\n");
		return 2;
	}

	printk("<1> traza 5\n");

	if( val >= 0x3E88 && val <= 0x4B07)
	{
		temp = val / 128.046;

	}else if( val >= 0xC88 && val <= 0x3E87 ){
		temp = val / 128.056;
	}else if( val >= 0x10 && val <= 0xC87 ){
		temp = val / 128.28;
	}else{
		temp = val / 240;
	}
	//snprintf(buf, 7, "%f", temp);
	
	//buf[0] = "1";
	printk("<1> temp = %f\n",temp);

   	return 1;
}
	/* Write to device (write to register) */
ssize_t temp_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos) {

	return 1;
}

module_init(temp_init);
module_exit(temp_exit);
