#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <asm/mman.h>
#include <asm/hardware.h>

//#include "peekpoke.h"
#include "prueba1.h"

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

int reg_open(struct inode *inode, struct file *filp);
int reg_release(struct inode *inode, struct file *filep);
ssize_t update_temp(struct file *filep, char *buf, size_t count, loff_t *f_pos); 
ssize_t reg_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos);

struct file_operations reg_fops = {
	read: update_temp,
	write: reg_write,
	open: reg_open,
	release: reg_release
};

int reg_major = 77;

int my_init(void){

	int result;
   //Register type of device: character (or block, net)
   result = register_chrdev(reg_major, "register", &reg_fops);
   if (result < 0){
  	printk("<1>Error loading temp driver\n");
      	return 1;
//  
   }
      printk("<1>Starting temp driver\n");
      return 0;
//   }
   //Lock access to REG_ADDR (just 1 byte)
   //Ex: #define REG_ADDR GPIO_PEDDR
   check_region(SSP_PAGE, 1); //REG_ADDR must be defined	

}

void driver_temp_exit(void) {
	//free major number	
	unregister_chrdev(reg_major, "register");
	//Unlock access to register
//	release_region(REG_ADDR, 1);
	printk("<1>Closing temp driver\n");
	//return 1;
}

/* Open device (as file) */
int reg_open(struct inode *inode, struct file *filp) {
	MOD_INC_USE_COUNT;
	printk("<1>Opening\n");
	return 0;
}
/* Close device (as file) */
int reg_release(struct inode *inode, struct file *filep) {
	MOD_DEC_USE_COUNT;
	printk("<1>Closing\n");
	return 0;
}

/* Read from device (read from register) */
/*
ssize_t reg_read(struct file *filep, char *buf, size_t count, loff_t *f_pos){
   *buf = inb(REG_ADDR);   //1-byte register
   return 1;
}*/

/* Write to device (write to register) */
ssize_t reg_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos){

//	outb(*buf, REG_ADDR);	//1-byte register
	return 1;
}
//ssize_t update_temp(struct file *filep, char *buf, size_t count, loff_t *f_pos){
ssize_t update_temp(struct file *filep, char *buf, size_t count, loff_t *f_pos){
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
	
	ssp_page = (unsigned char)SSP_PAGE;
	chip_select_page = (unsigned char)CHIP_SELECT_PAGE;
	/* 1.)   Set enable bit(SSE) in register SSPCR1*/
	POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 );

	/* 2.)   Write other SSP config registers(SSPCR0 & SSPCPSR)*/
	POKE32( (unsigned long)ssp_page, 0x0F ); 
	POKE32( (unsigned long)(ssp_page + SSPCPSR), 0xFE ); 

	/* 3.)   Clear the enable bit(SSE) in register SSPCR1*/
	POKE32( (unsigned long)(ssp_page + SSPCR1), 0x00 ); 
	//usleep(10000); //let the lines settle

	/* 4.)   Set the enable bit(SSE) in register SSPCR1*/
	POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 ); 

	/* Done with configuration now lets read the current temp...*/

	//enable the chip select
	POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DDR), 0x04 );
	POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DATA), 0x00 );


	//send read temp command
	POKE32( (unsigned long)(ssp_page + SSP_DATA), 0x8000 );
	//usleep(1000);
	
	unsigned long i;
	i = 0;
	while(i < 100000){
		i++;
	}

	//disable chip select
	POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DDR), 0x00 );

	//read the temp
	val = PEEK32( (unsigned long)(ssp_page + SSP_DATA) );

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
	snprintf(buf, 7, "%f", temp);

   	return 1;
}

module_init(my_init);
module_exit(driver_temp_exit);
