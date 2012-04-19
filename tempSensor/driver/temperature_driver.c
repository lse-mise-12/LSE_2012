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
int temp_major = 75;


int temp_init (void){
	int result;

	//Register type of device: character (or block, net)

	result = register_chrdev(temp_major, "temperature", &temp_fops);
	if (result < 0){
		printk("<1>Fail major number for temp_Driver = %d\n", temp_major);
		return result;
	}else{
		printk("<1>Good major number for temp_driver = %d\n", temp_major);		
	}

	printk("<1>Temperature driver initialized..\n");
	return result;
}

void temp_exit (void) {
	//free major number
	unregister_chrdev(temp_major, "temperature");
	//Unlock access to register
	//release_region(REG_ADDR_PEDR, REG_NUMB);
	printk("<1>Temperature driver exit succesfully\n");
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
	
	//MOD_INC_USE_COUNT;
	printk("<1>Initializing temp driver\n");
	return 0;
}

/* Close device (as file) */
int temp_release(struct inode *inode, struct file *filep) {
//	int sspcr1_val_clear = 0x00000000;
//	outl(sspcr1_val_clear,SSPCR1); //Clear SSE (disable SPI)
	//MOD_DEC_USE_COUNT;
	printk("<1>Closing temp driver\n");
	return 0;
}

/* Read from device (read from register) */
ssize_t temp_read(struct file *filep, char *buf, size_t count, loff_t *f_pos) {
	unsigned long val; //, aux;
	float temp;
//	volatile unsigned char *chip_select_page, *ssp_page;
	unsigned char isNegative = FALSE;

 	/* Lets intialize our pointers */
	/*CHIP_SELECT_PAGE = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, CHIP_SELECT_PAGE);
	assert(CHIP_SELECT_PAGE != MAP_FAILED);

	SSP_PAGE = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SSP_PAGE);
	assert(SSP_PAGE != MAP_FAILED);
*/
	/* 
	The EP9301 Users Manual says the following algorithm must 
	be used to configure and enable the SPI bus
	http://www-s.ti.com/sc/ds/tmp124.pdf
	*/
	
	//ssp_page = (unsigned char)SSP_PAGE;
	//chip_select_page = (unsigned char)CHIP_SELECT_PAGE;

	printk("Temperature Driver addresses: SSP_PAGE(%#lx) SSPCR1(%#x)\n",SSP_PAGE, SSPCR1);  //It should work, almost until this point
	/* 1.)   Set enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(SSP_PAGE + SSPCR1), 0x10 );
//	printk("");
	//outw(0x10, SSP_PAGE + SSPCR1);// No funciona si se hace la suma SSP_PAGE+sspcr1, ponemos el valor final en los in/out
	// 1
	printk("1. Temperature Driver: 0x10 to SSPCR1(%#x)\n", SSPCR1);  //It should work, almost until this point
	outw(0x10, SSPCR1);// No funciona si se hace la suma SSP_PAGE+sspcr1, ponemos el valor final en los in/out
//	outw(0x10, 0x808A0004);
///////////////////////////////////////////////////////////
//	val = inw(0x808A0004);
//	sprintf(buf, "%#lx", val);
//	return 4;
///////////////////////////////////////////////////////////
	/* 2.)   Write other SSP config registers(SSPCR0 & SSPCPSR)*/
	//POKE32( (unsigned long)SSP_PAGE, 0x0F ); 
	//POKE32( (unsigned long)(SSP_PAGE + SSPCPSR), 0xFE ); 
	// 2.A
	printk("2. Temperature Driver: 0x0F to SSP_PAGE(%#lx)\n", SSP_PAGE);  //It should work, almost until this point
	outw(0x0F, SSP_PAGE);
//	outw(0xFE, SSP_PAGE + SSPCPSR); 
	// 2.B
	printk("3. Temperature Driver: 0xFE to SSPCPSR(%#x)\n", SSPCPSR);  //It should work, almost until this point
	outw(0xFE, SSPCPSR); 
//	outw(0xFE, 0x808A0010);

//	outw(0xAA, 0x808A0010);
/*
	aux = inw(0x808A0010);
	printk("<1> traza aux: aux=%lu\n",aux);	
*/	



	/* 3.)   Clear the enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(SSP_PAGE + SSPCR1), 0x00 ); 
	printk("4. Temperature Driver: 0x00 to SSPCR1(%#x)\n", SSPCR1);
	//outw(0x00, 0x808A0004);
	outw(0x00, SSPCR1);
	//usleep(10000); //let the lines settle

	/* 4.)   Set the enable bit(SSE) in register SSPCR1*/
	//POKE32( (unsigned long)(SSP_PAGE + SSPCR1), 0x10 ); 
	//outw(0x10, 0x808A0004);
	printk("5. Temperature Driver: 0x10 to SSPCR1(%#x)\n", SSPCR1);
	outw(0x10, SSPCR1);
	/* Done with configuration now lets read the current temp...*/

	//enable the chip select
	//POKE32( (unsigned long)(CHIP_SELECT_PAGE + CHIP_SELECT_DDR), 0x04 );
	//outw(0x04, 0x808A0034);
	//POKE32( (unsigned long)(CHIP_SELECT_PAGE + CHIP_SELECT_DDR), 0x04 );
	printk("6. Temperature Driver: 0x04 to CHIP_SELECT_PAGE + CHIP_SELECT_DDR (%#lx)\n", CHIP_SELECT_PAGE + CHIP_SELECT_DDR);
	// 4.A
	outw(0x04, CHIP_SELECT_PAGE + CHIP_SELECT_DDR);
	// 4.B
	printk("7. Temperature Driver: 0x00 to CHIP_SELECT_PAGE + CHIP_SELECT_DATA (%#lx)\n", CHIP_SELECT_PAGE + CHIP_SELECT_DATA);
	outw(0x00, CHIP_SELECT_PAGE + CHIP_SELECT_DATA);
	//outw(0x00, 0x808A0030);

	//send read temp command
	//POKE32( (unsigned long)(SSP_PAGE + SSP_DATA), 0x8000 );
	printk("8. Temperature Driver: 0x8000 to SSP_PAGE + SSP_DATA(%#lx)\n", SSP_PAGE + SSP_DATA);
	//outw(0x8000, 0x808A0008);
	// 4.C
	outw(0x8000, SSP_PAGE + SSP_DATA);
	//usleep(1000);
	// while hasta que el flag de spi este correcto

	//disable chip select
	//POKE32( (unsigned long)(CHIP_SELECT_PAGE + CHIP_SELECT_DDR), 0x00 );
	//outw(0x00, 0x808A0034);
	printk("9. Temperature Driver: 0x00 to CHIP_SELECT_PAGE + CHIP_SELECT_DDR (%#lx)\n", CHIP_SELECT_PAGE + CHIP_SELECT_DDR);
	// 4.D
	outw(0x00, CHIP_SELECT_PAGE + CHIP_SELECT_DDR);
	//read the temp
	//val = PEEK32( (unsigned long)(SSP_PAGE + SSP_DATA) );
	//val = inw(0x808A0008);
	printk("Temperature Driver: SSP_PAGE + SSP_DATA(%#lx)\n", SSP_PAGE + SSP_DATA);
	val = inw(SSP_PAGE + SSP_DATA);
//	val = 0x05;
	printk("<1> traza 4: val=0x%lX\n",val);  //It should work, almost until this point
	sprintf(buf, "%lu", val);
	return 7;
	
/*
	//Lets check if the value is negative
	if( val <= 0xFFFF && val >= 0xE487 )
	{
		//perform two's complement
		val = ~val + 1;
		isNegative = TRUE;
	}else if( val <= 0x4B08 && val >= 0xE486 ){
		printk("<1>FAIL, invalid register value(out of range)...[Temp Driver]\n");
		return 2;
	}


	if( val >= 0x3E88 && val <= 0x4B07)
	{
		printk("<1> traza 5.1\n");
		temp = (float)(val) / 128.046;
		printk("<1> traza 5.2\n");

	}else if( val >= 0xC88 && val <= 0x3E87 ){
		printk("<1> traza 5.3\n");
		temp = (float)(val) / 128.056;
		printk("<1> traza 5.4\n");
	}else if( val >= 0x10 && val <= 0xC87 ){
		printk("<1> traza 5.5\n");
		temp = (float)(val) / 128.28;
		printk("<1> traza 5.6\n");
	}else{
		printk("<1> traza 5.7\n");
		temp = (float)(val) / 240;
		printk("<1> traza 5.8\n");
	}
	
	int a;
	a = temp * 1000;
	sprintf(buf, "%d", a);
//	sprintf(buf, "prueba snprintf\n");
//	*buf = 42;
	//return 1;
	
	
	//buf[0] = "1";
//	printk("<1> temp = %f\n",temp);

   	return 7; //strlen(buf);
*/
}
	/* Write to device (write to register) */
ssize_t temp_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos) {

	return 1;
}

module_init(temp_init);
module_exit(temp_exit);
