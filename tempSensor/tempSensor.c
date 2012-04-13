/********************************
 *Simple program to demonstrate
 *reading a temperature using 
 *tmp124 on a TS-7200
 *
 *EWD Oct. 22 2004
 *Eddie Dawyiduk <eddie@embeddedARM.com>
 *******************************/

#include "peekpoke.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include <errno.h>
#include <string.h>

/*Register addresses */ 
#define CHIP_SELECT_PAGE 0x80840000UL
#define SSP_PAGE         0x808A0000UL

/*Offsets*/
#define SSPCR1           0x04
#define SSPCPSR          0x10
#define CHIP_SELECT_DATA 0x30
#define CHIP_SELECT_DDR  0x34
#define SSP_DATA         0x08


#define STR_TEMP_LEN	10
#define WRITE_FREQ	60

int main(int argc, char **argv)
{
	unsigned long val;
	double temp;
	volatile unsigned char *chip_select_page, *ssp_page;
	unsigned char isNegative = FALSE;

	char str_temp[STR_TEMP_LEN];
	int ret;
	FILE *temp_file;
	
	int write_freq;
	
	write_freq = WRITE_FREQ;
	if(argc > 2){
		if(strncmp(argv[1],"--refresh",9) == 0){
			write_freq = atoi(argv[2]);
			if((write_freq < 5)||(write_freq > 300)){
				fprintf(stderr, "Usage: %s [--refresh secs]\n"
                                	"\t(refresh from 5 to 300)\n", argv[0]);
                        	exit(1);
			}
		}else{
			fprintf(stderr, "Usage: %s [--refresh secs]\n"
				"\t(refresh from 5 to 300)\n", argv[0]);
			exit(1);
		}
	}else if(argc == 2){
		fprintf(stderr, "Usage: %s [--refresh secs]\n"
                       "\t(refresh from 5 to 300)\n", argv[0]);
                exit(1);
	}
	fprintf(stderr, "Initialazing tempSensor with refreshing time = %d\n", write_freq);
	int fd = open("/dev/mem", O_RDWR|O_SYNC);
	if(fd < 0){
		fprintf(stderr,"error opening /dev/mem\n");
		exit(1);
	}	
	pid_t pid;
	if((pid = fork()) < 0){
		fprintf(stderr, "Error while demonizing\n");
		exit(1);
	}else if(pid > 0){
		exit(0);
	}
	setsid();

	chdir("/");

	if((pid = fork()) < 0){
                fprintf(stderr, "Error while demonizing\n");
                exit(1);
        }else if(pid > 0){
                exit(0);
        }
	if (freopen("/dev/null", "r", stdin) == NULL) 
    	{
        	fprintf(stderr, "Unable to replace stdin with /dev/null: %d\n",  errno); 
        	exit(EXIT_FAILURE); 
    	}
    	if (freopen("/dev/null", "w", stdout) == NULL) 
    	{
        	fprintf(stderr, "Unable to replace stdout with /dev/null: %d\n",  errno); 
        	exit(EXIT_FAILURE);
    	}
    	if (freopen("/dev/null", "w", stderr) == NULL)
    	{
        	fprintf(stderr, "Unable to replace stderr with /dev/null: %d\n",  errno);
        	exit(EXIT_FAILURE);
    	}

	while(1){
	
		sleep(write_freq);
		/* Lets intialize our pointers */
		chip_select_page = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, CHIP_SELECT_PAGE);
		assert(chip_select_page != MAP_FAILED);

		ssp_page = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SSP_PAGE);
		assert(ssp_page != MAP_FAILED);

		/* 
		 The EP9301 Users Manual says the following algorithm must 
		 be used to configure and enable the SPI bus
		 http://www-s.ti.com/sc/ds/tmp124.pdf
		*/
	
		/* 1.)	Set enable bit(SSE) in register SSPCR1*/
		POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 );
	
		/* 2.)	Write other SSP config registers(SSPCR0 & SSPCPSR)*/
		POKE32( (unsigned long)ssp_page, 0x0F ); 
		POKE32( (unsigned long)(ssp_page + SSPCPSR), 0xFE ); 
	
		/* 3.)	Clear the enable bit(SSE) in register SSPCR1*/
		POKE32( (unsigned long)(ssp_page + SSPCR1), 0x00 ); 
		//usleep(10000); //let the lines settle
		
		/* 4.)	Set the enable bit(SSE) in register SSPCR1*/
		POKE32( (unsigned long)(ssp_page + SSPCR1), 0x10 ); 
	
		/* Done with configuration now lets read the current temp...*/
	
		//enable the chip select
		POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DDR), 0x04 ); 
		POKE32( (unsigned long)(chip_select_page + CHIP_SELECT_DATA), 0x00 ); 
	
	
		//send read temp command
		POKE32( (unsigned long)(ssp_page + SSP_DATA), 0x8000 ); 
		usleep(1000);	
	
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
	
		} else if( val <= 0x4B08 && val >= 0xE486 ) 
		{
			printf("FAIL, invalid register value(out of range)...\n");
			return 2;
		}
	
		if( val >= 0x3E88 && val <= 0x4B07)
		{
			temp = val / 128.046;
	
		} else if( val >= 0xC88 && val <= 0x3E87 )
		{
			temp = val / 128.056;
	
		} else if( val >= 0x10 && val <= 0xC87 )
		{
			temp = val / 128.28;
		} else// => val >= 0x00 && val <= 0x0F 
		{
			temp = val / 240;
		} 

/*	
		//convert the temp to farenheit
		printf("\nThe temp is: %3.2f degrees Celsius\n", temp);
		temp = (temp * 1.8) + 32;
		printf("The temp is: %3.2f degrees Fahrenheit\n", temp);
*/		
		temp_file = fopen("/opt/lcd_coche/temperature", "wb");
		if(temp_file != NULL){
			memset(str_temp, '\0', STR_TEMP_LEN);
			snprintf(str_temp,STR_TEMP_LEN,"%3.2f",temp);
			ret = fwrite(str_temp, sizeof(char), strlen(str_temp), temp_file);
			fclose(temp_file);
		}

	}
	return 0;
		
}
