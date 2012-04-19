#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#define TRUE	1
#define FALSE 	0

int main(int argc, char * argv[]){

	//FILE *tfile;
	char temp[20];
	float temperatura;
	int ret;
	unsigned long val;	
	int fd;
	int isNegative = 0;
	
	/*tfile = fopen("/dev/temperature","r");
	if(tfile == NULL){
		fprintf(stderr, "Error abriendo /dev/register\n");
		exit(1);
	}*/
	fd = open("/dev/temperature", O_RDWR | O_ASYNC);
	if(fd < 0){
		fprintf(stderr, "Error abriendo /dev/temperature\n");
		perror("Error abriendo device");
		return 1;
	}
	memset(temp,'\0',sizeof(temp));
	ret = read(fd, temp, 1);

	val = atoi(temp);
	//fprintf(stderr, "Temperature = %s\n", temp);
	//fprintf(stderr, "Temperature = 0x%lX\n", val);
        //Lets check if the value is negative
        if( val <= 0xFFFF && val >= 0xE487 )
        {
                //perform two's complement
                val = ~val + 1;
                isNegative = TRUE;
        }else if( val <= 0x4B08 && val >= 0xE486 ){
                fprintf(stderr, "<1>FAIL, invalid register value(out of range)...\n");
                return 2;
        }


        if( val >= 0x3E88 && val <= 0x4B07)
        {
                temperatura = (float)(val) / 128.046;

        }else if( val >= 0xC88 && val <= 0x3E87 ){
                temperatura = (float)(val) / 128.056;
        }else if( val >= 0x10 && val <= 0xC87 ){
		temperatura = (float)(val) / 128.28;
        }else{
                temperatura = (float)(val) / 240;
        }

	if(isNegative)
		printf("Temperature = -%2.2f\n", temperatura);
	else
		printf("Temperature = %3.2f\n", temperatura);
		
	return 0;
}
