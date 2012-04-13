#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>


int main(int argc, char * argv[]){

	//FILE *tfile;
	char temp[20];
	int ret;
	
	int fd;
	
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
	fprintf(stderr, "fd = %d\n", fd);
	memset(temp,'\0',sizeof(temp));
	ret = read(fd, temp, sizeof(temp));

	fprintf(stderr, "Temperature = %s\n", temp);

	return 0;
}
