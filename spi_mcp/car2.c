#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syslog.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <termios.h> //puesto para pruebas

#include "mcp.h"
#include "car.h"
#include "spi.h"

float gps_loc[5];
FILE *GPS_file;
int gps_serial_fd = -1;

int verbose = 0;

int can_fd;

pthread_t pth_input;
struct termios ort;

// buffer ring  ¿¿¿ QUIZA MEJOR HACERLO CON NAMED_SOCKETS O FIFOS???
/*char can_buffer[MAX_LENGHT_TOKENS];
char *rbuf;
char *wbuf;
*/

// Manejardor de señales (para salir limpiamente)
void signal_handler(int signum){

	if((signum == SIGTERM)||(signum == SIGINT)){
		clean_exit(0);
	}else
		fprintf(stderr, "Signal %d received\n", signum);
	
	signal(signum, signal_handler);
}

// Salida limpia
void clean_exit(int ret){

    if(GPS_file != NULL)
    {
		  fprintf(stderr,"Closing GPS file\n");
        fclose(GPS_file);
        GPS_file = NULL;
    }
    if(gps_serial_fd > 0){
		fprintf(stderr,"Closing GPS port\n");
		close(gps_serial_fd);
		gps_serial_fd = -1;
	}

	if(pthread_cancel(pth_input) != 0){
		perror("pthread_cancel(pth_input) error");
	}
	if(pthread_join(pth_input, NULL) != 0){
		 perror("pth_input pthread_join() error");
	}else
		fprintf(stderr, "input thread finished\n");

	tcsetattr(STDIN_FILENO, TCSANOW, &ort); // solo necesario durante las pruebas, mientras este el thread de input

	exit(ret);
}

int main(int argc, char *argv[]){

	int ret,i;
	//int can_fd;
	int max_fd;

	fd_set rfds;
	struct timeval tv;

	char rbuf[512];
	int rbuf_size;
	
	time_t tt1;
	struct tm *tmPtr;
	char str_tiempo[80];
	time_t t_gps;

	CanMessage candata;
	candata.id=0x0000; // ID de la TS7400
	candata.dlc=CAN_DATA_LENGTH;
	candata.data[0]=0x10; // Protocol version
	candata.data[1]=0x02; // PIC_ID
	candata.data[2]=0x01; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
	candata.data[3]=0x00; // Secuence Number
	candata.data[4]=0xA4; // Variable
	candata.data[5]=0x00; // Value MSB
	candata.data[6]=0x00; // Value LSB
	candata.data[7]=0xFF; // CRC

	

	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	//////////////////////////
	// OPEN CAN DEVICE
	can_fd = open(CAN_DEV, O_RDWR);
	//can_fd = open("./kk", O_RDONLY);
	if(can_fd < 0){
		perror("Error opening CAN device");
		return -1;
	}
	//////////////////////////
	// OPEN GPS DEVICE
	gps_serial_fd = open_GPS(GPS_PORT,GPS_BITRATE);
	if(gps_serial_fd < 0){
		perror("Error opening GPS\n");
		//return -1;
	}
	///////////////////////////

	if(argc == 2){
		if(strncmp(argv[1],"-v",strlen(argv[1])) == 0){
			verbose = 1;
		}else if(strncmp(argv[1], "reset", strlen(argv[1]))==0){
			mcp_init(can_fd);
			printf("Reset enviado\n");
			return 0;
		}else{
			fprintf(stderr,"Usage: %s [-v]\n", argv[0]);
			return -1;
		}
	}

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	tcgetattr(STDIN_FILENO, &ort);
	ret = pthread_create(&pth_input, &attr, (void *)thread_input, NULL);
	if(ret){
		perror("input pthread_create failed");
		return -1;
	}

 // Descomentarlo cuando se abra el can device
	FD_ZERO(&rfds);
	FD_SET(can_fd, &rfds);
	if(gps_serial_fd > 0){
		FD_SET(gps_serial_fd, &rfds);
		if(can_fd > gps_serial_fd)
			max_fd = can_fd;
		else
			max_fd = gps_serial_fd;

		t_gps = time(NULL);
		tmPtr = localtime(&t_gps);
		strftime(str_tiempo,80,"%d/%m/%y %H:%M:%S", tmPtr);
	   	// Creo fichero de log para el gps (guardo protocolo NMEA)
		if((GPS_file = fopen("./gps.log", "wb+")) == NULL){
		   	fprintf(stderr, "Error opening GPS file (./gps.log)\n");
	  		syslog(LOG_DEBUG,"Error opening GPS file (./gps.log)");
		}else
		   	fprintf(GPS_file,"wsserver: Iniciando GPS Log (%s)\n",str_tiempo);
	}else
		max_fd = can_fd;

	tv.tv_sec = 2;
	tv.tv_usec = 0;
	rbuf_size = sizeof(rbuf);

	tt1 = time(NULL);

	fprintf(stderr, "max_fd = %d\n", max_fd);

	mcp_init(can_fd);
	printf("Sale de mcp_init()\n");
	//can_send(can_fd, &can, MCP_TXB0D0);		

	while(1){
		tv.tv_sec = 2;
		tv.tv_usec = 0;
		FD_ZERO(&rfds);
		FD_SET(can_fd, &rfds);
		if(gps_serial_fd > 0){
			FD_SET(gps_serial_fd, &rfds);
			if(can_fd > gps_serial_fd)
				max_fd = can_fd;
			else
				max_fd = gps_serial_fd;
		}else
			max_fd = can_fd;

		if(gps_serial_fd > 0){
			ret = select(max_fd + 1, &rfds, NULL, NULL, &tv);
		}else{
			ret = select(max_fd + 1, &rfds, NULL, NULL, &tv); // Puesto para pruebas
			//ret = select(max_fd + 1, &rfds, NULL, NULL, NULL);
		}

		fprintf(stderr,".");
		if(ret < 0){
			if(errno != EINTR)
				fprintf(stderr, "Select error %d\n", errno);
		}else{
			if(FD_ISSET(can_fd,&rfds)){ 
				memset(rbuf,'\0', rbuf_size);
				if((ret = read(can_fd, rbuf, CAN_DATA_LENGTH)) < 0){
					fprintf(stderr, "Error reading CAN\n");
				}else if(ret == 0){
					fprintf(stderr, "EOF reading CAN\n");
					//sleep(1);
				}else{
					handle_message(rbuf);
				}
			}/*else if(FD_ISSET(alarm_fd,&rfds)){
					
			}*/
z
			if((gps_serial_fd > 0)&&(time(NULL) - tt1 > GPS_FREQ)){
				if(FD_ISSET(gps_serial_fd,&rfds)){ 
					tt1 = time(NULL);
					GPS_parser();
				}
			}
		}
	}
	return 0;
}

void handle_message(char *message){
	int i;
	//fprintf(stderr,"%s\n",message); //PUESTO PARA DEBUG. POR AHORA SOLO MOSTRAMOS EL MSG
	//fprintf(stderr,".%s",message);
	printf("Datos de CAN: ");
	for(i=0;i<CAN_DATA_LENGTH;i++) {
		//printf("Dato[%d]=%02X ", i, message[i]);
		printf("%02X ", message[i]);
	}
	printf("\n");
}

int open_GPS(char *gps_port, int gps_bitrate){
	int gpsfd;

	gpsfd = open(gps_port, O_RDWR | O_NOCTTY | O_NDELAY);
   if(gpsfd < 0){
        perror("Error opening GPS port");
		  return -1;
    }else{
        struct termios my_termios;
        tcgetattr(gpsfd, &my_termios);
        tcflush(gpsfd, TCIFLUSH);
        switch(gps_bitrate){
        case 4800:
            my_termios.c_cflag = B4800 | CS8 | CREAD | CLOCAL | HUPCL;
            my_termios.c_iflag = IGNPAR;// | ICRNL;
            cfsetospeed(&my_termios, B4800);
            break;
        case 9600:
            my_termios.c_cflag = B9600 | CS8 | CREAD | CLOCAL | HUPCL;
            my_termios.c_iflag = IGNPAR;// | ICRNL;
            cfsetospeed(&my_termios, B9600);
            break;
        default:
            fprintf(stderr,"Invalid BitRate. Only 4800 and 9600 supported\n");
            syslog(LOG_DEBUG,"Open GPS failed");
				return -1;
        }
        tcsetattr(gpsfd, TCSANOW, &my_termios);
        printf("Port (%s) opened\n", gps_port);
        syslog(LOG_DEBUG,"GPS port opened");
    }
	return gpsfd;
}

/*  GPS  */
void GPS_parser(void){

	char read_buf1[50];
	char read_prev[150];
	char buffer_tmp[100];
	char *inicio_cadena;
	char *ret_carro;
	char *n_line = NULL;

	int n, num_elements;
	int buf1_size, prev_size, datos_GPS_size;

	char datos_GPS[MAX_CAN_TOKENS][MAX_LENGHT_TOKENS];
	int i = 0;

   prev_size = sizeof(read_prev);
   buf1_size = sizeof(read_buf1);
   datos_GPS_size = sizeof(datos_GPS[0]);


	 memset(read_buf1,'\0',buf1_size);
	 n = read(gps_serial_fd, read_buf1, buf1_size);//254
	 if(n < 0)
	 {
		  fprintf(stderr,"Port read failed\n");
	 }
	 else
	 {
		  // PARSEO DE CADENA
		  strncat(read_prev,read_buf1,prev_size-strlen(read_prev));  //SOLUCIONADO BUG DEBIDO A TAMAÑO DE read_prev DEMASIADO PEQUEÑO
		  if((n_line = strchr(read_prev,'\n')) != NULL)
		  {
				inicio_cadena = n_line + 1;
				if((ret_carro = strchr(inicio_cadena,'\r')) != NULL)
				{
					 *ret_carro = '\0';
					 if((strncmp("$GPGGA",inicio_cadena,6) == 0))
					 {
						  //fprintf(stderr,"%s\n",inicio_cadena); // DESCOMENTAR PARA VER LA LINEA LEIDA DEL GPS (DEBUG)
						  if(GPS_file != NULL)
						  {
								fprintf(GPS_file, "%s\n",inicio_cadena);
								fflush(GPS_file);
						  }
						  for(i=0; i<MAX_CAN_TOKENS; i++)
						  {
								memset(datos_GPS[i],'\0',datos_GPS_size);
						  }

						  num_elements = mystrparser(datos_GPS,inicio_cadena,',');

						  memset(gps_loc, 0, sizeof(float) * 5);

						  if(strncmp(datos_GPS[6],"0",1) != 0)
						  {
								int iloc_temp;
								float floc_temp;
								char *minutos;

								// LATITUD
								minutos = datos_GPS[2] + 2;
								floc_temp = atof(minutos);
								floc_temp /= 60;
								iloc_temp = (atoi(datos_GPS[2]))/100;
								//iloc_temp = lloc_temp / 100000;
								floc_temp += iloc_temp;
								gps_loc[0] = floc_temp;

								// LONGITUD
								minutos = datos_GPS[4] + 3;
								floc_temp = atof(minutos);
								floc_temp /= 60;
								iloc_temp = (atoi(datos_GPS[4]))/100;
								//iloc_temp = lloc_temp / 100000;
								floc_temp += iloc_temp;
								gps_loc[1] = floc_temp;

								// HDOP
								gps_loc[4] = atof(datos_GPS[8]);
								// ALTITUD
								gps_loc[2] = atof(datos_GPS[9]);

								// SIGNO: LATITUD SUR Y LONGITUD ESTE NEGATIVOS
								if(strncmp(datos_GPS[3],"S",1) == 0)
								{
									 gps_loc[0] = gps_loc[0] * (-1);
								}
								if(strncmp(datos_GPS[5],"E",1) == 0)
								{
									 gps_loc[1] = gps_loc[1] * (-1);
								}
								if(verbose > 0)
									 fprintf(stderr, "+++ DEBUG:\n"
												"\tLatitud=%.3f\n"
												"\tLongitud=%.3f\n"
												"\tAltitud=%.1f\n"
												"\tHDOP=%.1f\n", gps_loc[0], gps_loc[1], gps_loc[2], gps_loc[4]);

						  }
					 }
					 if(strlen(ret_carro + 1) > 0)
					 {
						  memset(buffer_tmp,'\0',sizeof(buffer_tmp));
						  strncpy(buffer_tmp,ret_carro+1,sizeof(buffer_tmp));
						  memset(read_prev,'\0',sizeof(read_prev));
						  strncpy(read_prev,buffer_tmp,sizeof(read_prev));
					 }
					 else
						  memset(read_prev,'\0',sizeof(read_prev));
				}
		  }
	 }
}

// Extrae los elementos de cadena separados por separador y los almacena en datos[][MAX_LENGHT_TOKENS]
// Devuelve el numero de elementos extraido
int mystrparser(char datos[MAX_CAN_TOKENS][MAX_LENGHT_TOKENS], char *cadena, char separador)
{
    char *index_ant;
    int i, j, size_datos;
    size_datos = sizeof(datos[0]);
    i = 0;
    j = 0;
    index_ant = &cadena[0];
    while((cadena[i] != '\0')&&(j<MAX_CAN_TOKENS-1))
    {
        if(cadena[i] == ',')
        {
            cadena[i] = '\0';
            strncpy(datos[j],index_ant,size_datos);
            j++;
            index_ant = &cadena[i] + 1;
        }
        i++;
    }
    strncpy(datos[j],index_ant, size_datos);
    return j+1;
}


/*void tigal_handler(int signum) {
	char icod, aux;
	if(signum == SIGIO){
		// tratamiento de la interrupcion TIGAL
		// Enviamos un mensaje al TIGAL para comprobar la fuente de la interrupción
		icod = mcp_readRegister(can_fd, MCP_CANSTAT);
		printf("Registro CANSTAT %02X\n",icod);	
		icod>>=1;
		icod&=0x07;
		switch(icod) {
			case 0x02: // WAK Interrupt
				printf("WAK Interrupt\n");
				clear_interrupt(INT_WAK);
				break;
			case 0x03: // TX0 Interrupt
				printf("TX0 Interrupt\n");
				clear_interrupt(INT_TX0);
				break;
			case 0x04: // TX1 Interrupt
				printf("TX1 Interrupt\n");
				clear_interrupt(INT_TX1);
				break;
			case 0x05: // TX2 Interrupt
				printf("TX2 Interrupt\n");
				clear_interrupt(INT_TX2);
				break;
			case 0x06: // RX0 Interrupt
				printf("RX0 Interrupt\n");
				// AQUI SE DEBE TRATAR LOS MENSAJES RECIBIDOS EN RX0
				clear_interrupt(INT_RX0);
				aux = mcp_readRegister(can_fd, MCP_RXB0CTRL+6);
				printf("Dato recibido en RXB0D0 = %02X\n",aux);
				aux = mcp_readRegister(can_fd, MCP_RXB0CTRL+7);
				printf("Dato recibido en RXB0D1 = %02X\n",aux);
				aux = mcp_readRegister(can_fd, MCP_RXB0CTRL);
				printf("Registro RXB0CTRL %02X\n",aux);
				aux = mcp_readRegister(can_fd, MCP_RXB0CTRL+5);
				printf("Registro RXB0DLC %02X\n",aux);
				aux = mcp_readStatus(can_fd);
				printf("Registro STATUS %02X\n",aux);		
				break;
			case 0x07: // RX1 Interrupt
				printf("RX1 Interrupt\n");
				// AQUI SE DEBE TRATAR LOS MENSAJES RECIBIDOS EN RX1
				clear_interrupt(INT_RX1);
				break;
			case 0x00: // Ninguna interrupción activada
				//printf("Atencion! Ha habido un disparo de la interrupción sin interrupcion en CAN\nICOD = %02X\n",icod);
				break;
			case 0x01: // ERR Interrupt. Asociada con CANINTE y ERRIE.
				// Leemos el Flag de Error para saber la fuente	de error			
				aux = mcp_readRegister(can_fd, MCP_EFLG);
				printf("Registro EFLG %02X\n",aux);
				switch(aux) {
					case 0x01: //TX OR RX WARNING (TEC o REC superior a 96)
					printf("TX OR RX WARNING (TEC or REC is equal or greater than 96)\n");
					break;
					case 0x02: //RX WARNING
					printf("RX WARNING Set when REC is equal to or greater than 96\n");					
					break;
					case 0x04: //TX WARNING
					printf("TX WARNING TEC is equal to or greater than 96\n");				
					break;
					case 0x08: //RX ERROR-PASSIVE
					printf("RX WARNING REC is equal to or greater than 128\n");				
					break;
					case 0x10: //TX ERROR-PASSIVE
					printf("TX WARNING TEC is equal to or greater than 128\n");				
					break;
					case 0x20: //TX BUS-OFF
					printf("Bit set when TEC reaches 255\n");				
					break;
					case 0x40: //RX0 Overflow
					printf("RX0 Overflow: Set when a valid message is received for RXB0 and CANINTF.RX0IF = 1\n");				
					break;
					case 0x80: //RX1 Overflow
					printf("RX1 Overflow: Set when a valid message is received for RXB1 and CANINTF.RX1IF = 1\n");				
					break;
					default:
					printf("Error inesperado: EFLG = %02X\n", aux);
					break;
				clear_interrupt(INT_MERR);
				clear_interrupt(INT_ERR);
				}
				break;
			default:
				printf("Atencion! Ha habido un error en la lectura de ICOD = %02X\n",icod);
				break;
		}
	}else{
		fprintf(stderr, "Signal %d received\n", signum);
	}
	signal(SIGIO, &tigal_handler); 
}
*/


#define KEY_z       0x7A
#define KEY_Z       0x5A
#define KEY_s       0x73
#define KEY_S       0x53
#define KEY_p       0x70
#define KEY_P       0x50
// THREAD DE TECLADO. SOLO PARA DEBUGAR
// CADA VEZ QUE SE PULSA LA TECLA z SE METE UN NUMERO EN EL BUFFER RING SIMULANDO LO QUE HARA EL PROCESO PRINCIPAL,
// QUE METERA EL MENSAJE QUE LEA DEL /DEV/CAN
void thread_input(void *arg)
{
   int keycode=0;

	CanMessage candata;

	while(1){
      pthread_testcancel();
      keycode=mygetch();
		//printf("%02X\n",keycode);
		if((keycode == KEY_s)||(keycode == KEY_S)){
				printf("Engine running!\n");
				// enviar orden de arranque
				candata.id=0x0004; // ID de la TS7400
				candata.dlc=CAN_DATA_LENGTH;
				candata.data[7]=0x10; // Protocol version
				candata.data[6]=0x04; // PIC_ID
				candata.data[5]=0x01; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
				candata.data[4]=0x00; // Secuence Number
				candata.data[3]=0xA4; // Variable
				candata.data[2]=0x01; // Value MSB
				candata.data[1]=0x00; // Value LSB
				candata.data[0]=0xFF; // CRC
				
				can_send(can_fd, &candata, 0);
		}else if((keycode == KEY_p)||(keycode == KEY_P)){
			// enviar orden de paro
			printf("Engine stopped\n");
			candata.id=0x0004; // ID de la TS7400
			candata.dlc=CAN_DATA_LENGTH;
			candata.data[7]=0x10; // Protocol version
			candata.data[6]=0x04; // PIC_ID
			candata.data[5]=0x01; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
			candata.data[4]=0x00; // Secuence Number
			candata.data[3]=0xA4; // Variable
			candata.data[2]=0x00; // Value MSB
			candata.data[1]=0x01; // Value LSB
			candata.data[0]=0xFF; // CRC
			can_send(can_fd, &candata, 0);
		}	
	}
}


int mygetch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
