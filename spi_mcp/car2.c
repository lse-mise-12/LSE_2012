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
#include <termios.h> 		//puesto para pruebas


#include "mcp.h"
#include "car.h"
#include "spi.h"

#define PIC_DEBUG		0
#define TEST_TIMEOUT	5

int test = 0;
int test_tout = TEST_TIMEOUT;
int test_pic = 1;

FILE *fila1 = NULL;
FILE *fila4 = NULL;

// variables
//		0: OFF
//		1: ON
//		2: AUTO
struct pic1{
	uint8_t security_ON;
	uint8_t sec1;
	uint8_t sec2;
	uint8_t wipers_ON;
	uint8_t wipers_auto;
	int16_t temp;
};
struct pic2{
	uint8_t lights_ON;
	uint8_t lights_auto;
	uint8_t distance;
};
struct pic3{
	uint8_t parking_ON;
};
struct pic4{
	uint8_t cruise_ctl;
	uint8_t velocity;
};

struct car_state{
	struct pic1 pic_1;
	struct pic2 pic_2;
	struct pic3 pic_3;
	struct pic4 pic_4;
}state;


//int sys_init(int desc);
void can_send_test(int);
int lcd_update(void);


float gps_loc[5];
FILE *GPS_file;
int gps_serial_fd = -1;

int verbose = 0;

int can_fd;

pthread_t pth_input;
struct termios ort;

#include "RemoteControl.c"

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

	if(fila1 != NULL) fclose(fila1);
	if(fila4 != NULL) fclose(fila4);
	exit(ret);
}

/*************************************/
/************* MAIN ******************/
/*************************************/

int main(int argc, char *argv[]){

	int ret;
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

	//remote control variables
	struct termios oldtio, newtio;
	int remote_fd = -1;
	//unsigned char state_remote = E0;
	remote_t remote_var;
	unsigned char buf[30];
	int res;



	struct car_state last_state;

	memset(&last_state,0,sizeof(last_state));

	remote_var.state_remote = E0;
	remote_var.button = BUTTON_OFF; 

	CanMessage candata;
/*
	candata.id=0x0004; // ID destino
	candata.dlc=CAN_DATA_LENGTH;
	candata.data[7]=0x10; // Protocol version
	candata.data[6]=0x00; // ID origen
	candata.data[5]=0x01; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
	candata.data[4]=0x00; // Secuence Number
	candata.data[3]=0xA4; // Variable
	candata.data[2]=0x01; // Value MSB
	candata.data[1]=0x00; // Value LSB
	candata.data[0]=0xFF; // CRC
*/
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	//////////////////////////
	// OPEN CAN DEVICE
	can_fd = open(CAN_DEV, O_RDWR);
	//can_fd = open("./kk", O_RDONLY);
	//////////////////////////
	
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


	/////////////////////////////////
	// OPEN 	REMOTE CONTROL DEVICE
	remote_fd = open(REMOTECONTROL_DEVICE,  O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (remote_fd <0) {
		perror(REMOTECONTROL_DEVICE); 
		//exit(-1); 
	}else
		SerialPort_Config(&remote_fd, &newtio, &oldtio);
	/////////////////////////////////

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

	fila1 = fopen("/opt/lcd_coche/fila1","wb");
	if(fila1 == NULL){
		perror("Error abriendo fila1");
		exit(1);
	}
	fila4 = fopen("/opt/lcd_coche/fila4","wb");
	if(fila1 == NULL){
		perror("Error abriendo fila4");
		exit(1);
	}
	
	lcd_update(); //PUESTO PARA DEBUG
	
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
	if(remote_fd > 0) FD_SET(remote_fd, &rfds);

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

	if(remote_fd > 0) max_fd = find_max(remote_fd) + 1;

	tv.tv_sec = 2;
	tv.tv_usec = 0;
	rbuf_size = sizeof(rbuf);

	tt1 = time(NULL);

	fprintf(stderr, "max_fd = %d\n", max_fd);

	///////////////////////////////
	// Inicialización TIGAL ///////
	///////////////////////////////
	mcp_init(can_fd);
	////////////////////////////////////////////////////////
	// Chequeo de comunicaciones con el resto del sistema //
	////////////////////////////////////////////////////////
	//sys_init(can_fd);
	//can_send_test(test_pic);

	while(1){
		tv.tv_sec = 0;
		tv.tv_usec = 200000;

		FD_ZERO(&rfds);
		FD_SET(can_fd, &rfds);
		if(remote_fd > 0) FD_SET(remote_fd, &rfds);

		if(gps_serial_fd > 0){
			FD_SET(gps_serial_fd, &rfds);
			if(can_fd > gps_serial_fd)
				max_fd = can_fd;
			else
				max_fd = gps_serial_fd;
		}//else
			//max_fd = can_fd;

		if(remote_fd > 0) max_fd = find_max(remote_fd);

		if(gps_serial_fd > 0){
			ret = select(max_fd + 1, &rfds, NULL, NULL, &tv);
		}else{
			ret = select(max_fd + 1, &rfds, NULL, NULL, &tv); // Puesto para pruebas
			//ret = select(max_fd + 1, &rfds, NULL, NULL, NULL);
		}
		if((test)&&(test_tout > 0)){
			//fprintf(stderr, ".");
			test_tout--;
			if(test_tout == 0){
				if(test_pic < 5){
					fprintf(stderr,"El PIC %d no responde\n", test_pic);
					// test time-out, enviar test al siguiente pic
					test_pic++;
					can_send_test(test_pic);
				}else{
					fprintf(stderr,"El PIC %d no responde\n", test_pic);
					fprintf(stderr,"Fin del test\n");
					test = 0;
				}
			}
		}
		fprintf(stderr,".");
		if(ret < 0){
			if(errno != EINTR)
				fprintf(stderr, "Select error %d\n", errno);
		}else{
			if(FD_ISSET(can_fd,&rfds)){ 
				fprintf(stderr, "#");
				memset(rbuf,'\0', rbuf_size);
				if((ret = read(can_fd, rbuf, CAN_DATA_LENGTH)) < 0){
					fprintf(stderr, "Error reading CAN\n");
				}else if(ret == 0){
					fprintf(stderr, "EOF reading CAN\n");
					//sleep(1);
				}else{
					handle_message(rbuf);
				}
			}else if(remote_fd > 0){
				if(FD_ISSET(remote_fd, &rfds)){
					res = read(remote_fd, buf, 255);
					remote_var = obtener_payload(buf, res, remote_var);					
    		     	candata.dlc=CAN_DATA_LENGTH;
		         candata.data[7]=0x10; // Protocol version
		         candata.data[6]=(uint8_t)0; // ID origen
      		   candata.data[5]=0x02; // MSG TYPE: WRITE
		         candata.data[4]=0x00; // Secuence Number
		         candata.data[3] = 0x04; // Variable
		         candata.data[2] = 0x00; // Value MSB
		         candata.data[0]=0xFF; // CRC
      		   candata.id=(uint16_t)1; // ID el mensaje
					if(remote_var.button == BUTTON_ON)
						//enviar segurity ON
		         	candata.data[1] = 0x01; // Value LSB
    	
					else if(remote_var.button == BUTTON_OFF) 
						// enviar mensaje de security OFF
		         	candata.data[1] = 0x00; // Value LSB

					fprintf(stderr,"Sending security\n");
         		can_send(can_fd, &candata, 0); 
				}
			}

			if((gps_serial_fd > 0)&&(time(NULL) - tt1 > GPS_FREQ)){
				if(FD_ISSET(gps_serial_fd,&rfds)){ 
					tt1 = time(NULL);
					GPS_parser();
				}
			}
			// ACTUALIZAR PANTALLA: velocidad, temperatura, motor ON/OFF, alarm ON/OFF
			if((state.pic_4.velocity != last_state.pic_4.velocity)||(state.pic_1.temp != last_state.pic_1.temp)||
				(state.pic_1.security_ON != last_state.pic_1.security_ON)||(state.pic_1.wipers_ON != last_state.pic_1.wipers_ON)||
				(state.pic_1.wipers_auto != last_state.pic_1.wipers_auto)||(state.pic_2.lights_ON != last_state.pic_2.lights_ON)||
				(state.pic_2.lights_auto != last_state.pic_2.lights_auto)||(state.pic_3.parking_ON != last_state.pic_3.parking_ON)||
				(state.pic_4.cruise_ctl != state.pic_4.cruise_ctl)){
				last_state.pic_4.velocity = state.pic_4.velocity;
				last_state.pic_1.temp = state.pic_1.temp;
				last_state.pic_1.security_ON = state.pic_1.security_ON;
				last_state.pic_1.wipers_ON = state.pic_1.wipers_ON;
				last_state.pic_1.wipers_auto = state.pic_1.wipers_auto;
				last_state.pic_2.lights_ON = state.pic_2.lights_ON;
				last_state.pic_2.lights_auto = state.pic_2.lights_auto;
				last_state.pic_3.parking_ON = state.pic_3.parking_ON;
				last_state.pic_4.cruise_ctl = state.pic_4.cruise_ctl;
				lcd_update();
			}
		}
	}
	return 0;
}

int lcd_update(void){

	int ret, len;
	char lcd_buffer[44];

	memset(lcd_buffer,'\0',sizeof(lcd_buffer));
	if((ret = fseek(fila1,0,SEEK_SET)) != 0){
		fprintf(stderr,"Error in fseek fila1\n");
		return -1;
	}
	snprintf(lcd_buffer,sizeof(lcd_buffer),"%2d C    %d km/h", state.pic_1.temp ,state.pic_4.velocity);
	//fprintf(stderr, "LCD: %s\n", lcd_buffer);
	if((ret = fwrite(lcd_buffer, sizeof(char), strlen(lcd_buffer), fila1)) != strlen(lcd_buffer)){
		fprintf(stderr, "Error actualizando LCD\n");
		return -1;
	}
	fflush(fila1);

	memset(lcd_buffer,'\0',sizeof(lcd_buffer));
	if((ret = fseek(fila4,0,SEEK_SET)) != 0){
		fprintf(stderr,"Error in fseek fila4\n");
		return -1;
	}

	if(state.pic_1.security_ON == 1){
		snprintf(lcd_buffer,sizeof(lcd_buffer),"  ON"); //, state.pic_1.temp ,state.pic_4.velocity);
	}else{
		snprintf(lcd_buffer,sizeof(lcd_buffer)," OFF"); //, state.pic_1.temp ,state.pic_4.velocity);
	}
	len = strlen(lcd_buffer);
	if(state.pic_1.wipers_ON == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"    ON"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"   OFF"); 
	}
	len = strlen(lcd_buffer);
	if(state.pic_1.wipers_auto == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"/A"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"/M"); 
	}
	len = strlen(lcd_buffer);
	if(state.pic_2.lights_ON == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"    ON"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"   OFF"); 
	}
	len = strlen(lcd_buffer);
	if(state.pic_2.lights_auto == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"/A"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"/M"); 
	}
	len = strlen(lcd_buffer);

	if(state.pic_3.parking_ON == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"     ON"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"    OFF"); 
	}
	len = strlen(lcd_buffer);

	if(state.pic_4.cruise_ctl == 1){
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"       ON"); 
	}else{
		snprintf(lcd_buffer + len,sizeof(lcd_buffer) - len,"      OFF"); 
	}
	len = strlen(lcd_buffer);

	fprintf(stderr, "state: %s\n",lcd_buffer);
	if((ret = fwrite(lcd_buffer, sizeof(char), strlen(lcd_buffer), fila4)) != strlen(lcd_buffer)){
		fprintf(stderr, "Error actualizando LCD\n");
		return -1;
	}
	fflush(fila4);
	return 0;
}

void can_send_test(int pic){

	CanMessage candata;
	
	test_tout = TEST_TIMEOUT;
	//test = 1;
	test = 0;

	candata.dlc=CAN_DATA_LENGTH;
	candata.data[7]=0x10; // Protocol version
	candata.data[5]=0x00; // MSG TYPE--> 0x00:TEST
	candata.data[4]=0x00; // Secuence Number
	candata.data[3]=0x00; // Variable
	candata.data[2]=0xA4; // Value MSB
	candata.data[1]=0x00; // Value LSB
	candata.data[0]=0xFF; // CRC

	candata.id=(uint16_t)pic; // ID destino
	candata.data[6]=(uint8_t)0; // ID origen

	fprintf(stderr, "Sending TEST to pic %d\n", pic);
	can_send(can_fd, &candata, 0);
	
}
/*
int sys_init(int desc)
{
	int i,j,ret,rbuf_size;
	char rbuf[100];
	CanMessage candata;
	fd_set rfds;
	struct timeval tv;
	
	candata.dlc=CAN_DATA_LENGTH;
	candata.data[7]=0x10; // Protocol version
	candata.data[5]=0x00; // MSG TYPE--> 0x00:TEST
	candata.data[4]=0x00; // Secuence Number
	candata.data[3]=0x00; // Variable
	candata.data[2]=0xA4; // Value MSB
	candata.data[1]=0x00; // Value LSB
	candata.data[0]=0xFF; // CRC

	for(i=1;i<2;i++)
	{
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		FD_ZERO(&rfds);
		FD_SET(desc, &rfds);

		candata.id=(uint16_t)i; // ID el mensaje
		candata.data[6]=(uint8_t)i; // ID origen
		can_send(desc, &candata, 0);
		
		ret = select(desc + 1, &rfds, NULL, NULL, &tv);
		if(ret < 0){
			if(errno != EINTR)
				fprintf(stderr, "Select error on sys_init\nerrno = %d\n", errno);
		}else{
			if(FD_ISSET(desc,&rfds)){ 
				memset(rbuf,'\0', rbuf_size);
				if((ret = read(desc, rbuf, CAN_DATA_LENGTH)) < 0){
					fprintf(stderr, "Error reading CAN\n");
				}else{
					if(ret == 0){
						fprintf(stderr, "EOF reading CAN\n");
						//sleep(1);
					}else{
						// Comprobamos que se trata destino de respuesta del test
						printf("Datos de respuesta de PIC%d:\n",i);
						for(j=0;j<CAN_DATA_LENGTH;j++) {						
							printf("rbuf[%d]=%02X ", j, rbuf[j]);
						}
						printf("\n");
						if((rbuf[5]==0x00)&&(rbuf[2]==0xA4)){ //Si el mensaje recibido es TEST, y Valor(MSB) = 0xA4 -> sistema OK
							printf("\nPIC nº%d OK\n", i);
						}
					}
				}
				
			}else{ //FD_ISNOTSET o se ha cumplido el timeout
				fprintf(stderr, "PIC nº%d NO responde", i);
			}
		}
	}
	return 0;
}
*/
void handle_message(char *message){
	//int i;
	CanMessage candata;	
/*
	printf("Datos de CAN: ");
	for(i=0;i<CAN_DATA_LENGTH;i++) {
		printf("%02X ", message[i]);
	}
	printf("\n");
*/	
	candata.proto_version = message[7];
	candata.picID = message[6];
	candata.type = message[5];
	candata.secNum = message[4];
	candata.variable = message[3];
	candata.valM = message[2];
	candata.valL = message[1];
	candata.crc = message[0];

#ifdef _DEBUG_
	if((candata.picID == PIC_DEBUG)||(PIC_DEBUG == 0)){
		fprintf(stderr,"RECEIVED: PIC: 0x%02X TYPE: 0x%02X SN: 0x%02X VAR: 0x%02X MSB: 0x%02X LSB: 0x%02X\n",candata.picID,candata.type,
								candata.secNum,candata.variable,candata.valM,candata.valL);
/*
		fprintf(stderr,"CAN Message:\n"
				"Proto. Version = 0x%02X\n"
				"PIC ID = 0x%02X\n"
				"Message Type = 0x%02X\n"
				"Secuence Number = 0x%02X\n"
				"Variable = 0x%02X\n"
				"Value MSB = 0x%02X\n"
				"Value LSB = 0x%02X\n"
				"CRC = 0x%02X\n\n",candata.proto_version,candata.picID,candata.type,
						candata.secNum,candata.variable,candata.valM,
						candata.valL,candata.crc);
*/
	}
#endif
	if(candata.type == 0x03){
		fprintf(stderr,"Error en PIC %d\n", candata.picID);
	}else if((candata.type == 0x00)&&(test)){ // test : cuando la TS envia un TEST a un pic este responde con un mensaje tipo TEST y el estado de sus sensores
											 // en el LSB. A continuación la TS envía la consulta TEST al siguiente pic, hasta llegar al último.

		fprintf(stderr,"PIC %d OK\n", candata.picID);
		switch(candata.picID){		 			
			case 0x01:
				if(test_pic == 1){
					// leer estados pic 1
					state.pic_1.security_ON = 	candata.valL & 0x01;	
					state.pic_1.sec1 			=	candata.valL & 0x02;	
					state.pic_1.sec2 			=	candata.valL & 0x04;	
					state.pic_1.wipers_ON 	= 	candata.valL & 0x08;	
					state.pic_1.wipers_auto = 	candata.valL & 0x10;	
					state.pic_1.temp 			= 	candata.valM;	
					// mandar test al pic 2
					can_send_test(++test_pic);
				}
				break;
			case 0x02:
				if(test_pic == 2){
					// leer estados pic 2
					state.pic_2.lights_ON	= 	candata.valL & 0x01;
					state.pic_2.lights_auto	= 	candata.valL & 0x02;
					state.pic_2.distance		= 	candata.valM;
					// mandar test al pic 3
					can_send_test(++test_pic);
				}
				break;
			case 0x03:
				if(test_pic == 3){
					// leer estados pic 3
					state.pic_3.parking_ON	= 	candata.valL & 0x01;
					// mandar test al pic 4
					can_send_test(++test_pic);
				}
				break;
			case 0x04:
				if(test_pic == 4){
					// leer estados pic 4
					state.pic_4.cruise_ctl	= 	candata.valL;
					state.pic_4.velocity		= 	candata.valM;
					// mandar test al pic 5
					can_send_test(++test_pic);
				}
				break;
			case 0x05:
				test = 0;
				fprintf(stderr,"End of test\n");
				break;
		}
	}else if(candata.type == 0x01){ // READ
		switch(candata.picID){
			case 0x01:
				if(candata.variable == 0x04){
					state.pic_1.security_ON = candata.valL;
					fprintf(stderr,"security_ON: %d\n", state.pic_1.security_ON);
				}else if(candata.variable == 0x05){
					state.pic_1.sec1 = candata.valL;
				}else if(candata.variable == 0x06){
					state.pic_1.sec2 = candata.valL;
				}else if(candata.variable == 0x07){
					state.pic_1.wipers_ON = candata.valL;
					fprintf(stderr,"wipers_ON: %d\n", state.pic_1.wipers_ON);
				}else if(candata.variable == 0x08){
					state.pic_1.wipers_auto = candata.valL;
					fprintf(stderr,"wipers_auto: %d\n", state.pic_1.wipers_auto);
				}else if(candata.variable == 0x11){
					state.pic_1.temp = ((uint16_t)(candata.valM) << 8) | candata.valL;
					fprintf(stderr,"Temp: %d\n", state.pic_1.temp);
				}
				break;
			case 0x02:
				if(candata.variable == 0x14){
					state.pic_2.lights_ON = candata.valL;
					fprintf(stderr,"ligths_ON: %d\n", state.pic_2.lights_ON);
				}else if(candata.variable == 0x28){
					state.pic_2.lights_auto = candata.valL;
					fprintf(stderr,"ligths_auto: %d\n", state.pic_2.lights_auto);
				}else if(candata.variable == 0x16){
					fprintf(stderr,"distance: %d\n", state.pic_2.distance);
					state.pic_2.distance = candata.valL;
			 	}	
				break;
			case 0x03:
				if(candata.variable == 0x19){
					fprintf(stderr,"parking_ON: %d\n", state.pic_3.parking_ON);
					state.pic_3.parking_ON = candata.valL;
				}
				break;
			case 0x04:
				if(candata.variable == 0x22){
					state.pic_4.velocity = candata.valL;
					fprintf(stderr,"velocity: %d\n", state.pic_4.velocity);
				}else if(candata.variable == 0x23){
					state.pic_4.cruise_ctl = candata.valL;
					fprintf(stderr,"cruise_ctl: %d\n", state.pic_4.cruise_ctl);
				}
				break;
			default:
				fprintf(stderr, "Unknown PIC ID %02X\n",candata.picID);
		}
	}else{
		//if(candata.picID == 0x00){
		// RECIBIDO TIPO DE MSG: WRITE
		if(candata.picID == 0x05){
			candata.dlc=CAN_DATA_LENGTH;
			candata.data[7]=0x10; // Protocol version
			candata.data[5]=0x02; // MSG TYPE: WRITE
			candata.data[4]=0x00; // Secuence Number
			candata.data[0]=0xFF; // CRC

			if(candata.variable == 0x14){
				// enviar orden de encendido de luces al PIC2
				candata.id=(uint16_t)2; // ID destino
				candata.data[6]=(uint8_t)0; // ID origen
				candata.data[3] = 0x14; // Variable
				candata.data[2] = 0x00; // Value MSB
				candata.data[1] = candata.valL; // Value LSB
				can_send(can_fd, &candata, 0);
				fprintf(stderr,"encender luces\n");	
			}else if(candata.variable == 0x28){
				// enviar orden de luces AUTOMATICAS al PIC2
				candata.id=(uint16_t)2; // ID destino
				candata.data[6]=(uint8_t)0; // ID origen
				candata.data[3] = 0x28; // Variable
				candata.data[2] = 0x00; // Value MSB
				candata.data[1] = candata.valL; // Value LSB
				can_send(can_fd, &candata, 0);
			}else if(candata.variable == 0x13){
				// enviar orden de arranque al PIC4
				fprintf(stderr,"orden motor\n");
				candata.id=0x0004; // ID destino
				candata.dlc=CAN_DATA_LENGTH;
				candata.data[6]=0x00; // ID origen
				candata.data[3]=0x13; // Variable
				candata.data[2]=0x00; // Value MSB
				candata.data[1]=candata.valL; // Value LSB
				can_send(can_fd, &candata, 0);
			}else if(candata.variable == 0x19){
				// enviar orden de arranque al PIC3
				fprintf(stderr,"orden PARKING %d\n", candata.valL);
				candata.id=0x0003; // ID destino
				candata.dlc=CAN_DATA_LENGTH;
				candata.data[6]=0x00; // ID origen
				candata.data[3]=0x19; // Variable
				candata.data[2]=0x00; // Value MSB
				candata.data[1]=candata.valL;//0x01; // Value LSB
				can_send(can_fd, &candata, 0);
			}else if(candata.variable == 0x07){
				// enviar orden de encendido de wipers al PIC1
				candata.id=(uint16_t)1; // ID destino
				candata.data[6]=(uint8_t)0; // ID origen
				candata.data[3] = 0x07; // Variable
				candata.data[2] = 0x00; // Value MSB
				candata.data[1] = candata.valL; // Value LSB
				can_send(can_fd, &candata, 0);
				fprintf(stderr,"orden wipers\n");	
			}else if(candata.variable == 0x08){
				// enviar orden de wipers AUTOMATICAS al PIC1
				candata.id=(uint16_t)1; // ID destino
				candata.data[6]=(uint8_t)0; // ID origen
				candata.data[3] = 0x08; // Variable
				candata.data[2] = 0x00; // Value MSB
				candata.data[1] = candata.valL; // Value LSB
				can_send(can_fd, &candata, 0);
				fprintf(stderr,"orden wipers auto\n");	
			}
		}
	}
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
				candata.id=0x0004;//4; // ID destino
				candata.dlc=CAN_DATA_LENGTH;
				candata.data[7]=0x10; // Protocol version
				candata.data[6]=0x00;//4; // ID origen
				candata.data[5]=0x02; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
				candata.data[4]=0x00; // Secuence Number
				candata.data[3]=0x13; // Variable
				candata.data[2]=0x01; // Value MSB
				candata.data[1]=0x01; // Value LSB
				candata.data[0]=0xFF; // CRC
				
				can_send(can_fd, &candata, 0);
		}else if((keycode == KEY_p)||(keycode == KEY_P)){
			// enviar orden de paro
			printf("Engine stopped\n");
			candata.id=0x0004; // ID destino
			candata.dlc=CAN_DATA_LENGTH;
			candata.data[7]=0x10; // Protocol version
			candata.data[6]=0x00; // ID origen
			candata.data[5]=0x02; // MSG TYPE--> 0x00:TEST, 0x01:READ, 0x02:WRITE, 0x03:ERROR
			candata.data[4]=0x00; // Secuence Number
			candata.data[3]=0x13; // Variable
			candata.data[2]=0x00; // Value MSB
			candata.data[1]=0x00; // Value LSB
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
