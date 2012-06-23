
#include "RemoteControl.h"

void SerialPort_Config(int *fd, struct termios *newtio, struct termios *oldtio){

	/* save current port settings */
	tcgetattr(*fd, oldtio); 

	/* set new port settings for canonical input processing */
	newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio->c_iflag = IGNPAR;
	//newtio.c_iflag = 0;
	newtio->c_oflag = 0;
	newtio->c_lflag = 0;
	newtio->c_cc[VMIN] = (CAB_TINYOS + TAM_PAYLOAD); //MIN sets the number of characters to receive before the read is satisfied
	newtio->c_cc[VTIME] = 0; //(Time = (VTIME * 0.1) s)
	
	//Con esta funcion cargamos la nueva configuracion al puerto serial
	tcsetattr(*fd, TCSANOW, newtio);
	usleep(200000);	
	//ceramos el buffer	
	tcflush(*fd, TCIFLUSH);

}

int find_max(int remote){
	int max;

	max = gps_serial_fd;

//	printf("gps: %d\tcan: %d\tremote: %d\n", gps_serial_fd, can_fd, remote);

	if(can_fd > max) max = can_fd;
	if(remote > max) max = remote;

	return max;

}


/*
	Función donde obtenemos el PAYLOAD de los paquetes recibidos por el PUERTO SERIE
		- buf[] = vector donde se encuentran los datos recibidos por el puerto serie
		- res = cantidad de bytes recibidos en el Puerto Serie
		- state = ultimo estado en el que estuvimos
*/
remote_t obtener_payload(unsigned char buf[], const unsigned char res, remote_t remote_var){
	static unsigned char flag = FALSE;//, button= BUTTON_OFF;
	static unsigned char tam = 0, cantdatos = 0;
	unsigned int i = 0;

	for(i=0; i<res; i++){

		if(remote_var.state_remote == E1)
			if(buf[i] == 0x7e) remote_var.state_remote = E0;

		if((remote_var.state_remote != E0) && (remote_var.state_remote != END)){
			if(buf[i] == 0x7D){
				flag = TRUE;
			}else{
				if(flag == TRUE){
					flag = FALSE;
					if(buf[i] == 0x5E) buf[i] = 0x7E;
					else if(buf[i] == 0x5D)buf[i] = 0x7D;
				}
			}
		}

		if(flag == FALSE){
			switch(remote_var.state_remote){
			case E0:
				//Framing byte
				if(buf[i] == 0x7E) remote_var.state_remote = E1;
				//printf("/****************MSG recibido**************/\n");
				break;

			case TAMANO:
				tam = buf[i];
				printf("Tam:%d\t",tam);
				if(!tam) remote_var.state_remote = E0;
				else remote_var.state_remote++;
				break;
			
			case DATOS:
				if(cantdatos < tam){
					cantdatos++;
					if(cantdatos == tam) remote_var.state_remote++;
					printf("Data:0x%x\n", buf[i]);
					switch(cantdatos){
					case BUTTONSTATE:
						if(remote_var.button == BUTTON_ON){
							remote_var.button = BUTTON_OFF;
							printf("Security OFF\n");
						}else{
							remote_var.button = BUTTON_ON;
							printf("Security ON\n");
						}
						break;
					default:
						printf("Error:%d", buf[i]);
						break;
					}
				}
				break;

			case END:
				if(buf[i] == 0x7e){ 
					remote_var.state_remote = E0;
				}else remote_var.state_remote = END;
				cantdatos = tam = 0;
			default:
				remote_var.state_remote++;
			}//END Switch
		}//END (FLAG == TRUE)

	} //END FOR	

	//printf("State:%d\n", state);
	return remote_var;
}
