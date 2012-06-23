

#define BAUDRATE B115200 			//Velocidad de la Plataforma Telosb
#define REMOTECONTROL_DEVICE "/dev/ttyUSB0"			//Puerto

#define FALSE 	0
#define TRUE 	1
#define BUTTON_ON	   0x55
#define BUTTON_OFF 	0x66

//Estados
#define E0 			100
#define E1 			101
#define TAMANO 	107
#define DATOS 		110
#define END 		113
//some defines
#define CAB_TINYOS	13
#define TAM_PAYLOAD	1
#define BUTTONSTATE	1
#define NUM_DEVICE 	3

typedef struct{
	unsigned char state_remote;
	unsigned char button;
}remote_t;



