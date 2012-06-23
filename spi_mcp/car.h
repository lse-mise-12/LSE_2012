#ifndef _CAR_H
#define _CAR_H

#define MAX_LENGHT_TOKENS 64
#define MAX_CAN_TOKENS  20

#define GPS_PORT	"/dev/ttyUSB1"
#define GPS_BITRATE	4800
#define GPS_FREQ	2

#define CAN_DEV		"/root/spi_if"
void thread_can_handler(void *arg);
void thread_GPS(void *arg);
void GPS_thread_clean_up(void *arg);
void handle_message(char *message);

void clean_exit(int ret);
void signal_handler(int signum);
int mystrparser(char datos[][MAX_LENGHT_TOKENS], char *cadena, char separador);
// puesto para pruenas
void thread_input(void *arg);
int mygetch(void);
void GPS_parser(void);
int open_GPS(char *gps_port, int gps_bitrate);

#endif
