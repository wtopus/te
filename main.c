/*
 * main.c
 *
 * Created on: 2014. 4. 12.
 * Author: Seokyong Hong
 */

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "protocol.h"

#define SERIAL_PORT		"/dev/ttyACM0"
#define SERIAL_BAUD_RATE	B19200

static int serial_fd;
static int socket_fd;
static int thread_exit;
static struct termios options;
static struct sockaddr_in server_addr;
static struct sockaddr_in client_addr;
static pthread_mutex_t mutex;

static int init_serial(void);
static int init_socket(void);

static int process_command(char *message, int length);
static void *distance_monitor(void *data);

int main(int argc, char *argv[]) {
	int length, status;
	socklen_t sock_length;
	char message[1024];
	pthread_t monitor_handle;

	if(init_serial() || init_socket()) {
		fprintf(stderr, "Robot control server cannot run.\n");
		return 0;
	}

	if(pthread_mutex_init(&mutex, NULL)) {
		fprintf(stderr, "Mutex cannot be initialized.");
	}
	else {
		if(pthread_create(&monitor_handle, NULL, distance_monitor, (void *)NULL) < 0) {
			fprintf(stderr, "Distance monitoring thread cannot be launched.");
		}
	}

	printf("Robot control server starts.\n");

	while(1) {
		sock_length = sizeof(client_addr);
		length = recvfrom(socket_fd, message, sizeof(message), 0, (struct sockaddr *)&client_addr, &sock_length);
		if(length > 0)
			process_command(message, length);
	}

	thread_exit = 1;
	pthread_join(monitor_handle, (void **)&status);
	pthread_mutex_destroy(&mutex);
	close(serial_fd);
	close(socket_fd);

	return 0;
}

int init_serial(void) {
	serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
	if(serial_fd < 0) {
		fprintf(stderr, "Unable to open %s: %s.\n", SERIAL_PORT, strerror(errno));
		return serial_fd;
	}

	fcntl(serial_fd, F_SETFL, FNDELAY);
	tcgetattr(serial_fd, &options);
	cfsetispeed(&options, SERIAL_BAUD_RATE);
	cfsetospeed(&options, SERIAL_BAUD_RATE);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;
	options.c_lflag &= ~(ICANON | ECHO | ISIG);
	tcsetattr(serial_fd, TCSANOW, &options);

	return 0;
}

int init_socket(void) {
	socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(socket_fd < 0) {
		fprintf(stderr, "Unable to open datagram socket: %s.\n", strerror(errno));
		return socket_fd;
	}

	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(2048);

	bind(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));

	return 0;
}

int process_command(char *message, int length) {
	if(length != 4 || !message) {
		fprintf(stderr, "Unable to recognize datagram message.\n");
		return 1;
	}

	switch(message[1]) {
	case PROTOCOL_TYPE_MOTOR:
		pthread_mutex_lock(&mutex);
		write(serial_fd, message, 4);
		pthread_mutex_unlock(&mutex);
		break;
	}

	return 0;
}

void *distance_monitor(void *data) {
	unsigned char request[] = { PROTOCOL_OPEN_CHARACTER, PROTOCOL_TYPE_SENSOR, PROTOCOL_SENSOR_ULTRASONIC, PROTOCOL_CLOSE_CHARACTER };
	unsigned char response[4];

	while(!thread_exit) {
		if(client_addr.sin_port) {
			pthread_mutex_lock(&mutex);
			write(serial_fd, request, 4);
			read(serial_fd, response, sizeof(response));
			pthread_mutex_unlock(&mutex);
			sendto(socket_fd, response, sizeof(response), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
		}
		sleep(2);
	}

	return (void *)1;
}
