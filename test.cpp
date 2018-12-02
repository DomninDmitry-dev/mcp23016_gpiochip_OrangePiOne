/*
 * test.cpp
 *
 *  Created on: Jul 2, 2018
 *      Author: dmitry
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

/*----------------------------------------------------------------------------*/
static int GPIOExport(int pin)
{
	char buffer[5];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open export for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, 5, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
static int GPIOUnexport(int pin)
{
	char buffer[5];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open unexport for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, 5, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
static int GPIODirection(int pin, int dir)
{
	static const char s_directions_str[]  = "in\0out";
	char path[35];
	int fd;

	snprintf(path, 35, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		return(-1);
	}

	if (write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3) == -1) {
		fprintf(stderr, "Failed to set direction!\n");
		return(-1);
	}

	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
static int GPIOWrite(int pin, int value)
{
	static const char s_values_str[] = "01";

	char path[30];
	int fd;

	snprintf(path, 30, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open gpio value for writing!\n");
		return(-1);
	}

	if (write(fd, &s_values_str[LOW == value ? 0 : 1], 1) != 1) {
		fprintf(stderr, "Failed to write value!\n");
		return(-1);
	}

	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
int getCommands(void)
{
	char comm;

	while(comm = getchar()) {
		//switch(comm)
		//{
		//case ''
		//}
	}
}
/*----------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	printf("Test module mcp23016\n");

	/*printf("argc = %d\n", argc);
	for(int i=0; i<argc; i++)
		printf("argc[%d]; argv = %d\n", i, atoi(argv[i]));
	printf("---------------------------------------------\n");*/

	if(argc > 1)
	{
		/*
		 * Enable GPIO pins
		 */
		for(int i=1; i<argc; i++) {
			if(GPIOExport(atoi(argv[i])) == -1) {
				fprintf(stderr, "Error export port: %d\n", atoi(argv[i]));
				return(1);
			}
			printf("Export port: %d is OK!\n", atoi(argv[i]));
		}
	}
	else printf("No port number entered!\n");

	return getCommands();
}
/*----------------------------------------------------------------------------*/
