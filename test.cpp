/*
 * test.cpp
 *
 *  Created on: Jul 2, 2018
 *      Author: dmitry
 */

#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdio>

using namespace std;

/*----------------------------------------------------------------------------*/
#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

/*----------------------------------------------------------------------------*/
static unsigned short int offset = 0;
static char name[20];
static char ngpio[10];
static const char menustr[] = {
"1 - Export pin\n\
2 - Unexport pin\n\
3 - Set direction pin\n\
4 - Read 0/1 frot pin\n\
5 - Write 0/1 to pin\n\
q - quit"
};

/*----------------------------------------------------------------------------*/
static int GPIOExport(int pin)
{
	char buffer[5];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open export for writing!\n");
		return 1;
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
		return 1;
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
		return 1;
	}

	if (write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3) == -1) {
		fprintf(stderr, "Failed to set direction!\n");
		return 1;
	}

	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
static int GPIORead(int pin)
{
	char path[30];
	char value_str[3];
	int fd;

	snprintf(path, 30, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open gpio value for reading!\n");
		return 1;
	}

	if (read(fd, value_str, 3) == -1) {
		fprintf(stderr, "Failed to read value!\n");
		return 1;
	}

	close(fd);

	return(atoi(value_str));
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
		return 1;
	}

	if (write(fd, &s_values_str[LOW == value ? 0 : 1], 1) != 1) {
		fprintf(stderr, "Failed to write value!\n");
		return 1;
	}

	close(fd);
	return(0);
}
/*----------------------------------------------------------------------------*/
static int getCommands(void)
{
	char menu, comm[5];
	int pin = 0;

	// Input char
	//while(comm = getchar()) {
	//	if(comm == 0x0D) break;
	//	string[pos++] = comm;
	//}

	printf("%s\n", menustr);
	do {
		printf("Put number from menu: ");
		cin >> menu;
		switch(menu) {
		case '1': { // Export
				while(1) {
					printf("Export put number pin (q-quit): ");
					cin >> comm;
					if(comm[0] == 'q') break;
					pin = atoi(comm);
					if((pin < offset) || (pin > offset + atoi(ngpio))) {
						fprintf(stderr, "Error number pin!\n");
					}
					if(GPIOExport(pin) == -1) {
						fprintf(stderr, "Error export port: %d\n", pin);
					}
				}
				break;
			}
		case '2': { // Unexport
				while(1) {
					printf("Unexport put number pin (q-quit): ");
					cin >> comm;
					if(comm[0] == 'q') break;
					pin = atoi(comm);
					if((pin < offset) || (pin > offset + atoi(ngpio))) {
						fprintf(stderr, "Error number pin!\n");
					}
					if(GPIOUnexport(pin) == -1) {
						fprintf(stderr, "Error export port: %d\n", pin);
					}
				}
				break;
			}
		case '3': { // Direction
				char dir[5] = {0};
				while(1) {
					printf("Put number pin (q-quit): ");
					cin >> comm;
					if(comm[0] == 'q') break;
					pin = atoi(comm);
					if((pin < offset) || (pin > offset + atoi(ngpio))) {
						fprintf(stderr, "Error number pin!\n");
					}
					else {
						printf("Input direction in/out to number pin %d: ", pin);
						cin >> dir;
						if( (strstr(dir, "in") == 0) || (strstr(dir, "out") == 0) )
						{
							if(GPIODirection(pin, dir[0] == 'i' ? 0 : 1) == -1) {
								fprintf(stderr, "Error direction pin: %d\n", pin);
							}
						}
						else fprintf(stderr, "Error input direction!\n");
					}
				}
				break;
			}
		case '4': { // Read
				while(1) {
					int value;
					printf("Put number pin (q-quit): ");
					cin >> comm;
					if(comm[0] == 'q') break;
					pin = atoi(comm);
					if((pin < offset) || (pin > offset + atoi(ngpio))) {
						fprintf(stderr, "Error number pin!\n");
					}
					else {
						while(1) {
							printf("Press any keys for read state from number pin %d (q-quit) ", pin);
							comm[0] = getchar();
							if(comm[0] == 'q') break;
							value = GPIORead(pin);
							if(value == -1) {
								fprintf(stderr, "Error read from pin: %d\n", pin);
							}
							else {
								printf("Pin %d = %d\n", pin, value);
							}
						}
					}
				}
				break;
			}
		case '5': { // Write
				int value = 0;
				while(1) {
					printf("Put number pin (q-quit): ");
					cin >> comm;
					if(comm[0] == 'q') break;
					pin = atoi(comm);
					if((pin < offset) || (pin > offset + atoi(ngpio))) {
						fprintf(stderr, "Error number pin!\n");
					}
					else {
						while(1) {
							printf("Input value 0/1 to write to number pin %d (q-quit): ", pin);
							cin >> comm;
							if(comm[0] == 'q') break;
							value = atoi(comm);
							if(GPIOWrite(pin, value == 0 ? 0 : 1) == -1) {
								fprintf(stderr, "Error write to pin: %d\n", pin);
							}
						}
					}
				}
				break;
			}
		}
	}
	while(menu != 'q');
	return 0;
}
/*----------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	printf("Test module mcp23016\n");

	/*printf("---------------------------------------------\n");
	printf("argc = %d\n", argc);
	for(int i=0; i<argc; i++)
		printf("argc[%d]; argv = %d\n", i, atoi(argv[i]));
	printf("---------------------------------------------\n");*/

	if(argc == 2)
	{
		int fd;
		char path[50];
		// Set offset gpiochip
		offset = atoi(argv[1]);
		sprintf(name, "gpiochip%d", offset);
		printf("%s\n", name);
		sprintf(path, "/sys/class/gpio/%s/ngpio", name);
		fd = open(path, O_RDONLY);
		if(read(fd, ngpio, 10) == -1) {
			fprintf(stderr, "Failed from read ngpio! path: %s\n", path);
			close(fd);
			return 1;
		}
		printf("ngpio: %s\n", ngpio);
		close(fd);
	}
	else {
		printf("No offset pins entered!\n");
		return 0;
	}
	return getCommands();
}
/*----------------------------------------------------------------------------*/
