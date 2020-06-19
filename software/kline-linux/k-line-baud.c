#include <stdlib.h>

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#define DESIRED_BAUD	10400

struct serial_struct old_serinfo;
struct serial_struct new_serinfo;
struct termios old_termios;
struct termios new_termios;

int main(int argc, char **argv)
{
	int fd;
	char *device;

	if (argc < 2)
	{
		fprintf(stderr, "usage: %s device\n", argv[0]);
		exit(2);
	}
	device = argv[1];

	/* Open device */
	if ((fd = open(device, O_RDWR|O_NONBLOCK)) < 0)
	{
		perror(device);
		exit(1);
	}

	/* Get old serial settings. */
	if (ioctl(fd, TIOCGSERIAL, &old_serinfo) < 0)
	{
		perror("Cannot get serial info");
		exit(1);
	}

	/* Change settings for custom baud rate. */
	new_serinfo = old_serinfo;
	new_serinfo.custom_divisor
		= new_serinfo.baud_base / DESIRED_BAUD;
	new_serinfo.flags = (new_serinfo.flags & ~ASYNC_SPD_MASK)
		| ASYNC_SPD_CUST;

	/* Update serial settings with changes. */
	if (ioctl(fd, TIOCSSERIAL, &new_serinfo) < 0)
	{
		perror("Cannot set serial info");
		exit(1);
	}

	/* Get old terminal attributes. */
	if (tcgetattr(fd, &old_termios) < 0)
	{
		perror("Cannot get terminal attributes");
		exit(1);
	}

	/* Change settings to 38400 baud.  The driver will
	 * substitute this with the custom baud rate.  */
	new_termios = old_termios;
	cfsetispeed(&new_termios, B0);
	cfsetospeed(&new_termios, B38400);

	/* Update terminal attributes. */
	if (tcsetattr(fd, TCSANOW, &new_termios) < 0)
	{
		perror("Cannot set terminal attributes");
		exit(1);
	}


	while(1) {
        char tx_buffer[256];
        tx_buffer[0] = 0x33;

        write(fd, tx_buffer, 1);
        sleep(1);

	}

	return 0;
}
