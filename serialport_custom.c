#include <termio.h>
#include <err.h>
#include <linux/serial.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
    switch(baudrate) {
        B(50);     B(75);     B(110);    B(134);    B(150);
        B(200);    B(300);    B(600);    B(1200);   B(1800);
        B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
        B(57600);  B(115200); B(230400); B(460800); B(500000);
        B(576000); B(921600); B(1000000);B(1152000);B(1500000);
        default: return 0;
    }
#undef B
}

/* Open serial port in raw mode, with custom baudrate if necessary */
int serial_open(const char *device, int rate)
{
    struct termios options;
    struct serial_struct serinfo;
    int fd;
    int speed = 0;

    /* Open and configure serial port */
    if ((fd = open(device,O_RDWR|O_NOCTTY)) == -1)
        return -1;

    speed = rate_to_constant(rate);

    if (speed == 0) {
        /* Custom divisor */
        serinfo.reserved_char[0] = 0;
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            return -1;
        serinfo.flags &= ~ASYNC_SPD_MASK;
        serinfo.flags |= ASYNC_SPD_CUST;
        serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
        if (serinfo.custom_divisor < 1)
            serinfo.custom_divisor = 1;
        if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
            return -1;
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            return -1;
        if (serinfo.custom_divisor * rate != serinfo.baud_base) {
            warnx("actual baudrate is %d / %d = %f",
                    serinfo.baud_base, serinfo.custom_divisor,
                    (float)serinfo.baud_base / serinfo.custom_divisor);
        }
    }

    fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);
    cfsetispeed(&options, speed ?: B38400);
    cfsetospeed(&options, speed ?: B38400);
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 10;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &options) != 0)
        return -1;
    return fd;
}

int main(void){

    std::cout << "Serial Port Program" << std::endl;
    int fd = serial_open("/dev/ttyUSB0", 307200);
    if(fd == -1) /* Error Checking */
        std::cout << "Error! in Opening ttyUSB0" << std::endl;
    else
        std::cout << "ttyUSB0 Opened Successfully" << std::endl;

    char write_buffer[] = "hoge";
    int bytes_written = write(fd,write_buffer,sizeof(write_buffer));
    std::cout << write_buffer << " written to tty USB0 " << std::endl;
    std::cout << bytes_written << " Bytes written to ttyUSB0" << std::endl;

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer             */
    char read_buffer[32];    /* Buffer to store the data received              */
    int bytes_read = read(fd,&read_buffer, 3); /* Read the data (3 bytes) */
    std::cout << " Bytes Read -" << bytes_read << std::endl;
    std::cout << read_buffer << std::endl;

    close(fd);
    return 0;
}
