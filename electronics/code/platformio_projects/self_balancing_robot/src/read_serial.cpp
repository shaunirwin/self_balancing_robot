#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

struct TestPacket_t {
    float testVal;
};

// Function to configure the serial port
int openSerialPort(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes" << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = CS8 | CLOCAL | CREAD;     // 8-bit data, ignore modem control lines, enable receiver
    tty.c_iflag = IGNPAR;   // ignore parity errors
    tty.c_oflag = 0;        // no output processing
    tty.c_lflag = 0;        // disables canonical mode (therefore enables raw mode), echo, and other line processing features

    tcflush(fd, TCIFLUSH);  // flush input buffer
    tcsetattr(fd, TCSANOW, &tty);   // apply the settings immediately

    return fd;
}

int main() {
    const char* serialPort = "/dev/ttyACM0";
    int fd = openSerialPort(serialPort);
    if (fd == -1) return -1;

    TestPacket_t packet;
    while (true) {
        int bytesRead = read(fd, &packet, sizeof(packet));

        if (bytesRead == sizeof(packet)) {
            std::cout << "Received testVal: " << packet.testVal << std::endl;
        } else {
            std::cerr << "Incomplete packet received, bytes read: " << bytesRead << std::endl;
        }

        auto sleep_ms = 10;
        usleep(sleep_ms * 1000);  // Sleep before reading again
    }

    close(fd);
    return 0;
}
