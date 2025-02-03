#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyACM0"  // Adjust this to your serial device
#define BAUDRATE B115200


// struct DataPacket {
//     // float value;
//     // bool flag;
//     char a;
//     char b;
//     char c;
// } __attribute__((packed));      // ensure no padding

typedef struct {
  float testVal;
  char c;
  bool b;
} __attribute__((packed))  DataPacket;


int configureSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);  // Open serial port
    if (fd == -1) {
        std::cerr << "Error opening serial port!" << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes!" << std::endl;
        close(fd);
        return -1;
    }

    // Set baud rate
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // Configure for raw mode
    tty.c_cflag &= ~PARENB;        // No parity bit
    tty.c_cflag &= ~CSTOPB;        // One stop bit
    tty.c_cflag &= ~CSIZE;         
    tty.c_cflag |= CS8;            // 8-bit characters
    // tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading & ignore modem control lines

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode (disable canonical, echo, signals)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // Disable flow control and newline translation
    tty.c_oflag &= ~OPOST;         // Disable output processing

    tty.c_cc[VMIN] = 1;            // Minimum 1 character per read
    tty.c_cc[VTIME] = 1;           // Timeout for read in tenths of a second

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes!" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

void readSerial(int fd) {
    const uint DATA_LENGTH = sizeof(DataPacket);
    const auto PACKET_LENGTH = DATA_LENGTH + 2;
    char buffer[PACKET_LENGTH];  // We expect 3 characters "!X@"
    int index = 0;

    while (true) {
        char c;
        int n = read(fd, &c, 1);  // Read one byte

        if (n <= 0) {
            continue;
        }
        
        if ((index == 0) && (c != '!')) {
            continue;
        }
        
        {

            buffer[index++] = c;

            // Check if we received "!X@"
            if (index == PACKET_LENGTH) {
                std::cout << "whole buffer:" << buffer << std::endl;
                if (buffer[0] == '!' && buffer[PACKET_LENGTH-1] == '@') {
                    DataPacket data;
                    std::memcpy(&data, &buffer[1], sizeof(DataPacket));

                    std::cout << "Received valid message: " << data.testVal << ", " << data.b << ", " << data.c << std::endl;
                } else {
                    std::cerr << "Invalid packet received" << std::endl;
                }
                index = 0; // Reset buffer
            }
        } 
    }
}

int main() {
    int serial_fd = configureSerial(SERIAL_PORT);
    if (serial_fd == -1) {
        return -1;  // Exit if the serial port cannot be opened
    }

    readSerial(serial_fd);

    close(serial_fd);
    return 0;
}
