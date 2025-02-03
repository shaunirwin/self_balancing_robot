#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyACM0"  // Adjust this to your serial device
#define BAUDRATE B115200

typedef struct {
  long long packetID;
  int64_t microSecondsSinceBoot;
} __attribute__((packed)) PacketHeader_t;

// typedef struct {
//   float testVal;
//   char c;
//   bool b;
// } __attribute__((packed))  DataPacket;

typedef struct {
  // IMU estimates
  float pitch_accel;
  float pitch_gyro;
  float pitch_est;

  // wheel encoder measurements
  // float motor1DistanceMeas;
  uint motor1EncoderPulses;         // uint on this platform is long on esp32
  uint motor1EncoderPulsesDelta;
  // unsigned char motor1DirMeas;

  // float motor2DistanceMeas;
  uint motor2EncoderPulses;
  uint motor2EncoderPulsesDelta;
  // unsigned char motor2DirMeas;

  // gyro angular velocity measurement
  float pitch_velocity_gyro;

} __attribute__((packed)) DataPacket; //StateEstimatePacket_t;


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
    const uint HEADER_LENGTH = sizeof(PacketHeader_t);
    const uint DATA_LENGTH = sizeof(DataPacket);
    const auto PACKET_LENGTH = HEADER_LENGTH + DATA_LENGTH + 2;
    char buffer[PACKET_LENGTH];  // We expect "!<header packet><data packet>@"
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

            if (index == PACKET_LENGTH) {
                if (buffer[0] == '!' && buffer[PACKET_LENGTH-1] == '@') {
                    PacketHeader_t header;
                    DataPacket data;
                    std::memcpy(&header, &buffer[1], sizeof(PacketHeader_t));
                    std::memcpy(&data, &buffer[HEADER_LENGTH + 1], sizeof(DataPacket));

                    std::cout << "Received valid message: " << header.packetID << ":" << data.pitch_accel << ", " << data.motor1EncoderPulses << ", " << data.motor2EncoderPulses << std::endl;
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
