#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>

#include "data_structs.h"

#define SERIAL_PORT "/dev/ttyACM0"  // Adjust this to your serial device
#define BAUDRATE B115200

const uint ENCODER_PULSES_PER_REVOLUTION = 700*2;   // detects rising and falling edge of each pulse



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
    const uint DATA_LENGTH = sizeof(DataPacket_t);
    const auto PACKET_LENGTH = HEADER_LENGTH + DATA_LENGTH + 2;
    char buffer[PACKET_LENGTH];  // We expect "!<header packet><data packet>@"
    int index = 0;

    int64_t microSecondsSinceBootPrevious = 0;
    int motor1PulsesPrevious = 0;
    int motor2PulsesPrevious = 0;

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
                    DataPacket_t data;
                    
                    std::memcpy(&header, &buffer[1], sizeof(PacketHeader_t));
                    std::memcpy(&data, &buffer[HEADER_LENGTH + 1], sizeof(DataPacket_t));

                    const float timeDeltaSec = (header.microSecondsSinceBoot - microSecondsSinceBootPrevious) / 1e6;
                    const float motor1PulsesPerSec = 1.f * (data.state.motor1EncoderPulses - motor1PulsesPrevious) / timeDeltaSec;
                    const float motor2PulsesPerSec = 1.f * (data.state.motor2EncoderPulses - motor2PulsesPrevious) / timeDeltaSec;

                    std::cout << "Received message: " << header.packetID << ", " << (int) (header.microSecondsSinceBoot / 1e6) << "sec (" << std::setprecision(3) << std::setfill('0') << (1.f/timeDeltaSec) << "Hz):" << 
                        data.state.motor1EncoderPulses << " M1 pulses (" << std::round(motor1PulsesPerSec) << " pulses/sec), " << 
                        data.state.motor2EncoderPulses << " M2 pulses (" << std::round(motor2PulsesPerSec) << " pulses/sec), " << 
                        std::fixed  << std::internal <<  std::showpos << std::setw(6) << std::setprecision(2) << std::setfill(' ') <<
                        "gy: " << data.pitchInfo.gy * 180. / M_PI << " deg/s, " << 
                        "calib: " << data.pitchInfo.isCalibrated << ", " <<
                        "gyOffset: " << data.pitchInfo.gyroOffsetY * 180. / M_PI << " deg/s, " << 
                        "gyVel: " << data.pitchInfo.pitchVelocityGyro * 180. / M_PI << " deg/s, " << 
                        std::endl;
                    
                    microSecondsSinceBootPrevious = header.microSecondsSinceBoot;
                    motor1PulsesPrevious = data.state.motor1EncoderPulses;
                    motor2PulsesPrevious = data.state.motor2EncoderPulses;
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
