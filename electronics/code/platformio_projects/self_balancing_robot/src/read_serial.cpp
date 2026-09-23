#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <fstream>
#include <vector>
#include <chrono>
#include <cerrno>
#include <poll.h>

#include "data_structs.h"

#define SERIAL_PORT "/dev/ttyACM0"  // Adjust this to your serial device
#define BAUDRATE B115200



// typedef struct {
//     StateEstimatePacket_t state;
//     Contr

// } __attribute__((packed)) LogPacket_t;


typedef struct {
    long long packetID;

    int64_t microSecondsSinceBoot;

    float pitch_est;    // estimated pitch angle [rad]

    float motor1EncoderPulsesPerSec;
    float motor2EncoderPulsesPerSec;

    uint8_t dutyCycle1;
    uint8_t dutyCycle2;

    uint32_t controlIntervalUs;
    float averageControlIntervalUs;
    float averageAbsControlJitterUs;
    uint32_t maxAbsControlJitterUs;

    std::string toCSVRow() const {
      std::stringstream csvRow;
      csvRow << packetID << "," << microSecondsSinceBoot / 1000 << std::fixed << std::setprecision(3) << "," << 
        pitch_est * 180./M_PI << "," << motor1EncoderPulsesPerSec << "," << motor2EncoderPulsesPerSec << "," << 
        static_cast<int>(dutyCycle1) << "," << static_cast<int>(dutyCycle2) << "," <<
        controlIntervalUs << "," << averageControlIntervalUs << "," <<
        averageAbsControlJitterUs << "," << maxAbsControlJitterUs << "\n";
      return csvRow.str();
    }

    std::string getCSVHeader() const {
      std::string csvHeader{ "packetID,timeMs,pitch_est deg,motor1EncoderPulsesPerSec,motor2EncoderPulsesPerSec,dutyCycle1,dutyCycle2,controlIntervalUs,averageControlIntervalUs,averageAbsControlJitterUs,maxAbsControlJitterUs\n" };
      return csvHeader;
    }

} CSVRow_t;


template<typename T>
void writeBinaryData(const std::string& csvPath, const std::vector<T>& logPackets) {
    std::ofstream file(csvPath, std::ios::binary | std::ios::app);
    
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    file.write(reinterpret_cast<const char*>(logPackets.data()), logPackets.size());
}


void writeCSVHeader(const std::string& csvPath, const std::vector<CSVRow_t>& csvRows) {
    std::ofstream file(csvPath);
    
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    file << csvRows[0].getCSVHeader();
}


void writeCSVRows(const std::string& csvPath, const std::vector<CSVRow_t>& csvRows) {
    std::ofstream file(csvPath, std::ios::app);      // append mode
    
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    for (const auto csvRow : csvRows) {
        file << csvRow.toCSVRow();
    }
}



int configureSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);  // Open serial port
    if (fd == -1) {
        std::cerr << "Error opening " << port << ": " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error reading settings for " << port << ": "
                  << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // Disable all terminal input/output translations so arbitrary binary packet
    // bytes arrive unchanged.
    cfmakeraw(&tty);

    // Set baud rate
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | HUPCL);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error configuring " << port << ": "
                  << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

int readSerial(int fd, bool logToCSV, std::string csvPath) {

    const uint HEADER_LENGTH = sizeof(PacketHeader_t);
    const uint DATA_LENGTH = sizeof(DataPacket_t);
    const auto PACKET_LENGTH = HEADER_LENGTH + DATA_LENGTH + 2;
    constexpr int TELEMETRY_TIMEOUT_MS = 5000;
    char buffer[PACKET_LENGTH];  // We expect "!<header packet><data packet>@"
    int index = 0;
    uint packetsReceived = 0;
    uint64_t bytesReceived = 0;
    auto lastValidPacketTime = std::chrono::steady_clock::now();

    int64_t microSecondsSinceBootPrevious = 0;
    int motor1PulsesPrevious = 0;
    int motor2PulsesPrevious = 0;

    // to be logged to CSV
    const auto samplesPerLog = 500;
    std::vector<IMUPacket_t> imuPackets;
    std::vector<CSVRow_t> csvRows;

    while (true) {
        pollfd serialPoll {
            .fd = fd,
            .events = POLLIN,
            .revents = 0,
        };
        const int pollResult = poll(&serialPoll, 1, TELEMETRY_TIMEOUT_MS);

        if (pollResult < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "Error waiting for serial data: " << std::strerror(errno) << std::endl;
            return 1;
        }

        if (pollResult == 0) {
            if (packetsReceived == 0) {
                std::cerr << "No data received from " << SERIAL_PORT << " within "
                          << TELEMETRY_TIMEOUT_MS / 1000 << " seconds. Check the USB connection, "
                          << "port name, firmware, and that no serial monitor is using the port."
                          << std::endl;
            } else {
                std::cerr << "Telemetry stopped: no serial data received for "
                          << TELEMETRY_TIMEOUT_MS / 1000 << " seconds." << std::endl;
            }
            return 1;
        }

        if (serialPoll.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::cerr << "Serial connection to " << SERIAL_PORT << " was lost." << std::endl;
            return 1;
        }

        char c;
        int n = read(fd, &c, 1);  // Read one byte

        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Error reading " << SERIAL_PORT << ": "
                          << std::strerror(errno) << std::endl;
                return 1;
            }
            continue;
        }
        if (n == 0) {
            continue;
        }
        bytesReceived++;
        
        if ((index == 0) && (c != STX)) {
            continue;
        }
        
        {
            buffer[index++] = c;

            if (index == PACKET_LENGTH) {
                if (buffer[0] == STX && buffer[PACKET_LENGTH-1] == ETX) {
                    PacketHeader_t header;
                    DataPacket_t data;
                    
                    std::memcpy(&header, &buffer[1], sizeof(PacketHeader_t));
                    std::memcpy(&data, &buffer[HEADER_LENGTH + 1], sizeof(DataPacket_t));

                    const float timeDeltaSec = (header.microSecondsSinceBoot - microSecondsSinceBootPrevious) / 1e6;
                    const float motor1PulsesPerSec = 1.f * (data.state.motor1EncoderPulses - motor1PulsesPrevious) / timeDeltaSec;
                    const float motor2PulsesPerSec = 1.f * (data.state.motor2EncoderPulses - motor2PulsesPrevious) / timeDeltaSec;

                    if (packetsReceived == 0) {
                        std::cout << "Telemetry connection established." << std::endl;
                    }

                    if (packetsReceived % 1 == 0) {
                        std::cout << "Received message: " << header.packetID << ", " << (int) (header.microSecondsSinceBoot / 1e6) << "sec (" << std::setprecision(3) << std::setfill('0') << (1.f/timeDeltaSec) << "Hz):" << 
                        data.state.motor1EncoderPulses << " M1 pulses (" << std::round(motor1PulsesPerSec) << " pulses/sec), " << 
                        data.state.motor2EncoderPulses << " M2 pulses (" << std::round(motor2PulsesPerSec) << " pulses/sec), " << 
                        "control dt " << data.controlTiming.interval_us << " us, " <<
                        std::fixed << std::setprecision(1) <<
                        "avg " << data.controlTiming.average_interval_us << " us, " <<
                        "avg |jitter| " << data.controlTiming.average_abs_jitter_us << " us, " <<
                        "max |jitter| " << data.controlTiming.max_abs_jitter_us << " us" <<
                        // "ax: " << data.imu.ax << " m/s^2, " << 
                        // "az: " << data.imu.az << " m/s^2, " << 
                        // // "gy: " << data.imu.gy * 180. / M_PI << " deg/s, " << 
                        // "gy calib: " << data.pitchInfo.pitchVelocityGyro * 180. / M_PI << " deg/s, " << 
                        // "calib: " << data.pitchInfo.isCalibrated << ", " <<
                        // "gyOffset: " << data.pitchInfo.gyroOffsetY * 180. / M_PI << " deg/s, " << 
                        // "gyVel: " << data.pitchInfo.pitchVelocityGyro * 180. / M_PI << " deg/s, " << 
                        // "pitch accel: " << data.pitchInfo.pitchAccel * 180. / M_PI << " deg, " << 
                        // "pitch gyro: " << data.pitchInfo.pitchGyro * 180. / M_PI << " deg, " << 
                        // "pitch est: " << data.pitchInfo.pitchEst * 180. / M_PI << " deg, " << 
                        // "temp: " << data.imu.temp << " deg C" <<
                        std::endl;
                    }
                    
                    microSecondsSinceBootPrevious = header.microSecondsSinceBoot;
                    motor1PulsesPrevious = data.state.motor1EncoderPulses;
                    motor2PulsesPrevious = data.state.motor2EncoderPulses;

                    if (logToCSV) {
                        // imuPackets.push_back(data.imu);

                        // if (imuPackets.size() == samplesPerLog) {
                        //     writeBinaryData(csvPath, imuPackets);
                        //     imuPackets.clear();
                        // }

                        CSVRow_t csvRow {
                            .packetID = header.packetID,
                            .microSecondsSinceBoot = header.microSecondsSinceBoot,
                            .pitch_est = data.state.pitch_est,
                            // .motor1EncoderPulsesPerSec = motor1PulsesPerSec,
                            // .motor2EncoderPulsesPerSec = motor2PulsesPerSec,
                            .dutyCycle1 = data.control.motorOutput.dutyCycle1,
                            .dutyCycle2 = data.control.motorOutput.dutyCycle2,
                            .controlIntervalUs = data.controlTiming.interval_us,
                            .averageControlIntervalUs = data.controlTiming.average_interval_us,
                            .averageAbsControlJitterUs = data.controlTiming.average_abs_jitter_us,
                            .maxAbsControlJitterUs = data.controlTiming.max_abs_jitter_us,
                        };

                        csvRows.push_back(csvRow);

                        if (packetsReceived == 0) {
                            writeCSVHeader(csvPath, csvRows);
                        }

                        if (csvRows.size() == samplesPerLog) {
                            writeCSVRows(csvPath, csvRows);
                            csvRows.clear();
                            std::cout << "logging to csv..." << std::endl;
                        }
                    }

                    packetsReceived ++;
                    lastValidPacketTime = std::chrono::steady_clock::now();
                } else {
                    std::cerr << "Invalid packet received" << std::endl;
                }
                index = 0; // Reset buffer
            }
        }

        const auto timeSinceValidPacket = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - lastValidPacketTime);
        if (timeSinceValidPacket.count() >= TELEMETRY_TIMEOUT_MS) {
            if (packetsReceived == 0 && bytesReceived > 0) {
                std::cerr << "Serial bytes are arriving from " << SERIAL_PORT
                          << ", but no valid telemetry packet was decoded within "
                          << TELEMETRY_TIMEOUT_MS / 1000 << " seconds. Rebuild and upload the firmware "
                          << "so its packet format matches this reader." << std::endl;
            } else {
                std::cerr << "Telemetry packet decoding stopped for "
                          << TELEMETRY_TIMEOUT_MS / 1000 << " seconds." << std::endl;
            }
            return 1;
        }
    }
}


int main() {
    int serial_fd = configureSerial(SERIAL_PORT);
    if (serial_fd == -1) {
        return -1;  // Exit if the serial port cannot be opened
    }

    std::cout << "Opened " << SERIAL_PORT << "; waiting for telemetry..." << std::endl;

    bool logToCSV = false;
    // std::string csvPath { "imuData.log" };
    // std::string csvPath { "motorSpinUp.csv" };
    std::string csvPath { "data_tmp.csv" };

    const int readResult = readSerial(serial_fd, logToCSV, csvPath);

    close(serial_fd);
    return readResult;
}
