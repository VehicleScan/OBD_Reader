#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>

class OBDSensorData
{
private:
    int serial_fd;
    bool port_open;

    // Sensor data storage
    int coolant_temp;
    int engine_rpm;
    int vehicle_speed;
    int tire_pressure;
    int maf;
    std::string dtcs;
    bool dtc_cleared;

    // Helper function to get current time in milliseconds
    long long currentTimeMillis()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
                   system_clock::now().time_since_epoch())
            .count();
    }

public:
    OBDSensorData() : serial_fd(-1), port_open(false),
                      coolant_temp(0), engine_rpm(0), vehicle_speed(0),
                      tire_pressure(0), maf(0), dtc_cleared(false) {}

    ~OBDSensorData()
    {
        if (port_open)
            close(serial_fd);
    }

    bool connect(const std::string &port = "/dev/ttyACM0", int baud = B9600)
    {
        serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd < 0)
        {
            std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd, &tty) != 0)
        {
            std::cerr << "Error getting termios attributes: " << strerror(errno) << std::endl;
            return false;
        }

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8-bit characters
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON; // Non-canonical mode
        tty.c_lflag &= ~ECHO;   // No echo
        tty.c_lflag &= ~ECHOE;  // No erasure
        tty.c_lflag &= ~ECHONL; // No new-line echo
        tty.c_lflag &= ~ISIG;   // No signal chars

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST; // Raw output
        tty.c_oflag &= ~ONLCR; // No newline conversion

        tty.c_cc[VTIME] = 1; // 0.1 second timeout (1 decisecond)
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error setting termios attributes: " << strerror(errno) << std::endl;
            return false;
        }

        port_open = true;
        tcflush(serial_fd, TCIOFLUSH);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // Wait for Arduino to reset
        return true;
    }

    bool requestSensorReadings()
    {
        if (!port_open)
        {
            std::cerr << "Port not open for reading" << std::endl;
            return false;
        }

        tcflush(serial_fd, TCIFLUSH);

        // Send 'r' command
        if (write(serial_fd, "r\n", 2) != 2)
        {
            std::cerr << "Error sending 'r' command" << std::endl;
            return false;
        }

        // Read response with timeout
        char buffer[128];
        std::string response;
        long long start_time = currentTimeMillis();
        ssize_t bytes_read;

        while (currentTimeMillis() - start_time < 3000)
        {
            bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0)
            {
                buffer[bytes_read] = '\0';
                response += buffer;
                if (response.find('\n') != std::string::npos)
                {
                    break;
                }
            }
            else if (bytes_read < 0)
            {
                std::cerr << "Read error: " << strerror(errno) << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Received sensor data: " << response << std::endl;

        // Parse values
        std::istringstream iss(response);
        std::vector<int> values;
        std::string token;

        while (std::getline(iss, token, ','))
        {
            try
            {
                values.push_back(std::stoi(token));
            }
            catch (...)
            {
                std::cerr << "Error parsing value: " << token << std::endl;
            }
        }

        if (values.size() != 5)
        {
            std::cerr << "Invalid data format. Expected 5 values, got " << values.size() << std::endl;
            return false;
        }

        coolant_temp = values[0];
        engine_rpm = values[1];
        vehicle_speed = values[2];
        tire_pressure = values[3];
        maf = values[4];

        return true;
    }

    bool requestDTCs()
    {
        if (!port_open)
            return false;

        tcflush(serial_fd, TCIFLUSH);
        write(serial_fd, "d\n", 2);

        char buffer[256];
        std::string response;
        long long start_time = currentTimeMillis();
        ssize_t bytes_read;

        while (currentTimeMillis() - start_time < 5000)
        {
            bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0)
            {
                buffer[bytes_read] = '\0';
                response += buffer;
                if (response.find('\n') != std::string::npos)
                {
                    break;
                }
            }
            else if (bytes_read < 0)
            {
                std::cerr << "Read error: " << strerror(errno) << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Received DTC data: " << response << std::endl;

        if (!response.empty() && response.back() == '\n')
        {
            response.pop_back();
        }

        dtcs = response;
        return true;
    }

    bool clearDTCs()
    {
        if (!port_open) {
            std::cerr << "Port not open for clearing DTCs" << std::endl;
            return false;
        }

        tcflush(serial_fd, TCIFLUSH);
        if (write(serial_fd, "c\n", 2) != 2) {
            std::cerr << "Error sending 'c' command: " << strerror(errno) << std::endl;
            return false;
        }

        char response[3] = {0};
        long long start_time = currentTimeMillis();
        ssize_t bytes_read = 0;

        while (currentTimeMillis() - start_time < 3000 && bytes_read < 3) {
            ssize_t n = read(serial_fd, response + bytes_read, 3 - bytes_read);
            if (n > 0) {
                bytes_read += n;
            } else if (n < 0) {
                std::cerr << "Read error: " << strerror(errno) << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Clear response: " << response << std::endl;

        if (bytes_read == 3 && response[0] == '1' && response[1] == '\r' && response[2] == '\n') {
            dtc_cleared = true;
            return true;
        } else if (bytes_read == 3 && response[0] == '0' && response[1] == '\r' && response[2] == '\n') {
            dtc_cleared = false;
            std::cout << "DTC clear failed (Arduino returned '0')" << std::endl;
            return false;
        }

        std::cerr << "Invalid response or timeout. Bytes read: " << bytes_read << std::endl;
        dtc_cleared = false;
        return false;
    }

    // Getters for sensor data
    int getCoolantTemp() const { return coolant_temp; }
    int getEngineRPM() const { return engine_rpm; }
    int getVehicleSpeed() const { return vehicle_speed; }
    int getTirePressure() const { return tire_pressure; }
    int getMAF() const { return maf; }
    std::string getDTCs() const { return dtcs; }
    bool getDtcClearedStatus() const { return dtc_cleared; }
};

int main()
{
    OBDSensorData obd;

    std::cout << "Connecting to OBD adapter..." << std::endl;
    if (!obd.connect())
    {
        std::cerr << "Failed to connect to OBD adapter" << std::endl;
        return 1;
    }
    std::cout << "Connected successfully" << std::endl;

    while (true)
    {
        std::cout << "\n--- Reading sensor data ---" << std::endl;
        if (obd.requestSensorReadings())
        {
            std::cout << "Coolant Temp: " << obd.getCoolantTemp() << "°C\n"
                      << "Engine RPM: " << obd.getEngineRPM() << " RPM\n"
                      << "Vehicle Speed: " << obd.getVehicleSpeed() << " km/h\n"
                      << "Tire Pressure: " << obd.getTirePressure() << " PSI\n"
                      << "MAF: " << obd.getMAF() << " g/s" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read sensor data" << std::endl;
        }

        std::cout << "\n--- Reading DTCs ---" << std::endl;
        if (obd.requestDTCs())
        {
            std::cout << "DTCs: " << obd.getDTCs() << std::endl;
        }
        else
        {
            std::cerr << "Failed to read DTCs" << std::endl;
        }

        std::cout << "\n--- Clearing DTCs ---" << std::endl;
        if (obd.clearDTCs())
        {
            std::cout << "DTCs cleared successfully" << std::endl;
        }
        else
        {
            std::cerr << "Failed to clear DTCs" << std::endl;
        }

        std::cout << "\nWaiting 5 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    return 0;
}