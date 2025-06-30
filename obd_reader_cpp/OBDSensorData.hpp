#ifndef OBDSENSORDATA_HPP
#define OBDSENSORDATA_HPP

#include <string>
#include <vector>

class OBDSensorData {
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
    long long currentTimeMillis();

public:
    OBDSensorData();
    ~OBDSensorData();

    bool connect(const std::string& port = "/dev/ttyACM0", int baud = B9600);
    bool requestSensorReadings();
    bool requestDTCs();
    bool clearDTCs();

    // Getters for sensor data
    int getCoolantTemp() const;
    int getEngineRPM() const;
    int getVehicleSpeed() const;
    int getTirePressure() const;
    int getMAF() const;
    std::string getDTCs() const;
    bool getDtcClearedStatus() const;
};

#endif // OBDSENSORDATA_HPP