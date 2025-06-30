#include "OBDSensorData.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    OBDSensorData obd;
    
    std::cout << "Connecting to OBD adapter..." << std::endl;
    if (!obd.connect()) {
        std::cerr << "Failed to connect to OBD adapter" << std::endl;
        return 1;
    }
    std::cout << "Connected successfully" << std::endl;
    
    while (true) {
        std::cout << "\n--- Reading sensor data ---" << std::endl;
        if (obd.requestSensorReadings()) {
            std::cout << "Coolant Temp: " << obd.getCoolantTemp() << "°C\n"
                      << "Engine RPM: " << obd.getEngineRPM() << " RPM\n"
                      << "Vehicle Speed: " << obd.getVehicleSpeed() << " km/h\n"
                      << "Tire Pressure: " << obd.getTirePressure() << " PSI\n"
                      << "MAF: " << obd.getMAF() << " g/s" << std::endl;
        } else {
            std::cerr << "Failed to read sensor data" << std::endl;
        }
        
        std::cout << "\n--- Reading DTCs ---" << std::endl;
        if (obd.requestDTCs()) {
            std::cout << "DTCs: " << obd.getDTCs() << std::endl;
        } else {
            std::cerr << "Failed to read DTCs" << std::endl;
        }
        
        std::cout << "\n--- Clearing DTCs ---" << std::endl;
        if (obd.clearDTCs()) {
            std::cout << "DTCs cleared successfully" << std::endl;
        } else {
            std::cerr << "Failed to clear DTCs" << std::endl;
        }
        
        std::cout << "\nWaiting 5 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    return 0;
}