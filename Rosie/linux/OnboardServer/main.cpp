
#include <iostream>
#include <string>

#include "dji_control.hpp"
#include "dji_status.hpp"

#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "vehicles/multirotor/controllers/OnboardDroneController.hpp"

using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;
using namespace Rosie::OnboardLib;

void printUsage() {
    cout << "Usage: DroneServer" << endl;
    cout << "Start the DroneServer using the 'PX4' settings in ~/Documents/AirSim/settings.json." << endl;
}

int main(int argc, const char* argv[])
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " is_simulation" << std::endl;
        std::cout << "\t where is_simulation = 0 or 1" << std::endl;
        return 1;
    }

    bool is_simulation = std::atoi(argv[1]) == 1;
    if (is_simulation)
        std::cout << "You are running in simulation mode." << std::endl;
    else
        std::cout << "WARNING: This is not simulation!" << std::endl;

    std::cout << "Start Onboard Server" << "\n";

    std::cout << "DJI::OSDK::Control::FlightCommand::startMotor = " << DJI::OSDK::Control::FlightCommand::startMotor << "\n";
    std::cout << "DJI::OSDK::Control::FlightCommand::stopMotor = " << DJI::OSDK::Control::FlightCommand::stopMotor << "\n";


    OnboardDroneController::ConnectionInfo connection_info;
    
    // read settings and override defaults
    auto settings_full_filepath = Settings::getUserDirectoryFullPath("settings.json");
    Settings& settings = Settings::singleton().loadJSonFile(settings_full_filepath);
    Settings child;
    if (settings.isLoadSuccess()) {
        settings.getChild("Onboard", child);

        connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
        connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
        connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
        connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);

    }
    else {
        std::cout << "Could not load settings from " << Settings::singleton().getFullFilePath() << std::endl;
        return 3;

    }

    OnboardDroneController onboard_drone;
    onboard_drone.initialize(connection_info, nullptr, is_simulation, argc, argv);
    onboard_drone.reset();

    RealMultirotorConnector connector(& onboard_drone);

    MultirotorApi server_wrapper(& connector);
    msr::airlib::MultirotorRpcLibServer server(&server_wrapper, connection_info.local_host_ip);
    
    //start server in async mode
    server.start(false);

    std::cout << "Server connected to Onboard port " << connection_info.serial_port << ", rate" << connection_info.baud_rate << std::endl;
    std::cout << "Hit Ctrl+C to terminate." << std::endl;

    std::vector<std::string> messages;
    while (true) {
        //check messages
        server_wrapper.getStatusMessages(messages);
        if (messages.size() > 1) {
            for (const auto& message : messages) {
                std::cout << message << std::endl;
            }
        }        

        constexpr static std::chrono::milliseconds MessageCheckDurationMillis(100);
        std::this_thread::sleep_for(MessageCheckDurationMillis);

        onboard_drone.reportTelemetry(100);
    }

    return 0;
}
