
#include <iostream>
#include <string>

#include "dji_control.hpp"
#include "dji_status.hpp"

#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "vehicles/multirotor/controllers/RealMultirotorConnector.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/multirotor/controllers/OnboardDroneController.hpp"
#include "common/Settings.hpp"


using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;
using namespace Rosie::OnboardLib;
void printUsage() {
    cout << "Usage: DroneServer" << endl;
    cout << "Start the DroneServer using the 'PX4' settings in ~/Documents/AirSim/settings.json." << endl;
}

int main(int argc, char** argv)
{
    if (argc > 2 || 0 > argc) { 
        std::cout << "Usage: " << argv[0] << " --s[imulation]" << std::endl;
        return 1;
    }

    bool is_simulation = false;
    if (argc == 2) { is_simulation = true; }
    if (is_simulation) { std::cout << "You are running in simulation mode." << std::endl; }   
    else { std::cout << "WARNING: This is not simulation!" << std::endl; }
        
    std::cout << "Start Onboard Server" << "\n";

    OnboardDroneController::ConnectionInfo connection_info;
    std::string host_ip = "127.0.0.1";
    int host_port = 41451;
    std::string onboard_version = "[Undefined]";

    
    std::cout << "Load Settings" << "\n";
    // read settings and override defaults
    auto onboardconfig_full_filepath = Settings::getUserDirectoryFullPath("UserConfig.txt");
    auto settings_full_filepath = Settings::getUserDirectoryFullPath("settings.json");
    Settings& settings = Settings::singleton().loadJSonFile(settings_full_filepath);
    Settings child;
    if (settings.isLoadSuccess()) {

        host_ip = settings.getString("ServerHostIp", host_ip);
        host_port = settings.getInt("ServerHostPort", host_port);


        std::cout << "Server Host: " << host_ip << "\n";
        std::cout << "Server Port: " << host_port << "\n";

        settings.getChild("OnboardSDK", child);

        onboard_version = child.getString("Version", onboard_version);
        
        connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);

        /*
        connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
        connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
        connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
        */
        
    }
    else {
        std::cout << "Could not load settings from " << Settings::singleton().getFullFilePath() << std::endl;
        return 3;
    }

    int onboard_argc = 2;

    char* onboard_argv[] = { argv[0], &onboardconfig_full_filepath[0] };

    std::cout << "Initialize flight controller" << "\n";

    OnboardDroneController onboard_drone;
    onboard_drone.initialize(connection_info, nullptr, is_simulation, onboard_argc, onboard_argv);
    onboard_drone.reset();

    RealMultirotorConnector connector(& onboard_drone);

    std::cout << "Start network server" << "\n";    
    MultirotorApi server_wrapper(& connector);
    msr::airlib::MultirotorRpcLibServer server(&server_wrapper, host_ip, host_port);
    
    //start server in async mode
    server.start(false);

    std::cout << "Server connected to Onboard" << std::endl;
    std::cout << "Hit Ctrl+C to terminate." << std::endl;

    std::cout << "Server started" << "\n";    

    std::vector<std::string> messages;
    while (true) {
        fprintf(stdout,".");
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

    std::cout << "Server stopped" << "\n";    
    
    return 0;
}