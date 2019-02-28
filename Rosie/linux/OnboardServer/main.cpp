
#include <iostream>
#include <string>

#include <dji_linux_helpers.hpp>
#include <dji_log.hpp>
#include <dji_control.hpp>
#include <dji_status.hpp>

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/multirotor/firmwares/dji/OnboardMultirotorApi.hpp"
#include "common/Settings.hpp"


using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;
using namespace Rosie::OnboardLib;
void printUsage() {
    cout << "Usage: OnboardServer" << endl;
    cout << "Start the OnboardServer using the 'OnboardSDK' settings in ~/Documents/AirSim/settings.json." << endl;
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
        
    std::cout << std::endl << "Start Onboard Server" << std::endl;
    DJI::OSDK::Log::instance().enableStatusLogging();
    DJI::OSDK::Log::instance().title(1, "STATUS").print(std::string("Start Onboard Server").c_str());    
    std::cout << std::endl;

    OnboardMultirotorApi::ConnectionInfo connection_info;
    std::string host_ip = "127.0.0.1";
    int host_port = 41451;
    std::string onboard_version = "[Undefined]";

    std::cout << "Load Settings" << std::endl;
    // read settings and override defaults
    auto onboardconfig_full_filepath = Settings::getUserDirectoryFullPath("UserConfig.txt");
    auto settings_full_filepath = Settings::getUserDirectoryFullPath("settings.json");
    Settings& settings = Settings::singleton().loadJSonFile(settings_full_filepath);
    Settings child;

    if (settings.isLoadSuccess()) 
    {
        host_ip = settings.getString("ServerHostIp", host_ip);
        host_port = settings.getInt("ServerHostPort", host_port);
        std::cout << "\n" << "\nServer Host: " << host_ip << "\n";
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
    else 
    {
        std::cout << "\nCould not load settings from " << Settings::singleton().getFullFilePath() << std::endl;
        return 3;
    }

    int onboard_argc = 2;
    char* onboard_argv[] = { argv[0], &onboardconfig_full_filepath[0] };

    std::cout << "Initialize flight controller" << std::endl;
    OnboardMultirotorApi onboard_api;
    onboard_api.initialize(connection_info, nullptr, is_simulation, onboard_argc, onboard_argv);
    onboard_api.reset();

    std::cout << "Start Onboard RPC server" << std::endl;
    
    ApiProvider api_provider(nullptr);
    api_provider.insert_or_assign("", &onboard_api, nullptr);
    msr::airlib::MultirotorRpcLibServer server(&api_provider, host_ip, host_port);
    server.start(false, 4);

    std::cout << std::endl << std::endl << "Onboard RPC Server started" << std::endl;
    std::cout << "Hit Ctrl+C to terminate." << std::endl;

    DSTATUS(std::string("Onboard server started").c_str());
    std::vector<std::string> messages;
    while (true) {
        //check messages
        onboard_api.getStatusMessages(messages);
        if (messages.size() > 1) {
            for (const std::string& message : messages) {
                DJI::OSDK::Log::instance().title(1, "STATUS").print(message.c_str());
            }
        }
        constexpr static std::chrono::milliseconds MessageCheckDurationMillis(100);
        std::this_thread::sleep_for(MessageCheckDurationMillis);
        onboard_api.sendTelemetry();
    }
    std::cout << std::endl << "Server Stopped" << std::endl;
    DSTATUS(std::string("Onboard server stopped").c_str());
    return 0;
}