
#include <iostream>
#include <string>

#include "dji_control.hpp"
#include "dji_status.hpp"

using namespace DJI::OSDK;
using namespace std;

int main()
{
    std::cout << "Start Onboard Server" << "\n";

    std::cout << "DJI::OSDK::Control::FlightCommand::startMotor = " << DJI::OSDK::Control::FlightCommand::startMotor << "\n";
    std::cout << "DJI::OSDK::Control::FlightCommand::stopMotor = " << DJI::OSDK::Control::FlightCommand::stopMotor << "\n";

    return 0;
}