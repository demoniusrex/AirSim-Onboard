// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_OnboardMultirotorApi_hpp
#define msr_airlib_OnboardMultirotorApi_hpp

#include <queue>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <exception>
#include <cmath>

#include "common/Common.hpp"
#include "common/common_utils/Timer.hpp"
#include "common/CommonStructs.hpp"
#include "common/VectorMath.hpp"
#include "common/AirSimSettings.hpp"
#include "vehicles/multirotor/MultiRotor.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/PidController.hpp"
#include "sensors/SensorCollection.hpp"

//sensors
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"

// onboard
#include "dji_control.hpp"
#include "dji_status.hpp"
#include "dji_log.hpp"
#include "dji_telemetry.hpp"
#include "dji_vehicle.hpp"

// onboard platform helper
#include "dji_linux_helpers.hpp"

using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

namespace Rosie { namespace OnboardLib {

class OnboardMultirotorApi : public MultirotorApiBase
{
public:
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::MultiRotor MultiRotor;

    struct ConnectionInfo {
        bool use_serial = true; 
        std::string serial_port = "*";
        int baud_rate = 115200;
        std::string ip_address = "127.0.0.1";
        int ip_port = 14560;
        std::string model = "Generic";
    };

public:
    virtual ~OnboardMultirotorApi() 
    {
        closeAllConnection();
    }

    //non-base interface specific to OnboardMultirotorApi
    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation, int argc, char** argv)
    {
        DiagnosticMessages diagnostic_messages;
        diagnostic_messages_ = diagnostic_messages;
    
        connection_info_ = connection_info;
        sensors_ = sensors;
        is_simulation_mode_ = is_simulation;

        try {
            //std::cout << "Initialize vehicle" << std::endl;
            //std::cout << "Initialize Onboard SDK Linux environment" << std::endl;
            linuxEnvironment = new LinuxSetup(argc, argv);
            if (linuxEnvironment == NULL)
            {
                throw std::runtime_error("Error initializing Onboard SDK Linux environment");
            }
            //std::cout << "Connect to flight controller" << std::endl;
            onboard_vehicle_ = linuxEnvironment->getVehicle();
            //connectToVideoServer();
            //std::cout << "Initialize vehicle data subscriptions" << std::endl;
            initializeOnboardSubscriptions();
            //std::cout << "Set position origin" << std::endl;
            setOrigin(false);
            is_avaiis_ready_lable_ = true;
            //std::cout << "Vehicle initialized" << std::endl;
        }
        catch (std::exception& ex) {
            is_ready_ = false;
            is_ready_message_ = Utils::stringf("Failed to connect to Onboard vehicle: %s", ex.what());
        }
    }

    Pose getMocapPose()
    {
        std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
        return mocap_pose_;
    }

    virtual const SensorCollection& getSensors() const override
    {
        return *sensors_;
    }

    virtual void reset() override
    {
        MultirotorApiBase::reset();
        resetState();
        was_reset_ = true;
        setNormalMode();
    }

    virtual void update() override
    {
        if (sensors_ == nullptr || onboard_vehicle_ == nullptr 
            || !(onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() == VehicleStatus::FlightStatus::ON_GROUND))
        {
            return;
        }

        const auto * distance = getDistance();
        if (distance) {
          const auto& distance_output = distance->getOutput();
          float pitch, roll, yaw;
          VectorMath::toEulerianAngle(distance_output.relative_pose.orientation, pitch, roll, yaw);
        }

        const auto gps = getGps();
        if (gps != nullptr) {
            const auto& gps_output = gps->getOutput();
            //send GPS
            if (gps_output.is_valid && gps_output.gnss.time_utc > last_gps_time_) {
                last_gps_time_ = gps_output.gnss.time_utc;
                Vector3r gps_velocity = gps_output.gnss.velocity;
                Vector3r gps_velocity_xy = gps_velocity;
                gps_velocity_xy.z() = 0;
                float gps_cog;
                if (Utils::isApproximatelyZero(gps_velocity.y(), 1E-2f) && Utils::isApproximatelyZero(gps_velocity.x(), 1E-2f))
                    gps_cog = 0;
                else
                    gps_cog = Utils::radiansToDegrees(atan2(gps_velocity.y(), gps_velocity.x()));
                if (gps_cog < 0)
                    gps_cog += 360;
            }
        }
        
        //must be done at the end
        if (was_reset_)
            was_reset_ = false;
    }

    virtual bool isReady(std::string& message) const override
    {
        if (!is_ready_)
            message = is_ready_message_;
        return is_ready_;
    }

    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        updateState();
        messages.clear();
        std::lock_guard<std::mutex> guard(status_text_mutex_);
        while (!status_messages_.empty()) {
            messages.push_back(status_messages_.front());
            status_messages_.pop();
        }
    }

    virtual Kinematics::State getKinematicsEstimated() const override
    {
        updateState();
        Kinematics::State state;
        
        Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
        Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type currentVelocity;
        Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type currentOrientation;
        Telemetry::TypeMap<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type currentAngularRate;
        Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type currentAccelerationGround;
        Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_BODY>::type currentAccelerationBody;
        Telemetry::Vector3f localOffset;

        currentSubscriptionGPS = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
        currentVelocity = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
        currentOrientation = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
        Telemetry::Vector3f eulerOrientation = toEulerAngle((static_cast<void*>(&currentOrientation)));
        currentAngularRate = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();
        currentAccelerationGround = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
        currentAccelerationBody = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_BODY>();

        localOffsetFromGpsOffset(localOffset,
                                static_cast<void*>(&currentSubscriptionGPS),
                                static_cast<void*>(&originGPS));

        state.pose.position = Vector3r(localOffset.x, localOffset.y, -localOffset.z);
        state.pose.orientation = VectorMath::toQuaternion(eulerOrientation.x, eulerOrientation.y, eulerOrientation.z);
        state.twist.linear = Vector3r(currentVelocity.data.x, currentVelocity.data.y, currentVelocity.data.z);
        state.twist.angular = Vector3r(currentAngularRate.x, currentAngularRate.y, currentAngularRate.z);
        state.accelerations.linear = Vector3r(currentAccelerationGround.x, currentAccelerationGround.y, currentAccelerationGround.z);
        state.accelerations.angular = Vector3r(currentAccelerationBody.x, currentAccelerationBody.y, currentAccelerationBody.z);
        return state;
    }

    virtual bool isApiControlEnabled() const override
    {
        return is_api_control_enabled_;
    }

    virtual void enableApiControl(bool is_enabled) override
    {
        checkVehicle();
        if (is_enabled) {
            onboard_vehicle_->obtainCtrlAuthority(1000);
            is_api_control_enabled_ = true;
            //addStatusMessage(std::string("Obtained Vehicle Control Authority"));
        }
        else {
            onboard_vehicle_->releaseCtrlAuthority(1000);
            is_api_control_enabled_ = false;
            //addStatusMessage(std::string("Released Vehicle Control Authority"));
        }
    }

    virtual Vector3r getPosition() const override
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
        Telemetry::Vector3f localOffset;
        currentSubscriptionGPS = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
        localOffsetFromGpsOffset(localOffset,
                                static_cast<void*>(&currentSubscriptionGPS),
                                static_cast<void*>(&originGPS));
        return Vector3r(localOffset.x, localOffset.y, -localOffset.z);
    }

    virtual Vector3r getVelocity() const override
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type currentVelocity;
        currentVelocity = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
        return Vector3r(currentVelocity.data.x, currentVelocity.data.y, currentVelocity.data.z);
    }

    virtual Quaternionr getOrientation() const override
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type currentOrientation;
        currentOrientation = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
        Telemetry::Vector3f euler = toEulerAngle((static_cast<void*>(&currentOrientation)));
        return VectorMath::toQuaternion(euler.x, euler.y, euler.z);
    }

    virtual LandedState getLandedState() const override
    {
        updateState();
        return current_state == VehicleStatus::FlightStatus::IN_AIR ? LandedState::Flying : LandedState::Landed;
    }

    virtual real_T getActuation(unsigned int rotor_index) const override
    {
        throw std::logic_error("Onboard Multirotor does not support function to read motor controls ");
    }

    virtual size_t getActuatorCount() const override
    {
        return RotorControlsCount;
    }

    virtual bool armDisarm(bool arm) override
    {
        SingleCall lock(this);
        checkVehicle();
        bool rc = false;
        if (arm)
        {
            onboard_vehicle_->control->armMotors(1000);
            setOrigin(true);
            addStatusMessage(std::string("Motors Armed"));
        }
        else 
        {
            onboard_vehicle_->control->disArmMotors(1000);
            addStatusMessage(std::string("Motors Disarmed"));
        }
        return rc;
    }

    virtual bool takeoff(float timeout_sec) override
    {
        SingleCall lock(this);

        checkVehicle();

        auto vec = getPosition();
        float z = vec.z() + getTakeoffZ();
        int  timeout = 100000;

        char func[50];

        // Telemetry: Verify the subscription
        addStatusMessage(std::string("Verify vehicle connection"));
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = onboard_vehicle_->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            throw VehicleMoveException("Takeoff failed. Unable to verify vehicle subscription");
        }

        // Start takeoff
        addStatusMessage(std::string("Start Takeoff"));
        ACK::ErrorCode takeoffStatus = onboard_vehicle_->control->takeoff(timeout);
        if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(takeoffStatus, func);
            throw VehicleMoveException("Takeoff failed. Error sending takeoff command.");
        }

        // First check: Motors started
        addStatusMessage(std::string("Check motors"));
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
                    VehicleStatus::FlightStatus::ON_GROUND &&
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_ENGINE_START &&
                motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            // fprintf(stderr, "Motors not started %d, %d\n", onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>(), onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() );
            updateState();
            usleep(1000000);
        }

        if (motorsNotStarted == timeoutCycles)
        {
            //std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
            // Cleanup
            // onboard_vehicle_->subscribe->removePackage(0, timeout);
            throw VehicleMoveException("Takeoff failed. Motors are not spinning.");
        }
        else
        {
            addStatusMessage(std::string("Motors spinning"));
        }

        // Second check: In air
        int stillOnGround = 0;
        timeoutCycles     = 110;

        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
                    VehicleStatus::FlightStatus::IN_AIR &&
                (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
                stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            updateState();
            usleep(100000);
        }

        if (stillOnGround == timeoutCycles)
        {
            //std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
            //            "motors are spinning."
            //        << std::endl;
            // Cleanup
            //onboard_vehicle_->subscribe->removePackage(0, timeout);
            throw VehicleMoveException("Takeoff failed. Aircraft is still on the ground, but the motors are spinning");
        }
        else
        {
            addStatusMessage(std::string("Ascending"));
        }

        // Final check: Finished takeoff
        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                    VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                    VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
        {
            
            updateState();
            sleep(1);
        }

        if (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_P_GPS ||
            onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            addStatusMessage(std::string("Successful takeoff"));
        }
        else
        {
            //std::cout
            //<< "Takeoff finished, but the aircraft is in an unexpected mode. "
            //    "Please connect DJI GO.\n";
            // onboard_vehicle_->subscribe->removePackage(0, timeout);
            throw VehicleMoveException("Takeoff finished, but the aircraft is in an unexpected mode.");
        }
        
        if (max_wait_seconds <= 0)
            return true; // client doesn't want to wait.

        return waitForZ(timeout_sec, z, getDistanceAccuracy());
    }

    virtual bool land(float timeout_sec) override
    {
        SingleCall lock(this);
        char func[50];

        updateState();
        checkVehicle();
        
        // Telemetry: Verify the subscription
        addStatusMessage(std::string("Verify vehicle connection"));
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = onboard_vehicle_->subscribe->verify(max_wait_seconds);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            throw VehicleMoveException("Unable to verify vehicle subscription");
        }

        // Start landing
        ACK::ErrorCode landingStatus = onboard_vehicle_->control->land(max_wait_seconds);
        if (ACK::getError(landingStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(landingStatus, func);
            throw VehicleMoveException("Error starting landing");
        }

        // First check: Landing started
        int landingNotStarted = 0;
        int timeoutCycles     = 20;

        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
                landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            updateState();
            usleep(100000);
        }

        if (landingNotStarted == timeoutCycles)
        {
            //std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
            throw VehicleMoveException("Landing failed. Aircraft is still in the air.");
        }
        else
        {
            addStatusMessage(std::string("Landing"));
        }

        // Second check: Finished landing
        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                    VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() ==
                    VehicleStatus::FlightStatus::IN_AIR)
        {
            updateState();
            sleep(1);
        }

        if (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_P_GPS ||
            onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            addStatusMessage(std::string("Successful landing"));
        }
        else
        {
            //std::cout
            //<< "Landing finished, but the aircraft is in an unexpected mode. "
            //    "Please connect DJI GO.\n";
            return false;
        }
        return true;
    }

    virtual bool goHome(float timeout_sec) override
    {
        SingleCall lock(this);
        char func[50];
        checkVehicle();
        bool rc = false;
        if (onboard_vehicle_ != nullptr) {
            ACK::ErrorCode takeoffStatus = onboard_vehicle_->control->goHome(60000);
            if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
            {
                ACK::getErrorCodeMessage(takeoffStatus, func);
                throw VehicleMoveException("goHome - timeout waiting for response from drone");
            }
            
            // First check: GoHome command started
            int commandNotStarted = 0;
            int timeoutCycles     = 20;

            while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                        VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME &&
                    commandNotStarted < timeoutCycles)
            {
                commandNotStarted++;
                updateState();
                usleep(100000);
            }

            if (commandNotStarted == timeoutCycles)
            {
                //std::cout << "Return home command failed. Aircraft is in incorrect mode." << std::endl;
                // Cleanup before return
                throw VehicleMoveException("Return home command failed. Aircraft is in incorrect mode.");
            }
            else
            {
                addStatusMessage(std::string("Returning home"));
            }

            // Second check: Finished landing
            while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                        VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME &&
                    onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() ==
                        VehicleStatus::FlightStatus::IN_AIR)
            {
                updateState();
                sleep(1);
            }

            if (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                    VehicleStatus::DisplayMode::MODE_P_GPS ||
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
                    VehicleStatus::DisplayMode::MODE_ATTITUDE)
            {
                addStatusMessage(std::string("Returned home"));
            }
            else
            {
                //std::cout
                //<< "Return home finished, but the aircraft is in an unexpected mode.\n";
            }
        }
        return rc;
    }

    virtual bool hover() override
    {
        SingleCall lock(this);
        bool rc = false;
        checkVehicle();
        onboard_vehicle_->control->emergencyBrake();
        while (!getCancelToken().isCancelled())
        {
            sleep(100);
        }
        return rc;
    }

    virtual GeoPoint getHomeGeoPoint() const override
    {
        updateState();
        return GeoPoint(rad2Deg(originGPS.latitude), rad2Deg(originGPS.longitude), originGPS.altitude);
    }

    virtual GeoPoint getGpsLocation() const override
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
        currentSubscriptionGPS = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
        return GeoPoint(rad2Deg(currentSubscriptionGPS.latitude), rad2Deg(currentSubscriptionGPS.longitude), currentSubscriptionGPS.altitude);
    }

    virtual void sendTelemetry(float last_interval = -1) override
    {
        return;
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //1 period of 50hz
    }

    virtual float getTakeoffZ() const override
    {
        return -1.1f;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;
    }

protected: //methods
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        // addStatusMessage(std::string("Received Command RollPitchZ"));
        checkVehicle();
        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_ANGLE |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_POSITION |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_BODY;
        if (pitch < 0.001f && pitch > -0.001f) 
        {
            pitch = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, pitch, roll, -z, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    virtual void commandRollPitchThrottle(float pitch, float roll, float throttle, float yaw_rate) override
    {
        // addStatusMessage(std::string("Received Command RollPitchThrottle,") + "vx:" + std::to_string(vx) + ",vy:" + std::to_string(vy) + ",vz:" + std::to_string(vz) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
        checkVehicle();
        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_ANGLE |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_THRUST |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_BODY;
        if (pitch < 0.001f && pitch > -0.001f) 
        {
            pitch = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, pitch, roll, throttle, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        // addStatusMessage(std::string("Received Command Velocity,") + "vx:" + std::to_string(vx) + ",vy:" + std::to_string(vy) + ",vz:" + std::to_string(vz) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
        checkVehicle();
        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_VELOCITY |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_VELOCITY |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_GROUND;
        if (vx < 0.001f && vx > -0.001f) 
        {
            vx = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, vx, vy, -vz, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        // addStatusMessage(std::string("Received Command VelocityZ,") + "vx:" + std::to_string(vx) + ",vy:" + std::to_string(vy) + ",z:" + std::to_string(z) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
        checkVehicle();
        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_VELOCITY |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_POSITION |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_GROUND; 
        if (vx < 0.001f && vx > -0.001f) 
        {
            vx = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, vx, vy, -z, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        // addStatusMessage(std::string("Received Command Position,") + "x:" + std::to_string(x) + ",y:" + std::to_string(y) + ",z:" + std::to_string(z) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
        checkVehicle();
        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_POSITION |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_POSITION |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_GROUND;
        Vector3r vec = getPosition();
        x = x - vec.x();
        y = y - vec.y();
        if (x < 0.001f && x > -0.001f) 
        {
            x = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, x, y, -z, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        static const MultirotorApiParams vehicle_params_;
        return vehicle_params_;
    }
    
    virtual void beforeTask() override
    {
        startOffboardMode();
    }
    virtual void afterTask() override
    {
        endOffboardMode();
    }

    const ImuBase* getImu()
    {
        return static_cast<const ImuBase*>(sensors_->getByType(SensorBase::SensorType::Imu));
    }

    const MagnetometerBase* getMagnetometer()
    {
        return static_cast<const MagnetometerBase*>(sensors_->getByType(SensorBase::SensorType::Magnetometer));
    }

    const BarometerBase* getBarometer()
    {
        return static_cast<const BarometerBase*>(sensors_->getByType(SensorBase::SensorType::Barometer));
    }

    const DistanceBase* getDistance()
    {
        return static_cast<const DistanceBase*>(sensors_->getByType(SensorBase::SensorType::Distance));
    }

    const GpsBase* getGps()
    {
        return static_cast<const GpsBase*>(sensors_->getByType(SensorBase::SensorType::Gps));
    }

private: // methods

    void addStatusMessage(const std::string& message)
    {
        std::lock_guard<std::mutex> guard_status(status_text_mutex_);
        //if queue became too large, clear it first
        if (status_messages_.size() > status_messages_MaxSize)
            Utils::clear(status_messages_, status_messages_MaxSize - status_messages_.size());
        status_messages_.push(message);
    }

    void checkVehicle() 
    {
        if (onboard_vehicle_ == nullptr) 
        {
            throw std::logic_error("Cannot perform operation when no vehicle is connected");
        }
    }

    void closeAllConnection()
    {
        close();
    }

    void close()
    {
        if (onboard_vehicle_ != nullptr) 
        {
            onboard_vehicle_->subscribe->removeLeftOverPackages();
            onboard_vehicle_->releaseCtrlAuthority(1000);
            onboard_vehicle_ = nullptr;
        }
    }

    void resetState()
    {
        //reset state
        is_any_heartbeat_ =  is_armed_ = false;
        is_controls_0_1_ = true;
        hil_state_freq_ = -1;
        actuators_message_supported_ = false;
        last_gps_time_ = 0;
        state_version_ = 0;
        current_state = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
        target_height_ = 0;
        is_api_control_enabled_ = false;
        thrust_controller_ = PidController();
        Utils::setValue(rotor_controls_, 0.0f);
        was_reset_ = false;
        mocap_pose_ = Pose::nanPose();
    }

    void updateState()
    {
        StatusLock lock(this);
        if (onboard_vehicle_ != nullptr) {
            int version = onboard_vehicle_->getLastReceivedFrame().recvInfo.version;
            if (version != state_version_)
            {
                current_state = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
                state_version_ = version;
            }
        }
    }

    void setOrigin(bool is_wellknown_origin)
    {
        if (onboard_vehicle_ != nullptr) 
        {
            // set origin point
            originGPS = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
            is_wellknown_origin_ = is_wellknown_origin;
        }
    }

    void initializeOnboardSubscriptions()
    {
        if (onboard_vehicle_ != nullptr) {
            //std::cout << "Initialize subscriptions" << std::endl;
            is_any_heartbeat_ = false;
            is_armed_ = false;
            is_controls_0_1_ = true;
            char func[50]; 
            int pkgIndex;
            Utils::setValue(rotor_controls_, 0.0f); 
            // Telemetry: Verify the subscription
            ACK::ErrorCode subscribeStatus;
            subscribeStatus = onboard_vehicle_->subscribe->verify(1000);
            if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
            {
                ACK::getErrorCodeMessage(subscribeStatus, func);
                throw std::runtime_error("Error verifying flight controller data");
            }
            {
                //std::cout << "Subscribe to Onboard SDK flight status and vehicle status topics" << std::endl;
                // Telemetry: Subscribe to flight status and mode at freq 10 Hz
                pkgIndex                  = 0;
                int       freq            = 10;
                Telemetry::TopicName topicList10Hz[] = { Telemetry::TOPIC_STATUS_FLIGHT,
                                                Telemetry::TOPIC_STATUS_DISPLAYMODE };
                int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
                bool enableTimestamp = false;

                bool pkgStatus = onboard_vehicle_->subscribe->initPackageFromTopicList(
                    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
                if (!(pkgStatus))
                {
                    throw std::runtime_error("Error initializing 10Hz flight controller subscription package");
                }
                subscribeStatus = onboard_vehicle_->subscribe->startPackage(pkgIndex, 1000);
                if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
                {
                    ACK::getErrorCodeMessage(subscribeStatus, func);
                    // Cleanup before return
                    onboard_vehicle_->subscribe->removePackage(pkgIndex, 1000);
                    throw std::runtime_error(func);
                }
            }
            {
                //std::cout << "Subscribe to Onboard SDK position, velocity and acceleration topics" << std::endl;
                // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
                // Hz
                pkgIndex                  = 1;
                int       freq            = 50;
                Telemetry::TopicName topicList50Hz[] = 
                { 
                    Telemetry::TOPIC_QUATERNION, 
                    Telemetry::TOPIC_GPS_FUSED, 
                    Telemetry::TOPIC_GPS_POSITION, 
                    Telemetry::TOPIC_VELOCITY,  
                    Telemetry::TOPIC_ANGULAR_RATE_FUSIONED, 
                    Telemetry::TOPIC_ACCELERATION_GROUND, 
                    Telemetry::TOPIC_ACCELERATION_BODY
                };
                int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
                bool      enableTimestamp = false;

                bool pkgStatus = onboard_vehicle_->subscribe->initPackageFromTopicList(
                    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
                if (!(pkgStatus))
                {
                    throw std::runtime_error("Error initializing 50Hz flight controller subscription package");
                }
                subscribeStatus =
                    onboard_vehicle_->subscribe->startPackage(pkgIndex, 1000);
                if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
                {
                    ACK::getErrorCodeMessage(subscribeStatus, func);
                    // Cleanup before return
                    onboard_vehicle_->subscribe->removePackage(pkgIndex, 1000);
                    throw std::runtime_error(func);
                }
            }
            //std::cout << "Started all flight controller subscriptions" << std::endl;
        }
    }

    bool startOffboardMode()
    {
        checkVehicle();
        // addStatusMessage(std::string("Obtain Vehicle Control Authority"));
        try 
        {
            onboard_vehicle_->obtainCtrlAuthority(1000);
        }
        catch (std::exception& ex) 
        {
            ensureSafeMode();
            addStatusMessage(std::string("Request control failed: ") + ex.what());
            return false;
        }
        return true;
    }

    void endOffboardMode()
    {
        onboard_vehicle_->releaseCtrlAuthority();
        ensureSafeMode();
        // addStatusMessage(std::string("Released Vehicle Control Authority"));
    }

    void ensureSafeMode()
    {
        if (onboard_vehicle_ != nullptr) 
        {
            const int state = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
            if (state == VehicleStatus::FlightStatus::ON_GROUND || state == VehicleStatus::FlightStatus::STOPED) 
            {
                return;
            }
            // do something
        }
    }

    void setNormalMode()
    {
        if (onboard_vehicle_ != nullptr) {
            std::lock_guard<std::mutex> guard(set_mode_mutex_);
            int mode = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();            
            if (mode != VehicleStatus::FlightStatus::IN_AIR)
            {
                return;
            }
            // call emergency brake
            onboard_vehicle_->control->emergencyBrake();
        }
    }

    /*! Very simple calculation of local NED offset between two pairs of GPS
    /coordinates. Accurate when distances are small.
    !*/
    void localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed, void* target, void* origin)
    {
        Telemetry::GPSFused*       subscriptionTarget;
        Telemetry::GPSFused*       subscriptionOrigin;
        double                     deltaLon;
        double                     deltaLat;

        subscriptionTarget = (Telemetry::GPSFused*)target;
        subscriptionOrigin = (Telemetry::GPSFused*)origin;
        deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }

    Telemetry::Vector3f toEulerAngle(void* quaternionData)
    {
        Telemetry::Vector3f    ans;
        Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

        double q2sqr = quaternion->q2 * quaternion->q2;
        double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
        double t1 = +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
        double t2 = -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
        double t3 = +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
        double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

        t2 = (t2 > 1.0) ? 1.0 : t2;
        t2 = (t2 < -1.0) ? -1.0 : t2;

        ans.x = asin(t2);
        ans.y = atan2(t3, t4);
        ans.z = atan2(t1, t0);

        return ans;
    }

    inline double rad2Deg(const double radians)
    {
        return (radians / M_PI) * 180.0;
    }

    inline double deg2Rad(const double degrees)
    {
        return (degrees / 180.0) * M_PI;
    }



public: // variables

protected: // variables

private: // variables

    static const int RotorControlsCount = 6;
    static const int messageReceivedTimeout = 10; ///< Seconds 

    size_t status_messages_MaxSize = 5000;

    // std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;

    std::mutex heartbeat_mutex_, mocap_pose_mutex_, set_mode_mutex_, status_text_mutex_, last_message_mutex_;
   
    LinuxSetup* linuxEnvironment = nullptr;

    ConnectionInfo connection_info_;
    bool is_any_heartbeat_, is_armed_;
    bool is_controls_0_1_; //Are motor controls specified in 0..1 or -1..1?
    float rotor_controls_[RotorControlsCount];
    std::queue<std::string> status_messages_;
    int hil_state_freq_;
    bool actuators_message_supported_;
    const SensorCollection* sensors_;
    uint64_t last_gps_time_;
    bool was_reset_;
    bool is_ready_;
    std::string is_ready_message_;
    Pose mocap_pose_;

    Vehicle* onboard_vehicle_;
    int state_version_;
    int current_state;
    
    Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type originGPS;
    bool is_wellknown_origin_;
    float target_height_;
    bool is_api_control_enabled_;
    bool is_simulation_mode_;
    PidController thrust_controller_;
    common_utils::Timer hil_message_timer_;
    common_utils::Timer sitl_message_timer_;
};

}} //namespace
#endif
