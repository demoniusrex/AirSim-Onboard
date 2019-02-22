// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_OnboardDroneController_hpp
#define msr_airlib_OnboardDroneController_hpp

#include <queue>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <exception>
#include <cmath>

// #include <dji_linux_helpers.hpp>

#include "common/Common.hpp"
#include "common/common_utils/Timer.hpp"
#include "common/CommonStructs.hpp"
#include "common/VectorMath.hpp"
#include "vehicles/multirotor/MultiRotor.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "controllers/PidController.hpp"

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

#include "dji_linux_helpers.hpp"

using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

namespace Rosie { namespace OnboardLib {

class OnboardDroneController : public DroneControllerBase
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
        /* Default values are requires so uninitialized instance doesn't have random values */

        bool use_serial = true; // false means use UDP instead
                                //Used to connect via HITL: needed only if use_serial = true
        std::string serial_port = "*";
        int baud_rate = 115200;

        //Used to connect to drone over UDP: needed only if use_serial = false
        std::string ip_address = "127.0.0.1";
        int ip_port = 14560;

        std::string model = "Generic";
    };

public:
    //required for pimpl
    OnboardDroneController();
    virtual ~OnboardDroneController();

    //non-base interface specific to OnboardDroneController
    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation, int argc, char** argv);
    ConnectionInfo getOnboardConnectionInfo();
    
    //TODO: get rid of below methods?
    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height);
    bool hasVideoRequest();

    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override;
    virtual void update() override;
    virtual size_t getVertexCount() override;
    virtual real_T getVertexControlSignal(unsigned int rotor_index) override;
    virtual void getStatusMessages(std::vector<std::string>& messages) override;
    virtual bool isAvailable(std::string& message) override;
    virtual bool isApiControlEnabled() override;
    virtual bool isSimulationMode() override;
    virtual void enableApiControl(bool is_enabled) override;
    virtual void setSimulationMode(bool is_set) override;
    virtual Pose getDebugPose() override;
    //*** End: VehicleControllerBase implementation ***//


    //*** Start: DroneControllerBase implementation ***//
public:
    virtual Kinematics::State getKinematicsEstimated() override;
    virtual Vector3r getPosition() override;
    virtual Vector3r getVelocity() override;
    virtual Quaternionr getOrientation() override;
    virtual LandedState getLandedState() override;
    virtual RCData getRCData() override;
    virtual void setRCData(const RCData& rcData) override;

    virtual bool armDisarm(bool arm, CancelableBase& cancelable_action) override;
    virtual bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override;
    virtual bool land(float max_wait_seconds, CancelableBase& cancelable_action) override;
    virtual bool goHome(CancelableBase& cancelable_action) override;
    virtual bool hover(CancelableBase& cancelable_action) override;
    virtual GeoPoint getHomeGeoPoint() override;
    virtual GeoPoint getGpsLocation() override;
    virtual void reportTelemetry(float renderTime) override;

    virtual float getCommandPeriod() override;
    virtual float getTakeoffZ() override;
    virtual float getDistanceAccuracy() override;

    virtual bool loopCommandPre() override;
    virtual void loopCommandPost() override;
protected:
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override;
    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override;
    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override;
    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override;
    const VehicleParams& getVehicleParams() override;
    //*** End: DroneControllerBase implementation ***//

private: //pimpl
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

struct OnboardDroneController::impl {
public:
    // static const int onboardFlightCommandStop = 9900;   ///< 
    static const int RotorControlsCount = 8;
    static const int messageReceivedTimeout = 10; ///< Seconds 

    size_t status_messages_MaxSize = 5000;

    // std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;
    std::shared_ptr<DroneControllerBase> onboard_vehicle_control_;

    std::mutex heartbeat_mutex_, mocap_pose_mutex_, set_mode_mutex_, status_text_mutex_, last_message_mutex_;
    OnboardDroneController* parent_;

    impl(OnboardDroneController* parent)
        : parent_(parent)
    {
    }

    LinuxSetup* linuxEnvironment = nullptr;

    //variables required for VehicleControllerBase implementation
    ConnectionInfo connection_info_;
    bool is_any_heartbeat_, is_armed_;
    bool is_controls_0_1_; //Are motor controls specified in 0..1 or -1..1?
    float rotor_controls_[RotorControlsCount];
    std::queue<std::string> status_messages_;
    int hil_state_freq_;
    bool actuators_message_supported_;
    const SensorCollection* sensors_;    //this is optional
    uint64_t last_gps_time_;
    bool was_reset_;
    Pose debug_pose_;
    std::string is_available_message_;
    bool is_available_;

    //additional variables required for DroneControllerBase implementation
    //this is optional for methods that might not use vehicle commands
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

    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation, int argc, char** argv)
    {
        connection_info_ = connection_info;
        sensors_ = sensors;
        is_simulation_mode_ = is_simulation;

        try {
            DSTATUS(std::string("Initialize Linux environment").c_str());
            linuxEnvironment = new LinuxSetup(argc, argv);
            if (linuxEnvironment == NULL)
            {
                throw std::runtime_error("Error initializing Linux environment");
            }

            
            DSTATUS(std::string("Connect to flight controller").c_str());
            onboard_vehicle_ = linuxEnvironment->getVehicle();
            //connectToVideoServer();
            initializeOnboardSubscriptions();
            setOrigin(false);
            
            is_available_ = true;
        }
        catch (std::exception& ex) {
            is_available_ = false;
            is_available_message_ = Utils::stringf("Failed to connect: %s", ex.what());
        }
    }

    bool isAvailable(std::string& message)
    {
        if (!is_available_)
            message = is_available_message_;
        return is_available_;
    }

    ConnectionInfo getOnboardConnectionInfo()
    {
        return connection_info_;
    }

    void normalizeRotorControls()
    {
        //if rotor controls are in not in 0-1 range then they are in -1 to 1 range in which case
        //we normalize them to 0 to 1 for PX4
        if (!is_controls_0_1_) {
            // change -1 to 1 to 0 to 1.
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (rotor_controls_[i] + 1.0f) / 2.0f;
            }
        }
        else {
            //this applies to PX4 and may work differently on other firmwares. 
            //We use 0.2 as idle rotors which leaves out range of 0.8
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = Utils::clip(0.8f * rotor_controls_[i] + 0.20f, 0.0f, 1.0f);
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
            std::cout << "Set origin point" << "\n";
            addStatusMessage(std::string("Set origin point").c_str());
        }
    }

    void initializeOnboardSubscriptions()
    {
        if (onboard_vehicle_ != nullptr) {
            DSTATUS(std::string("Initialize subscriptions").c_str());
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
            DSTATUS(std::string("Started flight controller subscriptions").c_str());
        }
    }

    // mavlinkcom::MavLinkHilSensor getLastSensorMessage()
    // {
    //     std::lock_guard<std::mutex> guard(last_message_mutex_);
    //     return last_sensor_message_;
    // }

    // mavlinkcom::MavLinkDistanceSensor getLastDistanceMessage()
    // {
    //     std::lock_guard<std::mutex> guard(last_message_mutex_);
    //     return last_distance_message_;
    // }

    // mavlinkcom::MavLinkHilGps getLastGpsMessage()
    // {
    //     std::lock_guard<std::mutex> guard(last_message_mutex_);
    //     return last_gps_message_;
    // }

    void setArmed(bool armed)
    {
        is_armed_ = armed;
        if (!armed) {
            //reset motor controls
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = 0;
            }
        }
    }

    void addStatusMessage(const std::string& message)
    {
        std::lock_guard<std::mutex> guard_status(status_text_mutex_);
        //if queue became too large, clear it first
        if (status_messages_.size() > status_messages_MaxSize)
            Utils::clear(status_messages_, status_messages_MaxSize - status_messages_.size());
        status_messages_.push(message);
    }

    real_T getVertexControlSignal(unsigned int rotor_index)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to read simulated motor controls while not in simulation mode");

        // std::lock_guard<std::mutex> guard(hil_controls_mutex_);
        //return rotor_controls_[rotor_index];
        return 0;
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
        debug_pose_ = Pose::nanPose();
    }

    //*** Start: VehicleControllerBase implementation ***//
    void reset()
    {
        resetState();
        was_reset_ = true;
        setNormalMode();
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

    void update()
    {
        if (sensors_ == nullptr || onboard_vehicle_ == nullptr 
            || !(onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() == VehicleStatus::FlightStatus::ON_GROUND))
        {
            return;
        }

        //send sensor updates
        // const auto& imu_output = getImu()->getOutput();
        // const auto& mag_output = getMagnetometer()->getOutput();
        // const auto& baro_output = getBarometer()->getOutput();

        // sendHILSensor(imu_output.linear_acceleration,
        //     imu_output.angular_velocity,
        //     mag_output.magnetic_field_body,
        //     baro_output.pressure * 0.01f /*Pa to Milibar */, baro_output.altitude);


        const auto * distance = getDistance();
        if (distance) {
          const auto& distance_output = distance->getOutput();
          float pitch, roll, yaw;
          VectorMath::toEulerianAngle(distance_output.relative_pose.orientation, pitch, roll, yaw);

        //   sendDistanceSensor(distance_output.min_distance / 100, //m -> cm
        //                      distance_output.max_distance / 100, //m -> cm
        //                      distance_output.distance,
        //                      0, //sensor type: //TODO: allow changing in settings?
        //                      77, //sensor id, //TODO: should this be something real?
        //                      pitch); //TODO: convert from radians to degrees?
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

                // sendHILGps(gps_output.gnss.geo_point, gps_velocity, gps_velocity_xy.norm(), gps_cog,
                //     gps_output.gnss.eph, gps_output.gnss.epv, gps_output.gnss.fix_type, 10);
            }
        }
        
        //must be done at the end
        if (was_reset_)
            was_reset_ = false;
    }

    void getStatusMessages(std::vector<std::string>& messages)
    {
        messages.clear();
        std::lock_guard<std::mutex> guard(status_text_mutex_);

        while (!status_messages_.empty()) {
            messages.push_back(status_messages_.front());
            status_messages_.pop();
        }
    }

    void closeAllConnection()
    {
        close();
    }

    //*** End: VehicleControllerBase implementation ***//

    bool hasVideoRequest()
    {
        // mavlinkcom::MavLinkVideoServer::MavLinkVideoRequest image_req;
        // return video_server_->hasVideoRequest(image_req);
        return false;
    }

    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
    {
        // const int MAVLINK_DATA_STREAM_IMG_PNG = 6;
        // video_server_->sendFrame(data, length, width, height, MAVLINK_DATA_STREAM_IMG_PNG, 0);
    }

    void setNormalMode()
    {
        if (onboard_vehicle_ != nullptr) {

            // remove MAV_MODE_FLAG_HIL_ENABLED flag from current mode 
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

    void close()
    {
        // if (video_server_ != nullptr)
        //     video_server_->close();

        if (onboard_vehicle_ != nullptr) {
            onboard_vehicle_->subscribe->removeLeftOverPackages();
            onboard_vehicle_->releaseCtrlAuthority(1000);
            onboard_vehicle_ = nullptr;
        }
    }

    //additional methods for DroneControllerBase
    void updateState()
    {
        StatusLock lock(parent_);
        if (onboard_vehicle_ != nullptr) {
            int version = onboard_vehicle_->getLastReceivedFrame().recvInfo.version;
            if (version != state_version_)
            {
                current_state = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
                state_version_ = version;
            }
        }
    }

    
    Kinematics::State getKinematicsEstimated()
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

        //TODO: how do we get angular acceleration?
        return state;
    }

    Vector3r getPosition()
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

    Vector3r getVelocity()
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type currentVelocity;
        currentVelocity = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
        return Vector3r(currentVelocity.data.x, currentVelocity.data.y, currentVelocity.data.z);
    }

    Quaternionr getOrientation()
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type currentOrientation;
        currentOrientation = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
        Telemetry::Vector3f euler = toEulerAngle((static_cast<void*>(&currentOrientation)));
        return VectorMath::toQuaternion(euler.x, euler.y, euler.z);
    }

    GeoPoint getHomeGeoPoint()
    {
        updateState();
        return GeoPoint(rad2Deg(originGPS.latitude), rad2Deg(originGPS.longitude), originGPS.altitude);
    }

    GeoPoint getGpsLocation()
    {
        updateState();
        Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
        currentSubscriptionGPS = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
        return GeoPoint(rad2Deg(currentSubscriptionGPS.latitude), rad2Deg(currentSubscriptionGPS.longitude), currentSubscriptionGPS.altitude);
    }

    LandedState getLandedState()
    {
        updateState();
        return current_state == VehicleStatus::FlightStatus::IN_AIR ? LandedState::Flying : LandedState::Landed;
    }

    inline double rad2Deg(const double radians)
    {
        return (radians / M_PI) * 180.0;
    }

    inline double deg2Rad(const double degrees)
    {
        return (degrees / 180.0) * M_PI;
    }

    //administrative

    bool armDisarm(bool arm, CancelableBase& cancelable_action)
    {
        unused(cancelable_action);
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

    bool isApiControlEnabled()
    {
        return is_api_control_enabled_;
    }

    bool isSimulationMode()
    {
        return is_simulation_mode_;
    }

    void enableApiControl(bool is_enabled)
    {
        checkVehicle();
        if (is_enabled) {
            onboard_vehicle_->obtainCtrlAuthority(1000);
            is_api_control_enabled_ = true;
            addStatusMessage(std::string("Obtained Vehicle Control Authority"));
        }
        else {
            onboard_vehicle_->releaseCtrlAuthority(1000);
            is_api_control_enabled_ = false;
            addStatusMessage(std::string("Released Vehicle Control Authority"));
        }
    }

    void setSimulationMode(bool is_set)
    {        
        is_simulation_mode_ = is_set;
    }

    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action)
    {
        unused(cancelable_action);
        checkVehicle();

        auto vec = getPosition();
        float z = vec.z() + getTakeoffZ();
        int  timeout = 100000;

        char func[50];

        // Telemetry: Verify the subscription
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
        addStatusMessage(std::string("Start Takeoff"));
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
            fprintf(stderr, "Motors not started %d, %d\n", onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>(), onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() );
            updateState();
            usleep(1000000);
        }

        if (motorsNotStarted == timeoutCycles)
        {
            std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
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
            std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                        "motors are spinning."
                    << std::endl;
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
            std::cout
            << "Takeoff finished, but the aircraft is in an unexpected mode. "
                "Please connect DJI GO.\n";
            // onboard_vehicle_->subscribe->removePackage(0, timeout);
            throw VehicleMoveException("Takeoff finished, but the aircraft is in an unexpected mode.");
        }
        
        if (max_wait_seconds <= 0)
            return true; // client doesn't want to wait.

        return parent_->waitForZ(max_wait_seconds, z, getDistanceAccuracy(), cancelable_action);
    }

    bool hover(CancelableBase& cancelable_action)
    {
        bool rc = false;
        checkVehicle();

        // if mission running pause mission
        onboard_vehicle_->control->emergencyBrake();
        //auto start_time = std::chrono::system_clock::now();
        while (!cancelable_action.isCancelled())
        {
        }
        // if mission was running resume mission
        return rc;
    }

    bool land(float max_wait_seconds, CancelableBase& cancelable_action)
    {
        char func[50];

        unused(cancelable_action);
        // bugbug: really need a downward pointing distance to ground sensor to do this properly, for now
        // we assume the ground is relatively flat an we are landing roughly at the home altitude.
        updateState();
        checkVehicle();
        
        // Telemetry: Verify the subscription
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
            std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
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
            std::cout
            << "Landing finished, but the aircraft is in an unexpected mode. "
                "Please connect DJI GO.\n";
            return false;
        }
        return true;
    }

    bool goHome(CancelableBase& cancelable_action)
    {
        char func[50];
  
        unused(cancelable_action);
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
                std::cout << "Return home command failed. Aircraft is in incorrect mode." << std::endl;
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
                std::cout
                << "Return home finished, but the aircraft is in an unexpected mode.\n";
            }
        }
        return rc;
    }

    void commandRollPitchZ(float pitch, float roll, float z, float yaw)
    {
        addStatusMessage(std::string("Received Command RollPitchZ"));
        checkVehicle();
        onboard_vehicle_->control->attitudeAndVertPosCtrl(roll, pitch, yaw, -z);
    }

    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
    {
        addStatusMessage(std::string("Received Command Velocity,") + "vx:" + std::to_string(vx) + ",vy:" + std::to_string(vy) + ",vz:" + std::to_string(vz) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
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

    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
    {
        addStatusMessage(std::string("Received Command VelocityZ,") + "vx:" + std::to_string(vx) + ",vy:" + std::to_string(vy) + ",z:" + std::to_string(z) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
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

    void commandPosition(float x, float y, float z, const YawMode& yaw_mode)
    {
        addStatusMessage(std::string("Received Command Position,") + "x:" + std::to_string(x) + ",y:" + std::to_string(y) + ",z:" + std::to_string(z) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));
        checkVehicle();
        
        Vector3r vec = getPosition();

        x = x - vec.x();
        y = y - vec.y();
        
        addStatusMessage(std::string("Use Position Offset,") + "x:" + std::to_string(x) + ",y:" + std::to_string(y) + ",z:" + std::to_string(z) + ",yaw or rate:" + std::to_string(yaw_mode.yaw_or_rate));

        uint8_t yaw_logic = DJI::OSDK::Control::YawLogic::YAW_RATE;
        if (!yaw_mode.is_rate)
        {
            yaw_logic = DJI::OSDK::Control::YawLogic::YAW_ANGLE;
        }
        uint8_t mode = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_POSITION |
            DJI::OSDK::Control::VerticalLogic::VERTICAL_POSITION |
            yaw_logic |
            DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_GROUND;
        if (x < 0.001f && x > -0.001f) 
        {
            x = 0.001f;
        }
        DJI::OSDK::Control::CtrlData flightControl(mode, x, y, -z, yaw_mode.yaw_or_rate);
        onboard_vehicle_->control->flightCtrl(flightControl);
    }

    RCData getRCData()
    {
        throw VehicleCommandNotImplementedException("getRCData() function is not yet implemented");
    }

    void setRCData(const RCData& rcData)
    {
        unused(rcData);
        //TODO: use RC data to control MavLink drone
    }

    bool validateRCData(const RCData& rc_data)
    {
        unused(rc_data);
        return true;
    }

    //drone parameters
    float getCommandPeriod()
    {
        return 1.0f / 50; //1 period of 50hz
    }
    float getTakeoffZ()
    {
        // pick a number, PX4 doesn't have a fixed limit here, but 3 meters is probably safe 
        // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
        return -1.1f;
    }
    float getDistanceAccuracy()
    {
        return 0.1f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }
    const VehicleParams& getVehicleParams()
    {
        return getInternalVehicleParams(); //defaults are good for generic quadrocopter.
    }
    //TODO: decouple DroneControllerBase, VehicalParams and SafetyEval
    const VehicleParams& getInternalVehicleParams()
    {
        static const VehicleParams vehicle_params_;
        return vehicle_params_; //defaults are good for DJI Matrice 100
    }

    void reportTelemetry(float renderTime)
    {
        if (onboard_vehicle_ == nullptr) {
            return;
        }
    }

    Pose getDebugPose()
    {
        std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
        return debug_pose_;
    }

    bool startOffboardMode()
    {
        checkVehicle();
        addStatusMessage(std::string("Obtain Vehicle Control Authority"));
        try {
            onboard_vehicle_->obtainCtrlAuthority(1000);
        }
        catch (std::exception& ex) {
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
        addStatusMessage(std::string("Released Vehicle Control Authority"));
    }

    void ensureSafeMode()
    {
        if (onboard_vehicle_ != nullptr) {
            const int state = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
            if (state == VehicleStatus::FlightStatus::ON_GROUND || !VehicleStatus::FlightStatus::STOPED) {
                return;
            }
        }
    }

    bool loopCommandPre()
    {
        return startOffboardMode();
    }

    void loopCommandPost()
    {
        endOffboardMode();
    }

    void checkVehicle() {
        if (onboard_vehicle_ == nullptr) {
            throw std::logic_error("Cannot perform operation when no vehicle is connected");
        }
    }


    /*! Very simple calculation of local NED offset between two pairs of GPS
    /coordinates.
        Accurate when distances are small.
    !*/
    void
    localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed, void* target, void* origin)
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
        double t1 =
            +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
        double t2 =
            -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
        double t3 =
            +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
        double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

        t2 = (t2 > 1.0) ? 1.0 : t2;
        t2 = (t2 < -1.0) ? -1.0 : t2;

        ans.x = asin(t2);
        ans.y = atan2(t3, t4);
        ans.z = atan2(t1, t0);

        return ans;
    }

}; //impl

//empty constructor required for pimpl
OnboardDroneController::OnboardDroneController()
{
    pimpl_.reset(new impl(this));
}

OnboardDroneController::~OnboardDroneController()
{
    pimpl_->closeAllConnection();
}

void OnboardDroneController::initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation, int argc, char** argv)
{
    pimpl_->initialize(connection_info, sensors, is_simulation, argc, argv);
}

OnboardDroneController::ConnectionInfo OnboardDroneController::getOnboardConnectionInfo()
{
    return pimpl_->getOnboardConnectionInfo();
}
void OnboardDroneController::sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
{
    pimpl_->sendImage(data, length, width, height);
}

bool OnboardDroneController::hasVideoRequest()
{
    return pimpl_->hasVideoRequest();
}

//*** Start: VehicleControllerBase implementation ***//
void OnboardDroneController::reset()
{
    DroneControllerBase::reset();
    pimpl_->reset();
}
void OnboardDroneController::update()
{
    DroneControllerBase::update();
    pimpl_->update();
}
real_T OnboardDroneController::getVertexControlSignal(unsigned int rotor_index)
{
    return pimpl_->getVertexControlSignal(rotor_index);
}
size_t OnboardDroneController::getVertexCount()
{
    return impl::RotorControlsCount;
}
void OnboardDroneController::getStatusMessages(std::vector<std::string>& messages)
{
    pimpl_->getStatusMessages(messages);
}
bool OnboardDroneController::isAvailable(std::string& message)
{
    return pimpl_->isAvailable(message);
}

//*** End: VehicleControllerBase implementation ***//



//DroneControlBase
Kinematics::State OnboardDroneController::getKinematicsEstimated()
{
    return pimpl_->getKinematicsEstimated();
}

Vector3r OnboardDroneController::getPosition()
{
    return pimpl_->getPosition();
}

Vector3r OnboardDroneController::getVelocity()
{
    return pimpl_->getVelocity();
}

Quaternionr OnboardDroneController::getOrientation()
{
    return pimpl_->getOrientation();
}

GeoPoint OnboardDroneController::getHomeGeoPoint()
{
    return pimpl_->getHomeGeoPoint();
}

GeoPoint OnboardDroneController::getGpsLocation()
{
    return pimpl_->getGpsLocation();
}

DroneControllerBase::LandedState OnboardDroneController::getLandedState()
{
    return pimpl_->getLandedState();
}
//administrative

bool OnboardDroneController::armDisarm(bool arm, CancelableBase& cancelable_action)
{
    return pimpl_->armDisarm(arm, cancelable_action);
}

void OnboardDroneController::enableApiControl(bool is_enabled)
{
    pimpl_->enableApiControl(is_enabled);
}
void OnboardDroneController::setSimulationMode(bool is_set)
{
    pimpl_->setSimulationMode(is_set);
}
bool OnboardDroneController::isApiControlEnabled()
{
    return pimpl_->isApiControlEnabled();
}
bool OnboardDroneController::isSimulationMode()
{
    return pimpl_->isSimulationMode();
}

bool OnboardDroneController::takeoff(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->takeoff(max_wait_seconds, cancelable_action);
}

bool OnboardDroneController::hover(CancelableBase& cancelable_action)
{
    return pimpl_->hover(cancelable_action);
}

bool OnboardDroneController::land(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->land(max_wait_seconds, cancelable_action);
}

bool OnboardDroneController::goHome(CancelableBase& cancelable_action)
{
    return pimpl_->goHome(cancelable_action);
}

void OnboardDroneController::commandRollPitchZ(float pitch, float roll, float z, float yaw)
{
    return pimpl_->commandRollPitchZ(pitch, roll, z, yaw);
}
void OnboardDroneController::commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocity(vx, vy, vz, yaw_mode);
}
void OnboardDroneController::commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocityZ(vx, vy, z, yaw_mode);
}
void OnboardDroneController::commandPosition(float x, float y, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandPosition(x, y, z, yaw_mode);
}

RCData OnboardDroneController::getRCData()
{
    return pimpl_->getRCData();
}
void OnboardDroneController::setRCData(const RCData& rcData)
{
    return pimpl_->setRCData(rcData);
}

bool OnboardDroneController::loopCommandPre()
{
    return pimpl_->loopCommandPre();
}

void OnboardDroneController::loopCommandPost()
{
    pimpl_->loopCommandPost();
}

//drone parameters
float OnboardDroneController::getCommandPeriod()
{
    return pimpl_->getCommandPeriod();
}
float OnboardDroneController::getTakeoffZ()
{
    return pimpl_->getTakeoffZ();
}
float OnboardDroneController::getDistanceAccuracy()
{
    return pimpl_->getDistanceAccuracy();
}
const VehicleParams& OnboardDroneController::getVehicleParams()
{
    return pimpl_->getVehicleParams();
}
//TODO: decouple DroneControllerBase, VehicalParams and SafetyEval

void OnboardDroneController::reportTelemetry(float renderTime)
{
    return pimpl_->reportTelemetry(renderTime);
}

Pose OnboardDroneController::getDebugPose()
{
    return pimpl_->getDebugPose();
}

}} //namespace
#endif
