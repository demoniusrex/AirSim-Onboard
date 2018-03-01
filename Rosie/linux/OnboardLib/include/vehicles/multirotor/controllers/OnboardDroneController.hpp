// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_OnBoardDroneController_hpp
#define msr_airlib_OnBoardDroneController_hpp

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
#include "vehicles/multirotor//MultiRotor.hpp"
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

#include "dji_telemetry.hpp"
#include "dji_vehicle.hpp"

using namespace DJI::OSDK;
using namespace std;
using namespace msr::airlib;

namespace Rosie { namespace OnboardLib {

class OnBoardDroneController : public DroneControllerBase
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
    OnBoardDroneController();
    virtual ~OnBoardDroneController();

    //non-base interface specific to OnBoardDroneController
    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation);
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

struct OnBoardDroneController::impl {
public:
    // static const int onboardFlightCommandStop = 9900;   ///< 
    static const int RotorControlsCount = 8;
    static const int messageReceivedTimeout = 10; ///< Seconds 

    size_t status_messages_MaxSize = 5000;

    // std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;
    std::shared_ptr<DroneControllerBase> onboard_vehicle_control_;

    std::mutex heartbeat_mutex_, mocap_pose_mutex_, set_mode_mutex_, status_text_mutex_, last_message_mutex_;
    OnBoardDroneController* parent_;

    impl(OnBoardDroneController* parent)
        : parent_(parent)
    {
    }

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
    std::shared_ptr<Vehicle> onboard_vehicle_;
    int state_version_;
    int current_state;
    float target_height_;
    bool is_api_control_enabled_;
    bool is_simulation_mode_;
    PidController thrust_controller_;
    common_utils::Timer hil_message_timer_;
    common_utils::Timer sitl_message_timer_;

    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
    {
        connection_info_ = connection_info;
        sensors_ = sensors;
        is_simulation_mode_ = is_simulation;

        try {
            openAllConnections();
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

    void initializeOnboardSubscriptions()
    {
        if (onboard_vehicle_ != nullptr) {
            is_any_heartbeat_ = false;
            is_armed_ = false;
            is_controls_0_1_ = true;
            Utils::setValue(rotor_controls_, 0.0f);
            //TODO: main_node_->setMessageInterval(...);
            // connection_->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
            //     unused(connection);
            //     processMavMessages(msg);
            // });

            // listen to the other mavlink connection also
            // auto mavcon = onboard_vehicle_->getConnection();
            // if (mavcon != connection_) {
            //     mavcon->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
            //         unused(connection);
            //         processMavMessages(msg);
            //     });
            // }
        }
    }

    void connect()
    {
        createOnboardConnection(connection_info_);
        initializeOnboardSubscriptions();
    }

    void createOnboardConnection(const ConnectionInfo& connection_info)
    {
        if (connection_info.use_serial) {
            createOnboardSerialConnection(connection_info.serial_port, connection_info.baud_rate);
        }
        else {
            createOnboardUdpConnection(connection_info.ip_address, connection_info.ip_port);
        }
        //Uncomment below for sending images over MavLink
        //connectToVideoServer();
    }

    void createOnboardUdpConnection(const std::string& ip, int port)
    {
        close();

        if (ip == "") {
            throw std::invalid_argument("UdpIp setting for Onboard flight controller is invalid.");
        }

        if (port == 0) {
            throw std::invalid_argument("UdpPort setting for Onboard flight controller has an invalid value.");
        }

        addStatusMessage(Utils::stringf("Connecting to Onboard flight controller UDP port %d, IP %s", port, ip.c_str()));
        // connection_ = mavlinkcom::MavLinkConnection::connectRemoteUdp("hil", connection_info_.local_host_ip, ip, port);
        // hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        // hil_node_->connect(connection_);
        addStatusMessage(std::string("Connected to Onboard flight controller over UDP."));

        onboard_vehicle_ = std::make_shared<Vehicle>();

        // onboard_vehicle_->connect(connection_);
        // onboard_vehicle_->startHeartbeat();
    }

    void createOnboardSerialConnection(const std::string& port_name, int baud_rate)
    {
        close();

        std::string port_name_auto = port_name;
        
        if (port_name_auto == "") {
            throw std::invalid_argument("Serial port for Onboard flight controller is empty. Please set it in settings.json.");
        }

        if (baud_rate == 0) {
            throw std::invalid_argument("Baud rate specified in settings.json is 0 which is invalid");
        }

        addStatusMessage(Utils::stringf("Connecting to Onboard flight controller over serial port: %s, baud rate %d ....", port_name_auto.c_str(), baud_rate));
        // connection_ = mavlinkcom::MavLinkConnection::connectSerial("hil", port_name_auto, baud_rate);
        // connection_->ignoreMessage(mavlinkcom::MavLinkAttPosMocap::kMessageId); //TODO: find better way to communicate debug pose instead of using fake Mocap messages
        // hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        // hil_node_->connect(connection_);
        addStatusMessage("Connected to Onboard flight controller over serial port.");

        onboard_vehicle_ = std::make_shared<Vehicle>();
        // onboard_vehicle_->connect(connection_); // in this case we can use the same connection.
        // onboard_vehicle_->startHeartbeat();
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

    // void processMavMessages(const mavlinkcom::MavLinkMessage& msg)
    // {
    //     if (msg.msgid == HeartbeatMessage.msgid) {
    //         std::lock_guard<std::mutex> guard_heartbeat(heartbeat_mutex_);

    //         //TODO: have MavLinkNode track armed state so we don't have to re-decode message here again
    //         HeartbeatMessage.decode(msg);
    //         bool armed = (HeartbeatMessage.base_mode & static_cast<uint8_t>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED)) > 0;
    //         setArmed(armed);
    //         if (!is_any_heartbeat_) {
    //             is_any_heartbeat_ = true;
    //             if (HeartbeatMessage.autopilot == static_cast<uint8_t>(mavlinkcom::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4) &&
    //                 HeartbeatMessage.type == static_cast<uint8_t>(mavlinkcom::MAV_TYPE::MAV_TYPE_FIXED_WING)) {
    //                 // PX4 will scale fixed wing servo outputs to -1 to 1
    //                 // and it scales multi rotor servo outpus to 0 to 1.
    //                 is_controls_0_1_ = false;
    //             }
    //         } else if (is_simulation_mode_ && !is_hil_mode_set_) {
    //             setHILMode();
    //         }
    //     } else if (msg.msgid == StatusTextMessage.msgid) {
    //         StatusTextMessage.decode(msg);
    //         //lock is established by below method
    //         addStatusMessage(std::string(StatusTextMessage.text));
    //     } else if (msg.msgid == CommandLongMessage.msgid) {
    //         CommandLongMessage.decode(msg);
    //         if (CommandLongMessage.command == static_cast<int>(mavlinkcom::MAV_CMD::MAV_CMD_SET_MESSAGE_INTERVAL)) {
    //             int msg_id = static_cast<int>(CommandLongMessage.param1 + 0.5);
    //             if (msg_id == 115) { //HIL_STATE_QUATERNION
    //                 hil_state_freq_ = static_cast<int>(CommandLongMessage.param2 + 0.5);
    //             }
    //         }
    //     } else if (msg.msgid == HilControlsMessage.msgid) {
    //         if (!actuators_message_supported_) {
    //             std::lock_guard<std::mutex> guard_controls(hil_controls_mutex_);

    //             HilControlsMessage.decode(msg);
    //             //is_arned_ = (HilControlsMessage.mode & 128) > 0; //TODO: is this needed?
    //             rotor_controls_[0] = HilControlsMessage.roll_ailerons;
    //             rotor_controls_[1] = HilControlsMessage.pitch_elevator;
    //             rotor_controls_[2] = HilControlsMessage.yaw_rudder;
    //             rotor_controls_[3] = HilControlsMessage.throttle;
    //             rotor_controls_[4] = HilControlsMessage.aux1;
    //             rotor_controls_[5] = HilControlsMessage.aux2;
    //             rotor_controls_[6] = HilControlsMessage.aux3;
    //             rotor_controls_[7] = HilControlsMessage.aux4;

    //             normalizeRotorControls();
    //         }
    //     }
    //     else if (msg.msgid == HilActuatorControlsMessage.msgid) {
    //         actuators_message_supported_ = true;

    //         std::lock_guard<std::mutex> guard_actuator(hil_controls_mutex_);    //use same mutex as HIL_CONTROl

    //         HilActuatorControlsMessage.decode(msg);
    //         //is_arned_ = (HilControlsMessage.mode & 128) > 0; //TODO: is this needed?
    //         for (auto i = 0; i < 8; ++i) {
    //             rotor_controls_[i] = HilActuatorControlsMessage.controls[i];
    //         }
    //         normalizeRotorControls();
    //     }
    //     //else ignore message
    // }

   

    real_T getVertexControlSignal(unsigned int rotor_index)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to read simulated motor controls while not in simulation mode");

        // std::lock_guard<std::mutex> guard(hil_controls_mutex_);
        //return rotor_controls_[rotor_index];
        return NULL;
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
        const auto& imu_output = getImu()->getOutput();
        const auto& mag_output = getMagnetometer()->getOutput();
        const auto& baro_output = getBarometer()->getOutput();

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

    void openAllConnections()
    {
        close(); //just in case if connections were open
        resetState(); //reset all variables we might have changed during last session

        connect();

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
        const int MAVLINK_DATA_STREAM_IMG_PNG = 6;
        // video_server_->sendFrame(data, length, width, height, MAVLINK_DATA_STREAM_IMG_PNG, 0);
    }

    void setNormalMode()
    {
        if (onboard_vehicle_ != nullptr) {

            // remove MAV_MODE_FLAG_HIL_ENABLED flag from current mode 
            std::lock_guard<std::mutex> guard(set_mode_mutex_);
            int mode = onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
            
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

    /*
        struct GlobalPosition {
            float lat = 0, lon = 0, alt = 0; 
        };
        struct Vector3 {
            float x = 0, y = 0, z = 0; // in NED (north, east, down) coordinates
        };
        struct LocalPose {
            Vector3 pos;  // in NEU (north, east, down) coordinates
            float q[4] = { 0 }; //qauternion
        };

        struct AttitudeState {
            float roll = 0, pitch = 0, yaw = 0, roll_rate = 0, yaw_rate = 0, pitch_rate = 0;
            uint64_t updated_on = 0;
        } attitude;

        struct GlobalState {
            GlobalPosition pos;
            Vector3 vel;
            int alt_ground = 0;
            float heading = 0;
            uint64_t updated_on = 0;
        } global_est;

        struct RCState {
            int16_t rc_channels_scaled[16] = { 0 };
            unsigned char rc_channels_count = 0;
            unsigned char rc_signal_strength = 0;
            uint64_t updated_on = 0;
        } rc;

        struct ServoState {
            unsigned int servo_raw[8] = { 0 };
            unsigned char servo_port = 0;
            uint64_t updated_on = 0;
        } servo;

        struct ControlState {
            float actuator_controls[16] = { 0 };
            unsigned char actuator_mode = 0, actuator_nav_mode = 0;
            bool landed = 0;
            bool armed = false;
            bool offboard = false;
            uint64_t updated_on = 0;
        } controls;

        struct LocalState {
            Vector3 pos; // in NEU (north, east, up) coordinates (positive Z goes upwards).
            Vector3 lin_vel;
            Vector3 acc;
            uint64_t updated_on;
        } local_est;

        struct MocapState {
            LocalPose pose;
            uint64_t updated_on = 0;
        } mocap;

        struct AltitudeState {
            uint64_t updated_on = 0;
            float altitude_monotonic = 0, altitude_amsl = 0, altitude_terrain = 0;
            float altitude_local = 0, altitude_relative = 0, bottom_clearance = 0;
        } altitude;

        struct VfrHud {
            float true_airspeed = 0; // in m/s
            float groundspeed = 0; // in m/s
            float altitude = 0; // MSL altitude in meters
            float climb_rate = 0; // in m/s
            int16_t heading = 0; // in degrees w.r.t. north
            uint16_t throttle = 0; // in percent, 0 to 100
        } vfrhud;

        struct HomeState {
            GlobalPosition global_pos;
            LocalPose local_pose;
            Vector3 approach; // in NEU (north, east, up) coordinates (positive Z goes upwards).
            bool is_set = false;
        } home;

        struct Stats {	//mainly for debugging purposes
            int last_read_msg_id = 0;
            uint64_t last_read_msg_time = 0;
            int last_write_msg_id = 0;
            uint64_t last_write_msg_time = 0;
            std::string debug_msg;
        } stats;


    */
    
    Kinematics::State getKinematicsEstimated()
    {
        updateState();
        Kinematics::State state;
        //TODO: reduce code duplication below?
        state.pose.position = Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
        state.pose.orientation = VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
        state.twist.linear = Vector3r(current_state.local_est.lin_vel.x, current_state.local_est.lin_vel.y, current_state.local_est.lin_vel.z);
        state.twist.angular = Vector3r(current_state.attitude.roll_rate, current_state.attitude.pitch_rate, current_state.attitude.yaw_rate);
        state.pose.position = Vector3r(current_state.local_est.acc.x, current_state.local_est.acc.y, current_state.local_est.acc.z);
        //TODO: how do we get angular acceleration?
        return state;
    }

    Vector3r getPosition()
    {
        updateState();
        return Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
    }

    Vector3r getVelocity()
    {
        updateState();
        return Vector3r(current_state.local_est.lin_vel.x, current_state.local_est.lin_vel.y, current_state.local_est.lin_vel.z);
    }

    Quaternionr getOrientation()
    {
        updateState();
        return VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
    }

    GeoPoint getHomeGeoPoint()
    {
        updateState();
        if (current_state.home.is_set)
            return GeoPoint(current_state.home.global_pos.lat, current_state.home.global_pos.lon, current_state.home.global_pos.alt);
        else
            return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }

    GeoPoint getGpsLocation()
    {
        updateState();
        return GeoPoint(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.global_est.pos.alt);
    }


    LandedState getLandedState()
    {
        updateState();
        return current_state.controls.landed ? LandedState::Landed : LandedState::Flying;
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
        }
        else 
        {
            onboard_vehicle_->control->disArmMotors(1000);
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
        }
        else {
            onboard_vehicle_->releaseCtrlAuthority(1000);
            is_api_control_enabled_ = false;
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

        bool rc = false;
        auto vec = getPosition();
        float z = vec.z() + getTakeoffZ();
        int  timeout = 1000;

        char func[50];
        int  pkgIndex;

        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = onboard_vehicle_->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            throw VehicleMoveException("Takeoff failed. Unable to verify vehicle subscription");
        }

        // Start takeoff
        std::cout << "Start Takeoff" << std::endl;
        ACK::ErrorCode takeoffStatus = onboard_vehicle_->control->takeoff(timeout);
        if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(takeoffStatus, func);
            throw VehicleMoveException("Takeoff failed. Error sending takeoff command.");
        }

        // First check: Motors started
        std::cout << "Check motors" << std::endl;
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while (onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
                    VehicleStatus::FlightStatus::ON_GROUND &&
                onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_ENGINE_START &&
                motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            fprintf(stderr, "Motors not started %d, %d\n", onboard_vehicle_->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>(), onboard_vehicle_->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() );
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
            std::cout << "Motors spinning...\n";
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
            std::cout << "Ascending...\n";
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
            std::cout << "Successful takeoff!\n";
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

        onboard_vehicle_->missionManager->wpMission->pause();
        onboard_vehicle_->control->emergencyBrake();

        //auto start_time = std::chrono::system_clock::now();
        while (!cancelable_action.isCancelled())
        {

        }
        onboard_vehicle_->missionManager->wpMission->resume(1000);
        return rc;
    }

    bool land(float max_wait_seconds, CancelableBase& cancelable_action)
    {
        char func[50];
        int  pkgIndex;

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
            std::cout << "Landing...\n";
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
            std::cout << "Successful landing!\n";
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
                std::cout << "Returning Home...\n";
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
                std::cout << "Returned home!\n";
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
        if (target_height_ != -z) {
            // these PID values were calculated experimentally using AltHoldCommand n MavLinkTest, this provides the best
            // control over thrust to achieve minimal over/under shoot in a reasonable amount of time, but it has not
            // been tested on a real drone outside jMavSim, so it may need recalibrating...
            thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
            target_height_ = -z;
        }
        checkVehicle();
        //auto state = onboard_vehicle_->getVehicleState();
        //float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
        //onboard_vehicle_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, thrust);
    }
    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
    {
        checkVehicle();
        //float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        //onboard_vehicle_->moveByLocalVelocity(vx, vy, vz, !yaw_mode.is_rate, yaw);
    }
    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
    {
        checkVehicle();
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        //onboard_vehicle_->moveByLocalVelocityWithAltHold(vx, vy, z, !yaw_mode.is_rate, yaw);
    }
    void commandPosition(float x, float y, float z, const YawMode& yaw_mode)
    {
        checkVehicle();
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        //onboard_vehicle_->moveToLocalPosition(x, y, z, !yaw_mode.is_rate, yaw);
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
        return -3.0f;
    }
    float getDistanceAccuracy()
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
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

    Pose getDebugPose()
    {
        std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
        return debug_pose_;
    }

    bool startOffboardMode()
    {
        checkVehicle();
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
}; //impl

   //empty constructor required for pimpl
OnBoardDroneController::OnBoardDroneController()
{
    pimpl_.reset(new impl(this));
}

OnBoardDroneController::~OnBoardDroneController()
{
    pimpl_->closeAllConnection();
}

void OnBoardDroneController::initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
{
    pimpl_->initialize(connection_info, sensors, is_simulation);
}

OnBoardDroneController::ConnectionInfo OnBoardDroneController::getOnboardConnectionInfo()
{
    return pimpl_->getOnboardConnectionInfo();
}
void OnBoardDroneController::sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
{
    pimpl_->sendImage(data, length, width, height);
}

bool OnBoardDroneController::hasVideoRequest()
{
    return pimpl_->hasVideoRequest();
}

//*** Start: VehicleControllerBase implementation ***//
void OnBoardDroneController::reset()
{
    DroneControllerBase::reset();
    pimpl_->reset();
}
void OnBoardDroneController::update()
{
    DroneControllerBase::update();
    pimpl_->update();
}
real_T OnBoardDroneController::getVertexControlSignal(unsigned int rotor_index)
{
    return pimpl_->getVertexControlSignal(rotor_index);
}
size_t OnBoardDroneController::getVertexCount()
{
    return impl::RotorControlsCount;
}
void OnBoardDroneController::getStatusMessages(std::vector<std::string>& messages)
{
    pimpl_->getStatusMessages(messages);
}
bool OnBoardDroneController::isAvailable(std::string& message)
{
    return pimpl_->isAvailable(message);
}

//*** End: VehicleControllerBase implementation ***//



//DroneControlBase
Kinematics::State OnBoardDroneController::getKinematicsEstimated()
{
    return pimpl_->getKinematicsEstimated();
}

Vector3r OnBoardDroneController::getPosition()
{
    return pimpl_->getPosition();
}

Vector3r OnBoardDroneController::getVelocity()
{
    return pimpl_->getVelocity();
}

Quaternionr OnBoardDroneController::getOrientation()
{
    return pimpl_->getOrientation();
}

GeoPoint OnBoardDroneController::getHomeGeoPoint()
{
    return pimpl_->getHomeGeoPoint();
}

GeoPoint OnBoardDroneController::getGpsLocation()
{
    return pimpl_->getGpsLocation();
}

DroneControllerBase::LandedState OnBoardDroneController::getLandedState()
{
    return pimpl_->getLandedState();
}
//administrative

bool OnBoardDroneController::armDisarm(bool arm, CancelableBase& cancelable_action)
{
    return pimpl_->armDisarm(arm, cancelable_action);
}


void OnBoardDroneController::enableApiControl(bool is_enabled)
{
    pimpl_->enableApiControl(is_enabled);
}
void OnBoardDroneController::setSimulationMode(bool is_set)
{
    pimpl_->setSimulationMode(is_set);
}
bool OnBoardDroneController::isApiControlEnabled()
{
    return pimpl_->isApiControlEnabled();
}
bool OnBoardDroneController::isSimulationMode()
{
    return pimpl_->isSimulationMode();
}

bool OnBoardDroneController::takeoff(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->takeoff(max_wait_seconds, cancelable_action);
}

bool OnBoardDroneController::hover(CancelableBase& cancelable_action)
{
    return pimpl_->hover(cancelable_action);
}

bool OnBoardDroneController::land(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->land(max_wait_seconds, cancelable_action);
}

bool OnBoardDroneController::goHome(CancelableBase& cancelable_action)
{
    return pimpl_->goHome(cancelable_action);
}

void OnBoardDroneController::commandRollPitchZ(float pitch, float roll, float z, float yaw)
{
    return pimpl_->commandRollPitchZ(pitch, roll, z, yaw);
}
void OnBoardDroneController::commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocity(vx, vy, vz, yaw_mode);
}
void OnBoardDroneController::commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocityZ(vx, vy, z, yaw_mode);
}
void OnBoardDroneController::commandPosition(float x, float y, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandPosition(x, y, z, yaw_mode);
}

RCData OnBoardDroneController::getRCData()
{
    return pimpl_->getRCData();
}
void OnBoardDroneController::setRCData(const RCData& rcData)
{
    return pimpl_->setRCData(rcData);
}

bool OnBoardDroneController::loopCommandPre()
{
    return pimpl_->loopCommandPre();
}

void OnBoardDroneController::loopCommandPost()
{
    pimpl_->loopCommandPost();
}

//drone parameters
float OnBoardDroneController::getCommandPeriod()
{
    return pimpl_->getCommandPeriod();
}
float OnBoardDroneController::getTakeoffZ()
{
    return pimpl_->getTakeoffZ();
}
float OnBoardDroneController::getDistanceAccuracy()
{
    return pimpl_->getDistanceAccuracy();
}
const VehicleParams& OnBoardDroneController::getVehicleParams()
{
    return pimpl_->getVehicleParams();
}
//TODO: decouple DroneControllerBase, VehicalParams and SafetyEval

Pose OnBoardDroneController::getDebugPose()
{
    return pimpl_->getDebugPose();
}

}} //namespace
#endif