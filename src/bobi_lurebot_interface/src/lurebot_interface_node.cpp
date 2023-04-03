#include <ros/ros.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/MotorVelocitiesStamped.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/ReturnVelocity.h>
#include <bobi_msgs/ReturnTemperature.h>
#include <bobi_msgs/ReturnDroppedMsgs.h>
#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/ProximitySensors.h>
#include <bobi_msgs/DroppedMessages.h>
#include <bobi_msgs/DutyCycle.h>
#include <bobi_msgs/Temperature.h>

#include <simpleble/SimpleBLE.h>
#include <simpledbus/base/Exceptions.h>
#include <bobi_lurebot_interface/lurebot_ble_details.hpp>

#include <iostream>
#include <sstream>

#include <chrono>
#include <mutex>

#include <random>

struct LurebotConfig {
    int rate = 65;
    double panic_time = 0.15;
    bool enable_ir = false;
    bool ret_vel = true;
    bool ret_temp = false;
    bool ret_dropped_msgs = true;
    bool check_heartbeat = true;
    double max_acceleration = 1.0;
    int pwm_duty_cycle_perc = 55;
    int bt_adapter = 0;
    std::string device_uuid;
};

LurebotConfig get_lurebot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    LurebotConfig cfg;
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<double>("panic_time", cfg.panic_time, cfg.panic_time);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("ret_vel", cfg.ret_vel, cfg.ret_vel);
    nh->param<bool>("ret_temp", cfg.ret_temp, cfg.ret_temp);
    nh->param<bool>("check_heartbeat", cfg.check_heartbeat, cfg.check_heartbeat);
    nh->param<bool>("ret_dropped_msgs", cfg.ret_dropped_msgs, cfg.ret_dropped_msgs);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
    nh->param<int>("pwm_duty_cycle_perc", cfg.pwm_duty_cycle_perc, cfg.pwm_duty_cycle_perc);
    nh->param<int>("bt_adapter", cfg.bt_adapter, cfg.bt_adapter);
    nh->getParam("device_uuid", cfg.device_uuid);
    return cfg;
}

class BLEInterface {
public:
    BLEInterface(std::shared_ptr<ros::NodeHandle> nh, LurebotConfig cfg)
        : _nh(nh),
          _cfg(cfg),
          _left_motor_cm_per_s(0.),
          _right_motor_cm_per_s(0.),
          _id_counter(0),
          _num_timeouts(0)
    {
        // subs/pubs
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &BLEInterface::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _dropped_msgs_pub = nh->advertise<bobi_msgs::DroppedMessages>("dropped_msgs", 1);
        _current_motor_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("current_velocities", 1);
        _current_temp_pub = nh->advertise<bobi_msgs::Temperature>("motor_temperature", 1);

        // services
        _enable_ir_srv = _nh->advertiseService("enable_ir", &BLEInterface::_enable_ir_srv_cb, this);
        _enable_ret_vel_srv = _nh->advertiseService("ret_vel", &BLEInterface::_enable_ret_vel_srv_cb, this);
        _enable_ret_temp_srv = _nh->advertiseService("ret_temp", &BLEInterface::_enable_ret_temp_srv_cb, this);
        _enable_ret_dropped_msgs_srv = _nh->advertiseService("ret_dropped_msgs", &BLEInterface::_enable_ret_dropped_msgs_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &BLEInterface::_set_max_accel_srv_cb, this);
        _set_duty_cycle_srv = _nh->advertiseService("set_duty_cycle", &BLEInterface::_set_duty_cycle_srv_cb, this);

        // initialize BLE peripheral
        auto adapter_list = SimpleBLE::Adapter::get_adapters();
        assert(adapter_list.size() && "No adapter was found.");
        _adapter = std::move(adapter_list[cfg.bt_adapter]);

        _adapter.set_callback_on_scan_start([]() { ROS_INFO("BLE scan started."); });
        _adapter.set_callback_on_scan_stop([]() { ROS_INFO("BLE scan stopped."); });
        _adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral peripheral) {
            std::cout << peripheral.identifier() << " " << peripheral.address() << std::endl;
            if (/*peripheral.identifier() == DEVICE_NAME &&*/
                peripheral.address() == cfg.device_uuid) {
                _peripheral = peripheral;
                _peripheral.set_callback_on_connected([]() { ROS_INFO("BLE device successfully connected."); });
                _peripheral.set_callback_on_disconnected([]() { ROS_WARN("BLE device disconnected."); });
            }
        });
        _adapter.scan_for(2500);
        while (!_peripheral.initialized()) {
            ROS_ERROR("Trying to intitialize BT peripheral");
            ros::Rate wait(1);
            wait.sleep();
        }
        assert(_peripheral.initialized());
        _peripheral.connect();
        _init_notifications();
        _init_robot_cfg();
    }

    ~BLEInterface()
    {
        if (_peripheral.is_connected()) {
            _peripheral.disconnect();
        }
    }

    void refresh()
    {
        MotorCmd cmd;
        // cmd.cmds[1] = toUnsigned<uint16_t>(0 + (std::rand() % (1000 - 0 + 1)) - 500);
        // cmd.cmds[2] = toUnsigned<uint16_t>(0 + (std::rand() % (1000 - 0 + 1)) - 500);

        if (_no_comm_time * 1 / _cfg.rate > _cfg.panic_time) {
            cmd.cmds[0] = 0;
            cmd.cmds[1] = toUnsigned<uint16_t>(0);
            cmd.cmds[2] = toUnsigned<uint16_t>(0);
            cmd.cmds[3] = static_cast<uint16_t>(_cfg.max_acceleration * 100);
        }
        else {
            cmd.cmds[0] = _id_counter;
            cmd.cmds[1] = toUnsigned<uint16_t>(_left_motor_cm_per_s * 10);
            cmd.cmds[2] = toUnsigned<uint16_t>(_right_motor_cm_per_s * 10);
            cmd.cmds[3] = static_cast<uint16_t>(_cfg.max_acceleration * 100);
        }

        if (_accel_cm_per_ssq > 0.) {
            cmd.cmds[3] = static_cast<uint16_t>(_accel_cm_per_ssq);
        }

        _send_velocities(cmd);
        ++_no_comm_time;

        if (_id_counter >= 100) {
            _id_counter = 0;
        }
        else {
            ++_id_counter;
        }
    }

protected:
    void _init_notifications()
    {
        ROS_INFO("Connected to Lurebot:");
        {
            LurebotName msg;
            SimpleBLE::ByteArray bytes = _peripheral.readBytes(INFO_SRV_UUID, DEVICE_NAME_CHAR_UUID);
            if (bytes.size()) {
                std::copy(bytes.begin(), bytes.end(), msg.bytes);
                ROS_INFO("\tName: %s", msg.cbytes);
            }
        }
        {
            LurebotFWVersion msg;
            SimpleBLE::ByteArray bytes = _peripheral.readBytes(INFO_SRV_UUID, FW_VERSION_CHAR_UUID);
            if (bytes.size()) {
                std::copy(bytes.begin(), bytes.end(), msg.bytes);
                ROS_INFO("\tFW version: %s", msg.cbytes);
            }
        }

        _peripheral.notify(DROPPED_MSGS_SRV_UUID, DROPPED_MSGS_COUNT_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
            if (bytes.size()) {
                DroppedMsg msg;
                std::copy(bytes.begin(), bytes.end(), msg.bytes);

                bobi_msgs::DroppedMessages rosmsg;
                rosmsg.dropped = msg.counters[0];
                rosmsg.total = msg.counters[1];
                _dropped_msgs_pub.publish(rosmsg);
            }
        });

        _peripheral.notify(MOTOR_VEL_SRV_UUID, CURRENT_VEL_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
            if (bytes.size()) {
                MotorSpeeds msg;
                std::copy(bytes.begin(), bytes.end(), msg.bytes);

                bobi_msgs::MotorVelocities rosmsg;
                // divide by 1000, this is to convert from mm/s to m/s
                rosmsg.left = toSigned<float>(msg.cmds[0]) / 1000.;
                rosmsg.right = toSigned<float>(msg.cmds[1]) / 1000.;
                rosmsg.resultant = std::sqrt(rosmsg.left * rosmsg.left + rosmsg.right * rosmsg.right);
                _current_motor_vel_pub.publish(rosmsg);
            }
        });

        _peripheral.notify(TEMPERATURE_SRV_UUID, TEMPERATURE_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
            if (bytes.size()) {
                Temperature msg;
                std::copy(bytes.begin(), bytes.end(), msg.bytes);
                bobi_msgs::Temperature rosmsg;
                rosmsg.temp = toSigned<float>(msg.cmds[0]) / 10.;
                _current_temp_pub.publish(rosmsg);
            }
        });

        if (_cfg.check_heartbeat) {
            _peripheral.notify(HEARTBEAT_SRV_UUID, HEARTBEAT_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
                if (bytes.size()) {
                    if (bytes[0]) {
                        ROS_WARN("Heartbeat");
                    }
                    else {
                        ROS_ERROR("No heartbeat");
                    }
                }
            });
        }
    }

    void _init_robot_cfg()
    {

        if (_cfg.ret_vel) {
            _enable_ret_vel(1);
        }

        if (_cfg.ret_temp) {
            _enable_ret_temp(1);
        }

        if (_cfg.ret_dropped_msgs) {
            _enable_dropped_msgs(1);
        }

        _set_max_accel(_cfg.max_acceleration * 100);
        _set_duty_cycle(_cfg.pwm_duty_cycle_perc);
        // TODO: enable ir
    }

    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        _no_comm_time = 0;
        _left_motor_cm_per_s = motor_velocities->left * 100.;
        _right_motor_cm_per_s = motor_velocities->right * 100.;
        _accel_cm_per_ssq = motor_velocities->acceleration * 100;
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        // TODO:
        ROS_WARN("Not available in this version");
        return true;
    }

    bool _enable_ret_vel_srv_cb(
        bobi_msgs::ReturnVelocity::Request& req,
        bobi_msgs::ReturnVelocity::Response& res)
    {
        _enable_ret_vel(req.enable);
        return true;
    }

    bool _enable_ret_temp_srv_cb(
        bobi_msgs::ReturnTemperature::Request& req,
        bobi_msgs::ReturnTemperature::Response& res)
    {
        _enable_ret_temp(req.enable);
        return true;
    }

    bool _enable_ret_dropped_msgs_srv_cb(
        bobi_msgs::ReturnDroppedMsgs::Request& req,
        bobi_msgs::ReturnDroppedMsgs::Response& res)
    {
        _enable_dropped_msgs(req.enable);
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        _set_max_accel(req.max_acceleration * 100);
        return true;
    }

    bool _set_duty_cycle_srv_cb(
        bobi_msgs::DutyCycle::Request& req,
        bobi_msgs::DutyCycle::Response& res)
    {
        _set_duty_cycle(req.low_perc);
        return true;
    }

private:
    void _send_velocities(const MotorCmd& cmd)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray bytes(sizeof(cmd.bytes));
        std::copy(cmd.bytes, cmd.bytes + sizeof(cmd.bytes), bytes.begin());

        try {
            _peripheral.write_command(MOTOR_VEL_SRV_UUID, DESIRED_VEL_CHAR_UUID, bytes);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            if (_num_timeouts % 10 == 0) {
                ROS_WARN("Timeouts caught: %ld", _num_timeouts + 1);
            }
            ++_num_timeouts;
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

    void _enable_ret_vel(uint8_t enable)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {enable};

        try {
            _peripheral.write_command(MOTOR_VEL_SRV_UUID, RETURN_CURRENT_VEL_CHAR_UUID, byte);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            ROS_WARN("Send timed out: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

    void _enable_ret_temp(uint8_t enable)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {enable};

        try {
            _peripheral.write_command(TEMPERATURE_SRV_UUID, RETURN_CURRENT_TEMP_CHAR_UUID, byte);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            ROS_WARN("Send timed out: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

    void _enable_dropped_msgs(uint8_t enable)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {enable};

        try {
            _peripheral.write_command(DROPPED_MSGS_SRV_UUID, RETURN_DROPPED_MSGS_CHAR_UUID, byte);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            ROS_WARN("Send timed out: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

    void _set_max_accel(uint8_t accel)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {accel};

        try {
            _peripheral.write_command(ACCEL_SRV_UUID, MAX_ACCEL_CHAR_UUID, byte);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            ROS_WARN("Send timed out: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

    void _set_duty_cycle(uint8_t duty)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {duty};

        try {
            _peripheral.write_command(DUTY_CYCLE_SRV_UUID, SET_DUTY_CYCLE_CHAR_UUID, byte);
        }
        catch (SimpleDBus::Exception::SendFailed& e) {
            ROS_WARN("Send timed out: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_WARN("Caught exception, but skipping: %s", e.what());
        }
    }

protected:
    SimpleBLE::Peripheral _peripheral;
    SimpleBLE::Adapter _adapter;

    std::shared_ptr<ros::NodeHandle> _nh;
    LurebotConfig _cfg;

    float _left_motor_cm_per_s;
    float _right_motor_cm_per_s;
    float _accel_cm_per_ssq;
    uint16_t _id_counter;
    std::mutex _peripheral_lock;

    uint _no_comm_time;
    uint64_t _num_timeouts;

    ros::Subscriber _motor_vel_sub;
    ros::Publisher _current_motor_vel_pub;
    ros::Publisher _current_temp_pub;
    ros::Publisher _dropped_msgs_pub;
    ros::Publisher _proximity_sensor_pub;
    ros::ServiceServer _enable_ir_srv;
    ros::ServiceServer _enable_ret_vel_srv;
    ros::ServiceServer _enable_ret_temp_srv;
    ros::ServiceServer _enable_ret_dropped_msgs_srv;
    ros::ServiceServer _set_max_accel_srv;
    ros::ServiceServer _set_duty_cycle_srv;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lurebot_interface_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    LurebotConfig cfg = get_lurebot_config(nh);
    BLEInterface blei(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {
        // auto loop_start = std::chrono::high_resolution_clock::now();

        blei.refresh();
        ros::spinOnce();
        loop_rate.sleep();

        // auto loop_stop = std::chrono::high_resolution_clock::now();
        // std::chrono::microseconds loop_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_stop - loop_start);
        // ROS_INFO("Loop time: %ld", loop_time.count());
    }

    return 0;
}