#include <ros/ros.h>
#include <bobi_fishbot_v4_interface/dbus_interface.h>

#include <bobi_msgs/MotorVelocities.h>

#include <iostream>

using namespace bobi::dbus;

class FishbotV4 {
public:
    FishbotV4(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh)
    {
        // ros setup
        std::string fw_path;
        _nh->param<std::string>("fw_path", fw_path, "");
        _nh->param<int>("robot_id", _robot_id, -1);
        std::string fishbot_name;
        _nh->param<std::string>("fishbot_name", fishbot_name, "");
        _fishbot_name = QString(fishbot_name.c_str());

        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &FishbotV4::_motor_velocity_cb, this);

        // send firmware to the robot
        _dbus_intf.load_script(fw_path.c_str());
    }

    void set_velocities(int speed_left, int speed_right)
    {
        QString event = "MotorControl" + _fishbot_name;
        Values data;
        data.append(speed_left);
        data.append(speed_right);
        _dbus_intf.send_event_name(event, data);
    }

protected:
    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        int left_motor_cm_per_s = motor_velocities->left * 100;
        int right_motor_cm_per_s = motor_velocities->right * 100;
        set_velocities(left_motor_cm_per_s, right_motor_cm_per_s);
    }

    DBusInterface _dbus_intf;

    std::shared_ptr<ros::NodeHandle> _nh;
    ros::Subscriber _motor_vel_sub;

    int _robot_id;
    QString _fishbot_name;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fishbot_interface_v4_node");
    QCoreApplication a(argc, argv);

    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    int r;
    nh->param("rate", r, 10);
    ros::Rate rate(r);

    FishbotV4 robot(nh);

    while (ros::ok()) {
        a.processEvents();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
