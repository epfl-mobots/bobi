#include <ros/ros.h>

#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/ProximitySensors.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <iostream>

class DummyLurebotInterface {
public:
    DummyLurebotInterface(std::shared_ptr<ros::NodeHandle> nh)
        : _nh(nh)
    {
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &DummyLurebotInterface::_motor_velocity_cb, this);

        _cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // TODO: not implemented yet
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);

        _pose_pub = nh->advertise<bobi_msgs::PoseVec>("robot_poses", 1);
        _odom_sub = _nh->subscribe("odom", 1, &DummyLurebotInterface::_odom_cb, this);
    }

protected:
    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        double Vr = motor_velocities->right;
        double Vl = motor_velocities->left;
        double l = 0.0451; // This is the distance between the wheels for the fisbot_v5
        double w = (Vr - Vl) / l; // angular velocity
        double v = (Vr + Vl) / 2;

        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        _cmd_vel_pub.publish(msg);
    }

    void _odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        bobi_msgs::PoseStamped pose;
        pose.pose.xyz.x = msg->pose.pose.position.x;
        pose.pose.xyz.y = msg->pose.pose.position.y;
        pose.pose.xyz.z = msg->pose.pose.position.z;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(pose.pose.rpy.roll, pose.pose.rpy.pitch, pose.pose.rpy.yaw);

        pose.header.stamp = ros::Time::now();
        bobi_msgs::PoseVec pv;
        pv.poses.push_back(pose);
        _pose_pub.publish(pv);
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    ros::Subscriber _motor_vel_sub;
    ros::Publisher _cmd_vel_pub;
    ros::Publisher _proximity_sensor_pub;

    ros::Subscriber _odom_sub;
    ros::Publisher _pose_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lurebot_dummy_interface_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    DummyLurebotInterface lurebot(nh);

    int rate;
    nh->param<int>("dummy_interface_node/rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}