#include <ros/ros.h>

#include <bobi_control/velocity_control.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_controller_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    int id;
    nh->param<int>("robot_id", id, -1);

    std::string pose_topic;
    nh->param<std::string>("pose_topic", pose_topic, "naive_poses");

    ROS_INFO("Robot %d controller using %s topic for pose info", id, pose_topic.c_str());

    VelocityControl ctrl(nh, id, pose_topic);

    int rate;
    nh->param<int>("velocity_control/rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ctrl.spin_once();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}