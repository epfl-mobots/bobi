#include <ros/ros.h>

#include <bobi_control/apriori_control.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "apriori_controller_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    int id;
    nh->param<int>("robot_id", id, -1);

    std::string pose_topic;
    nh->param<std::string>("pose_topic", pose_topic, "naive_poses");

    float wheel_radius;
    nh->param<float>("wheel_radius", wheel_radius, -1);

    float wheel_distance;
    nh->param<float>("wheel_distance", wheel_distance, -1);

    assert(wheel_radius > 0);
    assert(wheel_distance > 0);

    ROS_INFO("Robot %d controller using %s topic for pose info", id, pose_topic.c_str());

    APrioriControl ctrl(nh, id, pose_topic, wheel_radius, wheel_distance);

    int rate;
    nh->param<int>("apriori_control/rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ctrl.spin_once();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}