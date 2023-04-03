#include <ros/ros.h>

#include <bobi_vision/robot_tracking.hpp>

#include <cassert>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tracking_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    RobotTracking rt(nh);

    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (!rt.read()) {
            continue;
        }

        rt.process();
        rt.publish(header);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}