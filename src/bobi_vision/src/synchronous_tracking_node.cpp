#include <ros/ros.h>

#include <bobi_vision/individual_tracking.hpp>
#include <bobi_vision/robot_tracking.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "synchronous_tracking_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    IndividualTracking it(nh);
    RobotTracking rt(nh);

    int rate = 30;
    ros::Rate loop_rate(rate); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        bool it_success = it.read();
        bool rt_success = rt.read();

        it.process();
        rt.process();

        it.publish(header);
        rt.publish(header);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
