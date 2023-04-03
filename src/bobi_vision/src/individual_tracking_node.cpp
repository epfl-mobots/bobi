#include <ros/ros.h>
#include <bobi_vision/individual_tracking.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "individual_tracking_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    IndividualTracking it(nh);

    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (!it.read()) {
            continue;
        }

        it.process();
        it.publish(header);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}