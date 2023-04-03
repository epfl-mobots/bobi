#include <ros/ros.h>

#include <bobi_vision/trajectory_identification.hpp>

#include <bobi_msgs/SpeedEstimateVec.h>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_identification_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    TrajectoryIdentification ti(nh, 2);

    ros::Publisher pose_pub = nh->advertise<bobi_msgs::PoseVec>("filtered_poses", 1);
    ros::Publisher speed_pub = nh->advertise<bobi_msgs::SpeedEstimateVec>("speed_estimates", 1);

    int rate = 30;
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        bool was_filtered;
        std::vector<bobi_msgs::PoseStamped> poses;
        std::tie(poses, was_filtered) = ti.filter();

        bobi_msgs::PoseVec pv;
        pv.poses = poses;
        pv.is_filtered = was_filtered;
        pose_pub.publish(pv);

        auto history = ti.filtered_list();
        bobi_msgs::SpeedEstimateVec sev;
        if (history.size() > 1) {
            auto poses_t_1 = *std::next(history.begin());
            size_t end = std::min(poses.size(), poses_t_1.size());
            for (size_t i = 0; i < end; ++i) {
                double dt = poses[i].header.stamp.toSec() - poses_t_1[i].header.stamp.toSec();
                dt = std::max(dt, 1. / rate);
                double speed = std::sqrt(std::pow(poses[i].pose.xyz.x - poses_t_1[i].pose.xyz.x, 2) + std::pow(poses[i].pose.xyz.y - poses_t_1[i].pose.xyz.y, 2)) / dt;
                sev.speeds.push_back(speed);
            }
        }
        speed_pub.publish(sev);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}