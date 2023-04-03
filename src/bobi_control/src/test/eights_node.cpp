#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/TargetPose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eights_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    float radius1, radius2;
    std::vector<float> center;
    float waypoint_angle;
    int rate;
    float shift_angle;

    nh->param<float>("eights/radius1", radius1, 0.08);
    nh->param<float>("eights/radius2", radius2, 0.08);
    nh->param<std::vector<float>>("eights/center", center, {0., 0.});
    nh->param<float>("eights/waypoint_angle", waypoint_angle, 0.26);
    nh->param<int>("eights/rate", rate, 10);
    nh->param<float>("eights/shift_angle", shift_angle, 0.785);

    ros::Publisher wp_pub = nh->advertise<bobi_msgs::TargetPose>("target_position", 1);

    int num_waypoints = static_cast<int>(2. * M_PI / waypoint_angle);
    std::vector<bobi_msgs::TargetPose> waypoints;

    std::vector<std::vector<float>> centers;
    centers.resize(8);
    for (size_t i = 0; i < centers.size(); ++i) {
        centers[i] = {
            radius1 * std::cos(-shift_angle * i) + center[0],
            radius1 * std::sin(-shift_angle * i) + center[1],
            radius2 * std::cos(-shift_angle * i + M_PI) + center[0],
            radius2 * std::sin(-shift_angle * i + M_PI) + center[1],
            (float)(-shift_angle * i + M_PI),
            (float)(-shift_angle * i)};
    }

    for (size_t j = 0; j < centers.size(); ++j) {

        int sign = 1;
        if (j % 2 == 1) {
            sign = -1;
        }

        for (size_t i = 0; i < num_waypoints; ++i) {
            bobi_msgs::TargetPose wp;
            wp.target.pose.xyz.x = radius1 * std::cos(sign * waypoint_angle * i + centers[j][4]) + centers[j][0];
            wp.target.pose.xyz.y = radius1 * std::sin(sign * waypoint_angle * i + centers[j][4]) + centers[j][1];
            waypoints.push_back(wp);
        }

        for (size_t i = 0; i < num_waypoints; ++i) {
            bobi_msgs::TargetPose wp;
            wp.target.pose.xyz.x = radius2 * std::cos(-sign * waypoint_angle * i + centers[j][5]) + centers[j][2];
            wp.target.pose.xyz.y = radius2 * std::sin(-sign * waypoint_angle * i + centers[j][5]) + centers[j][3];
            waypoints.push_back(wp);
        }
    }

    ros::Rate loop_rate(rate);
    int iter = 0;
    while (ros::ok()) {
        wp_pub.publish(waypoints[iter++]);
        if (iter == waypoints.size()) {
            iter = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}