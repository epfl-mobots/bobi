#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/TargetPose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circular_trajectory_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    float radius;
    std::vector<float> center;
    float waypoint_angle;
    int rate;
    bool cw;
    nh->param<float>("circular_trajectory/radius", radius, 0.25);
    nh->param<std::vector<float>>("circular_trajectory/center", center, {0., 0.});
    nh->param<float>("circular_trajectory/waypoint_angle", waypoint_angle, 0.26);
    nh->param<int>("circular_trajectory/rate", rate, 2);
    nh->param<bool>("circular_trajectory/cw", cw, true);

    ros::Publisher wp_pub = nh->advertise<bobi_msgs::TargetPose>("target_position", 1);

    int sign = 1;
    if (!cw) {
        sign = -1;
    }

    int num_waypoints = static_cast<int>(2. * M_PI / waypoint_angle);
    std::vector<bobi_msgs::TargetPose> waypoints;
    for (int i = 0; i < num_waypoints; ++i) {
        bobi_msgs::TargetPose wp;
        wp.target.pose.xyz.x = radius * std::cos(sign * waypoint_angle * i) + center[0];
        wp.target.pose.xyz.y = radius * std::sin(sign * waypoint_angle * i) + center[1];
        waypoints.push_back(wp);
    }

    ros::Rate loop_rate(rate);
    int iter = 0;
    while (ros::ok()) {
        ROS_INFO("(x=%f, y=%f)", waypoints[iter].target.pose.xyz.x, waypoints[iter].target.pose.xyz.y);
        wp_pub.publish(waypoints[iter++]);
        if (iter == num_waypoints) {
            iter = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}