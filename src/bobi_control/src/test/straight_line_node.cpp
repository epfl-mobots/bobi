#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/TargetPose.h>

bobi_msgs::PoseStamped start, stop, cur_waypoint;
std::atomic<bool> flag = false;
ros::Publisher pub;

void pose_cb(const bobi_msgs::PoseVec::ConstPtr& msg)
{
    if (msg->poses.size() == 0) {
        return;
    }
    auto euc_distance = [](const bobi_msgs::PoseStamped& lpose, const bobi_msgs::PoseStamped& rpose) {
        return std::sqrt(std::pow(lpose.pose.xyz.x - rpose.pose.xyz.x, 2.) + std::pow(lpose.pose.xyz.y - rpose.pose.xyz.y, 2.));
    };

    if (euc_distance(msg->poses[0], cur_waypoint) <= 0.005) {
        flag = !flag;

        if (flag) {
            cur_waypoint = stop;
        }
        else {
            cur_waypoint = start;
        }
        pub.publish(cur_waypoint);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "straight_line_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    std::vector<float> vstart, vstop;
    nh->getParam("straight_line/start", vstart);
    nh->getParam("straight_line/stop", vstop);
    start.header.stamp = ros::Time::now();
    start.pose.xyz.x = vstart[0];
    start.pose.xyz.y = vstart[1];
    stop.header.stamp = ros::Time::now();
    stop.pose.xyz.x = vstop[0];
    stop.pose.xyz.y = vstop[1];
    cur_waypoint = start;

    pub = nh->advertise<bobi_msgs::TargetPose>("target_position", 1);
    ros::Subscriber rpose = nh->subscribe("robot_poses", 1, pose_cb);

    ros::Rate loop_rate(10);
    int iter = 0;
    while (ros::ok()) {
        bobi_msgs::TargetPose t;
        t.target = cur_waypoint;
        pub.publish(t);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}