#include <ros/ros.h>

#include <bobi_msgs/MotorVelocities.h>

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "timed_motor_vel_node");
    ros::NodeHandle nh;

    int num_repetitions;
    double delay;
    double vel;
    nh.param<int>("timed_motor_vel/num_repetitions", num_repetitions, 5);
    nh.param<double>("timed_motor_vel/delay", delay, 5.);
    nh.param<double>("timed_motor_vel/velocity", vel, 0.02);

    ros::Publisher pub = nh.advertise<bobi_msgs::MotorVelocities>("set_velocities", 2);

    for (int i = 0; i < num_repetitions; ++i) {
        int sign = 1;
        if (i % 2 == 1) {
            sign = -1;
        }

        bobi_msgs::MotorVelocities go;
        go.left = sign * vel;
        go.right = sign * vel;
        pub.publish(go);

        ros::spinOnce();
        ros::Duration(delay).sleep();
    }
    bobi_msgs::MotorVelocities zero;
    zero.left = 0.0;
    zero.right = 0.0;
    pub.publish(zero);

    ros::spinOnce();
    ros::Duration(delay).sleep();

    return 0;
}