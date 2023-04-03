#ifndef BOBI_FILTERING_METHOD_BASE_HPP
#define BOBI_FILTERING_METHOD_BASE_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/ConvertCoordinates.h>

#include <numeric>

#define INVALID -1000
#define BL 0.035
#define MAX_DIST_PER_TS 0.03
#define MIN_DIST_PER_DUPL 0.008

namespace bobi {
    class FilteringMethodBase {
    public:
        using AgentPose = std::vector<bobi_msgs::PoseStamped>;
        using AgentPoseList = std::list<AgentPose>;

        FilteringMethodBase(bool force_robot_position = false) : _force_robot_position(force_robot_position)
        {
            std::srand(time(NULL));
            _min_history_len = 1;
        }

        float euc_distance(const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o)
        {
            return std::sqrt(
                std::pow(o.pose.xyz.x - n.pose.xyz.x, 2.)
                + std::pow(o.pose.xyz.y - n.pose.xyz.y, 2.));
        }

        float angle_sim(const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o)
        {
            return std::abs(_angle_to_pipi(n.pose.rpy.yaw - o.pose.rpy.yaw));
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
        }

        int min_history_len() const
        {
            return _min_history_len;
        }

    protected:
        double _angle_to_pipi(double angle)
        {
            while (true) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
                if (abs(angle) <= M_PI) {
                    break;
                }
            }
            return angle;
        }

        void copy_pose(const bobi_msgs::PoseStamped& in, bobi_msgs::PoseStamped& target)
        {
            target.pose.xyz = in.pose.xyz;
            target.pose.rpy = in.pose.rpy;
            std::copy(in.pose.contours.begin(), in.pose.contours.end(), std::back_inserter(target.pose.contours));
            target.pose.is_filtered = in.pose.is_filtered;
            target.pose.is_swapped = in.pose.is_swapped;
            target.header.stamp = in.header.stamp;
        }

        bool _force_robot_position;
        int _min_history_len;
    }; // namespace bobi
} // namespace bobi

#endif
