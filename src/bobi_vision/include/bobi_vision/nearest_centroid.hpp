#ifndef BOBI_NEAREST_CENTROID_HPP
#define BOBI_NEAREST_CENTROID_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

namespace bobi {

    class NearestCentroid : public FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        NearestCentroid(bool force_robot_position = true) : FilteringMethodBase(force_robot_position)
        {
            _min_history_len = 1;
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            if (!individual_poses.size()) {
                return;
            }

            auto r0 = robot_poses.begin();
            auto t0 = individual_poses.begin();
            size_t sidx = num_robots;

            if (num_robots > 0 && r0->size() == num_robots) {
                for (size_t i = 0; i < r0->size(); ++i) {
                    float min_dist = std::numeric_limits<float>::infinity();
                    int min_idx = INVALID;
                    for (size_t j = 0; j < t0->size(); ++j) {
                        float dist = euc_distance((*r0)[i], (*t0)[j]);
                        if (dist < min_dist) {
                            min_dist = dist;
                            min_idx = j;
                        }
                    }

                    if (min_idx != INVALID && min_idx != i) {
                        bobi_msgs::PoseStamped tmp;
                        copy_pose((*t0)[min_idx], tmp);
                        copy_pose((*t0)[i], (*t0)[min_idx]);
                        copy_pose(tmp, (*t0)[i]);
                        if (_force_robot_position) {
                            copy_pose((*r0)[i], (*t0)[i]);
                            (*t0)[i].pose.is_swapped = true;
                        }
                    }
                }
            }

            AgentPose copy;
            if (t0->size()) {

                copy.resize(t0->size());
                for (size_t i = 0; i < copy.size(); ++i) {
                    copy_pose((*t0)[i], copy[i]);
                }
            }
            else {
                if (num_robots && r0->size()) {
                    copy.resize(r0->size());
                    t0->resize(r0->size());
                    for (size_t i = 0; i < copy.size(); ++i) {
                        copy_pose((*r0)[i], copy[i]);
                        copy_pose((*r0)[i], (*t0)[i]);
                    }
                }
            }
        }
    };
} // namespace bobi

#endif
