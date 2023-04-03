#ifndef BOBI_HUNGARIAN_BASED_HPP
#define BOBI_HUNGARIAN_BASED_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <bobi_vision/hungarian_algorithm.hpp>
#include <bobi_vision/nearest_centroid.hpp>

#include <opencv2/opencv.hpp>

#define MAX_BL_IN_PX 160
// #define ENABLE_SPLIT_CONTOURS

namespace bobi {

    class HungarianBased : public NearestCentroid {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        HungarianBased(bool force_robot_position = true) : NearestCentroid(force_robot_position)
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

#ifdef ENABLE_SPLIT_CONTOURS
            if (copy.size() < num_agents) {
                std::vector<std::vector<cv::Point>> contours;
                for (const bobi_msgs::PoseStamped& pose : *t0) {
                    std::vector<cv::Point> c;
                    for (auto point : pose.pose.contours) {
                        cv::Point p;
                        p.x = point.x;
                        p.y = point.y;
                        c.push_back(p);
                    }
                    contours.push_back(c);
                }

                _split_big_contours(copy, *t0, contours, num_robots, false);
                t0->resize(copy.size());
                for (size_t i = 0; i < copy.size(); ++i) {
                    copy_pose(copy[i], (*t0)[i]);
                }
            }
#endif

            if (individual_poses.size() > 1 && num_agents > 1 && t0->size() > num_robots) {
                auto t1 = std::next(individual_poses.begin());
                if (t1->size() > num_robots) {
                    std::vector<std::vector<double>> cost_mat;
                    cost_mat.resize(copy.size() - num_robots);
                    for (size_t i = num_robots; i < copy.size(); ++i) {
                        cost_mat[i - num_robots].resize(t1->size() - num_robots);
                        for (size_t j = num_robots; j < t1->size(); ++j) {
                            double dist = euc_distance(copy[i], (*t1)[j]);
                            double asim = angle_sim(copy[i], (*t1)[j]);

                            double w_dist = 0.95;
                            double w_hdg = 0.05;
                            double cost = w_dist * dist + w_hdg * asim;
                            cost_mat[i - num_robots][j - num_robots] = cost;
                        }
                    }

                    std::vector<int> assignment_idcs;
                    _ha.solve(cost_mat, assignment_idcs);

                    std::vector<size_t> not_matched;
                    for (size_t i = 0; i < cost_mat.size(); ++i) {
                        bobi_msgs::PoseStamped p;
                        if (assignment_idcs[i] >= 0) {
                            copy_pose(copy[i + num_robots], (*t0)[t0->size() - 1]);
                        }
                        else {
                            not_matched.push_back(i + num_robots);
                        }
                    }

                    for (size_t i = 0; i < not_matched.size(); ++i) {
                        bobi_msgs::PoseStamped p;
                        copy_pose(copy[not_matched[i]], p);
                        t0->push_back(p);
                    }
                }
            }

            if (t0->size() > num_agents) {
                t0->resize(num_agents);
            }
        }

    protected:
        algo::HungarianAlgorithm _ha;

        void _split_big_contours(AgentPose& copy, const AgentPose& l, const std::vector<std::vector<cv::Point>>& contours, int num_robots, bool is_robot = true)
        {
            size_t added = 0;
            for (size_t i = num_robots; i < l.size(); ++i) {
                float area = cv::contourArea(contours[i]);
                if (area > MAX_BL_IN_PX) {
                    int dubs = static_cast<int>(area / MAX_BL_IN_PX);
                    for (int j = 0; j < dubs; ++j) {
                        copy.insert(copy.begin() + i + added, bobi_msgs::PoseStamped());
                        copy_pose(l[i], copy[i + added]);
                        ++added;
                    }
                }
            }
        }
    };
} // namespace bobi

#endif
