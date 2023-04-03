#ifndef BOBI_REPLAY_HPP
#define BOBI_REPLAY_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <numeric>
#include <algorithm>

#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#define INVALID -1000

namespace bobi {

    class Replay : public FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        Replay(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<FilteringMethodBase> filter)
            : _filter(filter), _current_iter(0)
        {
            std::string replay_file;
            double pix2m;
            assert(nh->getParam("replay/replay_file_path", replay_file) && "Replay file invalid");
            assert(nh->getParam("top_camera/camera_px_width_undistorted", _center[0]));
            assert(nh->getParam("top_camera/camera_px_height_undistorted", _center[1]));
            assert(nh->getParam("top_camera/pix2m", pix2m));
            assert(nh->getParam("replay/radius", _radius));
            assert(nh->getParam("replay/scale", _scale));
            assert(nh->getParam("replay/cols_iter", _cols_iter));
            assert(nh->getParam("replay/num_ind_replay", _num_ind_replay));
            _center[0] = _center[0] / 2. * pix2m;
            _center[1] = _center[1] / 2. * pix2m;
            ROS_INFO("Replaying trajectories in setup with center (%f, %f) (in m)", _center[0], _center[1]);
            _load_from_file(_replay_data, replay_file);
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            int start_idx = 1;
            if (_cols_iter < 3) {
                start_idx = 0;
            }

            auto t0 = individual_poses.begin();
            for (size_t i = start_idx; i < _num_ind_replay; ++i) {
                bobi_msgs::PoseStamped p;
                p.pose.xyz.x = _replay_data(_current_iter, i * _cols_iter + 1) * _scale + _center[0];
                p.pose.xyz.y = _replay_data(_current_iter, i * _cols_iter + 2) * _scale + _center[1];
                p.pose.rpy.yaw = _replay_data(_current_iter, i * _cols_iter + 3);
                p.header.stamp = ros::Time::now();
                (*t0).push_back(p);
            }
            ++_current_iter;
            if (_current_iter == _replay_data.rows()) {
                _current_iter = 0;
            }

            _filter->operator()(individual_poses, robot_poses, num_agents + _num_ind_replay, num_robots);
        }

    protected:
        template <typename M>
        void _load_from_file(M& m, const std::string& filename, int skip = 0,
            const char& delim = ' ') const
        {
            auto values = _load_from_file(filename, skip, delim);
            m.resize(values.size(), values[0].size());
            for (size_t i = 0; i < values.size(); ++i)
                for (size_t j = 0; j < values[i].size(); ++j)
                    m(i, j) = values[i][j];
        }

        std::vector<std::vector<double>> _load_from_file(const std::string& filename, int skip = 0,
            const char& delim = ' ') const
        {
            std::ifstream ifs(filename.c_str());
            assert(ifs.good() && "Invalid file path");

            std::string line;
            std::vector<std::vector<double>> v;
            while (std::getline(ifs, line)) {
                if (skip > 0) {
                    --skip;
                    continue;
                }

                std::stringstream line_stream(line);
                std::string cell;
                std::vector<double> line;
                while (std::getline(line_stream, cell, delim)) {
                    (cell == "NAN") ? line.push_back(std::nan(cell.c_str()))
                                    : line.push_back(std::stod(cell));
                }
                v.push_back(line);
            }
            assert(!v.empty() && "Empty file");
            return v;
        }

        std::shared_ptr<FilteringMethodBase> _filter;
        Eigen::MatrixXd _replay_data;
        int _current_iter;
        std::array<float, 2> _center;
        float _radius;
        float _scale;
        int _cols_iter;
        int _num_ind_replay;
    };
} // namespace bobi

#endif
