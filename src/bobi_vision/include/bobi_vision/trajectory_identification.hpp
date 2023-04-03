#ifndef BOBI_TRAJECTORY_IDENTIFICATION_HPP
#define BOBI_TRAJECTORY_IDENTIFICATION_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/NumAgents.h>
#include <bobi_msgs/GetNumAgents.h>
#include <bobi_vision/filtering_method_base.hpp>
#include <dynamic_reconfigure/server.h>

#include <bobi_vision/TrajectoryIdentificationConfig.h>
#include <bobi_vision/trajectory_identification_method_factory.hpp>

#include <bobi_vision/coordinate_mapper.hpp>

#include <mutex>

namespace bobi {

    class TrajectoryIdentification {

        using AgentPose = std::vector<bobi_msgs::PoseStamped>;
        using AgentPoseList = std::list<AgentPose>;

    public:
        TrajectoryIdentification(std::shared_ptr<ros::NodeHandle> nh, size_t matrix_len = 5)
            : _nh(nh),
              _matrix_len(matrix_len),
              _filter(new FilteringMethodBase()),
              _init_successful(false)
        {
            dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig>::CallbackType f;
            f = boost::bind(&TrajectoryIdentification::_config_cb, this, _1, _2);
            _config_server.setCallback(f);

            std::tie(_basic_tracker, std::ignore) = _method_factory(_nh, 0, _force_robot_position);

            std::tie(_points_bottom, _points_top, _top_cfg, _bottom_cfg) = init_coordinate_mapper(nh);
            _bottom2top = std::shared_ptr<CoordinateMapper>(new CoordinateMapper(_nh,
                "None",
                _points_bottom,
                _points_top,
                _top_cfg.camera_matrix,
                _top_cfg.distortion_coeffs,
                _top_cfg.pix2m,
                _top_cfg.camera_px_width_undistorted,
                _top_cfg.camera_px_height_undistorted));

            _num_agents_srv = _nh->serviceClient<bobi_msgs::GetNumAgents>("get_num_agents");
            _num_agents_srv.waitForExistence();

            bobi_msgs::GetNumAgents srv;
            if (_num_agents_srv.call(srv)) {
                _num_agents = srv.response.info.num_agents;
                _num_robots = srv.response.info.num_robots;
                _num_virtu_agents = srv.response.info.num_virtu_agents;
            }

            _num_agents_sub = _nh->subscribe("num_agents_update", 1, &TrajectoryIdentification::_num_agents_cb, this);

            // Pose subscribers
            _naive_poses_sub = _nh->subscribe("naive_poses", 1, &TrajectoryIdentification::_naive_pose_cb, this);
            _robot_poses_sub = _nh->subscribe("robot_poses", 1, &TrajectoryIdentification::_robot_pose_cb, this);
        }

        std::tuple<AgentPose, bool> filter()
        {
            std::lock_guard<std::mutex> g1(_cfg_mutex);
            std::lock_guard<std::mutex> g2(_rb_mutex);
            std::lock_guard<std::mutex> g3(_ind_mutex);

            AgentPose p;
            bool is_filtered = false;
            if (_filtered_pose_list.size()) {
                _filter->operator()(_filtered_pose_list, _filtered_robot_pose_list, _num_agents, _num_robots);
                p = _filtered_pose_list.front();
                is_filtered = true;
            }

            return std::tuple(p, is_filtered);
        }

        AgentPoseList filtered_list()
        {
            return _filtered_pose_list;
        }

    protected:
        void _num_agents_cb(const bobi_msgs::NumAgents::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_cfg_mutex);
            _num_agents = msg->num_agents;
            _num_robots = msg->num_robots;
            _num_virtu_agents = msg->num_virtu_agents;
        }

        void _naive_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_ind_mutex);

            std::vector<bobi_msgs::PoseStamped> copies(pose_vec_ptr->poses.size());

            if (copies.size()) {
                std::copy(pose_vec_ptr->poses.begin(), pose_vec_ptr->poses.end(), copies.begin());
                _filtered_pose_list.push_front(copies);
                if (_filtered_pose_list.size() > _matrix_len) {
                    _filtered_pose_list.pop_back();
                }
            }
        }

        void _robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_rb_mutex);

            std::vector<bobi_msgs::PoseStamped> copies(pose_vec_ptr->poses.size());
            if (copies.size()) {
                std::copy(pose_vec_ptr->poses.begin(), pose_vec_ptr->poses.end(), copies.begin());
            }

            for (size_t i = 0; i < copies.size(); ++i) {
                geometry_msgs::Point converted_p;
                // copies[i].pose.xyz = _convert_bottom2top(copies[i].pose.xyz);
                copies[i].pose.xyz = _bottom2top->convert(copies[i].pose.xyz);
            }

            _filtered_robot_pose_list.push_front(copies);
            if (_filtered_robot_pose_list.size() > _matrix_len) {
                _filtered_robot_pose_list.pop_back();
            }
        }

        void _config_cb(bobi_vision::TrajectoryIdentificationConfig& config, uint32_t level)
        {
            std::lock_guard<std::mutex> guard(_cfg_mutex);
            bool success;
            _force_robot_position = config.force_robot_position;
            std::tie(_filter, success) = _method_factory(_nh, config.method, _force_robot_position);
            if (success) {
                ROS_INFO("Trajectory Identification method changed");
                _filtered_pose_list.clear();
                _filtered_robot_pose_list.clear();
                _init_successful = false;
            }
            else {
                ROS_INFO("Using default Trajectory Identification method");
            }
        }

        geometry_msgs::Point _convert_bottom2top(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_bottom2top_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        size_t _num_agents;
        size_t _num_robots;
        size_t _num_virtu_agents;
        bool _force_robot_position;
        bool _init_successful;

        std::shared_ptr<FilteringMethodBase> _filter;
        std::shared_ptr<FilteringMethodBase> _basic_tracker;
        size_t _matrix_len;
        AgentPoseList _filtered_pose_list;
        AgentPoseList _filtered_robot_pose_list;

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _naive_poses_sub;
        ros::Subscriber _robot_poses_sub;
        ros::ServiceClient _bottom2top_srv;
        std::mutex _ind_mutex;
        std::mutex _rb_mutex;

        ros::ServiceClient _num_agents_srv;
        ros::Subscriber _num_agents_sub;

        std::vector<cv::Point2d> _points_bottom;
        std::vector<cv::Point2d> _points_top;
        top::CameraConfig _top_cfg;
        bottom::CameraConfig _bottom_cfg;
        std::shared_ptr<CoordinateMapper> _bottom2top;

        dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig> _config_server;
        std::mutex _cfg_mutex;
        TrajectoryIdentificationMethodFactory _method_factory;
    };

} // namespace bobi

#endif
