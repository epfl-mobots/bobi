#ifndef BOBI_CONTROLLLER_BASE_HPP
#define BOBI_CONTROLLLER_BASE_HPP

#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/ConvertCoordinates.h>
#include <bobi_msgs/TargetPose.h>
#include <bobi_vision/coordinate_mapper.hpp>

#include <cassert>

namespace bobi {

    class ControllerBase {
    public:
        ControllerBase(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic)
            : _nh(nh),
              _ros_dt(0),
              _dt(0),
              _id(id),
              _desired_speed(0.),
              _desired_acceleration(0.),
              _pose_topic(pose_topic),
              _prev_stamp(ros::Time::now())
        {
            _pose_sub = _nh->subscribe(_pose_topic, 1, &ControllerBase::_pose_cb, this);
            _target_vel_sub = _nh->subscribe("target_velocities", 1, &ControllerBase::_target_velocities_cb, this);
            _target_pos_sub = _nh->subscribe("target_position", 1, &ControllerBase::_target_pose_cb, this);

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

            _top2bottom = std::shared_ptr<CoordinateMapper>(new CoordinateMapper(_nh,
                "None",
                _points_top,
                _points_bottom,
                _bottom_cfg.camera_matrix,
                _bottom_cfg.distortion_coeffs,
                _bottom_cfg.pix2m,
                _bottom_cfg.camera_px_width_undistorted,
                _bottom_cfg.camera_px_height_undistorted));

            _target_velocities.left = 0.;
            _target_velocities.right = 0.;
            _target_pose.pose.xyz.x = -1;
            _target_pose.pose.xyz.y = -1;
            _target_pose.pose.rpy.yaw = 0.;
        }

        virtual void spin_once()
        {
            auto n = ros::Time::now();
            _ros_dt = n - _prev_stamp;
            _dt = std::max(_ros_dt.toSec(), 1e-6);
            _prev_stamp = n;
        }

        const std::string get_pose_topic() const
        {
            return _pose_topic;
        }

        bobi_msgs::PoseStamped convert_top2bottom(const bobi_msgs::PoseStamped& pose)
        {
            bobi_msgs::PoseStamped res;
            res.pose.xyz = _top2bottom->convert(pose.pose.xyz);
            res.pose.rpy = pose.pose.rpy;
            res.header = pose.header;
            return res;
        }

        bobi_msgs::PoseStamped convert_bottom2top(const bobi_msgs::PoseStamped& pose)
        {
            bobi_msgs::PoseStamped res;
            res.pose.xyz = _bottom2top->convert(pose.pose.xyz);
            res.pose.rpy = pose.pose.rpy;
            res.header = pose.header;
            return res;
        }

    protected:
        template <typename T>
        int _sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        double _angle_to_pipi(double angle)
        {
            while (abs(angle) > M_PI) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
            }
            return angle;
        }

        void _pose_cb(const bobi_msgs::PoseVec::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_pose_mtx);
            assert(_id >= 0);
            assert(_id <= msg->poses.size());
            if (msg->poses.size() == 0) {
                return;
            }

            _prev_pose = _pose;
            _pose = (*msg).poses[_id];
        }

        void _target_velocities_cb(const bobi_msgs::MotorVelocities::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_tvel_mtx);
            _target_velocities = *msg;
        }

        void _target_pose_cb(const bobi_msgs::TargetPose::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_tpos_mtx);
            _target_pose = msg->target;
            _desired_speed = msg->desired_speed;
            // _desired_acceleration = msg->desired_acceleration;
            if (msg->is_top && _pose_topic == "robot_poses") {
                _target_pose = convert_top2bottom(_target_pose);
            }
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        const int _id;
        const std::string _pose_topic;

        std::vector<cv::Point2d> _points_bottom;
        std::vector<cv::Point2d> _points_top;
        top::CameraConfig _top_cfg;
        bottom::CameraConfig _bottom_cfg;
        std::shared_ptr<CoordinateMapper> _bottom2top;
        std::shared_ptr<CoordinateMapper> _top2bottom;

        ros::Subscriber _target_vel_sub;
        std::mutex _tvel_mtx;
        bobi_msgs::MotorVelocities _target_velocities;

        ros::Subscriber _target_pos_sub;
        std::mutex _tpos_mtx;
        bobi_msgs::PoseStamped _target_pose;
        float _desired_speed;
        float _desired_acceleration;

        ros::Subscriber _pose_sub;
        std::mutex _pose_mtx;
        bobi_msgs::PoseStamped _pose;
        bobi_msgs::PoseStamped _prev_pose;

        ros::Time _prev_stamp;
        ros::Duration _ros_dt;
        double _dt;
    };

} // namespace bobi

#endif