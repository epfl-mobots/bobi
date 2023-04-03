#ifndef BOBI_VELOCITY_CONTROL_HPP
#define BOBI_VELOCITY_CONTROL_HPP

#include <bobi_control/controller_base.hpp>

#include <dynamic_reconfigure/server.h>
#include <bobi_control/VelocityControlConfig.h>

namespace bobi {
    class VelocityControl : public ControllerBase {
    public:
        VelocityControl(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic)
            : ControllerBase(nh, id, pose_topic),
              _prev_error{0, 0},
              _integral{0, 0}
        {
            dynamic_reconfigure::Server<bobi_control::VelocityControlConfig>::CallbackType f;
            f = boost::bind(&VelocityControl::_config_cb, this, _1, _2);
            _cfg_server.setCallback(f);

            _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("set_velocities", 1);
        }

        void spin_once()
        {
            if (_prev_stamp > ros::Time(0)) {
                std::lock_guard<std::mutex> pose_guard(_pose_mtx);
                std::lock_guard<std::mutex> vel_guard(_tvel_mtx);

                if (_target_velocities.left == 0. && _target_velocities.right == 0.) {
                    _set_vel_pub.publish(_target_velocities);
                    return;
                }

                double yaw = _pose.pose.rpy.yaw;
                double vx = ((_pose.pose.xyz.x - _prev_pose.pose.xyz.x) / _dt);
                double vy = ((_pose.pose.xyz.y - _prev_pose.pose.xyz.y) / _dt);
                double v = std::sqrt(std::pow(vy, 2.) + std::pow(vx, 2.));
                double w = _angle_to_pipi(_pose.pose.rpy.yaw - _prev_pose.pose.rpy.yaw) / _dt;
                double l = 0.0451;

                _current_velocities.left = (2 * v - w * l) / 2;
                _current_velocities.right = (2 * v + w * l) / 2;

                std::array<double, 2> error;
                error[0] = _target_velocities.left - _current_velocities.left;
                error[1] = _target_velocities.right - _current_velocities.right;

                // proportional
                std::array<double, 2> p;
                p[0] = _Kp[0] * error[0];
                p[1] = _Kp[1] * error[1];

                // integral
                if (_target_velocities != _prev_target) {
                    _integral = {0., 0.};
                    _prev_error = {0., 0.};
                    _prev_target = _target_velocities;
                }
                _integral[0] += error[0] * _dt;
                _integral[1] += error[1] * _dt;
                std::array<double, 2> i;
                i[0] = _Ki[0] * _integral[0];
                i[1] = _Ki[1] * _integral[1];

                // derivative
                std::array<double, 2> d;
                d[0] = _Kd[0] * (error[0] - _prev_error[0]) / _dt;
                d[1] = _Kd[1] * (error[1] - _prev_error[1]) / _dt;

                // clipping values
                bobi_msgs::MotorVelocities new_velocities;
                new_velocities.left = _clip(_current_velocities.left + (p[0] + i[0] + d[0]) / _scaler[0], 0);
                new_velocities.right = _clip(_current_velocities.right + (p[1] + i[1] + d[1]) / _scaler[1], 1);

                if (_verbose) {
                    ROS_INFO("--- dt = %f", _dt);
                    ROS_INFO("Current velocities(left, right) = (%f, %f)", _current_velocities.left, _current_velocities.right);
                    ROS_INFO("Target velocities(left, right) = (%f, %f)", _target_velocities.left, _target_velocities.right);
                    ROS_INFO("New velocities(left, right) = (%f, %f)", new_velocities.left, new_velocities.right);
                }

                _set_vel_pub.publish(new_velocities);
                _prev_error = error;
            }
        }

    protected:
        void _config_cb(bobi_control::VelocityControlConfig& config, uint32_t level)
        {
            ROS_INFO("Updated %s config", __func__);
            _Kp = {config.left_Kp, config.right_Kp};
            _Ki = {config.left_Ki, config.right_Ki};
            _Kd = {config.left_Kd, config.right_Kd};
            _lb = {config.left_lb, config.right_lb};
            _ub = {config.left_ub, config.right_ub};
            _scaler = {config.scaler_left, config.scaler_right};
            _verbose = config.verbose;
        }

        double _clip(double val, size_t idx)
        {
            val = std::min(val, _ub[idx]);
            val = std::max(val, _lb[idx]);
            return val;
        }

        dynamic_reconfigure::Server<bobi_control::VelocityControlConfig> _cfg_server;
        ros::Publisher _set_vel_pub;

        std::array<double, 2> _Kp, _Kd, _Ki;
        std::array<double, 2> _prev_error;
        std::array<double, 2> _integral;
        std::array<double, 2> _lb, _ub;
        std::array<double, 2> _scaler;
        bool _verbose;

        bobi_msgs::MotorVelocities _current_velocities;
        bobi_msgs::MotorVelocities _prev_target;
    };

} // namespace bobi

#endif
