#ifndef BOBI_LINEAR_SPEED_CONTROL_HPP
#define BOBI_LINEAR_SPEED_CONTROL_HPP

#include <bobi_control/controller_base.hpp>

#include <dynamic_reconfigure/server.h>
#include <bobi_control/APrioriControlConfig.h>

#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/DutyCycle.h>

#include <chrono>

namespace bobi {
    class APrioriControl : public ControllerBase {
    public:
        APrioriControl(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic, const float wheel_radius, const float wheel_distance)
            : ControllerBase(nh, id, pose_topic),
              _wheel_radius(wheel_radius),
              _wheel_distance(wheel_distance),
              _prev_error{0, 0},
              _integral{0, 0},
              _rotating(false),
              _using_robot_motor_feedback(false)
        {
            dynamic_reconfigure::Server<bobi_control::APrioriControlConfig>::CallbackType f;
            f = boost::bind(&APrioriControl::_config_cb, this, _1, _2);
            _cfg_server.setCallback(f);

            _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("set_velocities", 1);
            // _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("target_velocities", 1);
            _cur_vel_sub = nh->subscribe("current_velocities", 1, &APrioriControl::_current_vel_cb, this);

            _max_accel_cli = nh->serviceClient<bobi_msgs::MaxAcceleration>("set_max_acceleration");
            _set_duty_cycle_cli = nh->serviceClient<bobi_msgs::DutyCycle>("set_duty_cycle");

            int rate;
            nh->param<int>("apriori_control/rate", rate, 30);
            _dt_ideal = 1. / rate;

            nh->param<double>("apriori_control/rotation_accel", _rotation_acceleration, 1.5);
        }

        void spin_once()
        {
            ControllerBase::spin_once();

            if (_prev_stamp > ros::Time(0)) {
                std::lock_guard<std::mutex> pose_guard(_pose_mtx);
                std::lock_guard<std::mutex> pos_guard(_tpos_mtx);

                auto euc_distance = [](const bobi_msgs::PoseStamped& lpose, const bobi_msgs::PoseStamped& rpose) {
                    return std::sqrt(std::pow(lpose.pose.xyz.x - rpose.pose.xyz.x, 2.) + std::pow(lpose.pose.xyz.y - rpose.pose.xyz.y, 2.));
                };

                if (euc_distance(_pose, _target_pose) < _distance_threshold
                    || (_target_pose.pose.xyz.x < 0 || _target_pose.pose.xyz.y < 0)) {
                    _set_vel_pub.publish(bobi_msgs::MotorVelocities());
                    _integral = {0., 0.};
                    _prev_error = {0., 0.};
                    _rotating = false;
                    return;
                }

                bool use_mouse_wpt = false;
                if (_desired_speed == 0) {
                    use_mouse_wpt = true;
                }
                else if (_desired_speed > 0) {
                    use_mouse_wpt = false;
                }

                if (_target_pose.pose.xyz.x != _prev_target.pose.xyz.x
                    || _target_pose.pose.xyz.y != _prev_target.pose.xyz.y) {
                    _integral = {0., 0.};
                    _prev_error = {0., 0.};
                    _rotating = false;
                }

                if (_pose.pose.xyz.x == _prev_pose.pose.xyz.x
                    && _pose.pose.xyz.y == _prev_pose.pose.xyz.y
                    && _prev_v[0] != 0 && _prev_v[1] != 0) {
                    _pose.pose.xyz.x += _prev_v[0] * _dt_ideal;
                    _pose.pose.xyz.y += _prev_v[1] * _dt_ideal;
                }

                double yaw = _pose.pose.rpy.yaw;
                double vx = ((_pose.pose.xyz.x - _prev_pose.pose.xyz.x) / _dt_ideal);
                double vy = ((_pose.pose.xyz.y - _prev_pose.pose.xyz.y) / _dt_ideal);
                double v = std::sqrt(std::pow(vy, 2.) + std::pow(vx, 2.));
                double motor_v = std::sqrt(std::pow(_current_velocities.left, 2.) + std::pow(_current_velocities.right, 2.));
                _prev_v = {vx, vy, v};
                double w = _angle_to_pipi(_pose.pose.rpy.yaw - _prev_pose.pose.rpy.yaw) / _dt_ideal;
                const float l = _wheel_distance;
                const float radius = _wheel_radius;
                double theta = _angle_to_pipi(atan2(_target_pose.pose.xyz.y - _pose.pose.xyz.y, _target_pose.pose.xyz.x - _pose.pose.xyz.x));

                std::array<double, 2> error;
                error[0] = euc_distance(_pose, _target_pose);
                error[1] = _angle_to_pipi(yaw - theta);

                if (abs(error[1]) > _rotate_in_place_threshold_ub) {
                    if (!_rotating) {
                        _rotating = true;
                        _integral[0] = 0;
                        _integral[1] = 0;
                    }
                }
                else {
                    if (_rotating && abs(error[1]) <= _rotate_in_place_threshold_lb) {
                        _rotating = false;
                        _integral[0] = 0;
                        _integral[1] = 0;
                    }
                }

                // proportional
                std::array<double, 2> p;
                p[0] = _Kp[0] * error[0];
                p[1] = _Kp[1] * error[1];

                // integral
                _integral[0] += error[0] * _dt_ideal;
                _integral[1] += error[1] * _dt_ideal;
                std::array<double, 2> i;
                i[0] = _Ki[0] * _integral[0];
                i[1] = _Ki[1] * _integral[1];

                // derivative
                std::array<double, 2> d;
                d[0] = _Kd[0] * (error[0] - _prev_error[0]) / _dt_ideal;
                d[1] = _Kd[1] * (error[1] - _prev_error[1]) / _dt_ideal;

                // clipping values
                double v_hat;
                if (use_mouse_wpt) {
                    _desired_acceleration = 1.0;
                    p[0] = 0.072 * error[0];
                    double pid_v = ((p[0] + i[0] + d[0]) / _scaler[0]) / _dt_ideal;
                    v_hat = _clip(pid_v, 0);
                }
                else {
                    double pid_v = ((p[0] + i[0] + d[0]) / _scaler[0]) / _dt_ideal;
                    v_hat = _clip(pid_v + _Kp_ref * _desired_speed, 0);
                }
                double w_hat = _clip(((p[1] + i[1] + d[1]) * _scaler[1]) / _dt_ideal, 1);

                if (_rotating) {
                    v_hat = 0.;
                    _new_velocities.acceleration = _rotation_acceleration;
                }
                else {
                    _new_velocities.acceleration = 0; // this will set the default acceleration
                    // _new_velocities.acceleration = _desired_acceleration + 0.1;
                }

                _new_velocities.left = (2 * v_hat - w_hat * l) / 2.;
                _new_velocities.right = (2 * v_hat + w_hat * l) / 2.;

                if (_verbose) {
                    if (!_using_robot_motor_feedback) {
                        _current_velocities.left = (2 * v - w * l) / 2 * radius;
                        _current_velocities.right = (2 * v + w * l) / 2 * radius;
                    }
                    else {
                        _using_robot_motor_feedback = false;
                    }
                    _prev_v = {vx, vy, v};

                    ROS_INFO("--- dt = %f", _dt_ideal);
                    ROS_INFO("Current velocities(left, right) = (%f, %f)", _current_velocities.left, _current_velocities.right);
                    ROS_INFO("Current velocities(v, w) = (%f, %f)", v, w);
                    ROS_INFO("Target position(x, y) = (%f, %f)", _target_pose.pose.xyz.x, _target_pose.pose.xyz.y);
                    ROS_INFO("Current position(x, y) = (%f, %f)", _pose.pose.xyz.x, _pose.pose.xyz.y);
                    ROS_INFO("New velocities(left, right) = (%f, %f)", _new_velocities.left, _new_velocities.right);
                    ROS_INFO("New velocities(v, w) = (%f, %f)", v_hat, w_hat);
                    ROS_INFO("Error(linear, angular) = (%f, %f)", error[0], error[1]);
                }

                _set_vel_pub.publish(_new_velocities);
                _prev_error = error;
                _prev_target = _target_pose;
            }
        }

    protected:
        void _current_vel_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
        {
            _using_robot_motor_feedback = true;
            _current_velocities.left = motor_velocities->left / 100.;
            _current_velocities.right = motor_velocities->right / 100.;
        }

        void _config_cb(bobi_control::APrioriControlConfig& config, uint32_t level)
        {
            ROS_INFO("Updated %s config", __func__);
            _Kp = {config.Kp_v, config.Kp_w};
            _Kp_ref = config.Kp_ref;
            _Ki = {config.Ki_v, config.Ki_w};
            _Kd = {config.Kd_v, config.Kd_w};
            _lb = {config.lb_v, config.lb_w};
            _ub = {config.ub_v, config.ub_w};
            _scaler = {config.scaler_v, config.scaler_w};
            _distance_threshold = config.distance_threshold;
            _rotate_in_place_threshold_ub = config.rotate_in_place_threshold_ub;
            _rotate_in_place_threshold_lb = config.rotate_in_place_threshold_lb;
            _verbose = config.verbose;
        }

        double _clip(double val, size_t idx)
        {
            val = std::min(val, _ub[idx]);
            val = std::max(val, _lb[idx]);
            return val;
        }

        dynamic_reconfigure::Server<bobi_control::APrioriControlConfig> _cfg_server;
        ros::Publisher _set_vel_pub;
        ros::Subscriber _cur_vel_sub;

        ros::ServiceClient _max_accel_cli;
        ros::ServiceClient _set_duty_cycle_cli;

        std::array<double, 2> _Kp, _Kd, _Ki;
        double _Kp_ref;
        std::array<double, 2> _prev_error;
        std::array<double, 2> _integral;
        std::array<double, 2> _lb, _ub;
        std::array<double, 2> _scaler;
        double _distance_threshold;
        double _rotate_in_place_threshold_ub;
        double _rotate_in_place_threshold_lb;
        bool _verbose;

        const float _wheel_radius;
        const float _wheel_distance;
        bool _rotating;
        bool _using_robot_motor_feedback;
        float _dt_ideal;
        double _rotation_acceleration;

        bobi_msgs::PoseStamped _current_position;
        bobi_msgs::MotorVelocities _new_velocities;
        bobi_msgs::MotorVelocities _current_velocities;
        bobi_msgs::PoseStamped _prev_target;
        std::array<double, 3> _prev_v;
    };

} // namespace bobi

#endif
