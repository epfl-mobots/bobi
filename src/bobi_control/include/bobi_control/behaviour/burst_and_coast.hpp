#ifndef BOBI_BURST_AND_COAST_HPP
#define BOBI_BURST_AND_COAST_HPP

#include <bobi_control/controller_base.hpp>

#include <bobi_control/BurstAndCoastConfig.h>

#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/DutyCycle.h>
#include <bobi_msgs/Pose.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/SpeedEstimateVec.h>
#include <bobi_msgs/KickSpecs.h>
#include <bobi_msgs/TargetPose.h>
#include <bobi_msgs/BacIntervene.h>

#include <tools/random/random_generator.hpp>

#include <dynamic_reconfigure/server.h>

#include <Eigen/Core>

// #define SIMU_MODE
#define USE_BLOCKING_REJ

namespace bobi {
    namespace behaviour {

        namespace defaults {
            struct RummyIndividualParams {
                float radius = 24.5;

                // interactions
                bool use_closest_individual = false;
                int perceived_agents = 1;
                float gamma_rand = 0.3;
                float gamma_wall = 0.23;
                float gamma_sym = 0.;
                float gamma_asym = 0.05;

                float dw = 6.;
                float dc = 1.;
                float alpha_w = 0.;
                float gamma_attraction = 0.3;
                float gamma_alignment = 0.3;

                // kicks
                float vmean = 11.;
                float vmin = 1.;
                float coeff_peak_v = 1.2;
                float vmem = 0.85;
                float vmem12 = 0.6;
                float vcut = 35.;
                float taumean = 0.52;
                float taumin = 0.2;
                float tau0 = 0.8;

                bool reset_current_pose = false;
                bool use_reference_speed = true;
                bool lure_rescue_routine = true;
                float lure_rescue_thres = 7.;
                float ref_reset_thres = 7.;
                float early_kick_thres = 1.;
                bool verbose = false;
                bool pause = false;
                bool reset_with_robot_pose = false;

                // simu
                int itermax = 50;
            };
        } // namespace defaults

        enum Order {
            DECREASING,
            INCREASING
        };

        float ran3()
        {
            return tools::random_in_range(0., 1.);
        }

        using state_t = std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
        using const_state_t = const std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>&;

        class BurstAndCoast : public ControllerBase {
        public:
            BurstAndCoast(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic, defaults::RummyIndividualParams params = defaults::RummyIndividualParams())
                : ControllerBase(nh, id, pose_topic),
                  _params(params),
                  _current_time(0.),
                  _setup_center_top{0., 0.},
                  _setup_center_bottom{0., 0.},
                  _iters(0),
                  _lure_rescue(false),
                  _mean_speed(0.),
                  _reset_current_pose(false),
                  _bac_intervene_flag(false)

            {
                // Initialize ROS interface
                dynamic_reconfigure::Server<bobi_control::BurstAndCoastConfig>::CallbackType f;
                f = boost::bind(&BurstAndCoast::_config_cb, this, _1, _2);
                _cfg_server.setCallback(f);

                assert(nh->getParam("top_camera/pix2m", _top_pix2m));
                assert(nh->getParam("bottom_camera/pix2m", _bottom_pix2m));
                double cx, cy;
                assert(nh->getParam("top_camera/camera_px_width_undistorted", cx));
                assert(nh->getParam("top_camera/camera_px_height_undistorted", cy));
                _setup_center_top = {(cx / 2) * _top_pix2m, (cy / 2) * _top_pix2m};
                bobi_msgs::PoseStamped center_top;
                center_top.pose.xyz.x = _setup_center_top[0];
                center_top.pose.xyz.y = _setup_center_top[1];
                bobi_msgs::PoseStamped center_bottom = convert_top2bottom(center_top);
                _setup_center_bottom = {center_bottom.pose.xyz.x, center_bottom.pose.xyz.y};
                ROS_INFO("Starting with top center (%f, %f)", _setup_center_top[0], _setup_center_top[1]);
                ROS_INFO("Starting with bottom center (%f, %f)", _setup_center_bottom[0], _setup_center_bottom[1]);

                // Initialize model params and the first kick
                _current_time = 0;
                _tau = 0;
                _pose_in_cm.pose.xyz.x = 0;
                _pose_in_cm.pose.xyz.y = 0;
                _pose_in_cm.pose.rpy.yaw = 0;
                _reference_pose.xyz.x = 0;
                _reference_pose.xyz.y = 0;
                _reference_pose.rpy.yaw = 0;
                _reference_pose.rpy.pitch = -1;
                _gatt = _params.gamma_attraction;
                _gali = _params.gamma_alignment;
                _mspeed_coeff = 1.;
                // _reference_pose.rpy.pitch = 0;

                _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("set_velocities", 1);
                _set_target_pose_pub = nh->advertise<bobi_msgs::TargetPose>("target_position", 1);
                _set_target_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("target_velocities", 1);
                _kick_specs_pub = nh->advertise<bobi_msgs::KickSpecs>("kick_specs", 1);

                _bac_intervene_srv = nh->advertiseService("bac_intervene", &BurstAndCoast::_bac_intervene_srv_cb, this);

                // _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("target_velocities", 1);
                _cur_pose_sub = nh->subscribe("filtered_poses", 1, &BurstAndCoast::_individual_poses_cb, this);
                _speed_estimates_sub = nh->subscribe("speed_estimates", 1, &BurstAndCoast::_speed_estimates_cb, this);

                _max_accel_cli = nh->serviceClient<bobi_msgs::MaxAcceleration>("set_max_acceleration");
                _set_duty_cycle_cli = nh->serviceClient<bobi_msgs::DutyCycle>("set_duty_cycle");

                _last_pose_stamp = ros::Time::now();
            }

            void spin_once()
            {
                ControllerBase::spin_once();

                auto euc_distance = [](const bobi_msgs::PoseStamped& lpose, const bobi_msgs::PoseStamped& rpose) {
                    return std::sqrt(std::pow(lpose.pose.xyz.x - rpose.pose.xyz.x, 2.) + std::pow(lpose.pose.xyz.y - rpose.pose.xyz.y, 2.));
                };

                if (_prev_stamp > ros::Time(0)) {
                    std::lock_guard<std::mutex> pose_guard(_pose_mtx);
                    std::lock_guard<std::mutex> pos_guard(_tpos_mtx);

                    if (!_individual_poses.size()) {
                        if (_params.verbose) {
                            ROS_ERROR("No agent information");
                        }
                        return;
                    }

                    _current_time += _dt;

                    _pose_in_cm = _individual_poses[_id];
                    bobi_msgs::PoseStamped rpose = convert_bottom2top(_pose);
                    rpose.pose.xyz.x -= _setup_center_top[0];
                    rpose.pose.xyz.y -= _setup_center_top[1];
                    rpose.pose.xyz.x *= 100;
                    rpose.pose.xyz.y *= 100;

                    if (_params.pause) {
                        _reference_pose = rpose.pose;
                        return;
                    }

                    if (_reference_pose.rpy.pitch == -1) {
                        _reference_pose = _pose_in_cm.pose;
                        _reference_pose.rpy.pitch = 0;
                    }

                    if (_reset_current_pose) {
                        bobi_msgs::PoseStamped ref;
                        ref.pose = _reference_pose;
                        double ref_vs_real;
                        if (_params.reset_with_robot_pose) {
                            ref_vs_real = euc_distance(rpose, ref);
                        }
                        else {
                            ref_vs_real = euc_distance(_individual_poses[_id], ref);
                        }

                        if (_reset_current_pose && (ref_vs_real > _params.ref_reset_thres)) {
                            if (_params.reset_with_robot_pose) {
                                _reference_pose = rpose.pose;
                            }
                            else {
                                _reference_pose = _pose_in_cm.pose;
                            }

                            if (_params.verbose) {
                                ROS_WARN("Too far from reference trajectory %f", ref_vs_real);
                            }

                            _tau = 0;
                            _current_time = 0;
                        }
                    }

                    if (_params.lure_rescue_routine) {

                        double lure_vs_robot_dist = euc_distance(_individual_poses[_id], rpose);
                        if (lure_vs_robot_dist > _params.lure_rescue_thres || _lure_rescue) {
                            _reference_pose = _pose_in_cm.pose;
                            _reference_pose.rpy.pitch = 0;
                            if (_params.verbose) {
                                ROS_WARN("Lure detached (?). Lure rescue routine: enabled");
                            }
                            _lure_rescue = true;
                        }

                        if (lure_vs_robot_dist < 1. && _lure_rescue) {
                            _lure_rescue = false;
                            if (_params.verbose) {
                                ROS_WARN("Lure rescue routine: disabled");
                            }
                        }
                    }

                    { // check if we are close enough to do an early kick
                        bobi_msgs::PoseStamped desired;
                        desired.pose = _desired_pose;
                        double distance_to_goal = euc_distance(desired, rpose);
                        if (distance_to_goal < _params.early_kick_thres) {
                            _tau = 0; // kick earlier than expected if within range of the target position
                            if (_params.verbose) {
                                ROS_WARN("Kicking early");
                            }
                        }
                    }

                    if (_current_time >= _tau) {
                        if (_speeds.size() == 0) {
                            for (size_t i = 0; i < _individual_poses.size(); ++i) {
                                _speeds.push_back(_params.vmin / 100.);
                            }
                        }

                        _kicked = _kick();
                        if (!_kicked) {
#ifndef USE_BLOCKING_REJ
                            if ((_iters > _params.itermax)) {
                                bobi_msgs::PoseStamped target_pose;
                                target_pose.header.stamp = ros::Time::now();
                                float new_r = std::sqrt(_pose_in_cm.pose.xyz.x * _pose_in_cm.pose.xyz.x + _pose_in_cm.pose.xyz.y * _pose_in_cm.pose.xyz.y) * 0.98;
                                float theta = _angle_to_pipi(std::atan2(_pose_in_cm.pose.xyz.y, _pose_in_cm.pose.xyz.x) * 1.1);
                                target_pose.pose.xyz.x = new_r * std::cos(theta);
                                target_pose.pose.xyz.y = new_r * std::sin(theta);
                                target_pose.pose.xyz.x /= 100.; // position in m
                                target_pose.pose.xyz.y /= 100.; // position in m
                                target_pose.pose.xyz.x += _setup_center_top[0]; // offset by center x coordinate
                                target_pose.pose.xyz.y += _setup_center_top[1]; // offset by center y coordinate
                                _set_target_pose_pub.publish(target_pose);
                                _current_time = 0;
                                _tau = 0.5;
                            }

                            ++_iters;
                            _reset_current_pose = true;
#endif
                            return;
                        }
                        else {
                            _current_time = 0;
                            _iters = 0;
                        }

                        bobi_msgs::PoseStamped target_pose;
                        target_pose.header.stamp = ros::Time::now();
                        target_pose.pose = _desired_pose;

                        // Take care of scale
                        target_pose.pose.xyz.x /= 100.; // position in m
                        target_pose.pose.xyz.y /= 100.; // position in m
                        target_pose.pose.xyz.x += _setup_center_top[0]; // offset by center x coordinate
                        target_pose.pose.xyz.y += _setup_center_top[1]; // offset by center y coordinate

                        // store the kick specs for logs (and control?)
                        bobi_msgs::KickSpecs kick_specs;
                        kick_specs.agent.pose = _reference_pose;
                        kick_specs.agent.pose.xyz.x /= 100.; // position in m
                        kick_specs.agent.pose.xyz.y /= 100.; // position in m
                        kick_specs.agent.pose.xyz.x += _setup_center_top[0]; // offset by center x coordinate
                        kick_specs.agent.pose.xyz.y += _setup_center_top[1]; // offset by center y coordinate
                        // kick_specs.agent = convert_top2bottom(kick_specs.agent);

                        // kick_specs.neighs.poses.resize(_individual_poses.size() - _id - 1);
                        for (size_t i : _most_inf_idcs) {
#ifndef SIMU_MODE
                            if (i == _id) {
                                assert(false && "something went wrong");
                            }
#endif

                            bobi_msgs::PoseStamped p;
                            p.header = _individual_poses[i].header;
                            p.pose.rpy.yaw = _individual_poses[i].pose.rpy.yaw;
                            p.pose.xyz.x = _individual_poses[i].pose.xyz.x / 100.; // position in m
                            p.pose.xyz.y = _individual_poses[i].pose.xyz.y / 100.; // position in m
                            p.pose.xyz.x += _setup_center_top[0]; // offset by center x coordinate
                            p.pose.xyz.y += _setup_center_top[1]; // offset by center y coordinate
                            // p = convert_top2bottom(p);
                            kick_specs.neighs.poses.push_back(p);
                        }
                        kick_specs.target_x = target_pose.pose.xyz.x;
                        kick_specs.target_y = target_pose.pose.xyz.y;
                        kick_specs.dl = _speed * _tau + _params.dc;
                        kick_specs.dphi = _angle_to_pipi(target_pose.pose.rpy.yaw - _reference_pose.rpy.yaw);
                        kick_specs.phi = target_pose.pose.rpy.yaw;
                        kick_specs.tau = _tau;
                        kick_specs.tau0 = _params.tau0;
                        kick_specs.perceived = _params.perceived_agents;
                        kick_specs.intervene = _bac_intervene_flag;
                        kick_specs.gatt = _gatt;
                        kick_specs.gali = _gali;
                        _kick_specs_pub.publish(kick_specs);

                        _prev_reference_pose = _reference_pose;
                        _reference_pose = _desired_pose;
                        float prev_speed = _speed;
                        _speed = std::sqrt(std::pow(_reference_pose.xyz.x - _prev_reference_pose.xyz.x, 2)
                                     + std::pow(_reference_pose.xyz.y - _prev_reference_pose.xyz.y, 2))
                            / _tau;

                        _mean_speed = _speed / 100.; // speed in m/s
                        // _target_velocities.resultant = _mean_speed;
                        // _set_target_vel_pub.publish(_target_velocities);

                        target_pose = convert_top2bottom(target_pose);
                        bobi_msgs::TargetPose t;
                        t.target = target_pose;
                        t.desired_speed = _mean_speed;
                        t.desired_acceleration = std::abs(_speed - prev_speed) / _tau / 100.;
                        _set_target_pose_pub.publish(t);
                    }
                }
            }

        protected:
            void _individual_poses_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec)
            {
                auto start = ros::Time::now();
                if (pose_vec->poses.size()) {
                    _individual_poses.clear();
                }
                for (size_t i = 0; i < pose_vec->poses.size(); ++i) {
                    bobi_msgs::PoseStamped pose = pose_vec->poses[i];
                    pose.pose.xyz.x -= _setup_center_top[0];
                    pose.pose.xyz.y -= _setup_center_top[1];
                    pose.pose.xyz.x *= 100;
                    pose.pose.xyz.y *= 100;
                    _individual_poses.push_back(pose);
                }
            }

            void _speed_estimates_cb(const bobi_msgs::SpeedEstimateVec::ConstPtr& speed_vec)
            {
                _speeds.clear();
                if (speed_vec->speeds.size() > 0) {
                    std::copy(speed_vec->speeds.begin(), speed_vec->speeds.end(),
                        std::back_inserter(_speeds));
                }
            }

            bool _bac_intervene_srv_cb(
                bobi_msgs::BacIntervene::Request& req,
                bobi_msgs::BacIntervene::Response& res)
            {
                _bac_intervene_flag = req.enable;
                if (req.enable) {
                    _gatt = req.gatt;
                    _gali = req.gali;
                    _mspeed_coeff = req.mspeed_coeff;
                    res.success = true;
                }
                else {
                    _gatt = _params.gamma_attraction;
                    _gali = _params.gamma_alignment;
                    _mspeed_coeff = 1.;
                    res.success = false;
                }

                return true;
            }

            void _config_cb(bobi_control::BurstAndCoastConfig& config, uint32_t level)
            {
                ROS_INFO("Updated %s config", __func__);
                _params.radius = config.radius * 100;
                _params.alpha_w = config.alpha_w;
                _params.use_closest_individual = config.use_closest_individual;
                _params.perceived_agents = config.perceived_agents;
                _params.gamma_rand = config.gamma_rand;
                _params.gamma_wall = config.gamma_wall;
                _params.gamma_sym = config.gamma_sym;
                _params.gamma_asym = config.gamma_asym;
                _params.dw = config.dw;
                _params.dc = config.dc;
                _params.gamma_attraction = config.gamma_attraction;
                _params.gamma_alignment = config.gamma_alignment;
                _params.vmean = config.vmean;
                _params.vmin = config.vmin;
                _params.coeff_peak_v = config.coeff_peak_v;
                _params.vmem = config.vmem;
                _params.vcut = config.vcut;
                _params.taumean = config.taumean;
                _params.taumin = config.taumin;
                _params.tau0 = config.tau0;
                _params.itermax = config.itermax;
                _params.reset_current_pose = config.reset_current_pose;
                _params.use_reference_speed = config.use_reference_speed;
                _params.lure_rescue_routine = config.lure_rescue_routine;
                _params.lure_rescue_thres = config.lure_rescue_thres;
                _params.ref_reset_thres = config.ref_reset_thres;
                _params.early_kick_thres = config.early_kick_thres;
                _params.verbose = config.verbose;
                _params.pause = config.pause;
                _params.reset_with_robot_pose = config.reset_with_robot_pose;
                _reset_current_pose = _params.reset_current_pose;
                if (!_bac_intervene_flag) {
                    _gatt = _params.gamma_attraction;
                    _gali = _params.gamma_alignment;
                    _mspeed_coeff = 1.;
                }
            }

            dynamic_reconfigure::Server<bobi_control::BurstAndCoastConfig> _cfg_server;
            ros::Publisher _set_vel_pub;
            ros::Publisher _set_target_pose_pub;
            ros::Publisher _set_target_vel_pub;
            ros::Publisher _kick_specs_pub;
            ros::Subscriber _cur_pose_sub;
            ros::Subscriber _individual_poses_sub;
            ros::Subscriber _speed_estimates_sub;

            ros::ServiceServer _bac_intervene_srv;

            ros::ServiceClient _max_accel_cli;
            ros::ServiceClient _set_duty_cycle_cli;

            std::array<double, 2> _scaler;
            std::array<double, 2> _setup_center_top;
            std::array<double, 2> _setup_center_bottom;
            double _rotate_in_place_threshold_ub;
            double _rotate_in_place_threshold_lb;
            bool _verbose;
            bool _lure_rescue;
            float _mean_speed;
            bool _reset_current_pose;
            bool _bac_intervene_flag;
            ros::Time _last_pose_stamp;

            bobi_msgs::PoseStamped _pose_in_cm;
            bobi_msgs::MotorVelocities _new_velocities;
            bobi_msgs::MotorVelocities _current_velocities;
            bobi_msgs::PoseStamped _prev_target;
            std::array<double, 3> _prev_v;

            std::vector<bobi_msgs::PoseStamped> _individual_poses;
            std::vector<double> _speeds;

            double _top_pix2m;
            double _bottom_pix2m;

        private:
            bool _kick()
            {
                ++_num_kicks;

                double speed_in_cm;
                if (_reset_current_pose || _params.use_reference_speed) {
                    speed_in_cm = _speeds[_id] * 100;
                }
                else {
                    speed_in_cm = _speed;
                }

                auto state = _compute_state();
                auto neighs = _sort_neighbours(std::get<0>(state), _id, Order::INCREASING); // by distance

                float dphi_int, fw;
                std::tie(dphi_int, fw) = _compute_interactions(state, neighs);

                /**
                 *
                 * Step the model, this is the actual "movement" of the fish
                 *
                 **/
                float r_new;
#ifdef USE_BLOCKING_REJ
                do {
#endif
                    float theta = std::atan2(_reference_pose.xyz.y, _reference_pose.xyz.x);
                    float theta_w = _angle_to_pipi(_reference_pose.rpy.yaw - theta);

                    do {
                        float prob = ran3();
                        if (prob < _params.vmem) {
                            if (prob < _params.vmem12 && neighs.size()) {
                                // _speed = _speeds[cn_idx] * 100 * _params.coeff_peak_v;
                                _speed = 0.;
                                for (size_t idx : neighs) {
                                    _speed += _speeds[idx];
                                }
                                _speed /= _speeds.size();
                                _speed *= 100;
                            }
                            else {
                                _speed = speed_in_cm;
                            }
                        }
                        else {
                            _speed = _params.vmin + _params.vmean * (-log(ran3() * ran3() * ran3())) / 3.;
                        }
                    } while (_speed > _params.vcut); // speed
                    // _speed = std::min(_speed, _params.vcut);
                    _speed = std::max(_speed, _params.vmin);
                    _speed *= _mspeed_coeff;
                    _speed = std::max(_speed, _params.vmin);

                    float dtau = _params.taumean - _params.taumin;
                    _tau = _params.taumin - dtau * 0.5 * log(ran3() * ran3());

                    // cognitive noise
                    float gauss = std::sqrt(-2. * log(ran3())) * cos(2 * M_PI * ran3());
                    float phi_new = _reference_pose.rpy.yaw + dphi_int + _params.gamma_rand * gauss * (1. - _params.alpha_w * fw);
                    // float dl = _speed * _params.tau0 * (1. - std::exp(-_tau / _params.tau0)) + _params.dc;
                    float dl = _speed * _tau + _params.dc;
                    float x_new = _reference_pose.xyz.x + dl * std::cos(phi_new);
                    float y_new = _reference_pose.xyz.y + dl * std::sin(phi_new);
                    r_new = std::sqrt(x_new * x_new + y_new * y_new);

                    _desired_pose.xyz.x = x_new;
                    _desired_pose.xyz.y = y_new;
                    _desired_pose.rpy.yaw = _angle_to_pipi(phi_new);

#ifdef USE_BLOCKING_REJ
                    if (++_iters > _params.itermax) {
                        _iters = 0;
                        float dphiplus = 0.1 * (-log(ran3()));
                        if (theta_w > 0) {
                            _reference_pose.rpy.yaw = theta + M_PI_2 + dphiplus;
                        }
                        else {
                            _reference_pose.rpy.yaw = theta - M_PI_2 - dphiplus;
                        }
                        std::tie(dphi_int, fw) = _compute_interactions(state, neighs);
                    }
                } while (r_new > _params.radius);

                return true;
#else
                if (r_new > _params.radius) {
                    _tau = 0;
                    return false;
                }
                else {
                    return true;
                }
#endif
            }

            state_t _compute_state() const
            {
                size_t num_fish = _individual_poses.size();

                Eigen::VectorXd distances = Eigen::VectorXd::Zero(num_fish);
                Eigen::VectorXd psis_ij = Eigen::VectorXd::Zero(num_fish);
                Eigen::VectorXd psis_ji = Eigen::VectorXd::Zero(num_fish);
                Eigen::VectorXd dphis = Eigen::VectorXd::Zero(num_fish);

                for (uint i = 0; i < num_fish; ++i) {
                    if (_id == i) {
#ifndef SIMU_MODE
                        continue;
#endif
                    }

                    distances(i) = std::sqrt(
                        std::pow(_reference_pose.xyz.x - _individual_poses[i].pose.xyz.x, 2)
                        + std::pow(_reference_pose.xyz.y - _individual_poses[i].pose.xyz.y, 2));

                    psis_ij(i) = std::atan2(_individual_poses[i].pose.xyz.y - _reference_pose.xyz.y,
                                     _individual_poses[i].pose.xyz.x - _reference_pose.xyz.x)
                        - _reference_pose.rpy.yaw;

                    psis_ji(i) = std::atan2(_reference_pose.xyz.y - _individual_poses[i].pose.xyz.y,
                                     _reference_pose.xyz.x - _individual_poses[i].pose.xyz.x)
                        - _individual_poses[i].pose.rpy.yaw;

                    dphis(i) = _individual_poses[i].pose.rpy.yaw - _reference_pose.rpy.yaw;
                }

                return {distances, psis_ij, psis_ji, dphis};
            }

            std::tuple<float, float> _compute_interactions(const state_t& state, std::vector<int> neighs)
            {
                /**
                 *
                 * Compute interactions, i.e., attraction/repulsion between the current individual and its neighbours.
                 *
                 **/

                float theta = std::atan2(_reference_pose.xyz.y, _reference_pose.xyz.x);
                float current_radius_corrected = std::min(static_cast<float>(std::sqrt(_reference_pose.xyz.x * _reference_pose.xyz.x + _reference_pose.xyz.y * _reference_pose.xyz.y)), _params.radius);
                float rw = _params.radius - current_radius_corrected;

                // wall interaction
                float theta_w = _angle_to_pipi(_reference_pose.rpy.yaw - theta);
                float fw = std::exp(-std::pow(rw / _params.dw, 2));
                float ow = std::sin(theta_w) * (1. + 0.7 * std::cos(2. * theta_w));
                float dphiw = _params.gamma_wall * fw * ow;

                float dphiwsym = _params.gamma_sym * std::sin(2. * theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;
                float dphiwasym = _params.gamma_asym * std::cos(theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;

                dphiw += dphiwsym + dphiwasym;

                // fish interaction
                std::vector<float> dphi_fish;
                for (int j : neighs) {
                    float dij = std::get<0>(state)[j];
                    float psi_ij = std::get<1>(state)[j];
                    float dphi_ij = std::get<3>(state)[j];

                    float fatt = (dij - 3.) / 3. / (1. + std::pow(dij / 20., 2));
                    float oatt = std::sin(psi_ij) * (1. - 0.33 * std::cos(psi_ij));
                    // float eatt = 1. - 0.48 * std::cos(dphi_ij) - 0.31 * std::cos(2. * dphi_ij);
                    // float oatt = std::sin(psi_ij) * (1. + std::cos(psi_ij));
                    float eatt = 1.;
                    float dphiatt = _gatt * fatt * oatt * eatt;

                    // float fali = (dij + 3. ) / 6. * std::exp(-std::pow(dij / 20., 2));
                    float fali = std::exp(-std::pow(dij / 20., 2));
                    float oali = std::sin(dphi_ij) * (1. + 0.3 * std::cos(2. * dphi_ij));
                    float eali = 1. + 0.6 * std::cos(psi_ij) - 0.32 * std::cos(2. * psi_ij);
                    float dphiali = _gali * fali * oali * eali;

                    dphi_fish.push_back(dphiatt + dphiali);
                } // for each neighbour

                _most_inf_idcs.clear();
                _most_inf_idcs = neighs;
                if (!_params.use_closest_individual) {
                    std::sort(_most_inf_idcs.begin(), _most_inf_idcs.end(),
                        [&](size_t lidx, size_t ridx) -> bool {
                            return std::abs(dphi_fish[lidx]) < std::abs(dphi_fish[ridx]);
                        });
                    std::sort(dphi_fish.begin(), dphi_fish.end(), [](const float& lv, const float& rv) {
                        return std::abs(lv) > std::abs(rv);
                    });
                }

                size_t offset = std::min(dphi_fish.size(), static_cast<size_t>(_params.perceived_agents));
                float dphi_f = std::accumulate(dphi_fish.begin(), dphi_fish.begin() + offset, 0);
                float dphi_int = dphiw + dphi_f;

                return {dphi_int, fw};
            }

            std::vector<int> _sort_neighbours(const Eigen::VectorXd& values, const int kicker_idx, Order order) const
            {
                std::vector<int> neigh_idcs;
                for (int i = 0; i < values.rows(); ++i) {
                    if (i == kicker_idx) {
#ifndef SIMU_MODE
                        continue;
#endif
                    }
                    neigh_idcs.push_back(i);
                }

                std::sort(std::begin(neigh_idcs), std::end(neigh_idcs), [&](int lhs, int rhs) {
                    return (order == Order::INCREASING) ? (values(lhs) < values(rhs))
                                                        : (values(lhs) > values(rhs));
                });

                return neigh_idcs;
            }

            defaults::RummyIndividualParams _params;

            uint64_t _num_kicks;

            bool _kicked;

            int _iters;
            double _current_time;
            float _tau;
            float _speed;
            bobi_msgs::Pose _desired_pose;
            std::vector<int> _most_inf_idcs;

            bobi_msgs::Pose _reference_pose;
            bobi_msgs::Pose _prev_reference_pose;

            float _gatt;
            float _gali;
            float _mspeed_coeff;

        public:
            float get_tau()
            {
                return _tau;
            }

            float kicked()
            {
                return _kicked;
            }
        };

    } // namespace behaviour
} // namespace bobi

#endif