#ifndef BOBI_FRAME_INFO_HPP
#define BOBI_FRAME_INFO_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/TargetPose.h>
#include <bobi_msgs/ConvertCoordinates.h>
#include <bobi_msgs/NumAgents.h>
#include <bobi_msgs/GetNumAgents.h>
#include <bobi_vision/coordinate_mapper.hpp>
#include <bobi_msgs/KickSpecs.h>
#include <bobi_vision/mask_factory.hpp>

#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

#define FPS_WINDOW 30

namespace bobi {
    enum CameraLocation {
        NA,
        TOP,
        BOTTOM
    };

    class FrameInfo {
    public:
        FrameInfo(std::shared_ptr<ros::NodeHandle> nh)
            : _nh(nh),
              _filtering_fps(0.),
              _bottom_fps(0.)
        {
            _prev_time = ros::Time::now();

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

            _bottom2top_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_bottom2top");
            _top2bottom_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_top2bottom");
            _num_agents_srv = _nh->serviceClient<bobi_msgs::GetNumAgents>("get_num_agents");
            _bottom2top_srv.waitForExistence();
            _top2bottom_srv.waitForExistence();
            _num_agents_srv.waitForExistence();

            bobi_msgs::GetNumAgents srv;
            if (_num_agents_srv.call(srv)) {
                _num_agents = srv.response.info.num_agents;
                _num_robots = srv.response.info.num_robots;
                _num_virtu_agents = srv.response.info.num_virtu_agents;
                _init_colours();
            }

            _num_agents_sub = _nh->subscribe("num_agents_update", 1, &FrameInfo::_num_agents_cb, this);
            _target_position = _nh->advertise<bobi_msgs::TargetPose>("target_position", 1);

            _start_time = ros::Time::now().toSec();
        }

        void draw_all(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> individual_poses,
            std::vector<bobi_msgs::PoseStamped> robot_poses,
            bobi_msgs::PoseStamped target_position,
            bool ifiltered,
            bool rfiltered,
            CameraLocation camera_loc = CameraLocation::NA)
        {
            draw_fps(frame, camera_loc);
            draw_poses(frame, individual_poses, camera_loc, ifiltered, rfiltered);
            draw_robot_poses(frame, robot_poses, camera_loc, ifiltered, rfiltered);
            draw_center(frame, camera_loc);
            draw_mask(frame, camera_loc);
            draw_target(frame, target_position, camera_loc);
            draw_uptime(frame);
        }

        void draw_fps(cv::Mat& frame, const CameraLocation camera_loc)
        {
            ros::Time now = ros::Time::now();
            double dt = ros::Duration(now - _prev_time).toSec();
            _filtering_fps_window.push_back(dt);
            if (_filtering_fps_window.size() > FPS_WINDOW) {
                _filtering_fps_window.erase(_filtering_fps_window.begin());
            }
            float top_fps = std::accumulate(_filtering_fps_window.begin(), _filtering_fps_window.end(), 0.) / _filtering_fps_window.size();

            // {
            //     std::stringstream stream;
            //     stream << std::fixed << std::setprecision(1) << 1. / top_fps;
            //     cv::putText(frame,
            //         "FPS (processing): " + stream.str(),
            //         cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
            //         cv::FONT_HERSHEY_DUPLEX,
            //         0.5,
            //         CV_RGB(200, 0, 0),
            //         2);
            // }

            switch (camera_loc) {
            case CameraLocation::TOP: {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << _filtering_fps;
                cv::putText(frame,
                    "FPS (w/ filtering): " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 0, 0),
                    2);
            } break;

            case CameraLocation::BOTTOM: {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << _bottom_fps;
                cv::putText(frame,
                    "FPS: " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 0, 0),
                    2);
            } break;
            }

            _prev_time = now;
        }

        void draw_poses(cv::Mat& frame, std::vector<bobi_msgs::PoseStamped> poses, const CameraLocation camera_loc, bool ifiltered, bool rfiltered)
        {
            if (poses.size() > 0 && camera_loc == CameraLocation::TOP) {
                double cur_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_top_header.stamp).toSec();
                if (cur_fps != std::numeric_limits<double>::infinity()) {
                    _top_fps_window.push_back(cur_fps);
                    if (_top_fps_window.size() > FPS_WINDOW) {
                        _top_fps_window.erase(_top_fps_window.begin());
                    }
                    _filtering_fps = std::accumulate(_top_fps_window.begin(), _top_fps_window.end(), 0.) / _top_fps_window.size();
                }
                _prev_top_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];
                if (camera_loc == CameraLocation::BOTTOM) {
                    pose.pose.xyz = _top2bottom->convert(pose.pose.xyz);
                    pose.pose.xyz.x /= _bottom_pix2m;
                    pose.pose.xyz.y /= _bottom_pix2m;
                }
                else {
                    pose.pose.xyz.x /= _top_pix2m;
                    pose.pose.xyz.y /= _top_pix2m;

                    {
                        std::stringstream stream;
                        stream << std::fixed << std::setprecision(1) << _filtering_fps;

                        std::string msg;
                        cv::Scalar colour;
                        if (ifiltered) {
                            msg = "ID Assignment: ACTIVE";
                            colour = CV_RGB(0, 200, 0);
                        }
                        else {
                            msg = "ID Assignment: INACTIVE";
                            colour = CV_RGB(200, 0, 0);
                        }
                        cv::putText(frame,
                            msg,
                            cv::Point(frame.size().width * 0.03, frame.size().height * 0.97),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            colour,
                            2);
                    }
                }

                float r = 30.;
                cv::Point com(pose.pose.xyz.x, pose.pose.xyz.y);
                cv::Point hdg(pose.pose.xyz.x + r * std::cos(pose.pose.rpy.yaw), pose.pose.xyz.y + r * std::sin(pose.pose.rpy.yaw));

                std::lock_guard<std::mutex> guard(_cfg_mutex);
                cv::circle(frame, com, 3, _colours[i], 2);
                cv::arrowedLine(frame, com, hdg, _colours[i], 2, 8, 0, 0.2);
            }
        }

        void draw_robot_poses(cv::Mat& frame, std::vector<bobi_msgs::PoseStamped> poses, const CameraLocation camera_loc, bool ifiltered, bool rfiltered)
        {
            if (poses.size() > 0 && camera_loc == CameraLocation::BOTTOM) {
                double cur_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_bottom_header.stamp).toSec();
                if (cur_fps != std::numeric_limits<double>::infinity()) {
                    _bottom_fps_window.push_back(cur_fps);
                    if (_bottom_fps_window.size() > FPS_WINDOW) {
                        _bottom_fps_window.erase(_bottom_fps_window.begin());
                    }
                    _bottom_fps = std::accumulate(_bottom_fps_window.begin(), _bottom_fps_window.end(), 0.) / _bottom_fps_window.size();
                }
                _prev_bottom_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];

                if (camera_loc == CameraLocation::TOP) {
                    pose.pose.xyz = _bottom2top->convert(pose.pose.xyz);
                    pose.pose.xyz.x /= _top_pix2m;
                    pose.pose.xyz.y /= _top_pix2m;
                }
                else {
                    pose.pose.xyz.x /= _bottom_pix2m;
                    pose.pose.xyz.y /= _bottom_pix2m;

                    float r = 30.;
                    cv::Point com(pose.pose.xyz.x, pose.pose.xyz.y);
                    cv::Point hdg(pose.pose.xyz.x + r * std::cos(pose.pose.rpy.yaw), pose.pose.xyz.y + r * std::sin(pose.pose.rpy.yaw));

                    std::lock_guard<std::mutex> guard(_cfg_mutex);
                    if (_colours_below.size()) {
                        cv::circle(frame, com, 3, _colours_below[i], 2);
                        cv::arrowedLine(frame, com, hdg, _colours_below[i], 2, 8, 0, 0.2);
                    }
                }
            }
        }

        void draw_center(cv::Mat& frame, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::TOP:
                cv::circle(frame, cv::Point(frame.size().width / 2., frame.size().height / 2.), 2, cv::Scalar(31, 95, 255), cv::FILLED);
                break;
            case CameraLocation::BOTTOM:
            default:
                break;
            }
        }

        void draw_mask(cv::Mat& frame, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::TOP:
                _top_mask->draw_roi(frame, cv::Scalar(31, 95, 255));
                break;

            case CameraLocation::BOTTOM:
                _bottom_mask->draw_roi(frame, cv::Scalar(31, 95, 255));
                break;
            }
        }

        void draw_target(cv::Mat& frame, bobi_msgs::PoseStamped target, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::BOTTOM:
                if (target.pose.xyz.x >= 0 && target.pose.xyz.y >= 0) {
                    cv::drawMarker(frame, cv::Point(target.pose.xyz.x / _bottom_pix2m, target.pose.xyz.y / _bottom_pix2m), cv::Scalar(31, 95, 255), cv::MARKER_TILTED_CROSS, 8, 2);
                }
                break;

            case CameraLocation::TOP:
                if (target.pose.xyz.x >= 0 && target.pose.xyz.y >= 0) {
                    target.pose.xyz = _bottom2top->convert(target.pose.xyz);
                    target.pose.xyz.x /= _top_pix2m;
                    target.pose.xyz.y /= _top_pix2m;
                    cv::drawMarker(frame, cv::Point(target.pose.xyz.x, target.pose.xyz.y), cv::Scalar(31, 95, 255), cv::MARKER_TILTED_CROSS, 8, 2);
                }
                break;
            }
        }

        void draw_kick(cv::Mat& frame, bobi_msgs::KickSpecs kick, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::TOP: {
                // cv::Point start(kick.agent.pose.xyz.x / _top_pix2m, kick.agent.pose.xyz.y / _top_pix2m);
                // cv::Point end(kick.target_x / _top_pix2m, kick.target_y / _top_pix2m);
                // cv::drawMarker(frame, start, cv::Scalar(0, 200, 200), cv::MARKER_DIAMOND, 15, 2);
                // cv::arrowedLine(frame, start, end, cv::Scalar(0, 200, 200), 2, cv::LINE_8, 0, 0.15);

                if (kick.perceived > 0) {
                    int step;
                    if (kick.perceived == 1) {
                        step = 100;
                    }
                    else {
                        step = 255. / kick.perceived;
                    }

                    int neighs = std::min(kick.perceived, static_cast<int>(kick.neighs.poses.size()));
                    for (int i = 0; i < neighs; ++i) {
                        bobi_msgs::PoseStamped pose = kick.neighs.poses[i];
                        cv::Point com(pose.pose.xyz.x / _top_pix2m, pose.pose.xyz.y / _top_pix2m);
                        cv::circle(frame, com, 8, cv::Scalar(0, std::min(step * (i + 1), 255), 0), 2);
                    }
                }

            } break;

            case CameraLocation::BOTTOM: {
                if (kick.perceived > 0) {
                    int step;
                    if (kick.perceived == 1) {
                        step = 100;
                    }
                    else {
                        step = 255. / kick.perceived;
                    }

                    int neighs = std::min(kick.perceived, static_cast<int>(kick.neighs.poses.size()));
                    for (int i = 0; i < neighs; ++i) {
                        bobi_msgs::PoseStamped pose = kick.neighs.poses[i];
                        pose.pose.xyz = _top2bottom->convert(pose.pose.xyz);
                        cv::Point com(pose.pose.xyz.x / _bottom_pix2m, pose.pose.xyz.y / _bottom_pix2m);
                        cv::circle(frame, com, 8, cv::Scalar(0, std::max(step * (i + 1), 255), 0), 2);
                    }
                }

            } break;
            }
        }

        void draw_uptime(cv::Mat& frame)
        {
            double now = ros::Time::now().toSec();
            unsigned long int dt = (now - _start_time);
            unsigned long int h = dt / 3600;
            unsigned long int min = (dt / 60) % 60;
            unsigned long int s = dt % 60;

            {
                std::stringstream stream;
                stream << std::fixed << std::setfill('0') << std::setw(2) << h << ":" << std::setw(2) << min << ":" << std::setw(2) << s;
                cv::putText(frame,
                    "Uptime: " + stream.str(),
                    cv::Point(frame.size().width * 0.7, frame.size().height * 0.05),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 200, 0),
                    2);
            }
        }

        void set_top_pix2m(double coeff)
        {
            _top_pix2m = coeff;
        }

        void set_bottom_pix2m(double coeff)
        {
            _bottom_pix2m = coeff;
        }

        double get_top_pix2m() const
        {
            return _top_pix2m;
        }

        double get_bottom_pix2m() const
        {
            return _bottom_pix2m;
        }

        geometry_msgs::Point convert_top2bottom(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_top2bottom_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        geometry_msgs::Point convert_bottom2top(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_bottom2top_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        void set_top_mouse_coords(int x, int y)
        {
            std::lock_guard<std::mutex> guard(_topw_mutex);
            _top_mouse_coords.first.x = x;
            _top_mouse_coords.first.y = y;
            _top_mouse_coords.second.x = x * _top_pix2m;
            _top_mouse_coords.second.y = y * _top_pix2m;
        }

        void set_bottom_mouse_coords(int x, int y)
        {
            std::lock_guard<std::mutex> guard(_bottomw_mutex);
            _bottom_mouse_coords.first.x = x;
            _bottom_mouse_coords.first.y = y;
            _bottom_mouse_coords.second.x = x * _bottom_pix2m;
            _bottom_mouse_coords.second.y = y * _bottom_pix2m;
        }

        void set_target(const bobi_msgs::PoseStamped& ps)
        {
            bobi_msgs::TargetPose t;
            t.target = ps;
            _target_position.publish(t);
        }

        void set_top_mask(bobi::MaskPtr mask)
        {
            _top_mask = mask;
        }

        void set_bottom_mask(bobi::MaskPtr mask)
        {
            _bottom_mask = mask;
        }

    protected:
        void _init_colours()
        {
            std::lock_guard<std::mutex> guard(_cfg_mutex);

            _colours.clear();
            _colours_below.clear();

            for (size_t i = 0; i < _num_robots; ++i) { // ! should be different for multiple robots
                _colours.push_back(cv::Scalar(0, 0, 255));
                _colours_below.push_back(cv::Scalar(0, 0, 255));
            }

            auto rand_in_range = [](int lb = 0, int ub = 255) { return (std::rand() % (ub - lb) + lb); };
            for (size_t i = _num_robots; i < _num_agents; ++i) {
                _colours.push_back(cv::Scalar(255, 0, 0));
                // _colours.push_back(cv::Scalar(rand_in_range(0, 250), rand_in_range(0, 250), rand_in_range(0, 250)));
            }
        }

        void _num_agents_cb(const bobi_msgs::NumAgents::ConstPtr& msg)
        {
            _num_agents = msg->num_agents;
            _num_robots = msg->num_robots;
            _num_virtu_agents = msg->num_virtu_agents;
            _init_colours();
        }

        static void _top_cam_mouse_cb(int event, int x, int y, int flags, void* data)
        {
            FrameInfo* fi = static_cast<FrameInfo*>(data);
            if (event == cv::EVENT_MOUSEMOVE) {
                fi->set_top_mouse_coords(x, y);
            }

            if (flags == (cv::EVENT_FLAG_CTRLKEY + cv::EVENT_LBUTTONDOWN)) {
                geometry_msgs::Point p, converted_p;
                p.x = x * fi->get_top_pix2m();
                p.y = y * fi->get_top_pix2m();
                converted_p = fi->convert_top2bottom(p);
                bobi_msgs::PoseStamped ps;
                ps.pose.xyz.x = converted_p.x;
                ps.pose.xyz.y = converted_p.y;
                ps.header.stamp = ros::Time::now();
                fi->set_target(ps);
            }
        }

        static void _bottom_cam_mouse_cb(int event, int x, int y, int flags, void* data)
        {
            FrameInfo* fi = static_cast<FrameInfo*>(data);
            if (event == cv::EVENT_MOUSEMOVE) {
                fi->set_bottom_mouse_coords(x, y);
            }

            if (flags == (cv::EVENT_FLAG_CTRLKEY + cv::EVENT_LBUTTONDOWN)) {
                float xm = x * fi->get_bottom_pix2m();
                float ym = y * fi->get_bottom_pix2m();
                bobi_msgs::PoseStamped ps;
                ps.pose.xyz.x = xm;
                ps.pose.xyz.y = ym;
                ps.header.stamp = ros::Time::now();
                fi->set_target(ps);
            }
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Publisher _target_position;

        std::vector<cv::Point2d> _points_bottom;
        std::vector<cv::Point2d> _points_top;
        top::CameraConfig _top_cfg;
        bottom::CameraConfig _bottom_cfg;
        std::shared_ptr<CoordinateMapper> _bottom2top;
        std::shared_ptr<CoordinateMapper> _top2bottom;
        ros::ServiceClient _top2bottom_srv;
        ros::ServiceClient _bottom2top_srv;

        ros::ServiceClient _num_agents_srv;
        ros::Subscriber _num_agents_sub;

        ros::Time _prev_time;
        size_t _num_agents;
        size_t _num_robots;
        size_t _num_virtu_agents;

        std::vector<cv::Scalar> _colours;
        std::vector<cv::Scalar> _colours_below;
        std::mutex _cfg_mutex;

        double _filtering_fps;
        double _bottom_fps;
        std_msgs::Header _prev_top_header;
        std_msgs::Header _prev_bottom_header;

        std::mutex _topw_mutex;
        std::mutex _bottomw_mutex;
        std::pair<cv::Point, cv::Point2f> _top_mouse_coords;
        std::pair<cv::Point, cv::Point2f> _bottom_mouse_coords;

        std::vector<float> _top_fps_window;
        std::vector<float> _bottom_fps_window;
        std::vector<float> _filtering_fps_window;

        double _top_pix2m;
        double _bottom_pix2m;
        unsigned long int _start_time;

        bobi::MaskPtr _top_mask;
        bobi::MaskPtr _bottom_mask;
    };
} // namespace bobi

#endif