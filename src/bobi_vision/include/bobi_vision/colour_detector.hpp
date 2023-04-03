#ifndef BOBI_COLOUR_DETECTOR_H
#define BOBI_COLOUR_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <bobi_vision/ColourDetectorConfig.h>

#include <numeric>

namespace bobi {
    namespace defaults {
        struct ColourDetectorConfig {
        };
    } // namespace defaults

    typedef std::vector<std::array<cv::Scalar, 2>> LedPairVec;
    typedef std::tuple<double, double, double, double, double, double> Tuple7d;
    typedef std::vector<std::array<Tuple7d, 2>> TuplePairVec;
    typedef std::vector<std::vector<cv::Point>> Contours;

    class ColourDetector {
    public:
        ColourDetector(const LedPairVec& led_colours, const TuplePairVec& hsv_thresholds, bool use_dynamic_reconf = false)
            : _led_colours(led_colours),
              _hsv_thresholds(hsv_thresholds)
        {
            _poses.resize(_led_colours.size());

            if (use_dynamic_reconf) {
                dynamic_reconfigure::Server<bobi_vision::ColourDetectorConfig>::CallbackType f;
                f = boost::bind(&ColourDetector::_config_cb, this, _1, _2);
                _cd_config_server = std::make_unique<dynamic_reconfigure::Server<bobi_vision::ColourDetectorConfig>>();
                _cd_config_server->setCallback(f);
            }

            // cv::namedWindow("front");
            // cv::namedWindow("back");
            // cv::namedWindow("blurred");
            // cv::namedWindow("morph");
        }

        std::vector<cv::Point3f> detect(cv::Mat& frame)
        {
            cv::Mat blurred;
            cv::blur(frame, blurred, cv::Size(4, 4));

            cv::Mat hsv;
            cv::cvtColor(blurred, hsv, cv::COLOR_RGB2HSV);

            cv::erode(blurred, blurred, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1)));
            cv::dilate(blurred, blurred, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1)));

            cv::Mat binary;
            std::vector<Contours> all_contours;
            std::vector<cv::Point3f> new_poses;
            for (size_t idx = 0; idx < _led_colours.size(); ++idx) {

                std::vector<cv::Point> new_contours;
                for (size_t k = 0; k < 2; ++k) {
                    const cv::Scalar& led = _led_colours[idx][k];
                    const Tuple7d& thres = _hsv_thresholds[idx][k];

                    cv::Scalar lb = cv::Scalar(
                        led[0] * std::get<0>(thres), led[1] * std::get<1>(thres), led[2] * std::get<2>(thres));
                    cv::Scalar ub = cv::Scalar(
                        led[0] * std::get<3>(thres), led[1] * std::get<4>(thres), 255 * std::get<5>(thres));

                    cv::inRange(blurred.clone(), lb, ub, binary);

                    // if (k == 0) {
                    //     cv::imshow("front", binary);
                    //     ROS_INFO_STREAM("Front (rgb): " << scalar_hsv_to_rgb(led));
                    //     ROS_INFO_STREAM("Front (lb): " << scalar_hsv_to_rgb(lb));
                    //     ROS_INFO_STREAM("Front (ub): " << scalar_hsv_to_rgb(ub));
                    // }
                    // else {
                    //     cv::imshow("back", binary);
                    //     ROS_INFO_STREAM("Back (rgb): " << scalar_hsv_to_rgb(led));
                    //     ROS_INFO_STREAM("Back (lb): " << scalar_hsv_to_rgb(lb));
                    //     ROS_INFO_STREAM("Back (ub): " << scalar_hsv_to_rgb(ub));
                    // }
                    // cv::imshow("blurred", blurred);

                    // cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1)));
                    // cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1)));
                    // cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1)));
                    // cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1)));

                    // cv::imshow("morph", binary);
                    // cv::waitKey(10);

                    Contours contours;
                    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                    std::vector<int> idcs(contours.size());
                    std::iota(idcs.begin(), idcs.end(), 0);
                    std::sort(idcs.begin(), idcs.end(), [&contours](int lhs, int rhs) {
                        return cv::contourArea(contours[lhs], false) > cv::contourArea(contours[rhs], false);
                    });

                    if (contours.size() >= 1) {
                        new_contours.push_back(_contour_center(contours[0]));
                    }
                }

                cv::Point3f pose = _poses[idx];
                if (new_contours.size() == 2) {
                    cv::Point3f candidate_pose;
                    candidate_pose.x = (new_contours[0].x + new_contours[1].x) / 2.;
                    candidate_pose.y = (new_contours[0].y + new_contours[1].y) / 2.;
                    candidate_pose.z = std::atan2((new_contours[0].y - new_contours[1].y), new_contours[0].x - new_contours[1].x);
                    if (candidate_pose.x >= 0 && candidate_pose.y >= 0) {
                        pose = candidate_pose;
                    }
                }
                new_poses.push_back(pose);
                all_contours.push_back({new_contours});
            }

            for (const auto& led_contours : all_contours) {
                for (auto& contour : led_contours) {
                    cv::circle(frame, _contour_center(contour), 5, cv::Scalar(0, 255, 0));
                }
            }

            _poses = new_poses;
            return _poses;
        }

    protected:
        inline cv::Point _contour_center(const std::vector<cv::Point> contour) const
        {
            cv::Moments m = cv::moments(contour, true);
            return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        }

        cv::Scalar scalar_hsv_to_rgb(cv::Scalar hsv_s)
        {
            cv::Mat rgb;
            cv::Mat hsv(1, 1, CV_8UC3, hsv_s);
            cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
            return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
        }

        cv::Scalar scalar_rgb_to_hsv(cv::Scalar rgb_s)
        {
            cv::Mat hsv;
            cv::Mat rgb(1, 1, CV_8UC3, rgb_s);
            cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
            return cv::Scalar(hsv.data[0], hsv.data[1], hsv.data[2]);
        }

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

        template <typename T>
        int _sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        void _config_cb(bobi_vision::ColourDetectorConfig& config, uint32_t level)
        {
            // TODO: this is for debugging and tuning colour values
            ROS_INFO("Updated config");

            int robot_id = config.robot_id;

            cv::Scalar front(config.front_h, config.front_s, config.front_v);
            cv::Scalar back(config.back_h, config.back_s, config.back_v);
            _led_colours[robot_id][0] = front;
            _led_colours[robot_id][1] = back;

            std::cout << config.back_h << std::endl;
            std::cout << config.back_s << std::endl;
            std::cout << config.back_v << std::endl;

            std::get<0>(_hsv_thresholds[robot_id][0]) = config.front_h_lb;
            std::get<1>(_hsv_thresholds[robot_id][0]) = config.front_s_lb;
            std::get<2>(_hsv_thresholds[robot_id][0]) = config.front_v_lb;

            std::get<3>(_hsv_thresholds[robot_id][0]) = config.front_h_ub;
            std::get<4>(_hsv_thresholds[robot_id][0]) = config.front_s_ub;
            std::get<5>(_hsv_thresholds[robot_id][0]) = config.front_v_ub;

            std::get<0>(_hsv_thresholds[robot_id][1]) = config.back_h_lb;
            std::get<1>(_hsv_thresholds[robot_id][1]) = config.back_s_lb;
            std::get<2>(_hsv_thresholds[robot_id][1]) = config.back_v_lb;

            std::get<3>(_hsv_thresholds[robot_id][1]) = config.back_h_ub;
            std::get<4>(_hsv_thresholds[robot_id][1]) = config.back_s_ub;
            std::get<5>(_hsv_thresholds[robot_id][1]) = config.back_v_ub;
        }

        LedPairVec _led_colours;
        TuplePairVec _hsv_thresholds;
        std::vector<cv::Point3f> _poses;

        std::unique_ptr<dynamic_reconfigure::Server<bobi_vision::ColourDetectorConfig>> _cd_config_server;

    }; // namespace bobi

} // namespace bobi

#endif