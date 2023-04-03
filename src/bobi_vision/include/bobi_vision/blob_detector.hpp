#ifndef BOBI_BLOB_DETECTOR_H
#define BOBI_BLOB_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

namespace bobi {
    namespace defaults {
        struct BlobDetectorConfig {
            // behavioural
            size_t num_agents = 5;
            size_t num_robots = 1;
            size_t num_virtu_agents = 0;

            // background subtractor
            size_t num_background_samples = 50;
            int bstor_history = 50;
            double var_threshold = 1;
            bool detect_shadows = false;
            float learning_rate = 0.05;
            float relearning_rate = 0.0;
            size_t min_contour_size = 10;
            size_t max_contour_size = 200;

            // tracking features
            double quality_level = 0.04;
            double min_distance = 2.;
            int block_size = 3;
            bool use_harris_dtor = true;
            double k = 0.04;

            // Binary threshold
            int threhold_value = 105;
            int threshold_new_value = 255;

            int el_dim_x = 3;
            int el_dim_y = 3;
            int el_x = 1;
            int el_y = 1;
            int erode_x = -1;
            int erode_y = -1;
            int close_x = -1;
            int close_y = -1;
            int dilate_x = -1;
            int dilate_y = -1;
            int erode_iters = 1;
            int close_iters = 1;
            int dilate_iters = 1;
        };
    } // namespace defaults

    class BlobDetector {
    public:
        BlobDetector(defaults::BlobDetectorConfig config = defaults::BlobDetectorConfig(), bool annotate_frame = false) : _config(config),
                                                                                                                          _background_stor(cv::createBackgroundSubtractorMOG2(_config.bstor_history, _config.var_threshold, _config.detect_shadows)),
                                                                                                                          _background_counter(0),
                                                                                                                          _annotate_frame(annotate_frame)
        {
        }

        std::pair<std::vector<cv::Point3f>, std::vector<std::vector<cv::Point>>> detect(cv::Mat& frame)
        {
            if (frame.channels() == 3) {
                cv::cvtColor(frame, _gray_frame, cv::COLOR_RGB2GRAY);
            }
            else {
                _gray_frame = frame.clone();
            }

            cv::Mat thres_frame;
            cv::threshold(_gray_frame, thres_frame, _config.threhold_value, _config.threshold_new_value, cv::THRESH_BINARY_INV);

            if (_background_counter < _config.num_background_samples) {
                _background_stor.get()->apply(thres_frame, _foreground_frame, _config.learning_rate);
                ++_background_counter;
                return std::make_pair(std::vector<cv::Point3f>(), std::vector<std::vector<cv::Point>>());
            }

            _background_stor.get()->apply(thres_frame, _foreground_frame, _config.relearning_rate);
            _remove_contours(_foreground_frame);

            {
                int an = 1;
                cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_config.el_dim_x, _config.el_dim_y), cv::Point(_config.el_x, _config.el_y));
                cv::erode(_foreground_frame, _foreground_frame, element, cv::Point(_config.erode_x, _config.erode_y), _config.erode_iters);
                cv::morphologyEx(_foreground_frame, _foreground_frame, cv::MORPH_CLOSE, element, cv::Point(_config.close_x, _config.close_y), _config.close_iters);
                cv::dilate(_foreground_frame, _foreground_frame, element, cv::Point(_config.dilate_x, _config.dilate_y), _config.dilate_iters);
            }
            _blob_frame = _foreground_frame.clone();

            std::vector<cv::Point2f> corners;
            std::vector<cv::Point2f> centers;
            std::vector<std::vector<cv::Point2f>> corners_in_contours;
            try {
                cv::goodFeaturesToTrack(_gray_frame,
                    corners,
                    _config.num_agents,
                    _config.quality_level,
                    _config.min_distance,
                    _foreground_frame,
                    _config.block_size,
                    _config.use_harris_dtor,
                    _config.k);
            }
            catch (const cv::Exception& e) {
                ROS_ERROR("OpenCV exception: %s", e.what());
            }

            std::vector<std::vector<cv::Point>> contours;
            _find_contours(_foreground_frame, frame, contours, corners, centers, corners_in_contours);
            return std::make_pair(_naive_pose2d(centers, corners_in_contours), contours);
        }

        void reset_background_detector()
        {
            _background_counter = 0;
        }

        cv::Mat get_blob_frame() const
        {
            return _blob_frame;
        }

        void set_config(const defaults::BlobDetectorConfig& config)
        {
            _config = config;
        }

    protected:
        void _remove_contours(cv::Mat& frame)
        {
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            try {
                cv::findContours(frame, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                // cv::findContours(frame, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
            }
            catch (const cv::Exception& e) {
                ROS_ERROR("OpenCV exception: %s", e.what());
            }

            std::vector<std::vector<cv::Point>> remove_countours;
            for (auto& contour : contours) {
                if (cv::contourArea(contour) < _config.min_contour_size || cv::contourArea(contour) > _config.max_contour_size) {
                    remove_countours.push_back(contour);
                }
            }

            if (remove_countours.size() > 0) {
                cv::drawContours(frame, remove_countours, -1, cv::Scalar(0, 0, 0), -1);
            }
        }

        void _find_contours(cv::Mat& frame,
            cv::Mat& annot_frame,
            std::vector<std::vector<cv::Point>>& contours,
            const std::vector<cv::Point2f>& corners,
            std::vector<cv::Point2f>& centers,
            std::vector<std::vector<cv::Point2f>>& corners_in_contours) const
        {
            std::vector<cv::Vec4i> hierarchy;
            try {
                cv::findContours(frame, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            }
            catch (const cv::Exception& e) {
                // ROS_ERROR("OpenCV exception: %s", e.what());
            }

            if (_annotate_frame && hierarchy.size() > 0) {
                int idx = 0;
                for (; idx >= 0; idx = hierarchy[idx][0]) {
                    cv::drawContours(annot_frame, contours, idx, cv::Scalar(0), 1, cv::LINE_8, hierarchy);
                }
            }

            std::vector<std::vector<cv::Point>> contours_poly;
            contours_poly.resize(contours.size());
            std::vector<cv::Point2f> corners_in_contour;
            for (size_t i = 0; i < contours.size(); ++i) {
                cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
                corners_in_contour.clear();
                double min_dist = std::numeric_limits<double>::infinity();
                cv::Point2f min_corner;

                for (auto& corner : corners) {
                    double contour_dist = cv::pointPolygonTest(contours[i], corner, true);
                    if (contour_dist >= 0) {
                        corners_in_contour.push_back(corner);
                    }
                }
                if (corners_in_contour.size() > 0) {
                    centers.push_back(_contour_center(contours[i]));
                    corners_in_contours.push_back(corners_in_contour);
                    // for (size_t j = 0; j < corners_in_contour.size(); ++j) {
                    //     centers.push_back(_contour_center(contours[i]));
                    //     // std::vector<cv::Point2f> closest_corner = {corners_in_contour[j]};
                    //     std::vector<cv::Point2f> closest_corner = {corners_in_contour[i]};
                    //     corners_in_contours.push_back(corners_in_contour);
                    // }
                }
            }

            if (_annotate_frame) {
                for (unsigned int i = 0; i < corners_in_contours.size(); ++i) {
                    cv::circle(annot_frame, corners_in_contours[i][0], 3, cv::Scalar(0), -1, 8, 0);
                    cv::circle(annot_frame, centers[i], 3, cv::Scalar(255), -1, 8, 0);
                }
            }
        }

        inline cv::Point2f _contour_center(const std::vector<cv::Point> contour) const
        {
            cv::Moments m = cv::moments(contour, true);
            return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        }

        inline std::vector<cv::Point3f> _naive_pose2d(const std::vector<cv::Point2f>& centers,
            const std::vector<std::vector<cv::Point2f>>& corners_in_contours) const
        {
            std::vector<cv::Point3f> poses(centers.size());
            for (size_t i = 0; i < poses.size(); ++i) {
                double theta = std::atan2(corners_in_contours[i][0].y - centers[i].y, corners_in_contours[i][0].x - centers[i].x);
                poses[i] = cv::Point3f(centers[i].x, centers[i].y, theta);
                // poses[i] = cv::Point3f(corners_in_contours[i][0].x, corners_in_contours[i][0].y, theta);
            }
            return poses;
        }

        cv::Mat _gray_frame;
        cv::Mat _foreground_frame;
        cv::Mat _blob_frame;
        bool _annotate_frame;
        defaults::BlobDetectorConfig _config;

    private:
        cv::Ptr<cv::BackgroundSubtractor> _background_stor;
        size_t _background_counter;
    };

} // namespace bobi

#endif