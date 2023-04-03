#ifndef BOBI_ARUCO_DETECTOR_HPP
#define BOBI_ARUCO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <cassert>

namespace bobi {
    class ArucoDetector {
    public:
        ArucoDetector();

        ArucoDetector(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const float marker_len)
            : _aruco_params(cv::aruco::DetectorParameters::create()),
              _aruco_dict(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)),
              _camera_matrix(camera_matrix),
              _distortion_coeffs(distortion_coeffs),
              _marker_len(marker_len)

        {
        }

        std::vector<cv::Vec6d> detect(const cv::Mat& img)
        {
            _current_ids.clear();
            _current_corners.clear();
            _rejected_candidates.clear();
            _current_poses.clear();

            cv::aruco::detectMarkers(img, _aruco_dict, _current_corners, _current_ids, _aruco_params, _rejected_candidates);
            cv::aruco::estimatePoseSingleMarkers(_current_corners, _marker_len, _camera_matrix, _distortion_coeffs, _rotation_vecs, _translation_vecs);

            for (int i = 0; i < _rotation_vecs.size(); ++i) {
                auto rvec = _rotation_vecs[i];
                auto tvec = _translation_vecs[i];
                tvec[1] = -tvec[1];
                tvec[2] = -tvec[2];

                cv::Mat rot;
                cv::Rodrigues(rvec, rot);
                cv::Vec3d rpy = rot2euler(rot);
                _current_poses.push_back(cv::Vec6d(tvec[0], tvec[1], tvec[2], rpy[0], rpy[1], rpy[2]));
            }

            return _current_poses;
        }

        void annotate_image(cv::Mat& img)
        {
            if (_current_corners.size()) {
                cv::aruco::drawDetectedMarkers(img, _current_corners, _current_ids, cv::Scalar(0, 0, 255));
                for (int i = 0; i < _rotation_vecs.size(); ++i) {
                    auto rvec = _rotation_vecs[i];
                    auto tvec = _translation_vecs[i];
                    cv::aruco::drawAxis(img, _camera_matrix, _distortion_coeffs, rvec, tvec, 0.1);
                }
            }
        }

        cv::Vec3d rot2euler(cv::Mat& rot) const
        {
            double sy = sqrt(rot.at<double>(0, 0) * rot.at<double>(0, 0) + rot.at<double>(1, 0) * rot.at<double>(1, 0));
            double x, y, z;
            if (sy >= 1e-6) {
                x = atan2(rot.at<double>(2, 1), rot.at<double>(2, 2));
                y = atan2(-rot.at<double>(2, 0), sy);
                z = atan2(rot.at<double>(1, 0), rot.at<double>(0, 0));
            }
            else {
                x = atan2(-rot.at<double>(1, 2), rot.at<double>(1, 1));
                y = atan2(-rot.at<double>(2, 0), sy);
                z = 0;
            }
            return cv::Vec3d(x, y, z);
        }

        void set_camera_matrix(const cv::Mat& matrix)
        {
            _camera_matrix = matrix;
        }

        void set_distortion_coeffs(const cv::Mat& matrix)
        {
            _distortion_coeffs = matrix;
        }

        cv::Mat get_camera_matrix() const
        {
            return _camera_matrix;
        }

        cv::Mat get_distortion_coeffs() const
        {
            return _distortion_coeffs;
        }

        std::vector<cv::Vec3d> get_rotation_vecs() const
        {
            return _rotation_vecs;
        }

        std::vector<cv::Vec3d> get_translation_vecs() const
        {
            return _translation_vecs;
        }

        const std::vector<int>& get_current_ids() const
        {
            return _current_ids;
        }

        const std::vector<std::vector<cv::Point2f>>& get_current_corners() const
        {
            return _current_corners;
        }

        const std::vector<std::vector<cv::Point2f>>& get_rejected_candidates() const
        {
            return _rejected_candidates;
        }

        const std::vector<cv::Vec6d>& get_current_poses() const
        {
            return _current_poses;
        }

        const std::vector<cv::Point2f> get_pixel_positions() const
        {
            std::vector<cv::Point2f> positions;
            for (const std::vector<cv::Point2f>& corners : _current_corners) {
                cv::Point2f p;
                p.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.;
                p.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.;
                positions.push_back(p);
            }
            return positions;
        }

    protected:
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

        std::vector<int> _current_ids;
        std::vector<std::vector<cv::Point2f>> _current_corners;
        std::vector<std::vector<cv::Point2f>> _rejected_candidates;
        std::vector<cv::Vec6d> _current_poses;

        cv::Ptr<cv::aruco::DetectorParameters> _aruco_params;
        cv::Ptr<cv::aruco::Dictionary> _aruco_dict;

        cv::Mat _camera_matrix;
        cv::Mat _distortion_coeffs;
        float _marker_len;
        std::vector<cv::Vec3d> _rotation_vecs;
        std::vector<cv::Vec3d> _translation_vecs;
    };

} // namespace bobi

#endif
