#ifndef BOBI_CAMERA_CONFIG_HPP
#define BOBI_CAMERA_CONFIG_HPP

#include <ros/ros.h>
#include <bobi_vision/colour_detector.hpp>

namespace bobi {

    namespace top {

        struct CameraConfig {
            bool using_file = false;
            std::string camera_dev_no = "2";
            int camera_px_width = 512;
            int camera_px_height = 512;
            int camera_px_width_undistorted = 507;
            int camera_px_height_undistorted = 508;
            int fps = 30;
            double pix2m = 0.001475;
            std::vector<double> camera_matrix = {
                783.55455, 0., 256.11758,
                0., 783.21002, 262.14992,
                0., 0., 1.};
            std::vector<double> distortion_coeffs = {-0.176236, 0.418800, 0.005023, -0.002432, 0.000000};

            std::string mask_type;
            std::vector<int> mask_specs;
        };

        CameraConfig get_camera_config(const ros::NodeHandle& nh)
        {
            // configuration for the top camera
            CameraConfig top_camera;
            nh.param<bool>("top_camera/using_file", top_camera.using_file, top_camera.using_file);
            nh.param<std::string>("top_camera/camera_dev_no", top_camera.camera_dev_no, top_camera.camera_dev_no);
            nh.param<int>("top_camera/camera_px_width", top_camera.camera_px_width, top_camera.camera_px_width);
            nh.param<int>("top_camera/camera_px_height", top_camera.camera_px_height, top_camera.camera_px_height);
            nh.param<int>("top_camera/camera_px_width_undistorted", top_camera.camera_px_width_undistorted, top_camera.camera_px_width_undistorted);
            nh.param<int>("top_camera/camera_px_height_undistorted", top_camera.camera_px_height_undistorted, top_camera.camera_px_height_undistorted);
            nh.param<int>("top_camera/fps", top_camera.fps, top_camera.fps);
            nh.param<double>("top_camera/pix2m", top_camera.pix2m, top_camera.pix2m);
            nh.param<std::vector<double>>("top_camera/camera_matrix", top_camera.camera_matrix, top_camera.camera_matrix);
            nh.param<std::vector<double>>("top_camera/distortion_coefficients", top_camera.distortion_coeffs, top_camera.distortion_coeffs);
            nh.param<std::string>("top_camera/mask_type", top_camera.mask_type, top_camera.mask_type);
            nh.param<std::vector<int>>("top_camera/mask", top_camera.mask_specs, top_camera.mask_specs);
            return top_camera;
        }

    } // namespace top

    namespace bottom {
        struct CameraConfig {
            bool using_file = false;
            std::string camera_dev_no = "2";
            int camera_px_width = 640;
            int camera_px_height = 480;
            int camera_px_width_undistorted = 617;
            int camera_px_height_undistorted = 388;
            int fps = 30;
            double pix2m = 0.002681818182;
            std::vector<double> camera_matrix = {
                354.63806, 0., 325.08967,
                0., 354.95442, 234.85626,
                0., 0., 1.};
            std::vector<double> distortion_coeffs = {-0.286276, 0.063449, 0.000008, -0.000346, 0.000000};
            bobi::LedPairVec led_colours;
            bobi::TuplePairVec hsv_thresholds;

            std::string mask_type;
            std::vector<int> mask_specs;
        };

        CameraConfig get_camera_config(const ros::NodeHandle& nh)
        {
            // configuration for the top camera
            CameraConfig bottom_camera;
            nh.param<bool>("bottom_camera/using_file", bottom_camera.using_file, bottom_camera.using_file);
            nh.param<std::string>("bottom_camera/camera_dev_no", bottom_camera.camera_dev_no, bottom_camera.camera_dev_no);
            nh.param<int>("bottom_camera/camera_px_width", bottom_camera.camera_px_width, bottom_camera.camera_px_width);
            nh.param<int>("bottom_camera/camera_px_height", bottom_camera.camera_px_height, bottom_camera.camera_px_height);
            nh.param<int>("bottom_camera/camera_px_width_undistorted", bottom_camera.camera_px_width_undistorted, bottom_camera.camera_px_width_undistorted);
            nh.param<int>("bottom_camera/camera_px_height_undistorted", bottom_camera.camera_px_height_undistorted, bottom_camera.camera_px_height_undistorted);
            nh.param<int>("bottom_camera/fps", bottom_camera.fps, bottom_camera.fps);
            nh.param<double>("bottom_camera/pix2m", bottom_camera.pix2m, bottom_camera.pix2m);
            nh.param<std::vector<double>>("bottom_camera/camera_matrix", bottom_camera.camera_matrix, bottom_camera.camera_matrix);
            nh.param<std::vector<double>>("bottom_camera/distortion_coefficients", bottom_camera.distortion_coeffs, bottom_camera.distortion_coeffs);

            std::vector<int> flat_led_colours;
            nh.param<std::vector<int>>("bottom_camera/led_colours", flat_led_colours, flat_led_colours);
            assert((flat_led_colours.size() % 6) == 0);

            for (size_t i = 0; i < flat_led_colours.size(); i += 6) {
                bottom_camera.led_colours.push_back(
                    {cv::Scalar(
                         flat_led_colours[0],
                         flat_led_colours[1],
                         flat_led_colours[2]),
                        cv::Scalar(
                            flat_led_colours[3],
                            flat_led_colours[4],
                            flat_led_colours[5])});
            }

            std::vector<double> flat_hsv_thresholds;
            nh.param<std::vector<double>>("bottom_camera/hsv_thresholds", flat_hsv_thresholds, flat_hsv_thresholds);
            assert((flat_hsv_thresholds.size() % 6) == 0);

            for (size_t i = 0; i < flat_hsv_thresholds.size(); i += 12) {
                bottom_camera.hsv_thresholds.push_back(
                    {std::make_tuple(
                         flat_hsv_thresholds[0], flat_hsv_thresholds[1], flat_hsv_thresholds[2],
                         flat_hsv_thresholds[3], flat_hsv_thresholds[4], flat_hsv_thresholds[5]),
                        std::make_tuple(
                            flat_hsv_thresholds[6], flat_hsv_thresholds[7], flat_hsv_thresholds[8],
                            flat_hsv_thresholds[9], flat_hsv_thresholds[10], flat_hsv_thresholds[11])});
            }

            nh.param<std::string>("bottom_camera/mask_type", bottom_camera.mask_type, bottom_camera.mask_type);
            nh.param<std::vector<int>>("bottom_camera/mask", bottom_camera.mask_specs, bottom_camera.mask_specs);

            return bottom_camera;
        }
    } // namespace bottom
} // namespace bobi

#endif
