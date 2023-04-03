
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <bobi_msgs/ConvertCoordinates.h>
#include <bobi_vision/coordinate_mapper.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_converter_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    std::vector<cv::Point2d> points_bottom;
    std::vector<cv::Point2d> points_top;
    top::CameraConfig top_cfg;
    bottom::CameraConfig bottom_cfg;
    std::tie(points_bottom, points_top, top_cfg, bottom_cfg) = init_coordinate_mapper(nh);

    CoordinateMapper top2bottom(nh, "convert_top2bottom", points_top, points_bottom, bottom_cfg.camera_matrix, bottom_cfg.distortion_coeffs, bottom_cfg.pix2m, bottom_cfg.camera_px_width_undistorted, bottom_cfg.camera_px_height_undistorted);
    CoordinateMapper bottom2top(nh, "convert_bottom2top", points_bottom, points_top, top_cfg.camera_matrix, top_cfg.distortion_coeffs, top_cfg.pix2m, top_cfg.camera_px_width_undistorted, top_cfg.camera_px_height_undistorted);

    ros::Rate rate(60);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}