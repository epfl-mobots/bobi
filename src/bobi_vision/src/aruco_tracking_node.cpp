#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/ConvertCoordinates.h>

#include <bobi_vision/aruco_detector.hpp>
#include <bobi_vision/mask_factory.hpp>

#include <bobi_vision/camera_config.hpp>

#include <cassert>

using namespace bobi;
using namespace top;

double angle_to_pipi(double angle)
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // get camera config from the parameter server
    CameraConfig camera_cfg = get_camera_config(nh);

    // set opencv stream
    std::string medium;
    cv::VideoCapture camera;

    if (camera_cfg.using_file) {
        camera = cv::VideoCapture(camera_cfg.camera_dev_no);
        medium = "Using video file: " + camera_cfg.camera_dev_no;
    }
    else {
        camera = cv::VideoCapture(std::stoi(camera_cfg.camera_dev_no), cv::CAP_GSTREAMER);
        medium = "Using camera device: " + camera_cfg.camera_dev_no;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_cfg.camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_cfg.camera_px_height);

    ROS_INFO("%s", medium.c_str());
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera/file");
        return 1;
    }

    // publisher for the image_transport wrapped image
    image_transport::Publisher raw_image_pub = it.advertise("top_camera/image_raw", 1);
    image_transport::Publisher undistorted_image_pub = it.advertise("top_camera/image_undistorted", 1);

    // coordinate conversion
    ros::ServiceClient top2bottom_srv = nh.serviceClient<bobi_msgs::ConvertCoordinates>("convert_top2bottom");

    // pose publisher
    ros::Publisher pose_pub;
    pose_pub = nh.advertise<bobi_msgs::PoseVec>("robot_poses", 1);

    cv::Mat distortion_coeffs = cv::Mat(1, 5, CV_64F);
    memcpy(distortion_coeffs.data, camera_cfg.distortion_coeffs.data(), camera_cfg.distortion_coeffs.size() * sizeof(double));
    cv::Mat camera_mat = cv::Mat(3, 3, CV_64F);
    memcpy(camera_mat.data, camera_cfg.camera_matrix.data(), camera_cfg.camera_matrix.size() * sizeof(double));

    cv::Rect roi;
    cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(camera_mat, distortion_coeffs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), 1., cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), &roi);

    ArucoDetector ar(camera_mat, distortion_coeffs, 0.08);

    cv::Mat frame;
    cv::Mat frame_und;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        camera >> frame;

        cv::undistort(frame, frame_und, camera_mat, distortion_coeffs, new_camera_mat);
        frame_und = frame_und(roi);

        // detect robots
        std::vector<cv::Vec6d> poses = ar.detect(frame_und); // TODO: here we should be using the pixel_positions so as to not use the ray traced pos

        ar.annotate_image(frame_und);

        // publish the poses of the individuals that were detected
        bobi_msgs::PoseVec pv;
        for (const cv::Vec6f& pose6d : poses) {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p.x = camera_cfg.camera_px_width * camera_cfg.pix2m / 2 + pose6d[0];
            conv.request.p.y = camera_cfg.camera_px_height * camera_cfg.pix2m / 2 - pose6d[1];

            if (!top2bottom_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }

            bobi_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.xyz.x = conv.response.converted_p.x;
            pose.pose.xyz.y = conv.response.converted_p.y;
            pose.pose.xyz.z = conv.response.converted_p.z;
            pose.pose.rpy.yaw = angle_to_pipi(pose6d[5] - M_PI / 2);
            pv.poses.push_back(pose);
        }

        if (poses.size() > 0) {
            pose_pub.publish(pv);
        }

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "bgr8", frame_und).toImageMsg();
        undistorted_image_pub.publish(undistorted_image_ptr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}