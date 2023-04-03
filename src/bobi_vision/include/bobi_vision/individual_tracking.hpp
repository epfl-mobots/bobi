#ifndef BOBI_INDIVIDUAL_TRACKING_HPP
#define BOBI_INDIVIDUAL_TRACKING_HPP

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_vision/blob_detector.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/GetNumAgents.h>
#include <bobi_msgs/NumAgents.h>

#include <bobi_vision/mask_factory.hpp>
#include <bobi_vision/camera_config.hpp>

#include <dynamic_reconfigure/server.h>
#include <bobi_vision/IndividualTrackerConfig.h>

namespace bobi {
    class IndividualTracking {
    public:
        IndividualTracking(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh), _it(*_nh)
        {
            // get camera config from the parameter server
            _camera_cfg = top::get_camera_config(*nh);

            // set opencv stream
            std::string medium;

            if (_camera_cfg.using_file) {
                _camera = cv::VideoCapture(_camera_cfg.camera_dev_no);
                medium = "Using video file: " + _camera_cfg.camera_dev_no;
            }
            else {
                // _camera = cv::VideoCapture(std::stoi(_camera_cfg.camera_dev_no), cv::CAP_GSTREAMER);
                _camera = cv::VideoCapture(std::stoi(_camera_cfg.camera_dev_no), cv::CAP_V4L2);
                medium = "Using camera device: " + _camera_cfg.camera_dev_no;
            }
            _camera.set(cv::CAP_PROP_FRAME_WIDTH, _camera_cfg.camera_px_width);
            _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _camera_cfg.camera_px_height);

            ROS_INFO("%s", medium.c_str());
            if (!_camera.isOpened()) {
                ROS_ERROR("ERROR: Could not open camera/file");
                assert(false);
            }

            // publisher for the image_transport wrapped image
            _raw_image_pub = _it.advertise("top_camera/image_raw", 1);
            _undistorted_image_pub = _it.advertise("top_camera/image_undistorted", 1);
            _masked_image_pub = _it.advertise("top_camera/image_masked", 1);
            _blob_pub = _it.advertise("top_camera/image_blobs", 1);

            // pose publisher
            _pose_pub = nh->advertise<bobi_msgs::PoseVec>("naive_poses", 1);

            _bd = std::make_unique<BlobDetector>(_cfg, true);

            _distortion_coeffs = cv::Mat(1, 5, CV_64F);
            memcpy(_distortion_coeffs.data, _camera_cfg.distortion_coeffs.data(), _camera_cfg.distortion_coeffs.size() * sizeof(double));
            _camera_mat = cv::Mat(3, 3, CV_64F);
            memcpy(_camera_mat.data, _camera_cfg.camera_matrix.data(), _camera_cfg.camera_matrix.size() * sizeof(double));

            _new_camera_mat = cv::getOptimalNewCameraMatrix(_camera_mat, _distortion_coeffs, cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), 1., cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), &_roi);
            ROS_WARN("Undistorted image ROI set to: %d x %d", _roi.size().width, _roi.size().height);

            _setup_mask = bobi::MaskFactory()(_camera_cfg.mask_type, _camera_cfg.mask_specs, cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), CV_8UC3);

            _num_agent_pub = nh->advertise<bobi_msgs::NumAgents>("num_agents_update", 1);

            dynamic_reconfigure::Server<bobi_vision::IndividualTrackerConfig>::CallbackType f;
            f = boost::bind(&IndividualTracking::_config_cb, this, _1, _2);
            _config_server.setCallback(f);

            _num_agent_srv = _nh->advertiseService("get_num_agents", &IndividualTracking::_num_agent_srv_cb, this);
        }

        bool read()
        {
            return _camera.read(_frame);
        }

        void process()
        {
            cv::cvtColor(_frame, _frame, cv::COLOR_BGR2GRAY); // enforce image_transport compatible grayscale

            cv::undistort(_frame, _frame_und, _camera_mat, _distortion_coeffs, _new_camera_mat);
            _frame_und = _frame_und(_roi);

            _masked_frame = _frame_und.clone();
            _setup_mask->roi(_masked_frame);

            // detect individuals
            std::vector<cv::Point3f> poses2d;
            std::vector<std::vector<cv::Point>> contours;
            std::tie(poses2d, contours) = _bd->detect(_masked_frame);
            if (poses2d.size()) {
                _poses2d = poses2d;
                _contours = contours;
            }

            std::vector<int> idcs_to_remove;
            for (size_t i = 0; i < _poses2d.size(); ++i) {
                cv::patchNaNs(_poses2d, -1);
                if (_poses2d[i].x < 0 || _poses2d[i].y < 0) {
                    idcs_to_remove.push_back(i);
                }
            }

            for (size_t i = 0; i < idcs_to_remove.size(); ++i) {
                auto it = _poses2d.begin();
                std::advance(it, idcs_to_remove[i] - i);
                _poses2d.erase(it);
            }
        }

        void publish(std_msgs::Header header)
        {
            // publish the poses of the individuals that were detected
            bobi_msgs::PoseVec pv;
            for (size_t i = 0; i < _poses2d.size(); ++i) {
                const cv::Point3f& pose2d = _poses2d[i];
                bobi_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.xyz.x = pose2d.x * _camera_cfg.pix2m;
                pose.pose.xyz.y = pose2d.y * _camera_cfg.pix2m;
                pose.pose.rpy.yaw = pose2d.z;
                for (const cv::Point point : _contours[i]) {
                    geometry_msgs::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    pose.pose.contours.push_back(p);
                }
                pv.poses.push_back(pose);
            }
            _pose_pub.publish(pv);

            // publish the image that contains the filtered image
            sensor_msgs::ImagePtr blob_image_ptr = cv_bridge::CvImage(header, "mono8", _bd->get_blob_frame()).toImageMsg();
            _blob_pub.publish(blob_image_ptr);

            // publish raw image
            sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "mono8", _frame).toImageMsg();
            _raw_image_pub.publish(raw_image_ptr);

            sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "mono8", _frame_und).toImageMsg();
            _undistorted_image_pub.publish(undistorted_image_ptr);

            sensor_msgs::ImagePtr masked_image_ptr = cv_bridge::CvImage(header, "mono8", _masked_frame).toImageMsg();
            _masked_image_pub.publish(masked_image_ptr);
        }

    protected:
        void _config_cb(bobi_vision::IndividualTrackerConfig& config, uint32_t level)
        {
            ROS_INFO("Updated config");
            bool num_agents_changed = (_cfg.num_agents != config.num_agents || _cfg.num_robots != config.num_robots || _cfg.num_virtu_agents != config.num_virtu_agents);
            _cfg.num_agents = config.num_agents;
            _cfg.num_robots = config.num_robots;
            _cfg.num_virtu_agents = config.num_virtu_agents;

            _cfg.num_background_samples = config.num_background_samples;
            _cfg.bstor_history = config.bstor_history;
            _cfg.var_threshold = config.var_threshold;
            _cfg.detect_shadows = config.detect_shadows;
            _cfg.learning_rate = config.learning_rate;
            _cfg.relearning_rate = config.relearning_rate;
            _cfg.min_contour_size = config.min_contour_size;
            _cfg.max_contour_size = config.max_contour_size;

            _cfg.quality_level = config.quality_level;
            _cfg.min_distance = config.min_distance;
            _cfg.block_size = config.block_size;
            _cfg.use_harris_dtor = config.use_harris_dtor;
            _cfg.k = config.k;

            _cfg.threshold_new_value = config.threshold_new_value;
            _cfg.threhold_value = config.threhold_value;

            _cfg.el_dim_x = config.el_dim_x;
            _cfg.el_dim_y = config.el_dim_y;
            _cfg.el_x = config.el_x;
            _cfg.el_y = config.el_y;
            _cfg.erode_x = config.erode_x;
            _cfg.erode_y = config.erode_y;
            _cfg.close_x = config.close_x;
            _cfg.close_y = config.close_y;
            _cfg.dilate_x = config.dilate_x;
            _cfg.dilate_y = config.dilate_y;
            _cfg.erode_iters = config.erode_iters;
            _cfg.close_iters = config.close_iters;
            _cfg.dilate_iters = config.dilate_iters;
            _bd->set_config(_cfg);
            if (config.reset_background_dtor) {
                _bd->reset_background_detector();
            }

            if (num_agents_changed) {
                bobi_msgs::NumAgents msg;
                msg.num_agents = _cfg.num_agents;
                msg.num_robots = _cfg.num_robots;
                msg.num_virtu_agents = _cfg.num_virtu_agents;
                _num_agent_pub.publish(msg);
            }
        }

        bool _num_agent_srv_cb(
            bobi_msgs::GetNumAgents::Request& req,
            bobi_msgs::GetNumAgents::Response& res)
        {
            res.info.num_agents = _cfg.num_agents;
            res.info.num_robots = _cfg.num_robots;
            res.info.num_virtu_agents = _cfg.num_virtu_agents;
            return true;
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        image_transport::ImageTransport _it;
        dynamic_reconfigure::Server<bobi_vision::IndividualTrackerConfig> _config_server;
        ros::ServiceServer _num_agent_srv;
        ros::Publisher _num_agent_pub;

        top::CameraConfig _camera_cfg;
        cv::VideoCapture _camera;

        // publisher for the image_transport wrapped image
        image_transport::Publisher _raw_image_pub;
        image_transport::Publisher _undistorted_image_pub;
        image_transport::Publisher _masked_image_pub;
        image_transport::Publisher _blob_pub;

        // pose publisher
        ros::Publisher _pose_pub;

        defaults::BlobDetectorConfig _cfg;
        std::unique_ptr<BlobDetector> _bd;

        cv::Mat _distortion_coeffs;
        cv::Mat _camera_mat;
        cv::Mat _new_camera_mat;

        cv::Rect _roi;
        bobi::MaskPtr _setup_mask;

        cv::Mat _frame;
        cv::Mat _frame_und;
        cv::Mat _masked_frame;

        std::vector<cv::Point3f> _poses2d;
        std::vector<std::vector<cv::Point>> _contours;
    };
} // namespace bobi

#endif