#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/TargetPose.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/ConvertCoordinates.h>
#include <bobi_vision/camera_config.hpp>

#include <mutex>

using namespace bobi;
using namespace top;

class ClickToWaypoint {
public:
    ClickToWaypoint(std::shared_ptr<ros::NodeHandle> nh)
        : _nh(nh), _it(*nh), _window_name("Click to add waypoints")
    {
        _individuals_image = _it.subscribe("top_camera/image_annot", 1, &ClickToWaypoint::_frame_update, this);
        cv::namedWindow(_window_name, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(_window_name, _top_cam_mouse_cb, this);
        _top2bottom_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_top2bottom");
        _top2bottom_srv.waitForExistence();
        _camera_cfg = get_camera_config(*_nh);
        _nh->param<double>("top_camera/pix2m", _top_pix2m, 0.001475);
        _target_position = _nh->advertise<bobi_msgs::TargetPose>("target_position", 1);
    }

    void show()
    {
        const std::lock_guard<std::mutex> lck(_img_ptr_lock);
        if (_cv_img_ptr != nullptr) {
            cv::imshow(_window_name, _cv_img_ptr->image);
        }
        cv::waitKey(10);
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

    double get_pix2m() const
    {
        return _top_pix2m;
    }

    void set_target(const bobi_msgs::PoseStamped& ps)
    {
        bobi_msgs::TargetPose t;
        t.target = ps;
        _target_position.publish(t);
    }

protected:
    void _frame_update(const sensor_msgs::ImageConstPtr& msg)
    {
        const std::lock_guard<std::mutex> lck(_img_ptr_lock);
        try {
            _cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    static void _top_cam_mouse_cb(int event, int x, int y, int flags, void* data)
    {
        ClickToWaypoint* ctw = static_cast<ClickToWaypoint*>(data);
        if (event == cv::EVENT_LBUTTONDOWN) {
            geometry_msgs::Point p, converted_p;
            p.x = x * ctw->get_pix2m();
            p.y = y * ctw->get_pix2m();
            converted_p = ctw->convert_top2bottom(p);
            bobi_msgs::PoseStamped ps;
            ps.pose.xyz.x = converted_p.x;
            ps.pose.xyz.y = converted_p.y;
            ps.header.stamp = ros::Time::now();
            ctw->set_target(ps);
        }
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    image_transport::ImageTransport _it;
    const std::string _window_name;
    CameraConfig _camera_cfg;

    image_transport::Subscriber _individuals_image;
    cv_bridge::CvImagePtr _cv_img_ptr;
    std::mutex _img_ptr_lock;
    ros::ServiceClient _top2bottom_srv;
    double _top_pix2m;
    ros::Publisher _target_position;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "click_to_waypoint_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    ClickToWaypoint ctw(nh);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ctw.show();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}