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
        : _nh(nh)
    {
        _rqt_mouse_event_cb = _nh->subscribe("top_camera/image_annot_mouse_left", 1, &ClickToWaypoint::_lmouse_click, this);
        _top2bottom_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_top2bottom");
        _top2bottom_srv.waitForExistence();
        _camera_cfg = get_camera_config(*_nh);
        _nh->param<double>("top_camera/pix2m", _top_pix2m, 0.001475);
        _target_position = _nh->advertise<bobi_msgs::TargetPose>("target_position", 1);
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
    void _lmouse_click(const geometry_msgs::Point& msg)
    {
        geometry_msgs::Point p, converted_p;
        p.x = msg.x * get_pix2m();
        p.y = msg.y * get_pix2m();
        converted_p = convert_top2bottom(p);
        bobi_msgs::PoseStamped ps;
        ps.pose.xyz.x = converted_p.x;
        ps.pose.xyz.y = converted_p.y;
        ps.header.stamp = ros::Time::now();
        set_target(ps);
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    CameraConfig _camera_cfg;

    ros::Subscriber _rqt_mouse_event_cb;
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
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}