#include <memory>
#include <iostream>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;


public:
    explicit StaticFramePublisher()
            : Node("initial_frame_setup")
    {
        this->declare_parameter("fixed_frames", rclcpp::PARAMETER_STRING_ARRAY);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // Publish static transforms once at startup
        this->publish_transform();
    }
private:
    void publish_transform()
    {
        auto res = this->get_parameter("fixed_frames").as_string_array();

        for (const auto& child_frame : res) {
            RCLCPP_INFO(this->get_logger(), "process child frame: %s", child_frame.c_str());
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = child_frame + "/odom";
            this->tf_static_broadcaster_->sendTransform(t);
        }
    }
};

int main(int argc, char * argv[])
{
    auto logger = rclcpp::get_logger("logger");
    // Pass parameters and initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    RCLCPP_INFO(logger,"The program exits.");
    return 0;
}