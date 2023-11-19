//
// Created by ryan on 11/18/23.
//
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DiffDriveCommandPublisher : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    void callback() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 20;
        twist_msg.angular.z = 0.0;
        this->publisher->publish(twist_msg);
        std::ostringstream msg;
        msg << "Publishing: Twist(linear.x=" << twist_msg.linear.x
            << ", angular.z=" << twist_msg.angular.z;

        // Comment the line below because it pollutes the console output.
        //RCLCPP_INFO(this->get_logger(), "%s", msg.str().c_str());
    }

public:
    DiffDriveCommandPublisher(): rclcpp::Node("diff_drive_command_publisher") {
        this->publisher = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_cmd", 10);
        this->timer = this->create_wall_timer(200ms, std::bind(&DiffDriveCommandPublisher::callback, this));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
