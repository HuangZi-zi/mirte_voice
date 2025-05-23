#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sound_trigger");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/play_sound", 10);

    std_msgs::msg::String msg;
    msg.data = "/home/amzi/Documents/RO47007/mirte_voice/obstacle.mp3";  // Path must exist on the robot

    rclcpp::WallRate loop_rate(1);
    for (int i = 0; i < 1; ++i) {  // Publish once
        publisher->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published play_sound command.");
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

