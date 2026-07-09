/// Sapphire ROS2 — main entry point
/// Minimal: init node, spin. All logic lives in SapphireRos.

#include <rclcpp/rclcpp.hpp>
#include <sapphire_ros2/sapphire_ros.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sapphire_ros2::SapphireRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
