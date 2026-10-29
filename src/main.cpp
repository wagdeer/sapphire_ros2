/// Sapphire ROS2 — main entry point
/// Minimal: init node, spin. All logic lives in SapphireRos.

#include <rclcpp/rclcpp.hpp>
#include <sapphire_ros2/sapphire_ros.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    spdlog::set_level(spdlog::level::warn);
    // Unset keeps warn, so accepted loops stay out of a normal terminal.
    // SAPPHIRE_LOG_LEVEL=info is how a bag replay sees `loop windows`.
    if (const char* level = std::getenv("SAPPHIRE_LOG_LEVEL")) {
        const std::string name(level);
        if (name == "trace") {
            spdlog::set_level(spdlog::level::trace);
        } else if (name == "debug") {
            spdlog::set_level(spdlog::level::debug);
        } else if (name == "info") {
            spdlog::set_level(spdlog::level::info);
        } else if (name == "warn") {
            spdlog::set_level(spdlog::level::warn);
        } else if (name == "error" || name == "err") {
            spdlog::set_level(spdlog::level::err);
        } else if (name == "critical") {
            spdlog::set_level(spdlog::level::critical);
        }
    }
    auto node = std::make_shared<sapphire_ros2::SapphireRos>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
