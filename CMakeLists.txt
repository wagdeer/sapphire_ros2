cmake_minimum_required(VERSION 3.16)
project(sapphire_ros2)

# ament defaults to C++14, override to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(SAPPHIRE_ENABLE_LTO
  "Enable IPO/LTO for Sapphire Release targets when supported" ON)

set(SAPPHIRE_ROS2_IPO_SUPPORTED FALSE)
if(SAPPHIRE_ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT SAPPHIRE_ROS2_IPO_SUPPORTED
    OUTPUT SAPPHIRE_ROS2_IPO_ERROR LANGUAGES CXX)
  if(NOT SAPPHIRE_ROS2_IPO_SUPPORTED)
    message(WARNING
      "SAPPHIRE_ENABLE_LTO is ON, but IPO/LTO is unavailable: "
      "${SAPPHIRE_ROS2_IPO_ERROR}")
  endif()
endif()

# ── Dependencies ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

# Find sapphire core — add_subdirectory in colcon workspace, find_package otherwise
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../sapphire/CMakeLists.txt")
  message(STATUS "sapphire core found as sibling package, using add_subdirectory")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sapphire ${CMAKE_BINARY_DIR}/sapphire)
else()
  find_package(sapphire REQUIRED)
  message(STATUS "sapphire core found via find_package")
endif()

# Adapter library
add_library(sapphire_ros2_adapter
    src/conversion.cpp
    src/sapphire_ros.cpp
)
target_include_directories(sapphire_ros2_adapter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sapphire_ros2_adapter PUBLIC sapphire)
ament_target_dependencies(sapphire_ros2_adapter
    builtin_interfaces rclcpp sensor_msgs nav_msgs geometry_msgs
    visualization_msgs tf2 tf2_ros
)
if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_ROS2_IPO_SUPPORTED)
  set_property(TARGET sapphire_ros2_adapter
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

add_executable(sapphire_ros_node src/main.cpp)
target_link_libraries(
  sapphire_ros_node
  sapphire_ros2_adapter
  spdlog::spdlog
)
ament_target_dependencies(sapphire_ros_node rclcpp)
if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_ROS2_IPO_SUPPORTED)
  set_property(TARGET sapphire_ros_node
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

# Install
install(TARGETS sapphire_ros_node DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS sapphire_ros2_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(sapphire_ros2_conversion_test
    test/conversion_test.cpp
  )
  if(TARGET sapphire_ros2_conversion_test)
    target_link_libraries(
      sapphire_ros2_conversion_test
      sapphire_ros2_adapter
    )
  endif()
endif()

ament_package()
