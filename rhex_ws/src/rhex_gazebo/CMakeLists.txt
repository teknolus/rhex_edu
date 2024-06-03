cmake_minimum_required(VERSION 3.8)
project(rhex_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO: Add support for ROS2 packages
find_package(ament_cmake REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(std_msgs REQUIRED)

install(
    DIRECTORY
        config launch models worlds
    DESTINATION
        share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
