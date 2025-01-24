cmake_minimum_required(VERSION 3.8)
project(arduinobot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp)

add_executable(simple_moveit_interface src/simple_moveit_interface.cpp)
ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter
  simple_moveit_interface
  simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
