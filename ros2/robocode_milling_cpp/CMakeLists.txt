cmake_minimum_required(VERSION 3.8)
project(robocode_milling_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(krl_lin_milling src/krl_lin_milling.cpp)
target_compile_features(krl_lin_milling PUBLIC cxx_std_17)
ament_target_dependencies(krl_lin_milling rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS krl_lin_milling DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY nc_program DESTINATION share/${PROJECT_NAME})

ament_package()
