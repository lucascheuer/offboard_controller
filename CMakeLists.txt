cmake_minimum_required(VERSION 3.5)
project(offboard_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)


include_directories(inc)

add_executable(offboard_controller
  src/offboard_controller.cpp
  src/controller.cpp
  src/min_snap_traj.cpp
  src/polynomial.cpp
)
target_link_libraries(offboard_controller OsqpEigen::OsqpEigen osqp::osqp)
ament_target_dependencies(offboard_controller rclcpp px4_msgs Eigen3 px4_ros_com geometry_msgs apriltag_msgs tf2 tf2_ros visualization_msgs nav_msgs trajectory_msgs)
install(TARGETS offboard_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

ament_package()