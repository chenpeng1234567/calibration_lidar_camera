cmake_minimum_required(VERSION 3.14)
project(lidar_camera_handeye_calibrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(handeye_calibrator_node src/handeye_calibrator_node.cpp)
target_include_directories(handeye_calibrator_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(handeye_calibrator_node
  builtin_interfaces geometry_msgs nav_msgs rclcpp std_msgs std_srvs tf2_ros)
target_link_libraries(handeye_calibrator_node Eigen3::Eigen)

install(TARGETS handeye_calibrator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
