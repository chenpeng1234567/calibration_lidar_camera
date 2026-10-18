cmake_minimum_required(VERSION 3.14)
project(orb_slam3_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../ORB_SLAM3" CACHE PATH
  "Path to the ORB_SLAM3 source tree")
set(ORB_SLAM3_LIBRARY "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
set(DBOW2_LIBRARY "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIBRARY "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so")

foreach(required_path
    "${ORB_SLAM3_ROOT}/include/System.h"
    "${ORB_SLAM3_LIBRARY}"
    "${DBOW2_LIBRARY}"
    "${G2O_LIBRARY}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR
      "Missing ORB-SLAM3 build artifact: ${required_path}. Run ORB_SLAM3/build.sh first, "
      "then configure with -DORB_SLAM3_ROOT=<path> if needed.")
  endif()
endforeach()

add_executable(mono_node src/mono_node.cpp)
target_include_directories(mono_node PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(mono_node
  cv_bridge geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2_ros)
target_link_libraries(mono_node
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  "${ORB_SLAM3_LIBRARY}"
  "${DBOW2_LIBRARY}"
  "${G2O_LIBRARY}"
  Boost::serialization OpenSSL::Crypto Eigen3::Eigen)
set_target_properties(mono_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")

install(TARGETS mono_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
