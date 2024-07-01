cmake_minimum_required(VERSION 3.8)
project(spoof_camera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(spoof_camera_node src/spoof_camera_node.cpp)
ament_target_dependencies(spoof_camera_node rclcpp sensor_msgs cv_bridge)

# Link OpenCV libraries
target_link_libraries(spoof_camera_node ${OpenCV_LIBRARIES})

install(TARGETS
  spoof_camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()