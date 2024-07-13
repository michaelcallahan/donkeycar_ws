cmake_minimum_required(VERSION 3.5)
project(image_streamer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find GStreamer manually
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(image_streamer src/image_streamer.cpp)
ament_target_dependencies(image_streamer rclcpp sensor_msgs cv_bridge OpenCV)

target_link_libraries(image_streamer
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

install(TARGETS
  image_streamer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
