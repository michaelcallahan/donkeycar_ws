cmake_minimum_required(VERSION 3.8)
project(camera_node_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcamera using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(/usr/include)
link_directories(/usr/share)
link_directories(/usr/lib)
link_directories(/usr/lib/aarch64-linux-gnu)
include_directories(/usr/include/libcamera)
link_directories(/usr/share/libcamera)
link_directories(/usr/lib/aarch64-linux-gnu/libcamera)

include_directories(${OpenCV_INCLUDE_DIRS} ${LIBCAMERA_INCLUDE_DIRS})

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge)

# Link libcamera and OpenCV libraries
target_link_libraries(camera_node ${OpenCV_LIBRARIES} ${LIBCAMERA_LIBRARIES})

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
