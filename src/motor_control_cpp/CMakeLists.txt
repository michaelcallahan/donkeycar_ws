cmake_minimum_required(VERSION 3.8)
project(motor_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Manually specify the paths for libgpiod
include_directories(/usr/include)
link_directories(/usr/lib)
include_directories(/usr/include/i2c)

# Find and include I2C-dev
#pkg_check_modules(I2CDEV REQUIRED i2c-dev)

include_directories(${I2CDEV_INCLUDE_DIRS})

add_executable(motor_control_node src/motor_control_node.cpp)
ament_target_dependencies(motor_control_node rclcpp std_msgs)

# Link libgpiod and I2C-dev libraries
target_link_libraries(motor_control_node gpiod i2c)

install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()