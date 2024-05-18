cmake_minimum_required(VERSION 3.8)
project(ros2_jointprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_pca9685 REQUIRED)

add_executable(ros2_jointprocessor src/ros2_jointprocessor.cpp)
ament_target_dependencies(ros2_jointprocessor rclcpp sensor_msgs ros2_pca9685)

install(TARGETS
  ros2_jointprocessor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
