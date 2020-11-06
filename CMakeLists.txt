cmake_minimum_required(VERSION 3.5)
project(e2e_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(no_dependency src/no_dependency.cpp)
ament_target_dependencies(no_dependency rclcpp sensor_msgs)
install(TARGETS
  no_dependency
  DESTINATION lib/${PROJECT_NAME})

add_executable(sub_dependency src/sub_dependency.cpp)
ament_target_dependencies(sub_dependency rclcpp sensor_msgs)
install(TARGETS
  sub_dependency
  DESTINATION lib/${PROJECT_NAME})

add_executable(timer_dependency src/timer_dependency.cpp)
ament_target_dependencies(timer_dependency rclcpp sensor_msgs)
install(TARGETS
  timer_dependency
  DESTINATION lib/${PROJECT_NAME})

add_executable(sensor_dummy src/sensor_dummy.cpp)
ament_target_dependencies(sensor_dummy rclcpp sensor_msgs)
install(TARGETS
  sensor_dummy
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

ament_package()
