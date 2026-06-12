cmake_minimum_required(VERSION 3.8)
project(roboracer_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(roboracer_interface_node
  src/roboracer_interface_node.cpp
)

target_include_directories(roboracer_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboracer_interface_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  nav_msgs
  std_msgs
)

install(TARGETS
  roboracer_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
