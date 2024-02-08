cmake_minimum_required(VERSION 3.8)
project(foil_state)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(custom_msg REQUIRED)


add_executable(foil_state_node src/foil_state_node.cpp)

ament_target_dependencies(foil_state_node
  rclcpp
  rosidl_default_generators
  geometry_msgs
  std_msgs
  sbg_driver
  custom_msg
)

target_include_directories(foil_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS foil_state_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
