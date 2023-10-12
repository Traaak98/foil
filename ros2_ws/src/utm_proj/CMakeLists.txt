cmake_minimum_required(VERSION 3.8)
project(utm_proj)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sbg_driver REQUIRED)

add_executable(utm_proj_node src/utm_proj_node.cpp)

ament_target_dependencies(utm_proj_node
  rclcpp
  rosidl_default_generators
  geometry_msgs
  sbg_driver
)

target_link_libraries(utm_proj_node "${cpp_typesupport_target}" proj)
target_include_directories(utm_proj_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS utm_proj_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
