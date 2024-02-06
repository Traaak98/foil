cmake_minimum_required(VERSION 3.8)
project(foil_consigne)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(foil_state_msg REQUIRED)
find_package(foil_objective_msg REQUIRED)
find_package(foil_consigne_msg REQUIRED)
find_package(foil_height_sensor_message REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(foil_consigne_node src/foil_consigne_node.cpp)

ament_target_dependencies(foil_consigne_node
  rclcpp
  std_msgs
  geometry_msgs
  foil_state_msg
  foil_objective_msg
  foil_consigne_msg
  foil_height_sensor_message
)

target_include_directories(foil_consigne_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(foil_consigne_node Eigen3::Eigen)

install(TARGETS foil_consigne_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
