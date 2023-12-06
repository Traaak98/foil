cmake_minimum_required(VERSION 3.8)
project(foil_consigne)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(foil_state_msg REQUIRED)
find_package(foil_objective_msg REQUIRED)

add_executable(foil_consigne_node src/foil_consigne_node.cpp)

ament_target_dependencies(foil_consigne_node
  rclcpp
  foil_state_msg
  foil_objective_msg
)

target_include_directories(foil_consigne_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS foil_consigne_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
