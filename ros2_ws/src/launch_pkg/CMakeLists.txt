cmake_minimum_required(VERSION 3.8)
project(launch_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(foil_state REQUIRED)
find_package(foil_objective REQUIRED)
find_package(foil_consigne REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
