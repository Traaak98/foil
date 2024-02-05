cmake_minimum_required(VERSION 3.8)
project(launch_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#find_package(esp_nuc REQUIRED)
find_package(foil_cmd_servo_msg REQUIRED)
find_package(foil_consigne REQUIRED)
find_package(foil_consigne_msg REQUIRED)
find_package(foil_height_sensor_message REQUIRED)
find_package(foil_objective REQUIRED)
find_package(foil_objective_msg REQUIRED)
find_package(foil_state REQUIRED)
find_package(foil_state_msg REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(uart_py REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
