cmake_minimum_required(VERSION 3.8)
project(slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bringup REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

#Python executables
install( 
  PROGRAMS scripts/nav2_command.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
