cmake_minimum_required(VERSION 3.5)
project(tianracer_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "rclcpp"
  "rclpy"
  "std_msgs"
  "visualization_msgs" 
  "tf2_geometry_msgs"
  "ackermann_msgs" 
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS 
  script/cmd_vel_to_ackermann_drive.py
  script/multi_goals.py
  script/wall_following.py
  DESTINATION lib/${PROJECT_NAME})
add_executable(L1_controller_v2 src/L1_controller_v2.cpp)
#target_link_libraries(L1_controller_v2 ${DEPENDENCIES})
ament_target_dependencies(L1_controller_v2 ${DEPENDENCIES})

ament_export_include_directories(include)
ament_package()
