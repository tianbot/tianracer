cmake_minimum_required(VERSION 3.5)
project(tianracer_competition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
ament_package()