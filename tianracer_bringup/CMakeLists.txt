cmake_minimum_required(VERSION 3.5)
project(tianracer_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)

string(ASCII 27 Esc)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/99.my_tianracer.dsv.in")
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/99.my_tianracer.dsv.in")
  message("--${Esc}[1;34m *** Add Customized Tianbot Environment Variables *** ${Esc}[0m")
else()
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/99.default_tianracer.dsv.in")
  message("--${Esc}[1;33m *** Attention! Using default Tianbot Environment Variables *** ${Esc}[0m")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()