cmake_minimum_required(VERSION 3.0.2)
project(tianracer_sim_referee)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/referee_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES scripts/_referee_core.so
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
