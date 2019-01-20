cmake_minimum_required(VERSION 2.8.3)
project(atom_interface)

find_package(
  catkin REQUIRED
  hardware_interface
  controller_manager
  roscpp
)

catkin_package(INCLUDE_DIRS include)
