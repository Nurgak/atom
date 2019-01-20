cmake_minimum_required(VERSION 2.8.3)
project(atom_interface_hardware)

find_package(
  catkin REQUIRED
  hardware_interface
  controller_manager
  roscpp
  atom_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/atom_interface_hardware_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  wiringPi
)
