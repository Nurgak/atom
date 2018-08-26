cmake_minimum_required(VERSION 2.8.3)
project(atom_hardware_interface)

find_package(
  catkin REQUIRED
  hardware_interface
  controller_manager
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(ATOMInterface include/ATOMInterface/ATOMInterface.cpp)
target_link_libraries(
  ATOMInterface
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/atom_hardware_interface_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ATOMInterface
  wiringPi
)

install(
  TARGETS ATOMInterface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
