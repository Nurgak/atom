cmake_minimum_required(VERSION 2.8.3)
project(atom_vrep_interface)

# Ignore warning flag as V-REP had an issue with an unused return value
add_compile_options(-Wno-unused-result)

find_package(
  catkin REQUIRED
  hardware_interface
  controller_manager
  roscpp
)

catkin_package()

# add "export VREP_ROOT="/home/.../V-REP_..." to ~/.bashrc
if(NOT DEFINED ENV{VREP_ROOT})
  message(FATAL_ERROR "Environment variable \$VREP_ROOT is not set. Please set it to point to the root directory of V-REP.")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  $ENV{VREP_ROOT}
  $ENV{VREP_ROOT}/programming/include
)

add_library(ATOMInterface include/ATOMInterface/ATOMInterface.cpp)
target_link_libraries(
  ATOMInterface
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/atom_vrep_interface_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ATOMInterface
)

install(
  TARGETS ATOMInterface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
