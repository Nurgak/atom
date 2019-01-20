cmake_minimum_required(VERSION 2.8.3)
project(atom_interface_vrep)

# Ignore warning flag as V-REP has an issue with an unused return value
add_compile_options(-Wno-unused-result)

find_package(
  catkin REQUIRED
  hardware_interface
  controller_manager
  roscpp
  atom_interface
)

catkin_package()

if(NOT DEFINED ENV{VREP_ROOT})
  message(FATAL_ERROR "Environment variable \$VREP_ROOT is not set. Please set it to point to the root directory of V-REP.")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  $ENV{VREP_ROOT}
  $ENV{VREP_ROOT}/programming/include
)

add_executable(${PROJECT_NAME}_node src/atom_interface_vrep_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
