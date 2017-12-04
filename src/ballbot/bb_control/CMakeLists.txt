cmake_minimum_required(VERSION 2.8.3)
project(bb_control)

#use cpp11 compiler standard.
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(bb_control_node
    include/bb_control/bb_control.hpp
    src/bb_control.cpp
    )

add_executable(omniwheel_control_node
      include/bb_control/bb_control.hpp
      src/3omniwheel_control.cpp
      )

## Specify libraries to link a library or executable target against
target_link_libraries(bb_control_node
   ${catkin_LIBRARIES}
)

target_link_libraries(omniwheel_control_node
   ${catkin_LIBRARIES}
)
