cmake_minimum_required(VERSION 3.0.2)
project(pid_contorl)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PID_DIR   "/home/pid_control")

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pid_contorl
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PID_DIR}/include

)

link_directories(
    ${TRT_DIR}/lib
    /usr/local/lib
)

add_executable(pid_test
  src/main.cpp 
  )

target_link_libraries(pid_test  ${catkin_LIBRARIES} pid)