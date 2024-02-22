cmake_minimum_required(VERSION 3.0.2)
project(pid)


add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

#
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pid
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${PROJECT_SOURCE_DIR}
)

file(GLOB_RECURSE pid_srcs ${PROJECT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE pid_include ${PROJECT_SOURCE_DIR}/*.h)

add_library(pid SHARED ${pid_srcs})

INSTALL(FILES ${pid_include} DESTINATION /home/pid_control/include)
INSTALL(TARGETS pid DESTINATION /home/pid_control/lib)