cmake_minimum_required(VERSION 2.8.3)
project(xbox_ctrl)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  include

  LIBRARIES
  xbox_ctrl

  CATKIN_DEPENDS
  roscpp

  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

find_package( Curses REQUIRED )
include_directories( ${CURSES_INCLUDE_DIRS} )

FIND_PATH(
    UDEV_INCLUDE_DIR
    libudev.h
    /usr/include
    /usr/local/include
    ${UDEV_PATH_INCLUDES}
)

FIND_LIBRARY(
    UDEV_LIBRARIES
    NAMES udev libudev
    PATHS
        /usr/lib${LIB_SUFFIX}
        /usr/local/lib${LIB_SUFFIX}
        ${UDEV_PATH_LIB}
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/gamepad.c
)
target_link_libraries(${PROJECT_NAME} ${UDEV_LIBRARIES} m c)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_old src/xbox_ctrl.cpp)
add_executable(${PROJECT_NAME}_node src/xbox_node.cpp)
add_executable(${PROJECT_NAME}_Client src/xbox_ctrl.cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_old
#    ${catkin_LIBRARIES} ${CURSES_LIBRARIES} ${PROJECT_NAME}
# )
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${CURSES_LIBRARIES} ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_Client
   ${catkin_LIBRARIES} ${CURSES_LIBRARIES} ${PROJECT_NAME}
)
