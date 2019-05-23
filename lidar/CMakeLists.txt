cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
add_message_files(
  FILES
  coor.msg
)

generate_messages(DEPENDENCIES
   std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(lidar_coor_node src/lidar_coor.cpp)
#add_dependenci

target_link_libraries(lidar_coor_node
  ${catkin_LIBRARIES}
)
