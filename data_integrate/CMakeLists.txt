cmake_minimum_required(VERSION 2.8.3)
project(data_integrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
)

find_package(OpenCV REQUIRED )
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
# add_executable(data_show_node src/data_show.cpp)
# add_dependencies(data_show_node core_msgs_generate_messages_cpp)


add_executable(demo_simple src/demov2.cpp)
add_dependencies(demo_simple core_msgs_generate_messages_cpp)
# target_link_libraries(data_show_node
#  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
# )
target_link_libraries(demo_simple
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(demo_return src/demo_return2.cpp)
add_dependencies(demo_return core_msgs_generate_messages_cpp)
target_link_libraries(demo_return
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(front src/front.cpp)
add_dependencies(front core_msgs_generate_messages_cpp)
target_link_libraries(front
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(main2 src/mainv2.cpp)
add_dependencies(main2 core_msgs_generate_messages_cpp)
target_link_libraries(main2
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(main src/main.cpp)
add_dependencies(main core_msgs_generate_messages_cpp)
target_link_libraries(main
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
