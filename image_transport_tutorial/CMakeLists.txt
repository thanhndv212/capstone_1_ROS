cmake_minimum_required(VERSION 2.8)
project(image_transport_tutorial)

add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport message_generation
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )


add_executable(my_publisher src/my_publisher.cpp)
add_dependencies(my_publisher core_msgs_generate_messages_cpp )
target_link_libraries(my_publisher   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
add_dependencies(my_subscriber core_msgs_generate_messages_cpp )
target_link_libraries(my_subscriber   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(my_subscriber_law_image src/my_subscriber_law_image.cpp)
add_dependencies(my_subscriber_law_image core_msgs_generate_messages_cpp)
target_link_libraries(my_subscriber_law_image   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)


# target_link_libraries(ball_image_transport_node
#   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
# )
# target_link_libraries(ball_detect_node_test
#   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
# )
