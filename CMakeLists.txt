cmake_minimum_required(VERSION 2.8.3)
project(dbow_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV 3 REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



catkin_package(
 INCLUDE_DIRS include
  LIBRARIES dbow_ros
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
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
  ${OpenCV_INCLUDE_DIRS}  
)
## Declare a C++ executable
add_executable(${PROJECT_NAME} src/dbow_driver src/dbow.cpp)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME} sensor_msgs_gencpp)