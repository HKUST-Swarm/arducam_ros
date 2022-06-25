cmake_minimum_required(VERSION 3.0.2)
project(arducam_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
)

#Find opencv
find_package(OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arducam_ros
#  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(arducam_driver
  src/arducam_driver.cpp
)

add_dependencies(arducam_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/arducam_ros_node.cpp)

add_dependencies(${PROJECT_NAME}_node arducam_driver ${catkin_EXPORTED_TARGETS})

target_link_libraries(arducam_driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  arducam_driver
  ${catkin_LIBRARIES}
)
