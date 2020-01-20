cmake_minimum_required(VERSION 2.8.3)
project(lucia_control_franka)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  #tf2_bullet
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prova_panda
  CATKIN_DEPENDS 
    roscpp 
    rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${franka_control_INCLUDE_DIRS}
  ${franka_visualization_INCLUDE_DIRS}
  ${franka_gripper_INCLUDE_DIRS}
  /opt/ros/kinetic/include/libfranka
  /usr/include/eigen3
  include
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")

add_executable(contr_panda src/contr_panda.cpp src/common.cpp src/library_panda.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against

target_link_libraries(contr_panda
   ${catkin_LIBRARIES}
   /usr/local/lib/libfranka.so.0.6.0
   /opt/ros/kinetic/lib/libeigen_conversions.so
 )



 

 


 
