cmake_minimum_required(VERSION 3.0.2)
project(oa_guider)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  drone_trajs
  guidance_law
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES prop_guidance
  CATKIN_DEPENDS roscpp std_msgs  sensor_msgs  nav_msgs drone_trajs  guidance_law
  #DEPENDS prop_guidance
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#add_executable(oagplan src/oagplanner.cpp)
#add_dependencies(oagplan  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(oagplan    ${catkin_LIBRARIES}   )

add_executable(oaguider src/oag_node.cpp src/oag_manager.cpp src/oagfsm.cpp)
add_dependencies(oaguider  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(oaguider    ${catkin_LIBRARIES}  )

add_executable(oagpredictor src/dtopub_node.cpp)
add_dependencies(oagpredictor  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(oagpredictor    ${catkin_LIBRARIES}   )

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})




