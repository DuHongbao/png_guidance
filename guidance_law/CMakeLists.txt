cmake_minimum_required(VERSION 3.0.2)
project(guidance_law)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  drone_trajs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES guidance_law
  CATKIN_DEPENDS roscpp std_msgs  sensor_msgs  nav_msgs drone_trajs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(guidance_law  src/guidance_law.cpp  src/vehicle.cpp )
add_dependencies(guidance_law  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(guidance_law    ${catkin_LIBRARIES}   )

#add_executable(png_node src/png.cpp src/vehicle.cpp)
#add_dependencies(png_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(png_node    ${catkin_LIBRARIES}  )

add_executable(test_node src/test.cpp  src/vehicle.cpp)
add_dependencies(test_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node    ${catkin_LIBRARIES}   )

