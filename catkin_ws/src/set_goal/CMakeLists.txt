cmake_minimum_required(VERSION 3.0.2)
project(set_goal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES set_goal
#  CATKIN_DEPENDS roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(SetGoal src/SetGoal.cpp)
target_link_libraries(SetGoal ${catkin_LIBRARIES})