cmake_minimum_required(VERSION 3.0.2)
project(set_goal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  message_generation
)

add_message_files(
  FILES
  NewGoal.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES set_goal
  CATKIN_DEPENDS roscpp tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(SetGoal src/SetGoal.cpp)
target_link_libraries(SetGoal ${catkin_LIBRARIES})
add_dependencies(SetGoal set_goal_generate_messages_cpp)