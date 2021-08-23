cmake_minimum_required(VERSION 3.0.2)
project(pick_e_delivery)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NewGoal.msg
  Pose.msg
  Timeout.msg
)

add_service_files(
  FILES
	setTooLongInterval.srv
	setWaitPackInterval.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pick_e_delivery
#  CATKIN_DEPENDS message_generation roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(SetGoal src/SetGoal.cpp)
#target_link_libraries(SetGoal ${catkin_LIBRARIES})
#add_dependencies(SetGoal pick_e_delivery_generate_messages_cpp)
#
#add_executable(Position src/Position.cpp)
#target_link_libraries(Position ${catkin_LIBRARIES})
#add_dependencies(Position pick_e_delivery_generate_messages_cpp)

add_executable(MainNode src/MainNode.cpp)
target_link_libraries(MainNode ${catkin_LIBRARIES})
add_dependencies(MainNode pick_e_delivery_generate_messages_cpp)