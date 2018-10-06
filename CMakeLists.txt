cmake_minimum_required(VERSION 2.8.3)
project(solar_buggy)

find_package(catkin REQUIRED 
  COMPONENTS
  geographic_msgs
  message_generation
  rospy
  std_msgs
  uuid_msgs
)

add_message_files(
  FILES
  Pose.msg
  RelationalWayPoint.msg
  Ultrasonic.msg
)

add_service_files(
  FILES
  Controller.srv
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  geographic_msgs 
  message_runtime 
  rospy 
  std_msgs
)
