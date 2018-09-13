cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  imu_3d
  roslib
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS cv_bridge message_generation roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
foreach(subdir
  reset
  akf
  transfer
)
add_subdirectory(${subdir})
endforeach()
