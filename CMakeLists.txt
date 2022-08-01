cmake_minimum_required(VERSION 3.0.2)
project(tennisball_demo_v2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tennisball_demo_v2
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_robot_info src/get_robot_info.cpp)
target_link_libraries(get_robot_info ${catkin_LIBRARIES})

add_executable(ur5e_motion src/ur5e_motion.cpp)
target_link_libraries(ur5e_motion ${catkin_LIBRARIES})

add_executable(ur5e_single_pose_motion src/ur5e_single_pose_motion.cpp)
target_link_libraries(ur5e_single_pose_motion ${catkin_LIBRARIES})

add_executable(ur5e_multi_pose_motion src/ur5e_multi_pose_motion.cpp)
target_link_libraries(ur5e_multi_pose_motion ${catkin_LIBRARIES})

add_executable(ur5e_add_obstacle src/ur5e_add_obstacle.cpp)
target_link_libraries(ur5e_add_obstacle ${catkin_LIBRARIES})

add_executable(ur5e_home src/ur5e_home.cpp)
target_link_libraries(ur5e_home ${catkin_LIBRARIES})