cmake_minimum_required(VERSION 2.8.3)
project(hear_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  genmsg
)

add_message_files(
  FILES
  BB_param.msg
  Info.msg
  MRFT_param.msg
  PID_param.msg
  Point2D.msg
  Pose.msg
  Poses.msg
  Waypoint.msg  
)

add_service_files(
   FILES
   get_float.srv
   get_int8.srv
   get_poses.srv
   Restricted_Norm_Settings.srv
   set_bool.srv
   set_float.srv
   set_floats.srv
   set_int.srv
   set_int8.srv
   set_point.srv
   set_points.srv
   set_pose.srv
   set_poses.srv
   set_vector.srv
   Update_Controller_BB.srv
   Update_Controller_MRFT.srv
   Update_Controller_PID.srv
   Update_Pose_Reference.srv
) 

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
    roscpp
    std_msgs
    geometry_msgs
)
