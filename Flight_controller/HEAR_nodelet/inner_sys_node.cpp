#include <ros/ros.h>
#include <nodelet/loader.h>

#include <iostream>

int main(int argc, char **argv){
  ros::init(argc, argv, "inner_sys_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  
  nodelet.load(ros::this_node::getName(), "hear_flight_controller/InnerSysNodelet", remap, nargv);
  
  ros::spin();
  std::cout << "finished node \n";
  return 0;
  }