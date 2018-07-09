#include <ros/ros.h>
#include "core.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "core_node");
  VisionCore ic;
  ros::spin();
  return 0;
}