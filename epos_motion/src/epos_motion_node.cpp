#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "epos_motion.hpp"

EposMotion em1(1, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);
EposMotion em2(2, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);

void MotionCallback(const geometry_msgs::Twist& msg)
{
  long sp1 = msg.linear.x;
  long sp2 = msg.linear.y;
  em1.MotionMove(sp1);
  em2.MotionMove(sp2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "epos_motion_node");
  ros::NodeHandle n("~");

  ros::Subscriber motion_sub;
  motion_sub = n.subscribe("/epos_motion/cmd_vel", 1, &MotionCallback);

  ros::spin();
  return 0;
}