#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "epos_motion.hpp"
#include "math.h"
//====motor define=====
#define ROBOT_RADIUS 0.15
#define WheelRadius  0.05
#define RPM_Max      6940
#define Gear         26
#define PWM_Range    127.0
#define PWM_Limit_Percent_Min   0.1
#define PWM_Limit_Percent_Max   0.9
#define RPM_Offset 0
#define PWM_MIN 20.0
#define PWM_MAX 103
#define x_inv
#define y_inv
#define yaw_inv 2.3251

//== Motor Contribution ==
//        robot direct
//
//    Motor A       Motor B
//       \           /
//        \         /
//           Robot
//             |
//             |
//           Motor C


EposMotion em1(1, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);
EposMotion em2(2, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);
EposMotion em3(3, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);

void OmniKinematics(const double x, const double y, const double w, double &fa, double &fb, double &fc)
{
  double desireAng = atan2(y, x);
  double velocity = hypot(x, y);
  fa = velocity * cos(150 - desireAng);
  fb = velocity * cos( 30 - desireAng);
  fc = velocity * cos(270 - desireAng);
}

void MotionCallback(const geometry_msgs::Twist& msg)
{
  long sp1 = msg.linear.x;
  long sp2 = msg.linear.y;
  long sp3 = msg.angular.z;
  em1.MotionMove(sp1);
  em2.MotionMove(sp2);
  em3.MotionMove(sp3);
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