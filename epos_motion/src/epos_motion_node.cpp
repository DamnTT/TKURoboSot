#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "epos_motion/Reset.h"
#include "epos_motion.hpp"
#include "math.h"
//====motor define=====
#define ROBOT_RADIUS 0.15
#define WheelRadius  0.05
#define RPM_Max      7000
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
//       (robot direct)
//             Y
//             |
//
//    Motor A       Motor B
//       \           /
//        \         /
//           Robot          --> X
//             |
//             |
//           Motor C


EposMotion em1(1, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);
EposMotion em2(2, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);
EposMotion em3(3, "EPOS2", "MAXON SERIAL V2", "USB", "USB0", 1000000);

void OmniKinematics(const double x, const double y, const double w, double &fa, double &fb, double &fc)
{
  double desireAng = atan2(x, y) * (180.0 / M_PI);
  double linear_vel = hypot(x, y) / 100 * RPM_Max;
  double angular_vel = w / 100 * RPM_Max;
  double radFa = (150 - desireAng) * (M_PI / 180.0);
  double radFb = ( 30 - desireAng) * (M_PI / 180.0);
  double radFc = (270 - desireAng) * (M_PI / 180.0);
  fa = linear_vel * cos(radFa) + angular_vel;
  fb = linear_vel * cos(radFb) + angular_vel;
  fc = linear_vel * cos(radFc) + angular_vel;
}

void MotionCallback(const geometry_msgs::Twist& msg)
{
  double sp1, sp2, sp3;
  double x = msg.linear.x;
  double y = msg.linear.y  * -1;
  double z = msg.angular.z * -1;
  x = (x < -100) ? -100 : x;  x = (x >  100) ?  100 : x;
  y = (y < -100) ? -100 : y;  y = (y >  100) ?  100 : y;
  z = (z < -100) ? -100 : z;  z = (z >  100) ?  100 : z;
  OmniKinematics(x, y, z, sp1, sp2, sp3);
  em1.MotionMove(sp1);
  em2.MotionMove(sp2);
  em3.MotionMove(sp3);
}

bool MotorReset(epos_motion::Reset::Request  &req,
                epos_motion::Reset::Response &res)
{
  unsigned int ulErrorCode = 0;
  bool state = false;
  std::string msg = "";
  ROS_INFO("Reset Motor Id: %d", req.id);
  switch(req.id)
  {
    case 1:
      em1.ResetController(&ulErrorCode);
      state = true;
      msg = "Rest Motor 1";
      break;
    case 2:
      em2.ResetController(&ulErrorCode);
      state = true;
      msg = "Rest Motor 2";
      break;
    case 3:
      em3.ResetController(&ulErrorCode);
      state = true;
      msg = "Rest Motor 3";
      break;
    default:
      msg = "Does not have this motor id";
      std::cout << msg << std::endl;
  }
  res.data = state;
  res.message = msg;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "epos_motion_node");
  ros::NodeHandle n("~");

  ros::Subscriber motion_sub;
  ros::ServiceServer motor_reset = n.advertiseService("/epos_motion/reset", MotorReset);
  motion_sub = n.subscribe("/epos_motion/cmd_vel", 1, &MotionCallback);

  ros::spin();
  return 0;
}