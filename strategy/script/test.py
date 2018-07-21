#!/usr/bin/env python
import rospy
from vision.msg import Object
from geometry_msgs.msg import Twist
import numpy as np
import math
import skfuzzy.control as ctrl
import trace
import orbit

CATCH_BALL_DIS = 39
ATTACK_RANGE_ANG = 5

tF = orbit.Fuzzy(42, 0)

def callback(data):
  dis = data.ball_dis
  ang = data.ball_ang

  pub = rospy.Publisher('motion/cmd_vel', Twist, queue_size=1)
  twist = Twist()

  ## Did not find ball
  if ang < -180 or ang > 180:
    twist.linear.x  = 0
    twist.linear.y  = 0
    twist.angular.z = 5
  else:
    oV, oW = tF.fuzzy(dis, ang)
    orbit_speed = 20
    twist.linear.x  = dis*math.sin(math.radians(ang+180))/abs(dis)*oV + orbit_speed
    twist.linear.y  = dis*math.cos(math.radians(ang))/abs(dis)*oV
    twist.angular.z = oW

  pub.publish(twist)

def listener():
  rospy.init_node('main', anonymous=True)
  rospy.Subscriber("vision/object", Object, callback)

  rospy.spin()

if __name__ == '__main__':
  try:
    listener()
  except rospy.ROSInterruptException:
    pass
