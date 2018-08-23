r""" robot_port_manager.py

* Author        |   Chu, Chen-You
* Date          |   2018/08/23
* Description   |   The port manager of robot 
* Contact       |   acgeusmile@gmail.com

"""
import rospy
from src.data_structure import data_structure
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from vision.msg import Object
from vision.msg import Two_point
from imu_3d.msg import inertia
from geometry_msgs.msg import Twist

__all__ = ['RobotPortManager']


class RobotPortManager(object):
    r""" The port manager of robot 
    function:
        updateRobotCmd():
            publish robot control command
        _rosSubscriber():
            ros subscriber decalaration
        _rosPublisher():
            ros publisher decalaration
        _initAttr():
            initialize all attrubute args
        _loadParam():
            load strategy parameter
        _rounding(value):
            round the value
    args:
        __robot_info:
            robot information
        __team_color:
            team color
        __robot_param:
            robot parameter
        __imu:
            inertial measurement unit data
    """
    def __init__(self):
        super(RobotPortManager, self).__init__()
        self._initAttr()
        self._loadParam()
        self._rosSubscriber()
        self._rosPublisher()

    def updateRobotCmd(self):
        motor_cmd = Twist()
        self.robot_info.cmd_vel.x = self._rounding(self.robot_info.cmd_vel.x)
        self.robot_info.cmd_vel.y = self._rounding(self.robot_info.cmd_vel.y)
        self.robot_info.cmd_vel.yaw = self._rounding(self.robot_info.cmd_vel.yaw)
        motor_cmd.linear.x = self.robot_info.cmd_vel.x
        motor_cmd.linear.y = self.robot_info.cmd_vel.y
        motor_cmd.angular.z = self.robot_info.cmd_vel.yaw
        shoot_cmd = Int32()
        shoot_cmd.data = self.robot_info.cmd_shoot
        self.pub_cmd_vel.publish(motor_cmd)

    def _rosSubscriber(self):
        rospy.Subscriber('FIRA/SaveParam', Int32, self._subSaveParam)
        rospy.Subscriber('FIRA/TeamColor', String, self._subTeamColor)
        rospy.Subscriber('vision/object', Object, self._subVision)
        rospy.Subscriber('interface/Two_point', Two_point, self._subTwoPoint)
        rospy.Subscriber('motion/remote', Bool, self._subManualCmd)
        rospy.Subscriber('/imu_3d', inertia, self._subIMU)

    def _rosPublisher(self):
        self.pub_cmd_vel = rospy.Publisher('motion/cmd_vel', Twist, queue_size=10) 
        self.pub_cmd_shoot = rospy.Publisher('motion/shoot', Int32, queue_size=10)

    def _initAttr(self):
        self.__robot_info = data_structure.RobotInfo()
        self.__team_color = 'Blue'
        self.__robot_param = data_structure.Param()
        self.__imu = 0
    
    def _loadParam(self):
        self.__robot_param.velocity_param = self._getParam('FIRA/SPlanning_Velocity', [2.2, 0.3, 50, 30, 20, 3, 144, 5])
        self.__robot_param.robot_number = self._getParam('FIRA/RobotNumber', 1)
        self.__robot_param.behavior.goalkeeper = self._getParam('FIRA_Behavior/Goalkeeper', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

    def _getParam(self, topic_name, init_data):
        if type(topic_name) != type(""):
            topic_name = str(topic_name)
        if rospy.has_param(topic_name):
            return rospy.get_param(topic_name)
        else:
            return init_data

    def _rounding(self, value):
        if abs(value) < 0.01:
            value = 0
        return value

    def _subVision(self, vision):
        if self.__team_color == 'Blue':
            self.__robot_info.goal.dis = vision.yellow_dis/100
            self.__robot_info.goal.ang = vision.yellow_ang
            self.__robot_info.op_goal.dis = vision.blue_dis/100
            self.__robot_info.op_goal.ang = vision.blue_ang
        elif self.__team_color == 'Yellow':
            self.__robot_info.goal.dis = vision.blue_dis/100
            self.__robot_info.goal.ang = vision.blue_ang
            self.__robot_info.op_goal.dis = vision.yellow_dis/100
            self.__robot_info.op_goal.ang = vision.yellow_ang
        else:
            raise ValueError('Team color must be either Blue or Yellow !!!')
        self.__robot_info.ball.dis = vision.ball_dis/100
        self.__robot_info.ball.ang = vision.ball_ang

    def _subTeamColor(self, team_color):
        self.__team_color = team_color.data

    def _subSaveParam(self, saveParam):
        self._loadParam()
    
    def _subTwoPoint(self, two_point):
        if self.__team_color == 'Blue':
            self.__robot_info.op_goal.ang_max = two_point.blue_ang_max
            self.__robot_info.op_goal.ang_min = two_point.blue_ang_min
            self.__robot_info.op_goal.left_dis = two_point.blue_left/100
            self.__robot_info.op_goal.right_dis = two_point.blue_right/100
        elif self.__team_color == 'Yellow':
            self.__robot_info.op_goal.ang_max = two_point.yellow_ang_max
            self.__robot_info.op_goal.ang_max = two_point.yellow_ang_min
            self.__robot_info.op_goal.left_dis = two_point.yellow_left/100
            self.__robot_info.op_goal.right_dis = two_point.yellow_right/100
        else:
            print("Unexpected input !!!")

    def _subManualCmd(self, manual_cmd):
        self.manual_cmd = manual_cmd.data

    def _subIMU(self, imu_data):
        self.__imu = imu_data.yaw

    @property
    def team_color(self):
        return self.__team_color
    
    @property
    def robot_info(self):
        return self.__robot_info
    @robot_info.setter
    def robot_info(self, value):
        self.__robot_info = value

    @ property
    def role(self):
        return self.__role

    @ property
    def imu(self):
        return self.__imu

    @ property
    def robot_param(self):
        return self.__robot_param