import rospy
# import roslib
from src.data_structure import data_structure
# import message 
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from vision.msg import Object
from vision.msg import Two_point
from imu_3d.msg import inertia
from geometry_msgs.msg import Twist

__all__ = ['RobotPortManager']

class RobotPortManager(object):
    """ The port manager of robot 
        
    """
    def __init__(self):
        super(RobotPortManager, self).__init__()
        self.updateData = True
        self.initAttr()
        self.loadParam()
        rospy.Subscriber('FIRA/SaveParam', Int32, self.subSaveParam)
        rospy.Subscriber('FIRA/TeamColor', String, self.subTeamColor)
        rospy.Subscriber('vision/object', Object, self.subVision)
        rospy.Subscriber('interface/Two_point', Two_point, self.subTwoPoint)
        rospy.Subscriber('motion/remote', Bool, self.subManualCmd)
        rospy.Subscriber('/imu_3d', inertia, self.subIMU)
        self.pub_cmd_vel = rospy.Publisher('motion/cmd_vel', Twist, queue_size=10) 
        self.pub_cmd_shoot = rospy.Publisher('motion/shoot', Int32, queue_size=10)

    def initAttr(self):
        self.__velocity_param = [2.2, 0.3, 50, 30, 20, 3, 144, 5]
        self.__robot_num = 0

        self.__robot_info = data_structure.RobotInfo()
        self.__team_color = 'Blue'
        self.__saveParam = True

        self.__robot_param = data_structure.Param()
        self.__imu = 0

    def subVision(self, vision):
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

    def subTeamColor(self, team_color):
        self.__team_color = team_color.data

    def subSaveParam(self, saveParam):
        self.loadParam()
    
    def subTwoPoint(self, two_point):
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

    def subManualCmd(self, manual_cmd):
        self.manual_cmd = manual_cmd.data

    def subIMU(self, imu_data):
        self.__imu = imu_data.yaw
    
    def loadParam(self):
        if rospy.has_param('FIRA/RobotNumber'):
            self.__robot_num = rospy.get_param('FIRA/RobotNumber')
        if rospy.has_param('FIRA/SPlanning_Velocity'):
            self.__velocity_param = rospy.get_param('FIRA/SPlanning_Velocity')
        if rospy.has_param('FIRA_Behavior/Goalkeeper'):
            self.__robot_param.behavior.goalkeeper = rospy.get_param('FIRA_Behavior/Goalkeeper')

    def updateRobotCmd(self):
        motor_cmd = Twist()
        self.robot_info.cmd_vel.x = self.rounding(self.robot_info.cmd_vel.x)
        self.robot_info.cmd_vel.y = self.rounding(self.robot_info.cmd_vel.y)
        self.robot_info.cmd_vel.yaw = self.rounding(self.robot_info.cmd_vel.yaw)

        motor_cmd.linear.x = self.robot_info.cmd_vel.x
        motor_cmd.linear.y = self.robot_info.cmd_vel.y
        motor_cmd.angular.z = self.robot_info.cmd_vel.yaw

        shoot_cmd = Int32()
        shoot_cmd.data = self.robot_info.cmd_shoot

        self.pub_cmd_vel.publish(motor_cmd)

    def rounding(self, value):
        if abs(value) < 0.01:
            value = 0
        return value

    @property
    def team_color(self):
        return self.__team_color
    
    @property
    def robot_info(self):
        return self.__robot_info
    @robot_info.setter
    def robot_info(self, value):
        self.__robot_info = value

    @property
    def robot_num(self):
        return self.__robot_num

    @ property
    def role(self):
        return self.__role

    @ property
    def velocity_param(self):
        return self.__velocity_param

    @ property
    def imu(self):
        return self.__imu

    @ property
    def robot_param(self):
        return self.__robot_param