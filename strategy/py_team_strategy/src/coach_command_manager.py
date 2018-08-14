import rospy
# import roslib
from src.data_structure import data_structure
from src.data_structure import const
# import message 
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from vision.msg import Object
from vision.msg import Two_point
from geometry_msgs.msg import Twist

__all__ = ['CoachCmdManager']

class CoachCmdManager(object):
    """ The port manager of robot 
        
    """
    def __init__(self):
        super(CoachCmdManager, self).__init__()
        self.initAttr()
        rospy.Subscriber('FIRA/GameState', Int32, self.subGameState)
        self.pub_role_1 = rospy.Publisher('FIRA/R1/Strategy/Coach/role', Int32, queue_size=10)
        self.pub_role_2 = rospy.Publisher('FIRA/R1/Strategy/Coach/role', Int32, queue_size=10)
        self.pub_role_3 = rospy.Publisher('FIRA/R1/Strategy/Coach/role', Int32, queue_size=10)


    def initAttr(self):
        self.game_state = const.GAMESTATE_HALT
        self.robot_1_role = const.ROLE_HALT
        self.robot_2_role = const.ROLE_HALT
        self.robot_3_role = const.ROLE_HALT

    def subGameState(self, game_state):
        self.game_state = game_state.data

    def pubRobotRole(self):
        role_1 = Int32(self.robot_1_role)

        self.pub_role_1.publish(role_1)
