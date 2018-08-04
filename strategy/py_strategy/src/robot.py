import rospy
import math
from .robot_port_manager import *
from src.fsm.base_fsm import BaseFSM

__all__ = ['SoccerRobot']

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180

class Robot(object):

    def __init__(self):
        pass

    def run(self):
        raise NotImplementedError('Subclass must implement abstract method')

class SoccerRobot(Robot):
    """ main class of robot strategy
    member function:
        __init__:
            initialize goal_keeper node
        run:
            separate thread of control by calling start()
    """
    def __init__(self, **kwarg):
        super(SoccerRobot, self).__init__()
        self.state = 0
        self.behavior = None
        self.interface = RobotPortManager()

    def run(self):
        self.getParam()
        self.behaviorDecision()
        self.pubMotionCommand()

    def getParam(self):
        if self.interface.saveParam == True:
            self.interface.loadParam()
            self.interface.saveParam = False

    def behaviorDecision(self):
        self._fsm.frame(self, self.state)

    def attachFSM(self, state, fsm):
        self.fsm = fsm
        self.curr_state = state

    def _changeState(self, new_state, new_fsm):
        self.curr_state = new_state
        self.fsm.exitState(self)
        self.fsm = new_fsm
        self.fsm.enterState(self)
        self.fsm.execState(self)
        self.fsm.transferState(self)

    def _keepState(self):
        self.fsm.execState(self)
        self.fsm.transferState(self)

    def _velocityNormalization(self):
        
        angle = self.interface.robot_info.cmd_vel.yaw
        velocity = math.hypot(self.interface.robot_info.cmd_vel.x, self.interface.robot_info.cmd_vel.y)
        if self.interface.robot_info.cmd_vel.x != 0:
            alpha = math.atan2(self.interface.robot_info.cmd_vel.y, self.interface.robot_info.cmd_vel.x) * RAD2DEG
        else:
            alpha = 0

        dis_max = self.interface.velocity_param[0]
        dis_min = self.interface.velocity_param[1]
        velocity_max = self.interface.velocity_param[2]
        velocity_min = self.interface.velocity_param[3]
        angular_velocity_max = self.interface.velocity_param[4]
        angular_velocity_min = self.interface.velocity_param[5]
        angle_max = self.interface.velocity_param[6]
        angle_min = self.interface.velocity_param[7]

        angle_out = angle 
        
        print(angle)
        if velocity == 0:
            pass
        elif velocity > dis_max:
            velocity = velocity_max
        elif velocity < dis_min:
            velocity = velocity_min
        else:
            velocity = (velocity_max - velocity_min) * (math.cos((((velocity - dis_min) / (dis_max-dis_min) - 1) * math.pi)) + 1 )/ 2 + velocity_min
        if angle == 0:
            pass
        elif abs(angle) > angle_max:
            angle_out = angular_velocity_max
        elif abs(angle) < angle_min:
            angle_out = angular_velocity_min
        else:
            angle_out = (angular_velocity_max - angular_velocity_min) * (math.cos((((angle - angle_min) / (angle_max-angle_min) - 1) * math.pi)) + 1 )/ 2 + angular_velocity_min
      
        if angle < 0:
            angle_out = -angle_out
        print(angle_out)

        self.interface.robot_info.cmd_vel.x = velocity * math.cos(alpha*DEG2RAD)
        self.interface.robot_info.cmd_vel.y = velocity * math.sin(alpha*DEG2RAD)
        self.interface.robot_info.cmd_vel.yaw = angle_out


    def pubMotionCommand(self):
        self._velocityNormalization()
        self.interface.updateRobotCmd()
