r""" robot.py

* Author        |   Chu, Chen-You
* Date          |   2018/08/23
* Description   |   The soccer robot class 
* Contact       |   acgeusmile@gmail.com

"""
import rospy
import math
from src.data_structure import const
from src.fsm.base_fsm import BaseFSM
from .robot_port_manager import *

__all__ = ['SoccerRobot']


class Robot(object):
    r""" Robot abstract class 

    """
    def __init__(self):
        pass

    def run(self):
        raise NotImplementedError('Subclass must implement abstract method')

class SoccerRobot(Robot):
    r""" The soccer robot class 
    function:
        run():
            execute the soccer robot procedure
        _initArgs():
            initialize all attrubute args
        attachFSM():
            attach finite-state machine to soccer robot
        _behaviorDecision():
            execute the finite-state machine
        _changeState():
            execute the procedure to change state
        _keepState():
            execute the procedure to keep state
        _velocityNormalization():
            velocity planning by using s function
        _pubRobotControlCommand():
            publish robot control command      
    args:
        state:
            the state of finite-state machine
        interface:
            all robot's information
        
    """
    def __init__(self, **kwarg):
        super(SoccerRobot, self).__init__()
        self._initArgs()

    def run(self):
        # self.interface._param("/FIRA_Behavior/Goalkeeper", 1)
        self._behaviorDecision()
        self._pubRobotControlCommand()

    def attachFSM(self, state, fsm):
        self.fsm = fsm
        self.curr_state = state
    
    def _initArgs(self):
        self.state = 0
        self.interface = RobotPortManager()

    def _behaviorDecision(self):
        self._fsm.frame(self, self.state)

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
            alpha = math.atan2(self.interface.robot_info.cmd_vel.y, self.interface.robot_info.cmd_vel.x) * const.RAD2DEG
        else:
            alpha = 0
        dis_max = self.interface.robot_param.velocity_param[0]
        dis_min = self.interface.robot_param.velocity_param[1]
        velocity_max = self.interface.robot_param.velocity_param[2]
        velocity_min = self.interface.robot_param.velocity_param[3]
        angular_velocity_max = self.interface.robot_param.velocity_param[4]
        angular_velocity_min = self.interface.robot_param.velocity_param[5]
        angle_max = self.interface.robot_param.velocity_param[6]
        angle_min = self.interface.robot_param.velocity_param[7]
        angle_out = angle
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
        self.interface.robot_info.cmd_vel.x = velocity * math.cos(alpha*const.DEG2RAD)
        self.interface.robot_info.cmd_vel.y = velocity * math.sin(alpha*const.DEG2RAD)
        self.interface.robot_info.cmd_vel.yaw = angle_out

    def _pubRobotControlCommand(self):
        self._velocityNormalization()
        self.interface.updateRobotCmd()