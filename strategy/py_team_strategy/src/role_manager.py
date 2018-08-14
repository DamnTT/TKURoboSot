import rospy
import math
from src.coach_command_manager import *
from src.data_structure import const

__all__ = ['RoleManager']

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180

class RoleManager(object):

    def __init__(self):
        
        self.coach_command = CoachCmdManager()

    def teamStrategy(self):
        
        if self.coach_command.game_state == const.GAMESTATE_HALT:
            self.coach_command.robot_1_role = const.ROLE_HALT
            self.coach_command.robot_2_role = const.ROLE_HALT
            self.coach_command.robot_3_role = const.ROLE_HALT
        elif self.coach_command.game_state == const.GAMESTATE_KICK_OFF:
            self.coach_command.robot_1_role = const.ROLE_GOALKEEPER
            self.coach_command.robot_2_role = const.ROLE_HALT
            self.coach_command.robot_3_role = const.ROLE_HALT
        
        ## special purpose
        
        elif self.coach_command.game_state == const.GAMESTATE_RUN_LOCATION:
            self.coach_command.robot_1_role = const.ROLE_RUN_LOCATION
            self.coach_command.robot_2_role = const.ROLE_RUN_LOCATION
            self.coach_command.robot_3_role = const.ROLE_RUN_LOCATION

    def pubRole(self):
        self.coach_command.pubRobotRole()