import rospy
import math
from src.coach_command_manager import *

__all__ = ['RoleManager']

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180

class RoleManager(object):

    def __init__(self):
        
        self.coach_command = CoachCmdManager()

    def teamStrategy(self):
        
        if self.coach_command.game_state == 0:
            self.coach_command.robot_1_role = 0
            self.coach_command.robot_2_role = 0
            self.coach_command.robot_3_role = 0
        elif self.coach_command.game_state == 1:
            self.coach_command.robot_1_role = 1
            self.coach_command.robot_2_role = 0
            self.coach_command.robot_3_role = 0

    def pubRole(self):
        self.coach_command.pubRobotRole()