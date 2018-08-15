import rospy
import math
from src.robot import SoccerRobot
from src.fsm.special_fsm import SpecialFSM

__all__ = ['Special']

class Special(SoccerRobot):

    def __init__(self, init_state):
        super().__init__()
        self._fsm = SpecialFSM(init_state)
        self.attachFSM(0, self._fsm.init_state)

    def run_dest_point(self):
        pass