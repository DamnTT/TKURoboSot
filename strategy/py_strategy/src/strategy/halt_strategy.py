import rospy
from src.robot import SoccerRobot
from src.fsm.halt_fsm import HaltFSM

__all__ = ['Halt']


class Halt(SoccerRobot): 

    def __init__(self):
        super().__init__()
        self._fsm = HaltFSM()
        self.attachFSM(0, self._fsm.init_state)

    def halt(self):
        self.interface.robot_info.cmd_vel.x = 0
        self.interface.robot_info.cmd_vel.y = 0
        self.interface.robot_info.cmd_vel.yaw = 0

    def brake(self):
        self.interface.robot_info.cmd_vel.x = 0
        self.interface.robot_info.cmd_vel.y = 0
        self.interface.robot_info.cmd_vel.yaw = 0