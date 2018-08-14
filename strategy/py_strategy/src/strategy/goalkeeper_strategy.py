import rospy
import math
from src.robot import SoccerRobot
from src.fsm.goalkeeper_fsm import GoalkeeperFSM

__all__ = ['Goalkeeper']

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180

class Goalkeeper(SoccerRobot):

    def __init__(self):
        super().__init__()
        self._fsm = GoalkeeperFSM()
        self.attachFSM(0, self._fsm.init_state)

    def imu_transfer(self, x, y, imu):
        x_ = x * math.cos(imu) - y * math.sin(imu)
        y_ = x * math.sin(imu) + y * math.cos(imu)
        return x_, y_

    def block(self):
        our_door = self.interface.robot_info.op_goal
        ball = self.interface.robot_info.ball
        imu = self.interface.imu

        near_door = our_door.left_dis if our_door.left_dis < our_door.right_dis else our_door.right_dis
        
        x = ball.dis * math.sin(ball.ang * DEG2RAD)
        y = 0
 
        self.interface.robot_info.cmd_vel.x, self.interface.robot_info.cmd_vel.y = self.imu_transfer(x, y, imu)

        absolute_ball_ang = ball.ang + imu*RAD2DEG

        if near_door < 0.6 and abs(absolute_ball_ang) > 5:
            self.interface.robot_info.cmd_vel.x = 0
            self.interface.robot_info.cmd_vel.y = 0
        
        self.interface.robot_info.cmd_vel.yaw = ball.ang
    
    def push(self):
        ball = self.interface.robot_info.ball
        self.interface.robot_info.cmd_vel.x = ball.dis * math.sin(ball.ang * DEG2RAD) * 1000  # rewrite here in feature
        self.interface.robot_info.cmd_vel.y = ball.dis * math.cos(ball.ang * DEG2RAD) * 1000  # rewrite here in feature
        # velocity * 1000 let robot using max velocity to push
        self.interface.robot_info.cmd_vel.yaw = ball.ang
    
    def stay(self):
        r""" Let robot stay in the center of our door and keep its front tracing the ball at the same time.
        """
        our_door = self.interface.robot_info.op_goal
        door_middle_diff = our_door.right_dis - our_door.left_dis
        imu = self.interface.imu

        if abs(door_middle_diff) <= 0.1:
             door_middle_diff = 0
        print(door_middle_diff)
        x = door_middle_diff
        y = 0

        self.interface.robot_info.cmd_vel.x, self.interface.robot_info.cmd_vel.y = self.imu_transfer(x, y, imu)

        # print(self.interface.robot_info.ball.ang)
        self.interface.robot_info.cmd_vel.yaw = self.interface.robot_info.ball.ang
    
    def blockFlyBall(self):
        pass
        # self.interface.robot_info.cmd_vel.x = self.interface.robot_info.ball.dis * math.sin(self.interface.robot_info.ball.ang * DEG2RAD)
        # self.interface.robot_info.cmd_vel.y = self.interface.robot_info.ball.dis * math.cos(self.interface.robot_info.ball.ang * DEG2RAD)
        # self.interface.robot_info.cmd_vel.yaw = self.interface.robot_info.ball.ang

    def returnDoor(self):

        ball = self.interface.robot_info.ball
        imu = self.interface.imu
        our_door = self.interface.robot_info.op_goal

        x = our_door.dis * math.sin(our_door.ang * DEG2RAD)
        y = our_door.dis * math.cos(our_door.ang * DEG2RAD)

        self.interface.robot_info.cmd_vel.x, self.interface.robot_info.cmd_vel.y = self.imu_transfer(x, y, imu)

        self.interface.robot_info.cmd_vel.yaw = self.interface.robot_info.ball.ang