from .base_fsm import *

__all__ = ['GoalkeeperFSM', 'HaltFSM']

BLOCK           =   1
PUSH            =   2
STAY            =   3
FLY_BALL        =   4
RETURN_DOOR     =   5


class GoalkeeperFSM(BaseFSM):

    def __init__(self):
        super().__init__()
        self._fsms.append(InitState())
        self._fsms.append(BlockState())
        self._fsms.append(PushState())
        self._fsms.append(StayState())

class InitState(AbstractFSM):
    def enterState(self, obj):
        print ('enter init!!')
    
    def execState(self, obj):
        print('initialize')

    def exitState(self, obj):
        print ('exit init!!')

    def transferState(self, obj):
        obj.state = STAY


class BlockState(AbstractFSM):
    def enterState(self, obj):
        print ('enter block!!')
    
    def execState(self, obj):
        obj.block()

    def exitState(self, obj):
        print ('exit block!!')

    def transferState(self, obj):
        our_door = obj.interface.robot_info.op_goal
        near_door = our_door.left_dis if our_door.left_dis < our_door.right_dis else our_door.right_dis
        ball = obj.interface.robot_info.ball

        dis_block_to_stay = obj.interface.robot_param.goalkeeper[5]
        dis_block_to_push = obj.interface.robot_param.goalkeeper[6]


        if ball.dis > dis_block_to_stay:
            obj.state = STAY
        elif ball.dis < dis_block_to_push:
            obj.state = PUSH
        else:
            obj.state = BLOCK

class PushState(AbstractFSM):
    def enterState(self, obj):
        print('enter push')
    
    def execState(self, obj):
        obj.push()

    def exitState(self, obj):
        print('exit push')

    def transferState(self, obj):
        our_door = obj.interface.robot_info.op_goal
        ball = obj.interface.robot_info.ball

        dis_our_door_ball = our_door.dis + ball.dis
        dis_push_to_return = obj.interface.robot_param.goalkeeper[7]
        
        if dis_our_door_ball > dis_push_to_return:
            obj.state = RETURN_DOOR
        else:
            obj.state = PUSH

class StayState(AbstractFSM):
    def enterState(self, obj):
        print('enter stay')
    
    def execState(self, obj):
        obj.stay()

    def exitState(self, obj):
        print('exit stay')

    def transferState(self, obj):
        ball = obj.interface.robot_info.ball
        dis_block_to_stay = obj.interface.robot_param.goalkeeper[5]

        if ball.dis < dis_block_to_stay:
            obj.state = BLOCK
        else:
            obj.state = STAY

class ReturnDoor(AbstractFSM):
    def enterState(self, obj):
        print('enter return_door')
    
    def execState(self, obj):
        obj.stay()

    def exitState(self, obj):
        print('exit return_door')

    def transferState(self, obj):
        ball = obj.interface.robot_info.ball
        our_door = obj.interface.robot_info.op_goal
        dis_return_to_stay = obj.interface.robot_param.goalkeeper[8]
        
        if our_door.dis < 1:
            obj.state = STAY
        else:
            obj.state = RETURN_DOOR
