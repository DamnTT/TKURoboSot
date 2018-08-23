r""" Structure of goal keeper strategy

"""
partition = '\n----------\n'

__all__ = ['RobotInfo', 'Param']


class Motion2D(object):
    r""" motion state
    attr:
        x: velocity of x axis
        y: velocity of y axis
        yaw: 
    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
    def __repr__(self):
        return 'x: ' + str(self.x) + partition + \
        'y: ' + str(self.y) + partition + \
        'yaw: ' + str(self.yaw) + partition

class Object(object):
    r""" object info structure
    basic attr:
        dis: distance of object
        ang: angle of object
    """
    def __init__(self):
        self.dis = 0
        self.ang = 0
    def __str__(self):
        return 'dis: ' + str(self.dis) + partition + \
        'ang: ' + str(self.ang) + partition

class Door(Object):
    r""" two point structure
    """
    def __init__(self):
        super(Door, self).__init__()
        self.ang_max = 0
        self.ang_min = 0
        self.left_dis = 0
        self.right_dis = 0
    def __str__(self):
        return 'ang_max: ' + str(self.ang_max) + partition + \
        'ang_min: ' + str(self.ang_min) + partition + \
        'left_dis: ' + str(self.left_dis) + partition + \
        'right_dis: ' + str(self.right_dis) + partition + \
        'dis: ' + str(self.dis) + partition + \
        'ang: ' + str(self.ang) + partition

class Vision(object):

    r""" Vision info class
    attr:
        pos:     position of robot
        ball:    information of ball
        goal:    information of goal
        op_goal: information of opposite goal
    """
    def __init__(self):
        self.pos = Object()
        self.ball = Object()
        self.goal = Door()
        self.op_goal = Door()

    def __str__(self):
        return 'pos:'+ partition + str(self.pos) + partition + \
        'ball:' + partition + str(self.ball) + partition +\
        'goal:' + partition + str(self.goal) + partition +\
        'op_goal:' + partition + str(self.op_goal) + partition
    

class RobotInfo(Vision):
    r""" robot info structure
    attr:
        cmd_vel: velocity command of robot
    """
    def __init__(self):
        super().__init__()
        self.cmd_vel = Motion2D()
        self.cmd_shoot = 0
        self.role = 0
    def __str__(self):
        return 'role:' + partition + str(self.role) + partition + \
        'cmd_vel:' + partition + str(self.cmd_vel) + partition + \
        'pos:'+ partition + str(self.pos) + partition + \
        'ball:' + partition + str(self.ball) + partition +\
        'goal:' + partition + str(self.goal) + partition +\
        'op_goal:' + partition + str(self.op_goal) + partition

class Behavior(object):

    def __init__(self):
        self.goalkeeper = []


class Param(object):

    def __init__(self):
        self.behavior = Behavior()
        self.velocity_param = []
        self.robot_number = 0
