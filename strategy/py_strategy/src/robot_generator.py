import rospy
from std_msgs.msg import Int32
from src.strategy.goalkeeper_strategy import *
from src.strategy.halt_strategy import *
from src.strategy.special import *
from src.data_structure import const

# role define
NO_EXISTS = -1

class RobotGenerator(object):

    def __init__(self):
        self.robot_list = [NO_EXISTS, NO_EXISTS, NO_EXISTS]
        self.update_role = [False, False, False]
        rospy.Subscriber('FIRA/R1/Strategy/Coach/role', Int32, self.subRole1st)
        rospy.Subscriber('FIRA/R2/Strategy/Coach/role', Int32, self.subRole2nd)
        rospy.Subscriber('FIRA/R3/Strategy/Coach/role', Int32, self.subRole3rd)

    def generateRobot(self, **kwargs):
        if 'robot_list' in kwargs:
            robot_list = kwargs['robot_list']
            if self.robot_list[kwargs['number']] == const.ROLE_HALT:
                robot_list[kwargs['number']] = Halt()
            elif self.robot_list[kwargs['number']] == const.ROLE_GOALKEEPER:
                robot_list[kwargs['number']] = Goalkeeper()
            
            ## special purpose
            elif self.robot_list[kwargs['number']] == const.ROLE_RUN_LOCATION:
                robot_list[kwargs['number']] = Special(const.ROLE_RUN_LOCATION)
        else:
            print('fuck you why')
            robot_list = []
            for robot in self.robot_list:
                if robot == NO_EXISTS:
                    robot_list.append(None)

        return robot_list
    
    def subRole1st(self, role):
        if self.robot_list[0] == role.data:
            pass
        else:
            self.robot_list[0] = role.data
            self.update_role[0] = True
            # self.update_role = True

    def subRole2nd(self, role):
        if self.robot_list[1] == role.data:
            pass
        else:
            self.robot_list[1] = role.data
            self.update_role[1] = True
            # self.update_role = True

    def subRole3rd(self, role):
        if self.robot_list[2] == role.data:
            pass
        else:
            self.robot_list[2] = role.data
            self.update_role[2] = True
            # self.update_role = True