import rospy
from std_msgs.msg import Int32
from src.strategy.goalkeeper_strategy import *
from src.strategy.halt_strategy import *
from stc.strategy.run_dest import *
from src.data_structure import const

# role define
NO_EXISTS = -1

class RobotGenerator(object):

    def __init__(self):
        self.robot_list = [NO_EXISTS, NO_EXISTS, NO_EXISTS]
        # self.update_role = [False, False, False]
        self.update_role = False
        rospy.Subscriber('FIRA/R1/Strategy/Coach/role', Int32, self.subRole1st)
        rospy.Subscriber('FIRA/R2/Strategy/Coach/role', Int32, self.subRole2nd)
        rospy.Subscriber('FIRA/R3/Strategy/Coach/role', Int32, self.subRole3rd)

    def selectRobot(self):
        robot_list = []
        for robot in self.robot_list:
            if robot == NO_EXISTS:
                pass
            elif robot == const.ROLE_HALT:
                robot_list.append(Halt())
            elif robot == const.ROLE_GOALKEEPER:
                robot_list.append(Goalkeeper())       
            
            ## special purpose
            elif robot == const.ROLE_RUN_LOCATION:
                pass

            else:
                raise ValueError('Unexpected role!!')

        return robot_list
    
    def subRole1st(self, role):
        if self.robot_list[0] == role.data:
            pass
        else:
            self.robot_list[0] = role.data
            # self.update_role[0] = True
            self.update_role = True

    def subRole2nd(self, role):
        if self.robot_list[1] == role.data:
            pass
        else:
            self.robot_list[1] = role.data
            # self.update_role[1] = True
            self.update_role = True

    def subRole3rd(self, role):
        if self.robot_list[2] == role.data:
            pass
        else:
            self.robot_list[2] = role.data
            # self.update_role[2] = True
            self.update_role = True
