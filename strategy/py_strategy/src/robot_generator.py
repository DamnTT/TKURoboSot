r""" robot_generator.py

* Author        |   Chu, Chen-You
* Date          |   2018/08/23
* Description   |   Generate the robot
* Contact       |   acgeusmile@gmail.com

"""
import rospy
from std_msgs.msg import Int32
from src.strategy.goalkeeper_strategy import *
from src.strategy.halt_strategy import *
from src.strategy.special import *
from src.data_structure import const


class RobotGenerator(object):
    r""" Generate the robot
    function:
        generateRobot():
            generate the robot's role list
        _rosSubscriber():
            ros subscriber decalaration
        _rosPublisher():
            ros publisher decalaration
        _initAttr():
            initialize all attrubute args
    args:
        robot_list:
            robot's role list
        update_role:
            list to check the robot_list change or not
    """
    def __init__(self):
        self._initAttr()
        self._rosPublisher()
        self._rosSubscriber()

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
            robot_list = []
            for robot in self.robot_list:
                if robot == const.NO_EXISTS:
                    robot_list.append(None)
        return robot_list

    def _rosSubscriber(self):
        rospy.Subscriber('FIRA/R1/Strategy/Coach/role', Int32, self._subRole1st)
        rospy.Subscriber('FIRA/R2/Strategy/Coach/role', Int32, self._subRole2nd)
        rospy.Subscriber('FIRA/R3/Strategy/Coach/role', Int32, self._subRole3rd)

    def _rosPublisher(self):
        pass
    
    def _initAttr(self):
        self.robot_list = [const.NO_EXISTS, const.NO_EXISTS, const.NO_EXISTS]
        self.update_role = [False, False, False]
    
    def _subRole1st(self, role):
        if self.robot_list[0] == role.data:
            pass
        else:
            self.robot_list[0] = role.data
            self.update_role[0] = True

    def _subRole2nd(self, role):
        if self.robot_list[1] == role.data:
            pass
        else:
            self.robot_list[1] = role.data
            self.update_role[1] = True

    def _subRole3rd(self, role):
        if self.robot_list[2] == role.data:
            pass
        else:
            self.robot_list[2] = role.data
            self.update_role[2] = True