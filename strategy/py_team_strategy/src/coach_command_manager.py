import rospy
import os

from tkinter import *
from tkinter.ttk import *

from src.data_structure import data_structure
from src.data_structure import const
# import message 
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from vision.msg import Object
from vision.msg import Two_point
from geometry_msgs.msg import Twist

__all__ = ['CoachCmdManager']

class CoachCmdManager(object):
    """ The port manager of robot 
        
    """
    def __init__(self):
        super(CoachCmdManager, self).__init__()
        self.initAttr()
        rospy.Subscriber('FIRA/GameState', Int32, self.subGameState)
        self.pub_role_1 = rospy.Publisher('FIRA/R1/Strategy/Coach/role', Int32, queue_size=10)
        self.pub_role_2 = rospy.Publisher('FIRA/R2/Strategy/Coach/role', Int32, queue_size=10)
        self.pub_role_3 = rospy.Publisher('FIRA/R3/Strategy/Coach/role', Int32, queue_size=10)

    def initAttr(self): 
        self.userInterface()
        self.game_state = const.GAMESTATE_HALT
        self.robot_1_role = const.ROLE_HALT
        self.robot_2_role = const.ROLE_HALT
        self.robot_3_role = const.ROLE_HALT

    def subGameState(self, game_state):
        self.game_state = game_state.data

    def pubRobotRole(self):
        role_1 = Int32(self.robot_1_role)
        role_2 = Int32(self.robot_2_role)
        role_3 = Int32(self.robot_3_role)

        if self.number == 1:
            if self.select == 1:
                self.pub_role_1.publish(role_1)
            elif self.select == 2:
                self.pub_role_2.publish(role_2)
            elif self.select == 3:
                self.pub_role_3.publish(role_3)
            else:
                raise ValueError('you must select case 1 ~ 3')
                
        elif self.number == 2:
            if self.select == 1:
                self.pub_role_1.publish(role_1)
                self.pub_role_2.publish(role_2)
            elif self.select == 2:
                self.pub_role_2.publish(role_2)
                self.pub_role_3.publish(role_3)
            elif self.select == 3:
                self.pub_role_1.publish(role_1)
                self.pub_role_3.publish(role_3)
            else:
                raise ValueError('you must select case 1 ~ 3')
        elif self.number == 3:
            self.pub_role_1.publish(role_1)
            self.pub_role_2.publish(role_2)
            self.pub_role_3.publish(role_3)
        else:
            raise ValueError('you must choose 1 to 3 robot')

    def userInterface(self):
        r"""  GUI for fun haha ~~~

        """

        def case_1():
            self.select = 1
            close_win()
        def case_2():
            self.select = 2
            close_win()
        def case_3():
            self.select = 3
            close_win()
        def single_robot():
            self.number = 1

            destoryObject(robot_number_button_1)
            destoryObject(robot_number_button_2)
            destoryObject(robot_number_button_3)
            destoryObject(robot_number_label)

            select_button_1_1.grid(column=0,row=1)
            select_button_1_2.grid(column=1,row=1) 
            select_button_1_3.grid(column=2,row=1)

            select_case_label.grid(column=1,row=0)
            
        def double_robot():
            self.number = 2

            destoryObject(robot_number_button_1)
            destoryObject(robot_number_button_2)
            destoryObject(robot_number_button_3)
            destoryObject(robot_number_label)

            select_button_2_1.grid(column=0,row=1)
            select_button_2_2.grid(column=1,row=1) 
            select_button_2_3.grid(column=2,row=1)

            select_case_label.grid(column=1,row=0)

        def triple_robot():
            self.number = 3

        def destoryObject(obj):
            obj.destroy()

        def close_win():
            win.quit()
            win.destroy()

        win = Tk()
        win.title("Team strategy GUI")
        win.resizable(0,0)

        robot_number_label = Label(win, text="choose the number of robot you want to control:")
        select_case_label = Label(win, text="choose which case you want:")

        robot_number_button_1 = Button(win, text="1", command=single_robot)   
        robot_number_button_2 = Button(win, text="2", command=double_robot)   
        robot_number_button_3 = Button(win, text="3", command=triple_robot)
        close_button = Button(win, text="close windows", command=close_win)
        
        select_button_1_1 = Button(win, text="robot 1", command=case_1) 
        select_button_1_2 = Button(win, text="robot 2", command=case_2) 
        select_button_1_3 = Button(win, text="robot 3", command=case_3) 

        select_button_2_1 = Button(win, text="robot 1 & robot3", command=case_1) 
        select_button_2_2 = Button(win, text="robot 2 & robot3", command=case_2) 
        select_button_2_3 = Button(win, text="robot 1 & robot3", command=case_3) 

        robot_number_label.grid(column=1,row=0)
        robot_number_button_1.grid(column=0,row=1)
        robot_number_button_2.grid(column=1,row=1) 
        robot_number_button_3.grid(column=2,row=1)
        close_button.grid(column=1,row=2)

        win.mainloop()