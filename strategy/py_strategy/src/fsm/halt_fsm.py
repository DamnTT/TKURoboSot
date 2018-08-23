r""" halt_fsm.py

* Author        |   Chu, Chen-You
* Date          |   2018/08/23
* Description   |   Halt finite state machine class
* Contact       |   acgeusmile@gmail.com

"""

from .base_fsm import *
from src.data_structure import const

__all__ = ['HaltFSM']

class HaltFSM(BaseFSM):
    r""" Halt finite state machine class
    
    """
    def __init__(self):
        super().__init__()
        self._fsms.append(InitState())
        self._fsms.append(HaltState())
        self._fsms.append(BrakeState())

class InitState(AbstractFSM):
    r""" Initial state class in role halt
    
    """
    def enterState(self, obj):
        print ('enter InitState!!')
    
    def execState(self, obj):
        print('initialize')

    def exitState(self, obj):
        print ('exit InitState!!')

    def transferState(self, obj):
        obj.state = const.FSM_HALT

class HaltState(AbstractFSM):
    r""" Halt state class in role halt
    
    """
    def enterState(self, obj):
        print ('enter HaltState!!')
    
    def execState(self, obj):
        obj.halt()

    def exitState(self, obj):
        print ('exit HaltState!!')
    
    def transferState(self, obj):
        pass

class BrakeState(AbstractFSM):
    r""" Brake state class in role halt
    
    """
    def enterState(self, obj):
        print ('enter BrakeState!!')
    
    def execState(self, obj):
        obj.brake()

    def exitState(self, obj):
        print ('exit BrakeState!!')

    def transferState(self, obj):
        pass