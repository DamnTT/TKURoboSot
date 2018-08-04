
from .base_fsm import *

__all__ = ['HaltFSM']

HALT = 1
BRAKE = 2

class HaltFSM(BaseFSM):

    def __init__(self):
        super().__init__()
        self._fsms.append(InitState())
        self._fsms.append(HaltState())
        self._fsms.append(BrakeState())

class InitState(AbstractFSM):
    def enterState(self, obj):
        print ('enter InitState!!')
    
    def execState(self, obj):
        print('initialize')

    def exitState(self, obj):
        print ('exit InitState!!')

    def transferState(self, obj):
        obj.state = HALT

class HaltState(AbstractFSM):
    def enterState(self, obj):
        print ('enter HaltState!!')
    
    def execState(self, obj):
        obj.halt()

    def exitState(self, obj):
        print ('exit HaltState!!')
    
    def transferState(self, obj):
        pass

class BrakeState(AbstractFSM):
    def enterState(self, obj):
        print ('enter BrakeState!!')
    
    def execState(self, obj):
        obj.brake()

    def exitState(self, obj):
        print ('exit BrakeState!!')

    def transferState(self, obj):
        pass