
from .base_fsm import *

__all__ = ['HaltFSM']

# assume 
SPECIAL_ROLE_OFFSET = 500

class SpecialFSM(BaseFSM):

    def __init__(self, init_state):
        super().__init__()
        self._fsms.append(InitState(init_state))
        self._fsms.append(RunDestPointState())

class InitState(AbstractFSM):
    def __init__(self, init_state):
        self.init_state = init_state - SPECIAL_ROLE_OFFSET

    def enterState(self, obj):
        print ('enter InitState!!')
    
    def execState(self, obj):
        print('initialize')

    def exitState(self, obj):
        print ('exit InitState!!')

    def transferState(self, obj):
        obj.state = self.init_state

class RunDestPointState(AbstractFSM):
    def enterState(self, obj):
        print ('enter RunDestPointState!!')
    
    def execState(self, obj):
        obj.run_dest_point()

    def exitState(self, obj):
        print ('exit RunDestPointState!!')
    
    def transferState(self, obj):
        pass
