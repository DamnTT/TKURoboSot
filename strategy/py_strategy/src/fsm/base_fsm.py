r""" base_fsm.py

* Author        |   Chu, Chen-You
* Date          |   2018/08/23
* Description   |   Base finite state machine class
* Contact       |   acgeusmile@gmail.com

"""
__all__ = ['BaseFSM', 'AbstractFSM']

class BaseFSM(object):
    r""" Base finite state machine class
    function:
        getFsm():
            get finite state machine
        frame():
            execute finite state machine
        addState(state):
            add state in finite state machine
        showState():
            show the state on the screen
    agrs:
        _fsms:
            finite state machine
    """
    def __init__(self):
        self._fsms = []
    
    def getFsm(self, state):
        return self._fsms[state]
    
    def frame(self, obj, state):
        if state == obj.curr_state:
            obj._keepState()
        else:
            obj._changeState(state, self._fsms[state])
    
    def addState(self, state):
        self._fsms.append(state)
    
    def showState(self):
        for state in self._fsms:
            print(state.__class__.__name__)

    @property
    def init_state(self):
        return self._fsms[0]
        
class AbstractFSM(object):
    r""" Abstract finite state machine class
    function:
        enterState(obj):
            The action while enter in this state
        execState(obj):
            The action while execute in this state
        exitState(obj):
            The action while exit in this state
        transferState(obj):
            The condition decide change to other state or not
    """
    def enterState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')
    
    def execState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')

    def exitState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')

    def transferState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')