__all__ = ['BaseFSM', 'AbstractFSM']

class BaseFSM(object):
    def __init__(self):
        self._fsms = []
    
    def getFsm(self, state):
        return self._fsms[state]

    @property
    def init_state(self):
        return self._fsms[0]
    
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

class AbstractFSM(object):
    def enterState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')
    
    def execState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')

    def exitState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')

    def transferState(self, obj):
        raise NotImplementedError('Subclass must implement abstract method')