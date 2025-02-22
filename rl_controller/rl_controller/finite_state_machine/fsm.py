from enum import Enum
from dataclasses import dataclass

@dataclass
class BuiltInEvent:
    DEFAULT_SIG = 1
    ENTRY_SIG = 2
    EXIT_SIG = 3
    USER_SIG = 4

@dataclass
class Status:
    TRAN_STATUS = 1
    HANDLED_STATUS = 2
    IGNORED_STATUS = 3
    INIT_STATUS = 4


"""
    while true:
        if event_queue
            event = event_queue.pop(0)
            state_machine.dispatch(event)
        
    
    def dispatch(self, event):
        prev_state = self.state
        status = self.state(event)

        if status is State.TRAN_STATUS:
            prev_state(self, BuiltInEvent.EXIT_SIG)
            self.state(BuiltInEvent.ENTRY_SIG)
    
    def initial_state(self, event):
        
        if event is BuiltInEvent.ENTRY_SIG:
            return NotImplemented
        elif event is BuiltInEvent.EXIT_SIG:
            return NotImplemented
        elif event is BuiltInEvent.INIT_SIG:

"""

class FSM(object):
    def __init__(self):
        self.state = self.initial_state

    def dispatch(self, event: BuiltInEvent) -> None:
        prev_state = self.state
        status = self.state(event)

        if status is Status.TRAN_STATUS:
            prev_state(BuiltInEvent.EXIT_SIG)
            self.state(BuiltInEvent.ENTRY_SIG)

    def transition_to(self, state) -> None:
        self.state = state

    def initial_state(self, event) -> Status:
        return NotImplemented