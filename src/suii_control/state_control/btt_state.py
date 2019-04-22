#!/usr/bin/env python

'''btt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State
from suii_control.state_machines.transition_name import Trans_To

from enum import Enum

class BTT_StateName(Enum):
    TASK = "task_state"
    MOVE = "move_state"
    WAIT = "wait_state"
    EXIT = "exit_state"
    EMRG = "emergency_state" 

class BTT_TransTo(Enum):
    TASK = "to_task"
    MOVE = "to_move"
    WAIT = "to_wait"
    EXIT = "to_exit"
    EMRG = "to_emergency" 

class BasicTransportationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(BasicTransportationState, self).__init__(fsm)
        
    def enter(self):
        super(BasicTransportationState, self).enter()
    
    def execute(self):
        super(BasicTransportationState, self).execute()
        self.fsm.transition(Trans_To.TSL)

    def exit(self):
        super(BasicTransportationState, self).exit()