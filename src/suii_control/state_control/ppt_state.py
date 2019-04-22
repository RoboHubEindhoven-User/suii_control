#!/usr/bin/env python

'''ppt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

from enum import Enum

class PPT_StateName(Enum):
    TASK = "task_state"
    MOVE = "move_state"
    WAIT = "wait_state"
    EXIT = "exit_state"
    EMRG = "emergency_state" 

class PPT_TransTo(Enum):
    TASK = "to_task"
    MOVE = "to_move"
    WAIT = "to_wait"
    EXIT = "to_exit"
    EMRG = "to_emergency" 

class PrecisionPlacementState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(PrecisionPlacementState, self).__init__(fsm)
        
    def enter(self):
        super(PrecisionPlacementState, self).enter()
    
    def execute(self):
        super(PrecisionPlacementState, self).execute()
        self.fsm.transition(Trans_To.TSL)

    def exit(self):
        super(PrecisionPlacementState, self).exit()