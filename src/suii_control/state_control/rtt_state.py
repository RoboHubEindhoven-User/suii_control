#!/usr/bin/env python

'''rtt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

from enum import Enum

class RTT_StateName(Enum):
    TASK = "task_state"
    MOVE = "move_state"
    WAIT = "wait_state"
    EXIT = "exit_state"
    EMRG = "emergency_state" 

class RTT_TransTo(Enum):
    TASK = "to_task"
    MOVE = "to_move"
    WAIT = "to_wait"
    EXIT = "to_exit"
    EMRG = "to_emergency" 

class RoundTableState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(RoundTableState, self).__init__(fsm)
        
    def enter(self):
        super(RoundTableState, self).enter()
    
    def execute(self):
        super(RoundTableState, self).execute()
        self.fsm.transition(Trans_To.TSL)

    def exit(self):
        super(RoundTableState, self).exit()