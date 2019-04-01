#!/usr/bin/env python

'''rtt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from sui_control.state_machines import State

class RoundTableState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(RoundTableState, self).__init__(fsm)
        
    def enter(self):
        super(RoundTableState, self).enter()
    
    def execute(self):
        super(RoundTableState, self).execute()
        self.fsm.transition("to_bnt")

    def exit(self):
        super(RoundTableState, self).exit()