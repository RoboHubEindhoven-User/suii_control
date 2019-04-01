#!/usr/bin/env python

'''ppt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from sui_control.state_machines import State

class PrecisionPlacementState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(PrecisionPlacementState, self).__init__(fsm)
        
    def enter(self):
        super(PrecisionPlacementState, self).enter()
    
    def execute(self):
        super(PrecisionPlacementState, self).execute()
        self.fsm.transition("to_rtt")

    def exit(self):
        super(PrecisionPlacementState, self).exit()