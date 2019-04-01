#!/usr/bin/env python

'''bmt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

class BasicManipulationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(BasicManipulationState, self).__init__(fsm)
        
    def enter(self):
        super(BasicManipulationState, self).enter()
    
    def execute(self):
        super(BasicManipulationState, self).execute()
        self.fsm.transition("to_btt")

    def exit(self):
        super(BasicManipulationState, self).exit()