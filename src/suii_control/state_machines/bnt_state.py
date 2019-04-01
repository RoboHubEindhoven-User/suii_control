#!/usr/bin/env python

'''bmt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

class BasicNavigationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(BasicNavigationState, self).__init__(fsm)
        
    def enter(self):
        super(BasicNavigationState, self).enter()
    
    def execute(self):
        super(BasicNavigationState, self).execute()
        self.fsm.transition("to_bmt")

    def exit(self):
        super(BasicNavigationState, self).exit()