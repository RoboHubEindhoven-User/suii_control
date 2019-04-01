#!/usr/bin/env python

'''btt_state.py: 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

class BasicTransportationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(BasicTransportationState, self).__init__(fsm)
        
    def enter(self):
        super(BasicTransportationState, self).enter()
    
    def execute(self):
        super(BasicTransportationState, self).execute()
        self.fsm.transition("to_ppt")

    def exit(self):
        super(BasicTransportationState, self).exit()