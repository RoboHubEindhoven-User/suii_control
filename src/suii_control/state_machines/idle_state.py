#!/usr/bin/env python

'''idle_state.py: An example state that inherit, implement and override the base state class functions 
'''

__author__      = "Zinkeng Thierry"

from suii_control.state_machines import State

class IdleState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(IdleState, self).__init__(fsm)
        
    def enter(self):
        super(IdleState, self).enter()
    
    def execute(self):
        super(IdleState, self).execute()
        self.fsm.transition("to_state_name")

    def exit(self):
        super(IdleState, self).exit()