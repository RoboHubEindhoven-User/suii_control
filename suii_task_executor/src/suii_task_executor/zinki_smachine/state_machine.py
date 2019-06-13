#!/usr/bin/env python

'''finite_state_machine.py: 
    - Contatins a Transition class used to transition from one state to the other.
    - Contains a Finmte state machine for setting the states, transition and executing the states actions
'''

__author__      = "Zinkeng Thierry"

import os, sys
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath('states'))))
from zinki_smachine import State

class Transition(object):
    ''' 
    The Transition class is used to transition between states
    '''

    def __init__(self, to_state):
        ''' Initialize the components. '''
        self.to_state = to_state

    def execute(self):
        ''' Prints a string as the action of the transition class'''
        print (">>> Transitioning to '%s' <<<" % (str(self.to_state)))

## ==================================================================================================

class StateMachine(object):
    ''' 
    The FSM has a computation mechanism with states and transition.
    it moves from one state to the other using transitions.
    '''

    def __init__(self, name=''):
        ''' Initialize the components. '''
        self.name        = name
        self.states      = {}
        self.transitions = {}
        self.curr_state  = None
        self.prev_state  = None
        self.trans       = None
    
    def add_state(self, state_name, state):
        ''' Adds a new state to the list of states
        Args:
            state_name (str): The name of the state passed in param 2
            state: The state instance/object
        '''
        self.states[state_name] = state

    def add_transition(self, trans_name, transition):
        ''' Adds a new transition to the list of transitions
        Args:
            trans_name (str): The name of the transition passed in param 2
            transition: The transition instance/object
        '''
        self.transitions[trans_name] = transition

    def set_state(self, state_name):
        ''' Change current state base on the state name
        Args:
            state_name (str): The name of the state to change to
        '''
        self.prev_state = self.curr_state
        self.curr_state = self.states[state_name]

    def transition(self, trans_name):
        ''' Change current transition base on the transition name 
        Args:
            trans_name (str): The name of the transition to change to
        '''
        self.trans = self.transitions[trans_name]

    def execute(self):
        ''' Changes states and transition. This function is to be called in a loop wherever the state machine is being used'''
        if(self.trans):
            self.curr_state.exit()
            self.trans.execute()
            self.set_state(self.trans.to_state)
            self.curr_state.enter()
            self.trans = None
        self.curr_state.execute()

    def __repr__(self):
        return self.name