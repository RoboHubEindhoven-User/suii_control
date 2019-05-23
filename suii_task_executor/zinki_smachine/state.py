#!/usr/bin/env python

'''state.py: 
    - Contatins a Transition class used to transition from one state to the other.
    - Contains a Finmte state machine for setting the states, transition and executing the states actions
'''

__author__      = "Zinkeng Thierry"

import rospy

class State(object):
    ''' An Object which provides some utility functions for the individual states within the state machine. '''

    def __init__(self, fsm):
        self.fsm = fsm

    def enter(self):
        ''' Used to process pre-conditions of the state '''
        if __debug__:
            rospy.loginfo ('Processing current state: ' + str(self.__str__()) + '...') # for debugging
        pass

    def on_event(self, event):
        ''' Handle events that are delegated to this State.
        Args:
            event: the event to be handle 
        '''
        pass
    
    def execute(self):
        ''' Used to process the main task of the state '''
        if __debug__:
            rospy.loginfo ('Executing current state: ' + str(self.__str__()) + '...') # for debugging
        pass

    def exit(self):
        ''' Used to process post-conditions of the state '''
        if __debug__:
            rospy.loginfo ('--> Exiting current state: ' + str(self.__str__())) # for debugging
        pass

    def __repr__(self):
        ''' Leverages the __str__ method to describe the State. '''
        return self.__str__()

    def __str__(self):
        ''' Returns the name of the State.  '''
        return self.__class__.__name__