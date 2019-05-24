#!/usr/bin/env python

'''state_machine_test.py:  
'''
# from sui_control.src.state_machine import *

__author__      = "Zinkeng Thierry"


import unittest
import rospy
from suii_control.state_machines import *

class StateMachineTest(unittest.TestCase):

    def test_add_state(self):
        print "Testing method: ", self._testMethodName
        fsm = StateMachine(self)
        bnt_state = BasicNavigationState(fsm)
        fsm.add_state("basic_navigation_test",    bnt_state)
        self.assertEqual(len(fsm.states), 1)

    def test_add_transition(self):
        print "Testing method: ", self._testMethodName
        fsm = StateMachine(self)
        fsm.add_transition("to_bnt", Transition("basic_navigation_test"))
        self.assertEqual(len(fsm.transitions), 1)

    def test_set_state(self):
        print "Testing method: ", self._testMethodName
        fsm = StateMachine(self)
        bnt_state = BasicNavigationState(fsm)
        bmt_state = BasicManipulationState(fsm)
        fsm.add_state("basic_navigation_test",    bnt_state)
        fsm.add_state("basic_manipulation_test",  bmt_state)
        fsm.set_state("basic_navigation_test")
        self.assertEqual(fsm.curr_state, bnt_state)


    def test_execute(self):
        print "Testing method: ", self._testMethodName
        fsm = StateMachine(self)
        bnt_state = BasicNavigationState(fsm)
        bmt_state = BasicManipulationState(fsm)
        btt_state = BasicTransportationState(fsm)

        fsm.add_state("basic_navigation_test",    bnt_state)
        fsm.add_state("basic_manipulation_test",  bmt_state)
        fsm.add_state("basic_transportation_test",  btt_state)

        fsm.add_transition("to_bnt", Transition("basic_navigation_test"))
        fsm.add_transition("to_bmt", Transition("basic_manipulation_test"))
        fsm.add_transition("to_btt", Transition("basic_transportation_test"))

        fsm.set_state("basic_navigation_test")
        
        fsm.execute()
        fsm.execute()

        self.assertEqual(fsm.curr_state, bmt_state)

if __name__ == '__main__':
    # init ros node because of the rospy loginfo function used in the state class
    # rospy.init_node('state_machine_test_node', anonymous=True)
    unittest.main()