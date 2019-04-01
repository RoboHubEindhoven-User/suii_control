#!/usr/bin/env python

'''global_state_control_node.py:  This class is the brain of the sui robot. it implements the general state machine
    integrating all nodes, sub states and modules necessary to make the sui robot functional
'''

__author__      = "Zinkeng Thierry"

import rospy
from suii_control.state_machines import *

class GlobalStateControlNode:
    def __init__(self):
        rospy.init_node('sui_control_node', anonymous=True)
        rospy.loginfo("initializing SUI Control Node...")

        # rospy.on_shutdown(self.shutdown_hook())
        self.fsm = StateMachine(self);
        self.rate = rospy.Rate(10)

        # States
        self.bnt_state = BasicNavigationState(self.fsm)
        self.fsm.add_state("basic_navigation_test",    self.bnt_state)
        self.fsm.add_state("basic_manipulation_test",  BasicManipulationState(self.fsm))
        self.fsm.add_state("basic_transportation_test",BasicTransportationState(self.fsm))
        self.fsm.add_state("precision_placement_test", PrecisionPlacementState(self.fsm))
        self.fsm.add_state("rotating_table_test",      RoundTableState(self.fsm))

        # Transitions
        self.fsm.add_transition("to_bnt", Transition("basic_navigation_test"))
        self.fsm.add_transition("to_bmt", Transition("basic_manipulation_test"))
        self.fsm.add_transition("to_btt", Transition("basic_transportation_test"))
        self.fsm.add_transition("to_ppt", Transition("precision_placement_test"))
        self.fsm.add_transition("to_rtt", Transition("rotating_table_test"))
        
        # set default state
        self.fsm.set_state("basic_navigation_test")

    def shutdown_hook(self):
        print("Global State Control Node shutting down...")


    def main(self):
        while not rospy.is_shutdown():
            try:
                self.fsm.execute()
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
#==================================================================================================
