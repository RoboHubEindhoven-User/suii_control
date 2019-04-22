#!/usr/bin/env python

'''global_state_control_node.py:  This class is the brain of the sui robot. it implements the general state machine
    integrating all nodes, sub states and modules necessary to make the sui robot functional
'''

__author__      = "Zinkeng Thierry"

import rospy
from suii_control.state_machines import *
from suii_control.state_control.bmt_state import *
from suii_control.state_control.bnt_state import *
from suii_control.state_control.btt_state import *
from suii_control.state_control.ppt_state import *
from suii_control.state_control.rtt_state import *

from suii_control.state_machines.state_name import *
from suii_control.state_machines.transition_name import *
from suii_control.state_control.refbox_communication_node import *
from suii_control.state_control.benchmark import *
from suii_control.utils.file_handler import FileHandler
from yaml import dump

class TestSelectState(State):
    def __init__(self, fsm, refbox, navigation_tasks, transportation_tasks, navigation_tasks_config, test_completed):
        super(TestSelectState, self).__init__(fsm)
        self.refbox                  = refbox
        self.navigation_tasks        = navigation_tasks
        self.transportation_tasks    = transportation_tasks
        self.navigation_tasks_config = navigation_tasks_config
        self.test_completed          = test_completed
        
    def enter(self):
        super(TestSelectState, self).enter()
    
    def execute(self):
        super(TestSelectState, self).execute()

        
        (phase_type, state_type) = self.refbox.get_benchmark_state()
        # nav_tasks = self.refbox.get_navigation_tasks()


        rospy.loginfo("benchmark: " + str(phase_type) + " ," + str(self.refbox.get_test()))

        if((state_type == BenchmarkStateProp.RUNNING) and (phase_type == BenchmarkPhaseProp.PREPARATION)):
            while(self.test_completed):
                self.test_completed.pop(0)
                
        elif((state_type == BenchmarkStateProp.RUNNING) and (phase_type == BenchmarkPhaseProp.EXECUTION)):
            if (self.test_completed):
                if(self.test_completed[0] == "Completed"):
                    return

            while not self.refbox.get_navigation_tasks():
                continue

            print("Please work, u piece of shit " + str(len(self.refbox.get_navigation_tasks())))
            if(self.refbox.get_test() == StateName.BNT):
                self.clear_list(self.navigation_tasks)
                nav_list = self.refbox.get_navigation_tasks()

                for i in range(len(nav_list)):
                    self.navigation_tasks.append(nav_list[i])

                self.assign_nav_location_pose()

                self.fsm.transition(Trans_To.BNT)
            else: 
                self.clear_list(self.transportation_tasks)
                trans_list = self.refbox.get_transportation_tasks()

                for i in range(len(trans_list)):
                    self.transportation_tasks.append(trans_list[i])

                self.assign_trans_location_pose()
                           
                if(self.refbox.get_test() == StateName.BMT):
                    self.fsm.transition(Trans_To.BMT)
                elif(self.refbox.get_test() == StateName.BTT):
                    self.fsm.transition(Trans_To.BTT)
                elif(self.refbox.get_test() == StateName.PPT):
                    self.fsm.transition(Trans_To.PPT)
                elif(self.refbox.get_test() == StateName.RTT):
                    self.fsm.transition(Trans_To.RTT)
        else:
            rospy.loginfo("Waiting for Refbox Benchmark Command...")

    def exit(self):
        super(TestSelectState, self).exit()

    def clear_list(self, item_list):
        while(item_list):
            item_list.pop(0)
    
    def assign_nav_location_pose(self):
        file_handler = FileHandler()
        waypoints = self.navigation_tasks_config

        i = 0
        while i < len(self.navigation_tasks):
            for j in range(len(waypoints)):
                if ((self.navigation_tasks[i].destination.area_type == waypoints[j].destination.area_type) and 
                    self.navigation_tasks[i].destination.instance_id == waypoints[j].destination.instance_id):

                    self.navigation_tasks[i].destination.position    = waypoints[j].destination.position
                    self.navigation_tasks[i].destination.orientation = waypoints[j].destination.orientation
                    break
            i += 1

    def assign_trans_location_pose(self):
        print("assign service area pose")

import yaml
class GlobalStateControlNode:
    def __init__(self):
        rospy.init_node('sui_control_node', anonymous=True)
        rospy.loginfo("initializing SUI Control Node...")
        
        self.refbox = RefboxCommunication()

        # rospy.on_shutdown(self.shutdown_hook())
        self.fsm = StateMachine(self);
        self.rate = rospy.Rate(10)
        self.navigation_tasks = []
        self.transportation_tasks = []
        self.navigation_tasks_config = None
        self.test_completed = []
        # print rospy.get_param('suii_control/src/suii_control/config/navigation_waypoints.yaml')
        # check before accessing
        file_handler = FileHandler()
        if rospy.has_param('/suii_control_node/path'):
            file_name = rospy.get_param('/suii_control_node/path')
            self.navigation_tasks_config = file_handler.load(file_name)
            # print yaml.dump(self.navigation_tasks_config)
        else:
            print 'Could not find parameter!'

        while(self.test_completed):
            self.test_completed.pop(0)


        # States
        self.bnt_state = BasicNavigationState(self.fsm, self.navigation_tasks, self.test_completed)
        self.fsm.add_state(StateName.BNT, self.bnt_state)
        self.fsm.add_state(StateName.BMT, BasicManipulationState(self.fsm, self.refbox))
        self.fsm.add_state(StateName.BTT, BasicTransportationState(self.fsm))
        self.fsm.add_state(StateName.PPT, PrecisionPlacementState(self.fsm))
        self.fsm.add_state(StateName.RTT, RoundTableState(self.fsm))
        self.fsm.add_state(StateName.EMR, EmergencyStopState(self.fsm))
        self.fsm.add_state(StateName.TSL, TestSelectState(self.fsm, self.refbox, self.navigation_tasks, self.transportation_tasks, self.navigation_tasks_config, self.test_completed))

        # Transitions
        self.fsm.add_transition(Trans_To.BNT, Transition(StateName.BNT))
        self.fsm.add_transition(Trans_To.BMT, Transition(StateName.BMT))
        self.fsm.add_transition(Trans_To.BTT, Transition(StateName.BTT))
        self.fsm.add_transition(Trans_To.PPT, Transition(StateName.PPT))
        self.fsm.add_transition(Trans_To.RTT, Transition(StateName.RTT))
        self.fsm.add_transition(Trans_To.EMR, Transition(StateName.EMR))
        self.fsm.add_transition(Trans_To.TSL, Transition(StateName.TSL))
        
        # set default state
        self.fsm.set_state(StateName.TSL)

    def shutdown_hook(self):
        print("Global State Control Node shutting down...")


    def run(self):
        while not rospy.is_shutdown():
            try:
                if (self.test_completed):
                    if(self.test_completed[0] == "Completed"):
                        print("------------------------All Tasks Completed Successfully------------------------\n")
                        # return
                
                self.fsm.execute()
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
#==================================================================================================
