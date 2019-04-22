#!/usr/bin/env python

'''
'''

__author__      = "Zinkeng Thierry"

import rospy
from atwork_ros_msgs.msg._BenchmarkState import *
from atwork_ros_msgs.msg._BenchmarkScenario import *
from atwork_ros_msgs.msg._Inventory import *
from atwork_ros_msgs.msg._TriggeredConveyorBeltStatus import *
from atwork_ros_msgs.msg._TaskInfo import *
# from atwork_ros_msgs.msg._ import LocationIdentifer
from suii_control.state_machines import StateName
from suii_control.state_control.benchmark import *
from suii_control.utils.object_data import ObjectData
from suii_control.utils.service_area import ServiceArea
from suii_control.utils.task_specification import TaskSpecification, NavigationTask
from yaml import dump, load

class RefboxCommunication:
    def __init__(self):
        
        rospy.loginfo("initializing SUI RefBox Communication...")

        rospy.Subscriber("/suii_refbox_client/benchmark_state", BenchmarkState, self.benchmark_callback)

        rospy.Subscriber("/suii_refbox_client/conveyor_belt_status", TriggeredConveyorBeltStatus, self.conveyor_callback)

        rospy.Subscriber("/suii_refbox_client/inventory", Inventory, self.inventory_callback)

        rospy.Subscriber("/suii_refbox_client/task_info", TaskInfo, self.task_callback)

        rospy.loginfo("Refbox node initializing complete!")

        self.benchmark      = BenchmarkState()
        self.conveyor_belt  = TriggeredConveyorBeltStatus()
        self.inventory      = Inventory()
        self.task_info          = TaskInfo()



    def benchmark_callback(self, benchmark_state_msg):
        self.benchmark = benchmark_state_msg
        # rospy.loginfo("benchmark received, Description: " + self.benchmark.scenario.description.data)

    def conveyor_callback(self, conveyor_belt_status_msg):
        # rospy.loginfo("Triggered Conveyor Belt Status received")
        self.conveyor_belt = conveyor_belt_status_msg

    def inventory_callback(self, inventory_msg):
        rospy.loginfo("Inventory received")
        rospy.loginfo("Test: " + str(self.benchmark.scenario.type.data) + ", Description: " + self.benchmark.scenario.description.data)
        self.inventory = inventory_msg

    def task_callback(self, task_info_msg):
        rospy.loginfo("Task Info received")
        self.task_info = task_info_msg
        

    def shutdown_hook(self):
        print("Refbox Communication shutting down...")

    def get_test(self):
        test_type = self.benchmark.scenario.type.data
        if(test_type == BenchmarkScenario.BNT):
            return StateName.BNT
        elif(test_type == BenchmarkScenario.BMT):
            return StateName.BMT
        elif(test_type == BenchmarkScenario.BTT):
            return StateName.BTT
        elif(test_type == BenchmarkScenario.PPT):
            return StateName.PPT
        elif(test_type == BenchmarkScenario.CBT):
            return StateName.RTT
        
        return ""

    def get_benchmark_state(self):
        phase_type = BenchmarkPhaseProp.PREPARATION
        state_type = BenchmarkStateProp.STOPPED

        if(self.benchmark.phase.data == BenchmarkState.EXECUTION):
            phase_type = BenchmarkPhaseProp.EXECUTION
        if(self.benchmark.phase.data == BenchmarkState.CALIBRATION):
            phase_type = BenchmarkPhaseProp.CALIBRATION
        if(self.benchmark.phase.data == BenchmarkState.PREPARATION):
            phase_type = BenchmarkPhaseProp.PREPARATION


        if(self.benchmark.state.data == BenchmarkState.RUNNING):
            state_type = BenchmarkStateProp.RUNNING
        if(self.benchmark.state.data == BenchmarkState.PAUSED):
            state_type = BenchmarkStateProp.PAUSED
        if(self.benchmark.state.data == BenchmarkState.FINISHED):
            state_type = BenchmarkStateProp.FINISHED
        if(self.benchmark.state.data == BenchmarkState.STOPPED):
            state_type = BenchmarkStateProp.STOPPED

        return phase_type, state_type

    def get_navigation_tasks(self):
        nav_list = []
        
        for i in range(len(self.task_info.tasks)):
            # string = load(str(self.task_info.tasks[i].navigation_task.wait_time))
            # print string
            nav_list.append(
                NavigationTask(
                    ServiceArea(
                        self.task_info.tasks[i].navigation_task.location.type.data,
                        self.get_location_str(self.task_info.tasks[i].navigation_task.location.type.data),
                        self.task_info.tasks[i].navigation_task.location.instance_id.data,
                        self.task_info.tasks[i].navigation_task.location.description.data,
                        self.task_info.tasks[i].navigation_task.orientation.data,
                        self.get_orientation_str(self.task_info.tasks[i].navigation_task.orientation.data)
                    ),
                    self.task_info.tasks[i].navigation_task.wait_time
                )
            )

        return nav_list
        
    
    def get_transportation_tasks(self):

        task_list = []
        
        for i in range(len(self.task_info.tasks)):
            tool_object = ObjectData(
                self.task_info.tasks[i].transportation_task.object.type.data,
                self.task_info.tasks[i].transportation_task.object.type_id.data,
                self.task_info.tasks[i].transportation_task.object.instance_id.data,
                self.task_info.tasks[i].transportation_task.object.description.data 
            )

            container =  ObjectData(
                self.task_info.tasks[i].transportation_task.container.type.data,
                self.task_info.tasks[i].transportation_task.container.type_id.data,
                self.task_info.tasks[i].transportation_task.container.instance_id.data,
                self.task_info.tasks[i].transportation_task.container.description.data 
            )

            source = ServiceArea(
                self.task_info.tasks[i].transportation_task.source.type.data,
                self.get_location_str(self.task_info.tasks[i].transportation_task.source.type.data),
                self.task_info.tasks[i].transportation_task.source.instance_id.data,
                self.task_info.tasks[i].transportation_task.source.description.data 
            )
            
            destination = ServiceArea(
                self.task_info.tasks[i].transportation_task.destination.type.data,
                self.get_location_str(self.task_info.tasks[i].transportation_task.destination.type.data),
                self.task_info.tasks[i].transportation_task.destination.instance_id.data,
                self.task_info.tasks[i].transportation_task.destination.description.data 
            )
        
            task_list.append(TaskSpecification(tool_object, container, source, destination))

        return task_list

    def get_location_str(self, location):
        if (location == 1):
            return "SH"
        if (location == 2):
            return "WS"
        if (location == 3):
            return "CB"
        if (location == 4):
            return "WP"
        if (location == 5):
            return "PP"
        if (location == 6):
            return "ROBOT"
        return ""

    def get_orientation_str(self, orientation):
        if (orientation == 1):
            return "NORTH"
        if (orientation == 2):
            return "EAST"
        if (orientation == 3):
            return "SOUTH"
        if (orientation == 4):
            return "WEST"
        return ""

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
                
#==================================================================================================
if __name__ == '__main__':
    refbox = RefboxCommunication()
    while not rospy.is_shutdown():
        refbox.run()
    