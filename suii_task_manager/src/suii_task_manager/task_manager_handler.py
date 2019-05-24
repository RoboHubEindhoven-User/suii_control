#! /usr/bin/env python

'''task_manager_handler.py: 
    - Implements the ROS interface between the Refbox Client and the Task Manager.
'''

__author__      = "Thanh, Thierry Zinkeng"

import rospy
from suii_task_manager.task_manager import TaskManager
from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task_list import TaskList
from atwork_ros_msgs.msg._TaskInfo import *

class TaskMangerHandler:
    def __init__(self):
        rospy.loginfo("initializing Task Manager Node...")
        # for rotating table test
        rospy.Subscriber("/suii_refbox_client/conveyor_belt_status", TriggeredConveyorBeltStatus, self.conveyor_callback)
        # fro receiving ROS tasks from the refbox client
        rospy.Subscriber("/suii_refbox_client/task_info", TaskInfo, self.task_callback)

        self.task_manager  = TaskManager()
        self.protocol      = TaskProtocol()
        self.task_list     = TaskList()
        self.conveyor_belt = TriggeredConveyorBeltStatus()

        rospy.loginfo("Task Manager node initialized!")

    def conveyor_callback(self, msg):
        self.conveyor_belt = msg

    def add_transportation_task (self, task):
        source = protocol.look_up_key(TaskProtocol.location_dict, task.transportation_task.source.type.data)
        if (source == -1):
            rospy.logerr("Look up failed for source")
            print("Source: " + task.transportation_task.source.description.data + "\n")
            return False
        
        destination = protocol.look_up_key(TaskProtocol.location_dict, task.transportation_task.destination.type.data)
        if (destination == -1):
            rospy.logerr("Destination look-up failed")
            print("Destination: " + task.transportation_task.destination.description.data + "\n") 
            return False
        
        object_to_pick = protocol.look_up_key(TaskProtocol.object_dict, task.transportation_task.object.description.data)
        if (object_to_pick == -1):
            rospy.logerr("Object look-up failed") 
            print("Object: " + task.transportation_task.object.description.data + "\n")
            return False
            
        container = -1
        if (task.transportation_task.container.description.data != ""):
            container = protocol.look_up_key(TaskProtocol.container_dict, task.transportation_task.container.description.data)
            if (container == -1):
                rospy.logerr("Container look-up failed") 
                print("Container: " + task.transportation_task.container.description.data + "\n")
                return False
        
        # create a new task
        tmp_task = Task()
        tmp_task.set_type(protocol.look_up_task_type_string("TRANSPORTATION"))
        tmp_task.set_source(source) 
        location = source if (source == -1) else destination
        tmp_task.set_destination(location)
        tmp_task.set_object(object_to_pick)
        tmp_task.set_container(container)
        self.task_list.add_task(temp)

        return True

    def add_navigation_task(self, task):
        destination = protocol.look_up_key(task.transportation_task.destination.description.data)
        if (destination == -1):
            rospy.logerr("Destination look-up failed")
            return False
        tmp_task = Task() 
        tmp_task.set_type(protocol.look_up_task_type_string("NAVIGATION")) 
        tmp_task.set_source(None) 
        tmp_task.setDest(destination)
        tmp_task.set_object(None)
        tmp_task.set_container(-1)
        task_list.add_task(temp)

        return True

    def process_one_task (self, task):
        if(task.type.data == 1): # TRANSPORTATION
            rospy.loginfo("Transportation Task received")
            return self.add_transportation_task(task)
        elif (task.type.data == 2): # NAVIGATION
            rospy.loginfo("Navigation task received")
            return self.add_navigation_task(task)
        return False

    def task_callback(self, msg):
        self.task_list.clear_task()
        task_info = TaskInfo()
        tasks = msg.tasks
        for task in tasks:
            if self.process_one_task(task)):
                rospy.loginfo("Task processed")
            else:
                return
                
        (result, self.task_list, task_list_wa) = task_manager.optimize_list(self.task_list)

        if (result):
            rospy.loginfo("Finished optimizing")
        else:
            rospy.logerr("Failed to optimize")
        if __debug__:
            rospy.loginfo("Final task list:")
            for task in task_list_wa:
                task.print_task_data()
