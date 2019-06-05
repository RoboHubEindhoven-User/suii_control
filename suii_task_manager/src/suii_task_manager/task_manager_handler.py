#! /usr/bin/env python

'''task_manager_handler.py: 
    - Implements the ROS interface between the Refbox Client and the Task Manager.
'''

__author__      = "Thanh, Thierry Zinkeng"

import rospy
from atwork_ros_msgs.msg._TriggeredConveyorBeltStatus import *
from atwork_ros_msgs.msg._TaskInfo import *
from suii_task_manager.task_manager import TaskManager
from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task_list import TaskList
from suii_task_manager.protocol.enum_task_type import TaskType
from suii_task_manager.task import Task
from suii_task_manager.refbox_converter import RefBoxConverter
from suii_task_manager.action_list import ActionList

class TaskMangerHandler:
    def __init__(self):
        rospy.loginfo("initializing Task Manager Node...")
        # for rotating table test
        rospy.Subscriber("/suii_refbox_client/conveyor_belt_status", TriggeredConveyorBeltStatus, self.conveyor_callback)
        # fro receiving ROS tasks from the refbox client
        rospy.Subscriber("/suii_refbox_client/task_info", TaskInfo, self.task_callback)

        self.task_manager  = TaskManager()
        self.task_list     = TaskList()
        self.conveyor_belt = TriggeredConveyorBeltStatus()

        rospy.loginfo("Task Manager node initialized!")

    def conveyor_callback(self, msg):
        self.conveyor_belt = msg

    def add_transportation_task(self, task):
        tmp_task = RefBoxConverter.transportation_task_to_task(task) 
        if (tmp_task is None):
            rospy.logerr("Cannot convert transportation task!")
            return False
        self.task_list.add_task(tmp_task) 
        rospy.loginfo("Task converted!")
        rospy.loginfo(tmp_task)
        return True
        
    def add_navigation_task(self, task):
        tmp_task = RefBoxConverter.navigation_task_to_task(task) 
        if (tmp_task is None):
            return False
        self.task_list.add_task(tmp_task) 
        return True

    def process_one_task (self, task):
        if(task.type.data == int(TaskType.TRANSPORTATION)): # TRANSPORTATION
            rospy.loginfo("Transportation Task received")
            return self.add_transportation_task(task)
        elif (task.type.data == int(TaskType.NAVIGATION)): # NAVIGATION
            rospy.loginfo("Navigation task received")
            return self.add_navigation_task(task)
        return False

    def task_callback(self, msg):
        self.task_list.clear_task()
        tasks = msg.tasks
        for task in tasks:
            if self.process_one_task(task):
                rospy.loginfo("Task processed")
            else:
                rospy.logerr("Failed to process task")
                return
        if __debug__:
            rospy.loginfo("Converted task list: ")
            print(self.task_list)

        result = ActionList()
        if (self.task_manager.optimize_list(self.task_list, result.action_list)):
            rospy.loginfo("Finished optimizing")
        else:
            rospy.logerr("Failed to optimize")
        if __debug__:
            rospy.loginfo("Final task list:")
            print(result)
