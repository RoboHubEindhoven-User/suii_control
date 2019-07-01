#! /usr/bin/env python

'''task_manager_handler.py: 
    - Implements the ROS interface between the Refbox Client and the Task Manager.
'''

__author__      = "Thanh, Thierry Zinkeng"

import rospy
from suii_msgs.msg import SuiiTask, SuiiTaskList

from atwork_ros_msgs.msg._TriggeredConveyorBeltStatus import *
from atwork_ros_msgs.msg._TaskInfo import *

from suii_task_manager.task_manager import TaskManager
from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_type import TaskType

from suii_mux_manager_comm.task_list import TaskList
from suii_mux_manager_comm.task import Task, TaskStatus
from suii_mux_manager_comm.action_list import ActionList
from refbox_converter import RefBoxConverter
from suii_mux_manager_comm.converter.mux_converter import MuxConverter

class TaskMangerHandler():
    def __init__(self):
        # Save params
        self.verbose = rospy.get_param('~verbose')
        self.yaml_path = rospy.get_param('~yaml_path')

        # Loggity log
        rospy.loginfo("Initializing Task Manager Node...")
        rospy.loginfo(" === PARAMETERS ==== ")
        rospy.loginfo("Navigation YAML: %s", self.yaml_path)
        rospy.loginfo("Verbose logging: %r", self.verbose)
        rospy.loginfo("===================")

        # for rotating table test
        # rospy.Subscriber("/suii_refbox_client/conveyor_belt_status", TriggeredConveyorBeltStatus, self.conveyor_callback)
        # fro receiving ROS tasks from the refbox client
        rospy.Subscriber("/suii_refbox_client/task_info", TaskInfo, self.task_callback)

        # The things
        self.task_manager  = TaskManager(holding_capacity=3, yaml_path=self.yaml_path, verbose=self.verbose)
        self.task_list_pub = rospy.Publisher('/suii_task_executor/input', SuiiTaskList, queue_size=10, latch=True)
        self.task_replan_sub = rospy.Subscriber('/suii_task_executor/replan', SuiiTaskList, self.replan_callback)
        # self.conveyor_belt = TriggeredConveyorBeltStatus()

        rospy.loginfo("Task Manager node initialized!")

    def task_callback(self, msg):
        rospy.loginfo("Tasks received. Converting to TaskObject...")
        self.task_manager.clear()                       # Clear shit
        self.task_cb_convert_data(msg)                  # Convert ros msg to TaskList()
        self.task_cb_optimize()                         # Send said TaskList() to TM
        self.send_to_mux(self.task_manager.output_list) # Send TM's ActionList() to Mux

    def replan_callback(self, msg):
        rospy.loginfo("Received replan data. Converting...")
        # Update data for TM
        self.task_manager.replan_list = MuxConverter.ros_to_action_list(msg)
        self.task_manager.error_index = msg.error_index

        # Replan 
        if (self.task_manager.replan()):
            self.send_to_mux(self.task_manager.output_list)
        else:
            rospy.logfatal("TM cannot handle this error.")

    def task_cb_convert_data(self, msg):
        converted_list = RefBoxConverter.ros_msg_to_task_list_object(msg)
        if converted_list is None:
            rospy.logerr("Cannot convert ROS msg to task list")
        else:
            rospy.loginfo("Converted to task list:")
            self.task_manager.initialize_list(converted_list)
            rospy.loginfo("##################")
            print(self.task_manager.task_list)
    
    def task_cb_optimize(self):
        optimize_ok = self.task_manager.optimize_list()
        if (optimize_ok):
            rospy.loginfo("Finished optimizing")
        else:
            rospy.logerr("Failed to optimize")

        rospy.loginfo("Final task list:")
        rospy.loginfo("##################")
        print(self.task_manager.output_list)

    def send_to_mux(self, result):
        if (len(result.task_list) != 0):
            rospy.loginfo("Publishing to mux...")
            # Converting action list to mux with status DISPATCHED
            mux_msg = MuxConverter.action_list_to_ros(result.task_list, int(TaskStatus.DISPATCHED))
            self.task_list_pub.publish(mux_msg)
        else:
            rospy.logwarn("Empty task list so didn't send to mux")

    def conveyor_callback(self, msg):
        self.conveyor_belt = msg

if __name__ == '__main__':
    rospy.init_node('suii_task_manager')
    tmh = TaskMangerHandler()
    rospy.spin()