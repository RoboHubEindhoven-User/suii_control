#! /usr/bin/env python
import rospy
from suii_task_manager.task_manager_handler import TaskMangerHandler

if __name__ == "__main__":
    rospy.init_node("suii_task_manager_node")
    node = TaskMangerHandler()
    rospy.spin()