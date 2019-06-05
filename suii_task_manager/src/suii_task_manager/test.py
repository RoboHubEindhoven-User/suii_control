#! /usr/bin/env python

import rospy
from task_manager_handler import TaskMangerHandler

rospy.init_node('suii_task_manager')
tmh = TaskMangerHandler()

while not rospy.is_shutdown():
    rospy.spin()