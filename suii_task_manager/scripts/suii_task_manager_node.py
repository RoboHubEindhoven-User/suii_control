#! /usr/bin/env python
import rospy
from suii_task_manager.task_manager import TaskManager

if __name__ == "__main__":
    rospy.init_node("suii_task_manager_node")
    node = TaskManager()
    while not rospy.is_shutdown():
        try:
            node.publish_tasks()
        except rospy.ROSInterruptException:
            pass