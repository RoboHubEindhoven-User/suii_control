#! /usr/bin/env python
from suii_task_executor import TaskExecutor
import rospy

if __name__ == "__main__":
    node = TaskExecutor()
    while not rospy.is_shutdown:
        node.execute()
        