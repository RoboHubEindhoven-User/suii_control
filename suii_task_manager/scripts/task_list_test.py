#!/usr/bin/env python

'''task_list_test.py:  
'''

__author__      = "Zinkeng Thierry"

import unittest
import rospy
# from suii_task_manager.src.suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task_protocol import TaskProtocol
# from suii_task_manager.task_protocol import TaskProtocol
# from zinki_smachine.src.zinki_smachine.state import State
# from suii_control.suii_task_manager.task_protocol import *
from task import Task
from task_with_action import TaskWithAction
from task_list import TaskList
from task_list import TaskList

class TaskListTest(unittest.TestCase):
    def test_add_task(self):
        print ("Testing method: " + str(self._testMethodName))
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(1, 1, 3, 2, 4)
        tl.add_task(t1)
        t2 = Task(1, 1, 3, 2, 4)
        tl.add_task(t2)
        t3 = Task(1, 1, 3, 2, 4)
        tl.add_task(t3)

        self.assertEqual(len(tl.task_list), 3)

    def test_remove_task(self):
        print ("Testing method: " + str(self._testMethodName))
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(1, 1, 3, 2, 4)
        tl.add_task(t1)
        t2 = Task(1, 1, 3, 2, 4)
        tl.add_task(t2)
        t3 = Task(1, 1, 3, 2, 4)
        tl.add_task(t3)
        result = tl.remove_task(t2)

        self.assertEqual(result, True)
        

if __name__ == '__main__':
    unittest.main()