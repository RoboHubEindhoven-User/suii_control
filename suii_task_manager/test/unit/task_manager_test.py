#!/usr/bin/env python

'''task_list_test.py:  
'''

__author__      = "Zinkeng Thierry"

# To import suii_task_manager
import sys, os
sys.path.append(os.path.abspath(os.path.join('../..', 'src/')))

import unittest
import rospy

from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task import Task
from suii_task_manager.task_with_action import TaskWithAction
from suii_task_manager.task_list import TaskList
from suii_task_manager.task_manager import TaskManager
from suii_task_manager.action_list import ActionList

class TaskManagerTest(unittest.TestCase):
    def test_constructor(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()
        self.assertEqual(tm.holding_capacity, tm.MAX_HOLDING_CAPACITY)
        tm = TaskManager(holding_capacity=5)
        self.assertEqual(tm.holding_capacity, 5)
    
    def test_format_drive(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()

        result = ActionList()
        dest_key = TaskProtocol.look_up_value(TaskProtocol.location_dict, "Workstation 1")
        action_key = TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "DRIVE")

        result.action_list = tm.format_drive(dest_key, result.action_list)
        self.assertEqual(len(result.action_list), 1)

        for item in result.action_list:
            self.assertEqual(item.action, action_key)
            self.assertEqual(item.action_str, "DRIVE")
            self.assertEqual(item.destination, dest_key)
            self.assertEqual(item.destination_str, "Workstation 1")

    def test_format_pick_task(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()

        result = ActionList()
        t = Task(t_type=1, source=2, destination=4, container=2, t_object=4)

        result.action_list = tm.format_pick_task(t, result.action_list)
        action_key = TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "PICK")

        self.assertEqual(len(result.action_list), 1)
        for item in result.action_list:
            self.assertEqual(item.action, action_key)
            self.assertEqual(item.action_str, "PICK")
            self.assertEqual(item.type, t.type)
            self.assertEqual(item.type_str, t.type_str)
            self.assertEqual(item.source, t.source)
            self.assertEqual(item.source_str, t.source_str)
            self.assertEqual(item.destination, t.destination)
            self.assertEqual(item.destination_str, t.destination_str)
            self.assertEqual(item.object, t.object)
            self.assertEqual(item.object_str, t.object_str)
            self.assertEqual(item.container, t.container)
            self.assertEqual(item.container_str, t.container_str)
    
    def test_format_pick (self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()
        tl = TaskList()
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        result = ActionList()
        result.action_list = tm.format_pick(tl, result.action_list)

        # If reached here, format_pick_task passed the test
        # So only assert len
        self.assertEquals(len(result.action_list), 3)
    
    def test_format_place_task(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()

        result = ActionList()
        t = Task(t_type=1, source=2, destination=4, container=2, t_object=4)

        result.action_list = tm.format_place_task(t, result.action_list)
        action_key = TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "PLACE")

        self.assertEqual(len(result.action_list), 1)

        for item in result.action_list:
            self.assertEqual(item.action, action_key)
            self.assertEqual(item.action_str, "PLACE")
            self.assertEqual(item.type, t.type)
            self.assertEqual(item.type_str, t.type_str)
            self.assertEqual(item.source, t.source)
            self.assertEqual(item.source_str, t.source_str)
            self.assertEqual(item.destination, t.destination)
            self.assertEqual(item.destination_str, t.destination_str)
            self.assertEqual(item.object, t.object)
            self.assertEqual(item.object_str, t.object_str)
            self.assertEqual(item.container, t.container)
            self.assertEqual(item.container_str, t.container_str)

    def test_format_place (self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()
        tl = TaskList()
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        result = ActionList()
        result.action_list = tm.format_place(tl, result.action_list)

        # If reached here, format_place_task passed the test
        # So only assert len
        self.assertEquals(len(result.action_list), 3)

    def test_format_set_of_tasks(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()
        tl = TaskList()
        
        t1 = Task(t_type=1, source=2, destination=4, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=3, destination=3, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=2, destination=4, t_object=3)
        tl.add_task(t3)
        
        result = ActionList()
        result.action_list = tm.format_set_of_tasks(tl, result.action_list)

        # for item in result:
        #     print(item)

        # TODO: Discuss workflow for this scenario
        # self.assertEquals(len(result), 3)

    def test_optimize_list(self):
        print ("Testing method: " + str(self._testMethodName))
        tm = TaskManager()
        tl = TaskList()
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        result = []
        result = tm.optimize_list(tl, result)
        for item in result:
            print(item)

    
if __name__ == '__main__':
    unittest.main()