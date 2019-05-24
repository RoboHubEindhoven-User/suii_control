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

class TaskListTest(unittest.TestCase):
    def test_get_task_index(self):
        print ("Testing method: " + str(self._testMethodName))
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)
        
        self.assertEqual(tl.get_task_index(t2.type, t2.source, t2.destination, t2.object, t2.container), 1)

    def test_is_empty(self):
        tl = TaskList()
        self.assertTrue(tl.is_empty())

    def test_is_full(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        self.assertTrue(tl.is_full())
    
    def test_clear_task(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        tl.clear_task()
        self.assertEqual(len(tl.task_list), 0)
        
    def test_add_task(self):
        print ("Testing method: " + str(self._testMethodName))
        tl = TaskList()
        tl.capacity = 1
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)

        # Check length
        self.assertEqual(len(tl.task_list), 1)

        # Check that all info is correct
        self.assertEqual(tl.task_list[0].type, 1)
        self.assertEqual(tl.task_list[0].source, 2)
        self.assertEqual(tl.task_list[0].destination, 4)
        self.assertEqual(tl.task_list[0].container, 2)
        self.assertEqual(tl.task_list[0].object, 4)

        # Exceed capacity, should not add
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        self.assertFalse(tl.add_task(t2))
        self.assertEqual(len(tl.task_list), 1)

    def test_remove_task(self):
        print ("Testing method: " + str(self._testMethodName))
        
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)
        
        # Check return val of method
        self.assertEqual(tl.remove_task(t2), True)

        # Check if t2 was really removed
        self.assertEqual(len(tl.task_list), 2)

    def test_get_next_obj_to_pick(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        t1.picked = True 
        retval, index, task = tl.get_next_obj_to_pick()

        self.assertTrue(retval)
        self.assertNotEqual(index, -1)
        self.assertLess(index, len(tl.task_list))
        self.assertFalse(task.picked)
    
    def test_get_tasks_by_source(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        source_2 = tl.get_tasks_by_source(2)
        
        for item in source_2.task_list:
            self.assertEqual(item.source, 2)

    def test_get_tasks_by_destination(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)

        destination_4 = tl.get_tasks_by_destination(4)
        
        for item in destination_4.task_list:
            self.assertEqual(item.destination, 4)

    def test_get_unique_destination(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)
        
        unique_dest = tl.get_unique_destination()
        
        self.assertEqual(len(unique_dest), 2)
        
        for item in unique_dest:
            self.assertTrue(item == 3 or item == 4)
    
    def test_get_unique_source(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=2, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=2, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=3, destination=4, container=2, t_object=3)
        tl.add_task(t3)
        
        unique_src = tl.get_unique_source()
        
        self.assertEqual(len(unique_src), 2)
        
        for item in unique_src:
            self.assertTrue(item == 2 or item == 3)

    def test_sort_by_src_and_dest(self):
        tl = TaskList()
        tl.capacity = 3
        
        t1 = Task(t_type=1, source=1, destination=4, container=2, t_object=4)
        tl.add_task(t1)
        t2 = Task(t_type=1, source=3, destination=3, container=5, t_object=2)
        tl.add_task(t2)
        t3 = Task(t_type=1, source=1, destination=3, container=2, t_object=3)
        tl.add_task(t3)
        
        # Check that they're not sorted
        self.assertEqual(tl.task_list.index(t1), 0)
        self.assertEqual(tl.task_list.index(t2), 1)
        self.assertEqual(tl.task_list.index(t3), 2)

        tl.sort_by_src_and_dest()

        # Check that they should be sorted
        # Right order is: t3 -> t1 -> t2
        self.assertEqual(tl.task_list.index(t3), 0)
        self.assertEqual(tl.task_list.index(t1), 1)
        self.assertEqual(tl.task_list.index(t2), 2)            

if __name__ == '__main__':
    unittest.main()