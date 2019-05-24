# !/usr/bin/env python

import unittest
from task import Task
from protocol.enum_task_action import TaskActionType
from task_with_action import TaskWithAction 

class TaskWithActionTest(unittest.TestCase):
    def test_set_action(self):
        print ("Testing method: " + str(self._testMethodName))
        twa = TaskWithAction()
        twa.set_action(TaskActionType.PICK)
        self.assertEqual(twa.action, TaskActionType.PICK)

    def test_copy_from_task(self):
        t = Task()
        t.set_type(1)
        t.set_source(1)
        t.set_destination(3)
        t.set_object(2)
        t.set_container(2)
        twa = TaskWithAction()
        twa.copy_from_task(t)

        self.assertEqual(twa.type, 1)
        self.assertEqual(twa.source, 1)
        self.assertEqual(twa.destination, 3)
        self.assertEqual(twa.object, 2)
        self.assertEqual(twa.container, 2)
        

if __name__ == '__main__':
    unittest.main()