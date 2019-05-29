#! /usr/bin/env python
from suii_task_manager.task_with_action import TaskWithAction 
from suii_task_manager.task import Task
from suii_task_manager.task_list import TaskList
from suii_task_manager.task_protocol import TaskProtocol

class ActionList:
    def __init__(self):
        self.action_list = []

    def __repr__(self):
        retstr = ''
        for action in self.action_list:
            retstr += repr(action) + '\n'
        return retstr