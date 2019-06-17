#! /usr/bin/env python
from task_with_action import TaskWithAction
from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_action import TaskActionType

## ===== ActionList ===== ##
# Object to manage a list of TaskWithAction[]
class ActionList():
    def __init__(self):
        self.task_list = []

    def __repr__(self):
        retstr = ''
        for action in self.task_list:
            retstr += repr(action) + '\n'
        return retstr

    def clear_task(self):
        del self.task_list[:]

    def make_duplicate(self):
        dup = ActionList()
        for item in self.task_list:
            dup.task_list.append(item.make_duplicate())
        return dup