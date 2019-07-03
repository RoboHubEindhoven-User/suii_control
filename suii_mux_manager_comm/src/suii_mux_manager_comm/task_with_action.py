#! /usr/bin/env python

from task import Task
from suii_protocol.task_protocol import TaskProtocol

## ===== TaskWithAction ===== ##
# Object to keep a MUX's command
# So: a task with a specific action: DRIVE, PICK, Etc
class TaskWithAction(Task):
    def __init__(self):
        Task.__init__(self)
        self.action = -1
        self.action_str = TaskProtocol.look_up_key(TaskProtocol.task_action_dict, self.action)

    def set_action(self, id):
        self.action = id
        self.action_str = TaskProtocol.look_up_key(TaskProtocol.task_action_dict, id)
    
    def copy_from_task(self, task):
        self.set_type(task.type)
        self.set_source(task.source)
        self.set_destination(task.destination)
        self.set_orientation(task.orientation)
        self.set_object(task.object)
        self.set_container(task.container)
    
    def make_duplicate(self):
        temp = TaskWithAction()
        temp.set_action(self.action)
        temp.set_source_dist(self.source_dist)
        temp.set_dest_dist(self.dest_dist)
        temp.set_type(self.type)
        temp.set_source(self.source)
        temp.set_destination(self.destination)
        temp.set_orientation(self.orientation)
        temp.set_object(self.object)
        temp.set_container(self.container)
        return temp

    def __eq__(self, other): # override '==' operator
        if (hasattr(other, 'action')):
            return Task.__eq__(self, other) and self.action == other.action
        else:
            return Task.__eq__(self, other)
    
    def __repr__(self):
        return "[%s] \n     %s" % (self.action_str, Task.__repr__(self))