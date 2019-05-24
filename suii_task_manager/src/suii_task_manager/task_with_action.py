#! /usr/bin/env python

from task import Task
from task_protocol import TaskProtocol

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
        self.set_object(task.object)
        self.set_container(task.container)
    
    def __repr__(self):
        return "<<'%s'>> %s" % (self.action_str, Task.__repr__(self))