
#! /usr/bin/env python
from task_protocol import TaskProtocol

class Task:
    def __init__(self):
        self.type            = -1
        self.type_str        = TaskProtocol.look_up_key(TaskProtocol.task_type_dict, self.type)
        self.source          = -1
        self.source_str      = TaskProtocol.look_up_key(TaskProtocol.location_dict, self.source)
        self.destination     = -1
        self.destination_str = TaskProtocol.look_up_key(TaskProtocol.location_dict, self.destination)
        self.object          = -1
        self.object_str      = TaskProtocol.look_up_key(TaskProtocol.object_dict, self.object)
        self.container       = -1
        self.container_str   = TaskProtocol.look_up_key(TaskProtocol.container_dict, self.container)
        self.picked          = False
        self.placed          = False
    
    def set_type(self, t_type):
        self.type = t_type
        self.type_str = TaskProtocol.look_up_key(TaskProtocol.task_type_dict, t_type)

    def set_source(self, src):
        self.source = src
        self.source_str = TaskProtocol.look_up_key(TaskProtocol.location_dict, src)

    def set_destination(self, dest):
        self.destination = dest
        self.destination_str = TaskProtocol.look_up_key(TaskProtocol.location_dict, dest)

    def set_object(self, obj):
        self.object = obj
        self.object_str = TaskProtocol.look_up_key(TaskProtocol.object_dict, obj)

    def set_container(self, container):
        self.container = container
        self.container_str = TaskProtocol.look_up_key(TaskProtocol.container_dict, container)

    def is_dest_same_as_src(self):
        return self.destination == self.source

    def is_equal_to(self, task):
        return task.type == self.type and task.source == self.source and task.destination == self.destination and task.object == self.object

    def __repr__(self):
        return ("Type: " + self.type_str + ", Source: " + self.source_str + ", Destination: " + self.destination_str + ", Object: " + self.object_str + ", Container: " + self.container_str)
    