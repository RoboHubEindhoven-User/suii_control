
#! /usr/bin/env python
from task_protocol import TaskProtocol

class Task:
    def __init__(self, t_type = -1, source = -1, destination = -1, container = -1, t_object = -1):
        self.type            = t_type
        self.type_str        = TaskProtocol.look_up_key(TaskProtocol.task_type_dict, self.type)
        self.source          = source
        self.source_str      = TaskProtocol.look_up_key(TaskProtocol.location_dict, self.source)
        self.destination     = destination
        self.destination_str = TaskProtocol.look_up_key(TaskProtocol.location_dict, self.destination)
        self.object          = t_object
        self.object_str      = TaskProtocol.look_up_key(TaskProtocol.object_dict, self.object)
        self.container       = container
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

    def __eq__(self, other): # override '==' operator
        return ((other.type == self.type) and (other.source == self.source) and (other.destination == self.destination) and (other.object == self.object) and (other.container == self.container))

    def __repr__(self): # string representation to use with print()
        return ("Type: " + self.type_str + ", Source: " + self.source_str + ", Destination: " + self.destination_str + ", Object: " + self.object_str + ", Container: " + self.container_str)
    