
#! /usr/bin/env python
from suii_protocol.task_protocol import TaskProtocol
from enum import Enum

class TaskStatus(Enum):
    INACTIVE = 1, 'INACTIVE'
    SCHEDULED = 2, 'SCHEDULED'
    DISPATCHED = 3, 'DISPATCHED'
    FAILED = 4, 'FAILED'

    def __new__(cls, value, name):
        member = object.__new__(cls)
        member._value_ = value
        member.fullname = name
        return member

    def __int__(self):
        return self.value

    def __str__(self):
        return self.fullname

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
        self.source_dist     = -1
        self.dest_dist       = -1
        self.status          = int(TaskStatus.INACTIVE)
        self.error_code      = -1 # not in use yet

    # Definition of can be picked (TaskList needs)
    def can_be_picked(self):
        return self.status == int(TaskStatus.INACTIVE)

    def status_to_scheduled(self):
        self.status = int(TaskStatus.SCHEDULED)

    def set_source_dist(self, distance):
        self.source_dist = distance
    
    def set_dest_dist(self, distance):
        self.dest_dist = distance
    
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
    
    def make_duplicate(self):
        temp = Task()
        temp.set_source_dist(self.source_dist)
        temp.set_dest_dist(self.dest_dist)
        temp.set_type(self.type)
        temp.set_source(self.source)
        temp.set_destination(self.destination)
        temp.set_object(self.object)
        temp.set_container(self.container)
        return temp

    def is_dest_same_as_src(self):
        return self.destination == self.source

    def __eq__(self, other): # override '==' operator
        return ((other.type == self.type) and (other.source == self.source) 
        and (other.destination == self.destination) 
        and (other.object == self.object) 
        and (other.container == self.container))

    def __repr__(self): # string representation to use with print()
        retstr = ''
        if self.type_str != TaskProtocol.ERROR_STRING:
            retstr += "Type: " + self.type_str + "; "
        if self.source_str != TaskProtocol.ERROR_STRING:
            retstr += "Source: " + self.source_str + "; "
        if self.destination_str != TaskProtocol.ERROR_STRING:
            retstr += "Destination: " + self.destination_str + "; "
        if self.object_str != TaskProtocol.ERROR_STRING:
            retstr += "Object: " + self.object_str + "; "
        if self.container_str != TaskProtocol.ERROR_STRING:
            retstr += "Container: " + self.container_str + "; "
        if self.source_dist != -1:
            retstr += "Dist. from source: " + str(self.source_dist) + "; "
        if self.dest_dist != -1:
            retstr += "Dist. from dest: " + str(self.dest_dist) + "; "
        return retstr
    