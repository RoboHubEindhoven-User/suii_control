
#! /usr/bin/env python
 
class Task:
    def def __init__(self):
        self.type            = -1
        self.type_str        = ""
        self.source          = -1
        self.source_str      = ""
        self.destination     = -1
        self.destination_str = ""
        self.object          = -1
        self.object_str      = ""
        self.container       = -1
        self.container_str   = ""
        self.picked          = False
        self.placed          = False
        self.protocol        = TaskProtocol()
        
def set_type(self, t_type):
    self.type = t_type
    self.type_str = self.protocol.look_up_task_type_id(t_type)

def set_source(self, src):
    self.source = src
    self.source_str = self.protocol.look_up_location_id(src)

def set_destination(self, dest):
    self.destination = dest
    self.destination_str = self.protocol.look_up_location_id(dest)

def set_object(self, obj):
    self.object = obj
    self.object_str = self.protocol.look_up_object_id(obj)

def set_container(self, container):
    self.container = container
    self.container_str = self.protocol.look_up_container_id(container)

def is_dest_same_as_src(void):
    return dest == src

def is_equal_to(self, task):
    return task.type == self.type and task.source == self.source and task.destination == self.destination and task.object == self.object

def print_task_data(void):
   print("Type: " + self.type_str + ", Source: " + self.source_str + ", Destination: " + self.destination_str + ", Object: " + self.object_str + ", Container: " + self.container_str + "\n")
  